/**
 * CAMERA 菜单（v4.1.3）
 *
 *   CAMERA
 *   ├─ ADD DEVICE          二级：扫描列表 + RESCAN/EXIT，FRONT 配对
 *   ├─ DEVICE MANAGEMENT   二级：已保存设备列表 + EXIT
 *   │    └─ 选中设备        三级：CONNECT / FORGET / BACK
 *   └─ EXIT
 *
 * 导航：MID=下移 FRONT=确定 MID长按=返回上一级 FRONT长按=清除全部配对
 */
#include "CameraPage.h"
#include "applog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <M5Unified.h>
#include <NimBLEDevice.h>
#include <esp_mac.h>
#include <host/ble_hs.h>

#include "compat.h"
#include "CameraList.h"
#include "Device.h"
#include "FurbleControl.h"
#include "Scan.h"

using namespace Furble;

namespace {

enum class Page { MENU, ADD, MGMT, DEVICE };
enum class CamState { IDLE, SCANNING, CONNECTING, ACTIVE, FAILED };

static Page s_page = Page::MENU;
static CamState s_state = CamState::IDLE;
static int s_cursor = 0;
static int s_devSel = 0;  // DEVICE 页对应设备下标
static volatile bool s_dirty = true;
static bool s_exitReq = false;

static bool s_gpsOk = false;
static int s_sats = 0;
static uint32_t s_sentCount = 0;
static uint32_t s_lastSentMs = 0;

static bool s_confirmClear = false;
static uint32_t s_confirmMs = 0;
static bool s_confirmForget = false;
static uint32_t s_forgetMs = 0;
static uint32_t s_failMs = 0;

// 进入添加/管理/清配对前都要先断开，等 Control 线程释放 Camera* 引用后
// 才能销毁列表，否则连接重试中的线程会踩到已释放对象（LoadProhibited）。
enum class Pending { NONE, ADD, MGMT, CLEAR };
static Pending  s_pending = Pending::NONE;
static uint32_t s_pendingMs = 0;

static bool busy(void) { return s_pending != Pending::NONE; }

static char s_snap[512];

static bool portrait(void) {
  auto &d = M5.Display;
  return d.width() < d.height();
}

static int rowY(int i) {
  return portrait() ? 52 + i * 32 : 22 + i * 18;
}

static int maxRows(void) {
  return 5;
}

// 以当前字体绘制，超过像素宽度时截断并追加 ".."（UTF-8 安全）
static void printClipped(int x, int y, int maxW, const char *s) {
  auto &d = M5.Display;
  if (d.textWidth(s) <= maxW) {
    d.setCursor(x, y);
    d.print(s);
    return;
  }
  char buf[64];
  strlcpy(buf, s, sizeof buf);
  size_t n = strlen(buf);
  const int ell = d.textWidth("..");
  while (n > 0) {
    do {
      n--;
    } while (n > 0 && ((unsigned char)buf[n] & 0xC0) == 0x80);
    buf[n] = 0;
    if (d.textWidth(buf) <= maxW - ell) break;
  }
  d.setCursor(x, y);
  d.print(buf);
  d.print("..");
}

static void uiRow(const char *s, int i, bool active) {
  auto &d = M5.Display;
  int y = rowY(i);
  if (portrait()) {
    if (active) d.fillRect(2, y - 3, d.width() - 4, 22, TFT_YELLOW);
    d.setFont(&fonts::efontCN_16);
    d.setTextSize(1);
    d.setTextColor(active ? TFT_BLACK : TFT_WHITE, active ? TFT_YELLOW : TFT_BLACK);
    printClipped(6, y, d.width() - 10, s);
  } else {
    if (active) d.fillRect(2, y, 236, 16, TFT_YELLOW);
    d.setFont(&fonts::efontCN_16);
    d.setTextSize(1);
    d.setTextColor(active ? TFT_BLACK : TFT_WHITE, active ? TFT_YELLOW : TFT_BLACK);
    printClipped(4, y, 232, s);
  }
  d.setTextSize(1);
}

static void uiTitle(const char *t) {
  auto &d = M5.Display;
  d.setFont(&fonts::efontCN_16);
  d.setTextSize(1);
  d.setTextColor(TFT_CYAN, TFT_BLACK);
  d.setCursor(4, portrait() ? 4 : 0);
  d.print(t);
  d.setTextSize(1);
}

static void uiStatus(const char *t, uint16_t color) {
  auto &d = M5.Display;
  d.setFont(&fonts::efontCN_16);
  d.setTextSize(1);
  d.setTextColor(color, TFT_BLACK);
  if (portrait()) {
    printClipped(4, 24, d.width() - 8, t);
  } else {
    printClipped(120, 2, d.width() - 124, t);
  }
}

static bool connected(void) {
  return Control::getInstance().getState() == Control::STATE_ACTIVE;
}

static int itemCount(void) {
  return (int)CameraList::size();
}

// 扫描前确保没有挂起的连接（否则 BLE 栈并发卡死）
static void ensureNoPendingConnect(void) {
  auto st = Control::getInstance().getState();
  if (st != Control::STATE_IDLE && st != Control::STATE_ACTIVE) {
    Control::getInstance().disconnect();
  }
}

static void startScan(void) {
  auto &scan = Scan::getInstance();
  scan.stop();
  scan.clear();
  scan.start(
      [](void *p) {
        (void)p;
        s_dirty = true;
      },
      nullptr);
  s_state = CamState::SCANNING;
  appLog("[CAM] scan started");
  s_dirty = true;
}

static void doClearNow(void) {
  Scan::getInstance().stop();
  CameraList::clear();
  CameraList::load();
  for (int guard = 0; guard < 16 && CameraList::size() > 0; guard++) {
    CameraList::remove(CameraList::get(0));
    CameraList::clear();
    CameraList::load();
  }
  NimBLEDevice::deleteAllBonds();
  Serial.println("[CAM] pairings + bonds cleared");
  appLog("[CAM] pairings cleared");
  s_confirmClear = false;
  s_state = CamState::IDLE;
  s_dirty = true;
}

static void runPending(void) {
  Pending p = s_pending;
  s_pending = Pending::NONE;
  switch (p) {
    case Pending::ADD:
      s_page = Page::ADD;
      s_cursor = 0;
      CameraList::clear();
      startScan();
      appLog("[CAM] add-device ready");
      break;
    case Pending::MGMT:
      s_page = Page::MGMT;
      s_cursor = 0;
      CameraList::clear();
      CameraList::load();
      appLog("[CAM] mgmt ready (%d)", itemCount());
      break;
    case Pending::CLEAR:
      doClearNow();
      break;
    default:
      break;
  }
  s_dirty = true;
}

// 先请求断开；Control 回到 IDLE（或 3 秒超时）后才销毁列表。
static void requestPending(Pending p) {
  s_pending = p;
  s_pendingMs = millis();
  appLog("[CAM] pending=%d", (int)p);
  auto &ctl = Control::getInstance();
  if (ctl.getState() != Control::STATE_IDLE) {
    ctl.disconnect();
  } else {
    runPending();
  }
  s_dirty = true;
}

static void connectOne(int index) {
  int n = itemCount();
  if (index < 0 || index >= n) return;
  ensureNoPendingConnect();
  auto &ctl = Control::getInstance();
  auto *cam = CameraList::get(index);
  Serial.printf("[CAM] connect %s\n", cam->getName().c_str());
  cam->setActive(true);
  ctl.addActive(cam);
  Scan::getInstance().stop();
  ctl.connectAll(false);
  s_state = CamState::CONNECTING;
  s_dirty = true;
}

// 忘记单个设备（s_devSel）
static void forgetOne(void) {
  int n = itemCount();
  if (s_devSel < 0 || s_devSel >= n) return;
  auto *cam = CameraList::get(s_devSel);
  Serial.printf("[CAM] forget %s\n", cam->getName().c_str());
  CameraList::remove(cam);
  CameraList::clear();
  CameraList::load();
  s_confirmForget = false;
  s_devSel = 0;
  s_page = Page::MGMT;
  s_cursor = 0;
  s_dirty = true;
}

// ---------------------------------------------------------------- 状态文本

static const char *statusLabel(void) {
  auto st = Control::getInstance().getState();
  static char b[48];
  if (busy()) return "断开中...";
  if (s_confirmClear) return "清配对? 是/否";
  if (s_confirmForget) return "忘记? 是/否";
  if (s_page == Page::DEVICE) {
    int n = itemCount();
    if (s_devSel < n) {
      snprintf(b, sizeof b, "设备 %.24s", CameraList::get(s_devSel)->getName().c_str());
      return b;
    }
  }
  if (st == Control::STATE_ACTIVE) {
    if (s_gpsOk) snprintf(b, sizeof b, "已连 %d星 推%lu", s_sats, (unsigned long)s_sentCount);
    else snprintf(b, sizeof b, "已连 无定位");
    return b;
  }
  if (st == Control::STATE_CONNECTING || st == Control::STATE_CONNECT) return "连接中...";
  if (st == Control::STATE_CONNECT_FAILED || s_state == CamState::FAILED) return "失败";
  if (s_state == CamState::SCANNING) return "扫描中...";
  return "待机";
}

static uint16_t statusColor(void) {
  auto st = Control::getInstance().getState();
  if (s_confirmClear || s_confirmForget) return TFT_RED;
  if (st == Control::STATE_ACTIVE) return TFT_GREEN;
  if (st == Control::STATE_CONNECT_FAILED || s_state == CamState::FAILED) return TFT_RED;
  return TFT_CYAN;
}

// ---------------------------------------------------------------- 页面行构建

static int visibleCamRows(int reserved) {
  return maxRows() - reserved;
}

static int buildRows(char rows[][48], int *topOut) {
  int nrows = 0;
  int n = itemCount();
  int top = 0;

  auto addRow = [&](const char *fmt, ...) {
    if (nrows >= 8) return;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(rows[nrows], sizeof rows[nrows], fmt, ap);
    va_end(ap);
    nrows++;
  };

  switch (s_page) {
    case Page::MENU:
      addRow("添加设备");
      addRow("设备管理");
      addRow("退出");
      break;

    case Page::ADD: {
      int vis = visibleCamRows(2);  // RESCAN + EXIT
      if (n > vis) {
        if (s_cursor < n) {
          top = s_cursor - vis + 1;
          if (top > n - vis) top = n - vis;
          if (top < 0) top = 0;
        } else {
          top = n - vis;
        }
      }
      int shown = n - top;
      if (shown > vis) shown = vis;
      for (int i = 0; i < shown; i++) addRow("%s", CameraList::get(top + i)->getName().c_str());
      addRow("重新扫描");
      addRow("退出");
      break;
    }

    case Page::MGMT: {
      int vis = visibleCamRows(1);  // EXIT
      if (n > vis) {
        if (s_cursor < n) {
          top = s_cursor - vis + 1;
          if (top > n - vis) top = n - vis;
          if (top < 0) top = 0;
        } else {
          top = n - vis;
        }
      }
      int shown = n - top;
      if (shown > vis) shown = vis;
      if (n == 0) {
        addRow("退出");
      } else {
        for (int i = 0; i < shown; i++) addRow("%s", CameraList::get(top + i)->getName().c_str());
        addRow("退出");
      }
      break;
    }

    case Page::DEVICE: {
      int n2 = itemCount();
      bool isConn = s_devSel < n2 && CameraList::get(s_devSel)->isActive();
      addRow(isConn ? "已连接" : "连接");
      if (n2 > 0 && s_devSel < n2 && CameraList::get(s_devSel)->isActive()) {
        // 已连接时该项仅作状态显示
      }
      addRow("忘记");
      addRow("返回");
      break;
    }
  }

  *topOut = top;
  return nrows;
}

static void drawPage(void) {
  auto &d = M5.Display;
  char rows[8][48];
  int top = 0;
  int nrows = buildRows(rows, &top);
  int n = itemCount();

  char snap[512];
  snprintf(snap, sizeof snap, "P%d|C%d|D%d|S%d|", (int)s_page, s_cursor, s_devSel, (int)s_state);
  size_t off = strlen(snap);
  for (int i = 0; i < nrows && off < sizeof snap - 50; i++)
    off += snprintf(snap + off, sizeof snap - off, "%s|", rows[i]);
  snprintf(snap + off, sizeof snap - off, "%s", statusLabel());

  if (strcmp(snap, s_snap) == 0) {
    s_dirty = false;
    return;
  }
  strlcpy(s_snap, snap, sizeof s_snap);

  d.fillScreen(TFT_BLACK);
  uiTitle("相机");
  uiStatus(statusLabel(), statusColor());

  // 高亮行
  int selRow = -1;
  if (s_page == Page::MENU || s_page == Page::DEVICE) {
    selRow = s_cursor;
  } else if (s_page == Page::ADD) {
    if (n == 0) selRow = s_cursor;  // RESCAN / EXIT
    else if (s_cursor < n) {
      if (s_cursor >= top && s_cursor < top + (nrows - 2)) selRow = s_cursor - top;
    } else {
      selRow = (nrows - 2) + (s_cursor - n);  // RESCAN / EXIT
    }
  } else if (s_page == Page::MGMT) {
    if (n == 0) selRow = 0;  // EXIT
    else if (s_cursor < n) {
      if (s_cursor >= top && s_cursor < top + (nrows - 1)) selRow = s_cursor - top;
    } else {
      selRow = nrows - 1;  // EXIT
    }
  }

  for (int i = 0; i < nrows && i < maxRows(); i++) uiRow(rows[i], i, i == selRow);


}

}  // namespace

namespace FurbCam {

void enter(void) {
  s_page = Page::MENU;
  s_cursor = 0;
  s_exitReq = false;
  s_pending = Pending::NONE;
  s_snap[0] = 0;
  s_dirty = true;
  appLog("[CAM] menu entered");
}

void leave(void) {
  Scan::getInstance().stop();
  Serial.println("[CAM] page left (link kept)");
}

void draw(void) {
  drawPage();
}

bool tick(uint32_t now) {
  auto st = Control::getInstance().getState();

  // 挂起操作：等 Control 释放 Camera* 引用后执行（最多等 3 s）
  if (s_pending != Pending::NONE &&
      (st == Control::STATE_IDLE || now - s_pendingMs > 3000)) {
    runPending();
  }
  if (st == Control::STATE_ACTIVE && s_state != CamState::ACTIVE) {
    s_state = CamState::ACTIVE;
    s_dirty = true;
    for (auto &t : Control::getInstance().getTargets()) {
      CameraList::save(t->getCamera());
    }
    Serial.println("[CAM] linked + saved");
  }
  if (st == Control::STATE_CONNECT_FAILED && s_state != CamState::FAILED) {
    s_state = CamState::FAILED;
    s_failMs = now;
    s_dirty = true;
  }
  if (s_state == CamState::FAILED && now - s_failMs > 3000 && s_page == Page::ADD) {
    ensureNoPendingConnect();
    startScan();
  }
  if (s_confirmClear && now - s_confirmMs > 5000) {
    s_confirmClear = false;
    s_dirty = true;
  }
  if (s_confirmForget && now - s_forgetMs > 5000) {
    s_confirmForget = false;
    s_dirty = true;
  }

  static uint32_t lastDraw = 0;
  if (s_dirty || now - lastDraw >= 1000) {
    lastDraw = now;
    drawPage();
  }

  if (s_exitReq) {
    s_exitReq = false;
    return true;
  }
  return false;
}

void front(void) {
  if (busy()) return;  // 等断开完成、列表就绪后再接受操作
  if (s_confirmClear) {
    requestPending(Pending::CLEAR);
    return;
  }
  if (s_confirmForget) {
    forgetOne();
    return;
  }

  int n = itemCount();

  switch (s_page) {
    case Page::MENU:
      if (s_cursor == 0) {  // ADD DEVICE
        requestPending(Pending::ADD);
      } else if (s_cursor == 1) {  // DEVICE MANAGEMENT
        requestPending(Pending::MGMT);
      } else {  // EXIT
        s_exitReq = true;
      }
      s_dirty = true;
      break;

    case Page::ADD: {
      if (n == 0) {
        if (s_cursor == 0) {  // RESCAN
          requestPending(Pending::ADD);
        } else {              // EXIT
          Scan::getInstance().stop();
          s_page = Page::MENU;
          s_cursor = 0;
        }
        s_dirty = true;
        break;
      }
      if (s_cursor < n) {
        connectOne(s_cursor);
      } else if (s_cursor == n) {  // RESCAN
        requestPending(Pending::ADD);
      } else {                     // EXIT
        Scan::getInstance().stop();
        s_page = Page::MENU;
        s_cursor = 0;
      }
      s_dirty = true;
      break;
    }

    case Page::MGMT:
      if (n == 0) {
        s_page = Page::MENU;
        s_cursor = 0;
      } else if (s_cursor < n) {  // 选中设备 → 三级菜单
        s_devSel = s_cursor;
        s_cursor = 0;
        s_page = Page::DEVICE;
      } else {                    // EXIT
        s_page = Page::MENU;
        s_cursor = 0;
      }
      s_dirty = true;
      break;

    case Page::DEVICE:
      if (s_cursor == 0) {        // CONNECT
        connectOne(s_devSel);
      } else if (s_cursor == 1) { // FORGET
        s_confirmForget = true;
        s_forgetMs = millis();
      } else {                    // BACK
        s_page = Page::MGMT;
        s_cursor = s_devSel;
      }
      s_dirty = true;
      break;
  }
}

void mid(void) {
  if (busy()) return;
  if (s_confirmClear) { s_confirmClear = false; s_dirty = true; return; }
  if (s_confirmForget) { s_confirmForget = false; s_dirty = true; return; }

  int n = itemCount();
  int total = 0;
  switch (s_page) {
    case Page::MENU:   total = 3; break;
    case Page::ADD:    total = (n > 0) ? n + 2 : 2; break;
    case Page::MGMT:   total = (n > 0) ? n + 1 : 1; break;
    case Page::DEVICE: total = 3; break;
  }
  if (total > 0) s_cursor = (s_cursor + 1) % total;
  s_dirty = true;
}

void back(void) {
  if (busy()) {  // 等待断开期间允许取消
    s_pending = Pending::NONE;
    s_page = Page::MENU;
    s_cursor = 0;
    s_dirty = true;
    appLog("[CAM] pending cancelled");
    return;
  }
  if (s_confirmClear) { s_confirmClear = false; s_dirty = true; return; }
  if (s_confirmForget) { s_confirmForget = false; s_dirty = true; return; }
  switch (s_page) {
    case Page::DEVICE:
      s_page = Page::MGMT;
      s_cursor = s_devSel;
      break;
    case Page::MGMT:
    case Page::ADD:
      Scan::getInstance().stop();
      s_page = Page::MENU;
      s_cursor = 0;
      break;
    case Page::MENU:
      s_exitReq = true;
      break;
  }
  s_dirty = true;
}

bool isActive(void) {
  return connected();
}

void requestClear(void) {
  if (connected() || CameraList::size() > 0) {
    s_confirmClear = true;
    s_confirmMs = millis();
    s_dirty = true;
  }
}

void clearPairings(void) {
  requestPending(Pending::CLEAR);
}

void getLinkInfo(LinkInfo &o) {
  o.active = connected();
  o.count = 0;
  o.name[0] = 0;
  o.gpsOk = s_gpsOk;
  o.sats = s_sats;
  o.sent = s_sentCount;
  if (o.active) {
    for (auto &t : Control::getInstance().getTargets()) {
      if (o.count == 0) strlcpy(o.name, t->getCamera()->getName().c_str(), sizeof o.name);
      o.count++;
    }
  }
}

void relayGPS(bool fix, double lat, double lon, float alt, int sats, int year, int month, int day,
              int hour, int minute, int second) {
  s_gpsOk = fix;
  s_sats = fix ? sats : 0;
  s_dirty = true;
  if (!fix) return;
  Camera::gps_t g = {lat, lon, (double)alt, (unsigned)sats};
  Camera::timesync_t t = {(unsigned)year, (unsigned)month, (unsigned)day,
                          (unsigned)hour, (unsigned)minute, (unsigned)second, 0};
  Control::getInstance().updateGPS(g, t);
  if (connected()) {
    s_sentCount++;
    s_lastSentMs = millis();
  }
}

}  // namespace FurbCam
