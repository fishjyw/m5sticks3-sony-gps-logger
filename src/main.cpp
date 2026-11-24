/*
 * GPS Track Logger for M5StickS3 + M5Stack GPS Unit v1.1 (AT6668)
 *
 * Version 3.2.0 (2026-11-22) — ported to ESP-IDF as gpscam v4.0.0.
 *
 * Records GPX tracks to internal LittleFS (~4.9MB), 1 point / 10 s.
 * Export: over USB serial. Run export_gpx.py on the PC (or send 'D' on the
 * serial port) to pull every .GPX off the device. A WiFi AP is NOT usable
 * on this toolchain: esp_wifi_init() panics with Arduino 2.0.7 qio_opi
 * prebuilt libs (confirmed with a minimal M5+WiFi firmware).
 * Downloaded .GPX -> geotag photos (GeoSetter / Lightroom / exiftool).
 *
 * PHYSICAL BUTTONS (M5Stack docs / M5Unified 0.2.29 / reseller spec):
 *   FRONT big key      = KEY1 = GPIO11 = M5.BtnA   (programmable)
 *   SIDE middle key    = KEY2 = GPIO12 = M5.BtnB   (programmable)
 *   SIDE edge key      = POWER key, wired to M5PM1 (PY32) PMIC, NOT to ESP32.
 *                        Hardware only: single click = power on / reset,
 *                        double click = power off, long hold = download mode.
 *                        Firmware must NOT touch it (polling its IRQ flags
 *                        would break the PMIC double-click power-off).
 *
 * MAIN SCREEN (135x240 portrait only since 4.5.0):
 *   Portrait rows: clock / state / elapsed, a divider, SAT + LAT + LON + ALT
 *   (one per line), a divider, AVG + MAX + BAT as size2, a divider, FREE +
 *   FILES, and the "MID:MENU" hint at the bottom.
 *   FRONT short : start / stop recording
 *   FRONT hold  : graceful power off (flush GPX first)
 *   MID   short : open the full-screen MENU page (blocked while recording)
 *   MID   hold  : OUTPUT export screen (shortcut)
 *
 * MENU PAGE (full-screen, MID enters it from the main screen):
 *   MID   short : move to the next item (wraps)
 *   FRONT short : OK - run the highlighted item
 *   The menu page AUTO-RETURNS to the main screen after 10 s idle.
 *   FILES       : English GPX list. MID = next row, FRONT = confirm.
 *                 Selecting a file opens a DELETE / CANCEL dialog.
 *                 A trailing "EXIT" row returns to the main screen.
 *                 Portrait shows 7 rows.
 *   SETUP       : English slider page: BRIGHTNESS / DIM AFTER / REC RATE /
 *                 AUTO OFF / GNSS SYS / ROTATE / EXIT. MID = next row,
 *                 FRONT = next value. REC RATE is linked to the main
 *                 screen's "%us/pt" field. Landscape draws label | slider |
 *                 value in three columns; portrait stacks label+value above a
 *                 full-width slider (three columns need 192 px > 135 px).
 *   OUTPUT      : choose a PC export client (browser / python / terminal)
 *   POWEROFF    : save track (if any) and shut down
 *   EXIT        : back to the main screen
 *   Long holds are ignored while on any sub page.
 *   Serial 'A'/'B' simulate FRONT/MID clicks for headless testing.
 *   Serial 'W'  : cycle rotation 0->1->2->3 and lock that axis
 *   Serial 'S'  : swap FRONT/MID roles if the physical map is reversed
 *
 * SUB PAGES: ALL labels are plain ASCII (Font0). No CJK font, no bitmap
 * glyph table, no sprite / DMA: those paths trip TG1WDT_SYS_RST on this
 * Arduino 2.0.7 + qio_opi toolchain. The LCD font stays Font0 forever.
 *
 * IDLE BEHAVIOUR:
 *   - Backlight dims to a very low level after DIM_AFTER_MS without a key.
 *   - The FIRST key press while dimmed ONLY wakes the screen: its click AND
 *     its long hold are discarded (wake-only guard on the whole down->up
 *     cycle). Press the key a second time to actually run its function.
 *   - A WAKE_GRACE_MS window starts when that wake press is released: the
 *     screen stays bright and the next press is NEVER treated as a wake,
 *     even if DIM AFTER is short. Any real click/hold clears the wake banner
 *     at once, so it cannot be repainted by the 1 Hz main-screen redraw.
 *   - After AUTO_OFF_MS idle AND not recording AND not on export screen AND
 *     USB unplugged -> save and power off. USB VBUS is checked once/second
 *     (cached) so exporting from a computer never triggers a reboot loop.
 *
 * EXPORT (OUTPUT menu):
 *   - Three host-side clients (browser Web Serial page / Python script /
 *     native shell + PowerShell scripts) all share ONE protocol: the PC
 *     sends 'D', the device streams #GPX_BEGIN|name / raw GPX / #GPX_END /
 *     #GPX_DONE|n over the same USB CDC link. No transport switching.
 *   - OUTPUT -> pick a client -> return to the main screen with a
 *     "USE <filename>" hint. NO dedicated full-screen waiting page: that
 *     page displayed during a CDC GPX burst trips TG1WDT_SYS_RST here.
 *     The chosen method is saved in settings.cfg and highlighted next time.
 *
 * FEEDBACK / BATTERY:
 *   - The built-in ES8311 / I2S speaker is NOT usable on this Arduino 2.0.7
 *     + qio_opi (Octal PSRAM) build: any I2S playback (M5Unified Speaker or
 *     raw ESP-IDF I2S) trips TG1WDT_SYS_RST. Start / stop recording therefore
 *     give distinct BACKLIGHT PULSE patterns instead (PROMPT in SETTINGS).
 *   - Battery <= 5% while recording stops the track after 2 consecutive
 *     checks and locks new recordings until it recovers above 10%.
 *
 * UI: partial redraw only (no full-screen clear in the loop) -> no flicker.
 * A "MID:MENU" hint line sits at the bottom of the main screen. Satellites
 * row is always yellow. Track files are named by GPS local time:
 * /YYYYMMDD_HHMMSS.GPX (UTC+8).
 *
 * v11:
 *   - GNSS constellation selector (SETUP -> GNSS SYS), CASIC $PCAS04 sent
 *     over the existing bidirectional UART and re-applied at every boot.
 *     The active mode is shown on the yellow satellites row.
 *   - Screen orientation (SETUP -> ROTATE): manual 4-position selector
 *     (see v3.0.1 below).
 *
 * v2.0.0:
 *   - Every page has a portrait layout (rowsPort). Landscape was removed
 *     in 4.5.0.
 *     Serial: N=cycle GNSS, W=cycle rotation.
 *
 * v3.0.1:
 *   - REMOVED gravity AUTO rotation. The 4-way detector proved unreliable
 *     (a wrong pair of landscape poses, and spurious turns while the device
 *     sat on a desk), so ROTATE is now a plain manual 4-position selector:
 *     0 DEG / 90 DEG / 180 DEG / 270 DEG, each mapping 1:1 to setRotation().
 *     The IMU is no longer polled at all (M5Unified still initialises it).
 *     settings.cfg gained a format marker so old AUTO/PORT/LAND values are
 *     migrated once instead of being misread as raw rotations.
 */

#include "compat.h"
#include <M5Unified.h>
#include <esp_log.h>
#include <esp_pm.h>
#include <esp_private/esp_clk.h>  // esp_clk_cpu_freq()
#include <soc/rtc.h>             // rtc_clk_cpu_freq_get_config()
#include <esp_system.h>          // esp_reset_reason()
#include <nvs_flash.h>

#include <NimBLEDevice.h>
#include <esp_mac.h>
#include <host/ble_hs.h>

#include "CameraList.h"
#include "CameraPage.h"
#include "Device.h"
#include "FurbleControl.h"

#ifndef GPS_RX_PIN
#define GPS_RX_PIN 9
#endif
#ifndef GPS_TX_PIN
#define GPS_TX_PIN 10
#endif
#ifndef GPS_BAUD
#define GPS_BAUD 115200
#endif
#ifndef TZ_OFFSET_MIN
#define TZ_OFFSET_MIN 480
#endif
#ifndef REC_INTERVAL_S
#define REC_INTERVAL_S 10
#endif
#ifndef HOLD_MS
#define HOLD_MS 1500
#endif
#ifndef DIM_AFTER_MS
#define DIM_AFTER_MS 30000
#endif
#ifndef BRIGHT_HIGH
#define BRIGHT_HIGH 200
#endif
// Dim level: 0..255. 12 is just barely readable in daylight; override with
// -D BRIGHT_DIM=N in platformio.ini if you want it brighter/darker.
#ifndef BRIGHT_DIM
#define BRIGHT_DIM 12
#endif
#ifndef AUTO_OFF_MS
#define AUTO_OFF_MS 300000
#endif
#ifndef BAT_LOW_PERCENT
#define BAT_LOW_PERCENT 5
#endif
#ifndef BAT_RECOVER_PERCENT
#define BAT_RECOVER_PERCENT 10
#endif
#ifndef BAT_CHECK_MS
#define BAT_CHECK_MS 10000
#endif
// Selection mode (yellow highlight on the bottom bar) auto-exits after this
// much idle time, measured from the last real key press - independent of the
// dim / auto-off timers so it cannot be disturbed by USB serial traffic.
#ifndef SELECT_TIMEOUT_MS
#define SELECT_TIMEOUT_MS 10000
#endif
// 2.0.1: any non-main page (MENU / FILES / SETUP / OUTPUT / DELETE) returns to
// the main screen after this much time with no key press, so an abandoned
// sub page can never sit there forever holding the screen on.
#ifndef SUBPAGE_TIMEOUT_MS
#define SUBPAGE_TIMEOUT_MS 30000
#endif
// Brightness slider steps (0..255). BRIGHT_HIGH stays the compile-time default.
static const uint16_t BRIGHT_STEPS[] = {40, 80, 120, 160, 200, 240};
static const int     BRIGHT_STEPS_N = 6;
// Dim-after slider: 5 / 10 / 15 / 30 s
static const uint16_t DIMAFTER_STEPS[] = {5, 10, 15, 30};
static const int      DIMAFTER_STEPS_N = 4;
// Record-interval slider: 1 / 5 / 10 / 30 / 60 s per point
static const uint16_t REC_STEPS[] = {1, 5, 10, 30, 60};
static const int      REC_STEPS_N = 5;
// Auto-off slider (seconds): 30s / 1 / 5 / 10 / 20 / 30 / 60 min. Longer
// options exist so a slow cold-start search never powers the unit off first.
static const uint16_t AUTOOFF_STEPS[] = {30, 60, 300, 600, 1200, 1800, 3600};
static const int      AUTOOFF_STEPS_N = 7;

// NOTE: UART2 MUST be begun BEFORE M5.begin(). M5Unified conflicts with
// HardwareSerial.begin if called afterwards (silent panic, WDT eats it).
static HardwareSerial GPSSerial(2);

#define GPScam_NAME "Sony Camera GPS Logger"
#define GPScam_VER_MAJOR 4
#define GPScam_VER_MINOR 6
#define GPScam_VER_PATCH 3
#define GPScam_VERSION_STR(maj, min, pat) \
  #maj "." #min "." #pat
#define GPScam_VERSION_NO_V \
  GPScam_VERSION_STR(GPScam_VER_MAJOR, GPScam_VER_MINOR, GPScam_VER_PATCH)
#define GPScam_VERSION "v" GPScam_VERSION_NO_V
// 设置页显示用的版本号（无 v 前缀），与 GPScam_VERSION 同源
#define FIRMWARE_VERSION GPScam_VERSION_NO_V
#define GPScam_BASE "record base v3.2.0 (2026-11-22)"

// ---------------- GPS state ----------------
struct GpsState {
  bool   timeValid = false;
  bool   valid = false;
  int    year = 0, month = 0, day = 0;
  int    hour = 0, minute = 0, second = 0;
  double lat = 0, lon = 0;
  float  alt = 0;
  int    sats = 0;
  int    satTotal = 0;
  int    quality = 0;
  uint32_t lastFixMs = 0;
  // --- Speed over ground (v2.0.0): RMC field 7 is knots, shown as km/h ---
  float    spdKt = 0;
  uint32_t spdMs = 0;          // when spdKt was last refreshed from RMC
  // --- Signal strength (SNR) layer (v15) ---
  bool     snrWindowOpen = false;
  uint32_t snrWindowStartMs = 0;
  int      snrSum = 0, snrCount = 0, snrPeak = 0;
  int      sysVisible[8] = {0};
  int      snrAvg = 0, snrMax = 0, snrVis = 0;
  uint32_t snrUpdateMs = 0;
};
static GpsState gps;

static int gsvTalkerIdx(const char *type) {
  if (type[0] == 'G' && type[1] == 'P') return 0;
  if (type[0] == 'G' && type[1] == 'B') return 1;
  if (type[0] == 'G' && type[1] == 'L') return 2;
  if (type[0] == 'G' && type[1] == 'A') return 3;
  if (type[0] == 'G' && type[1] == 'Q') return 4;
  return -1;
}
static void settleSnrWindow() {
  if (!gps.snrWindowOpen) return;
  gps.snrWindowOpen = false;
  gps.snrAvg = gps.snrCount > 0 ? (gps.snrSum / gps.snrCount) : 0;
  gps.snrMax = gps.snrPeak;
  gps.snrVis = 0;
  for (auto &v : gps.sysVisible) gps.snrVis += v;
  gps.snrUpdateMs = millis();
}

// ---------------- recording state ----------------
static bool     recording = false;
static File     trackFile;
static char     trackName[32] = "";
static uint32_t trackPoints = 0;
static int64_t  recStartSecs = -1;
static uint32_t lastPointMs = 0;

// ---------------- export state ----------------
static bool      exportMode = false;   // OUTPUT ready screen visible
static uint32_t  exportIdleMs = 0;
// All three host-side clients use the SAME USB CDC + 'D' protocol. OUTPUT
// only chooses which client to show instructions for, and remembers the
// last used one. No firmware-side transport switching happens here.
#define OUTPUT_METHODS 3
static const char *OUTPUT_LABELS[] = {
  "浏览器", "Python", "终端", "退出"
};
static const int OUTPUT_ROWS = 4;      // 3 methods + EXIT
static uint8_t   outputChoice = 0;     // last used method, saved in settings.cfg
static int       outputIndex  = 0;     // cursor in OUTPUT_LABELS
enum ExportPhase : uint8_t { EXP_WAIT = 0, EXP_SEND, EXP_DONE };
static uint8_t   exportPhase = EXP_WAIT;
static int       exportTotal = 0, exportSent = 0, exportLastCount = 0;
static int       exportDoneCount = 0;   // 上次导出文件数（导出页显示）
// Streaming GPX while the static full-screen OUTPUT page is displayed trips
// TG1WDT_SYS_RST on this Arduino 2.0.7 + qio_opi build (verified: the very
// same 'D' on the main screen is fine). The 'D' handler therefore returns
// to the proven-safe main screen BEFORE streaming, then shows a transient
// "EXPORT DONE N" through setStatus() - no painting inside the CDC burst.

// ---------------- display / menu / idle state ----------------
// v1.1.0: the horizontal bottom menu bar was replaced by a full-screen
// MENU sub page (MID enters it from the main screen, FRONT confirms).
enum UiMode : uint8_t { UI_MAIN = 0, UI_MENU, UI_FILES, UI_SETTINGS, UI_DELETE, UI_OUTPUT, UI_CAM, UI_EXPORT };
static UiMode   uiMode = UI_MAIN;
static int      menuIndex = 0;         // cursor in MENU_LABELS (menu sub page)
static uint32_t selectEnterMs = 0;    // last REAL key press in a select page
static int      settingsIndex = 0;
static bool     dimmed = false;
static uint32_t lastActivityMs = 0;
// After a wake-only press the NEXT key must always perform its action.
// Without this grace a short DIM AFTER (5 s) can dim the screen again before
// the user reacts to "WAKE - PRESS AGAIN", so the second press gets swallowed
// as another wake and the message shows up a second time.
#define WAKE_GRACE_MS 6000
static uint32_t wakeGraceUntilMs = 0;
static bool wakeGraceActive(uint32_t now) {
  return (int32_t)(wakeGraceUntilMs - now) > 0;
}
static bool     keySwap = false;       // serial 'S' swaps FRONT/MID roles
static bool     usbOverride = false;   // serial 'I' test: ignore USB presence
static bool     autoOffReason = false;// true = idle auto-off, false = key hold

// FILES sub page: GPX list + a trailing EXIT row. Names are full /paths so
// they can be passed straight to LittleFS.remove().
#define MAX_FILE_ITEMS 24
#define FILE_ROWS_P 7                  // visible rows in portrait
static int      fileRowsVisible = FILE_ROWS_P;
static char     fileNames[MAX_FILE_ITEMS][32];
static int      fileCount = 0;         // number of .GPX files
static int      fileIndex = 0;         // cursor: 0..fileCount (== EXIT row)
static int      fileTop   = 0;         // scroll offset
static int      deleteChoice = 0;      // 0 = DELETE, 1 = CANCEL
static bool     delArmed = false;      // DELETE 二次确认
static uint32_t delArmedMs = 0;

// Runtime settings: defaults from the macros, editable in SETUP, persisted
// in /settings.cfg so they survive a reboot.
static uint16_t recIntervalS = REC_INTERVAL_S;
static uint8_t  dimLevel     = BRIGHT_DIM;
static uint32_t dimAfterMs   = DIM_AFTER_MS;
static uint32_t autoOffMs    = AUTO_OFF_MS;   // 0 = never (legacy value)
static uint8_t  brightHigh   = BRIGHT_HIGH;   // normal (awake) brightness
static bool     promptOn     = true;

// ---------------- GNSS constellation selection (v11) ----------------
// AT6668 speaks NMEA 4.1 out and CASIC "$PCAS04,mode*cs" in. The strings
// below are taken from M5Stack's own MultipleSatellite.h and the CASIC
// protocol spec; every checksum was recomputed and matched. The command
// takes effect immediately (no reboot / no cold start required).
// mode 7 = GPS+BDS+GLONASS joint fixing, used as "AUTO" because it is the
// widest combination the CASIC spec defines. Galileo / QZSS are reachable
// as explicit single-system choices.
static const char *GNSS_CMDS[] = {
  "$PCAS04,7*1E\r\n",   // 0 AUTO : GPS+BDS+GLONASS
  "$PCAS04,1*18\r\n",   // 1 GPS
  "$PCAS04,2*1B\r\n",   // 2 BDS
  "$PCAS04,3*1A\r\n",   // 3 GPS+BDS
  "$PCAS04,4*1D\r\n",   // 4 GLONASS
  "$PCAS04,8*11\r\n",   // 5 GALILEO
  "$PCAS04,20*2B\r\n",  // 6 QZSS
};
static const char *GNSS_LABELS[] = {
  "AUTO", "GPS", "BDS", "GPS+BDS", "GLN", "GAL", "QZS"
};
// Long form for the portrait constellation line (2.0.0).
static const char *GNSS_DETAIL[] = {
  "GPS+BDS+GLN", "GPS ONLY", "BEIDOU ONLY", "GPS+BEIDOU",
  "GLONASS ONLY", "GALILEO ONLY", "QZSS ONLY"
};
static const int   GNSS_COUNT = 7;
static uint8_t     gnssMode = 0;       // index into GNSS_CMDS

static void sendGnssMode(uint8_t m) {
  if (m >= GNSS_COUNT) m = 0;
  // The UART is already bidirectional: setup() probed and locked rx/tx/baud.
  // Responses from the module are non-GGA/RMC/GSV lines and parseNMEA()
  // simply ignores them.
  GPSSerial.print(GNSS_CMDS[m]);
  Serial.printf("[GNSS] apply %s -> %s", GNSS_LABELS[m], GNSS_CMDS[m]);
}

// 4.5.0: 旋转/横屏支持全部移除，面板固定为原生 135x240 竖屏（rotation 0）。

static int      batteryLevel = -1;
static uint8_t  lowBatHits   = 0;
static bool     lowBatLock   = false;

// ---------------- time helpers (pure integer, no time_t) ----------------
static int64_t days_from_civil(int y, int m, int d) {
  y -= m <= 2;
  int64_t era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = y - era * 400;
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097LL + (int64_t)doe - 730426LL;
}
static void civil_from_days(int64_t z0, int &y, int &m, int &d) {
  z0 += 730426;
  int64_t era = (z0 >= 0 ? z0 : z0 - 146096) / 146097;
  unsigned doe = z0 - era * 146097;
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int64_t yy = (int64_t)yoe + era * 400;
  unsigned doy = doe - (yoe * 365 + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  m = mp + (mp < 10 ? 3 : -9);
  y = yy + (m <= 2);
}
static int64_t gpsEpochSecs(const GpsState &g) {
  return days_from_civil(g.year, g.month, g.day) * 86400LL
       + g.hour * 3600 + g.minute * 60 + g.second;
}
// Split GPS epoch + TZ into local calendar / clock parts.
static void localParts(int &yy, int &mm, int &dd, int &hh, int &mi, int &ss) {
  int64_t s = gpsEpochSecs(gps) + (int64_t)TZ_OFFSET_MIN * 60;
  int64_t days = s / 86400; if (s < 0 && s % 86400) days--;
  int rem = (int)(s - days * 86400);
  civil_from_days(days, yy, mm, dd);
  hh = rem / 3600; mi = (rem % 3600) / 60; ss = rem % 60;
}
static void fmtUTC(const GpsState &g, char *out, size_t n) {
  snprintf(out, n, "%04d-%02d-%02dT%02d:%02d:%02d.00Z",
           g.year, g.month, g.day, g.hour, g.minute, g.second);
}
static void fmtElapsed(int64_t secs, char *out, size_t n) {
  if (secs < 0) { snprintf(out, n, "--:--:--"); return; }
  int h = secs / 3600, m = (secs % 3600) / 60, s = secs % 60;
  if (h > 99) snprintf(out, n, "%4dh", h);
  else snprintf(out, n, "%02d:%02d:%02d", h, m, s);
}

// ---------------- NMEA parsing (with checksum) ----------------
static double nmeaToDeg(const char *v, const char *hemi) {
  if (!v || !*v) return 0;
  double raw = atof(v);
  int deg = (int)(raw / 100);
  double d = deg + (raw - deg * 100) / 60.0;
  if (hemi && (*hemi == 'S' || *hemi == 'W')) d = -d;
  return d;
}
static int atoi2(const char *p, int len) {
  int v = 0;
  for (int i = 0; i < len; i++) {
    if (p[i] >= '0' && p[i] <= '9') v = v * 10 + (p[i] - '0'); else break;
  }
  return v;
}
static bool nmeaChecksumOk(const char *line) {
  const char *star = strchr(line, '*');
  if (!star || star - line < 2) return false;
  uint8_t cs = 0;
  for (const char *p = line + 1; p < star; p++) cs ^= (uint8_t)*p;
  int want = (int)strtol(star + 1, nullptr, 16);
  return cs == want;
}
#define NF 24
static char fbuf[NF][16];
static int splitFields(char *s, int firstComma) {
  int n = 0, start = firstComma + 1;
  size_t len = strlen(s);
  for (size_t i = start; i <= len && n < NF; i++) {
    if (i == len || s[i] == ',') {
      size_t flen = i - start;
      if (flen > 15) flen = 15;
      memcpy(fbuf[n], s + start, flen);
      fbuf[n][flen] = 0;
      n++; start = i + 1;
    }
  }
  return n;
}
static void parseNMEA(char *line) {
  if (line[0] != '$') return;
  if (!nmeaChecksumOk(line)) return;
  char *star = strchr(line, '*');
  if (star) *star = 0;
  char *c1 = strchr(line, ',');
  if (!c1) return;
  size_t tlen = c1 - line - 1;
  char type[8] = {0};
  if (tlen > 6) tlen = 6;
  memcpy(type, line + 1, tlen);

  bool isGGA = !strcmp(type, "GGA") || !strcmp(type, "GNGGA") || !strcmp(type, "GPGGA");
  bool isRMC = !strcmp(type, "RMC") || !strcmp(type, "GNRMC") || !strcmp(type, "GPRMC");
  bool isGSV = strstr(type, "GSV");
  if (!isGGA && !isRMC && !isGSV) return;

  int n = splitFields(line, (int)(c1 - line));
  if (isGSV) {
    int talker = gsvTalkerIdx(type);
    int total = (n > 2) ? atoi(fbuf[2]) : 0;
    if (talker >= 0 && talker < 8) gps.sysVisible[talker] = total;
    int msgIdx = (n > 1) ? atoi(fbuf[1]) : 1;
    uint32_t now = millis();
    if (!gps.snrWindowOpen
        || (now - gps.snrWindowStartMs > 2000)
        || (msgIdx == 1 && now - gps.snrWindowStartMs > 150)) {
      gps.snrWindowOpen = true;
      gps.snrWindowStartMs = now;
      gps.snrSum = gps.snrCount = gps.snrPeak = 0;
      for (auto &v : gps.sysVisible) v = 0;
      if (talker >= 0 && talker < 8) gps.sysVisible[talker] = total;
    }
    for (int k = 6; k <= 18; k += 4) {
      if (k >= n) break;
      int snr = atoi(fbuf[k]);
      if (snr <= 0) continue;
      gps.snrSum += snr;
      gps.snrCount++;
      if (snr > gps.snrPeak) gps.snrPeak = snr;
    }
    if (total > gps.satTotal) gps.satTotal = total;
    return;
  }
  if (isGGA) {
    if (strlen(fbuf[0]) >= 6) {
      gps.hour = atoi2(fbuf[0], 2); gps.minute = atoi2(fbuf[0] + 2, 2); gps.second = atoi2(fbuf[0] + 4, 2);
    }
    gps.quality = atoi(fbuf[5]);
    gps.sats = atoi(fbuf[6]);
    if (*fbuf[8]) gps.alt = atof(fbuf[8]);
    if (gps.quality > 0 && *fbuf[1] && *fbuf[3]) {
      gps.lat = nmeaToDeg(fbuf[1], fbuf[2]);
      gps.lon = nmeaToDeg(fbuf[3], fbuf[4]);
      gps.valid = true;
      gps.lastFixMs = millis();
    }
  } else { // RMC
    if (strlen(fbuf[0]) >= 6) {
      gps.hour = atoi2(fbuf[0], 2); gps.minute = atoi2(fbuf[0] + 2, 2); gps.second = atoi2(fbuf[0] + 4, 2);
    }
    if (strlen(fbuf[8]) >= 6) {
      gps.day = atoi2(fbuf[8], 2); gps.month = atoi2(fbuf[8] + 2, 2); gps.year = 2000 + atoi2(fbuf[8] + 4, 2);
      gps.timeValid = true;
    }
    if (fbuf[1][0] == 'A' && *fbuf[2] && *fbuf[4]) {
      gps.lat = nmeaToDeg(fbuf[2], fbuf[3]);
      gps.lon = nmeaToDeg(fbuf[4], fbuf[5]);
      gps.valid = true;
      gps.lastFixMs = millis();
    }
    // Speed over ground: RMC field 6, in knots. (splitFields() starts after
    // the type token, so 0=time 1=status 2=lat 3=N/S 4=lon 5=E/W 6=knots
    // 7=course 8=date - field 7 is the COURSE, not the speed.)
    if (*fbuf[6]) { gps.spdKt = atof(fbuf[6]); gps.spdMs = millis(); }
  }
}
static char nmeaLine[128];
static int  nmeaPos = 0;
static void readGPS() {
  while (GPSSerial.available()) {
    char c = (char)GPSSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (nmeaPos > 0) { nmeaLine[nmeaPos] = 0; parseNMEA(nmeaLine); nmeaPos = 0; }
    } else if (nmeaPos < (int)sizeof(nmeaLine) - 1) {
      nmeaLine[nmeaPos++] = c;
    } else {
      nmeaPos = 0;
    }
  }
  if (gps.valid && millis() - gps.lastFixMs > 5000) gps.valid = false;
  // A stale RMC must not keep showing the last speed: drop it with the fix.
  if (!gps.valid) gps.spdKt = 0;
}

// ---------------- LittleFS / GPX ----------------
static bool fsOk = false;
static bool initFS() {
  if (!LittleFS.begin(true)) return false;
  fsOk = true;
  return true;
}
static void powerOffNow();             // defined later in the button section

// Enumerate the root dir instead of open()-ing a possibly missing path:
// LittleFS.exists() calls open() internally and logs a vfs_api ERROR on a
// first boot, which looks alarming on the serial console.
static void loadKeySwap() {
  if (!fsOk) return;
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  while (f) {
    const char *nm = f.name();
    if (!strcmp(nm, "keyswap.cfg") || !strcmp(nm, "/keyswap.cfg")) {
      keySwap = (f.read() == '1');
      f.close();
      return;
    }
    f = root.openNextFile();
  }
}
static void saveKeySwap() {
  if (!fsOk) return;
  File f = LittleFS.open("/keyswap.cfg", FILE_WRITE);
  if (f) { f.write(keySwap ? '1' : '0'); f.close(); }
}

// Cached once/second: an I2C VBUS read every 2 ms loop would be wasteful.
// Unknown (<0) counts as "plugged" so auto-off is skipped - safe default.
static bool usbPlugged() {
  if (usbOverride) return false;       // diagnostics only
  static uint32_t lastCheck = 0;
  static bool cached = true;
  if (millis() - lastCheck >= 1000) {
    lastCheck = millis();
    int16_t mv = M5.Power.getVBUSVoltage();
    cached = (mv < 0) ? true : (mv > 3500);
  }
  return cached;
}
// ---------------- CPU 电源计划 (4.6.0) ----------------
// 平时/记录中固定 80 MHz；按键、菜单操作、BLE 连接配对时临时升到 240 MHz，
// 20 秒无操作自动回落。ESP32-S3 的 APB 在 80/160/240 三档下都固定 80 MHz
// （见 IDF rtc_clk.c: rtc_clk_apb_freq_update(80*MHZ)），所以 GPS 串口、
// PMIC I2C、LCD SPI 和 Flash 时序都不受影响；BLE 要求 CPU >= 80 MHz，满足。
#define CPU_MHZ_IDLE  80
#define CPU_MHZ_BOOST 240
#define CPU_BOOST_MS  20000u
static bool     pmReady = false;
static uint8_t  cpuMhzNow = CPU_MHZ_IDLE;   // 记录当前档位（用于日志/诊断）
static uint32_t cpuBoostUntil = 0;

// 直接重配 PM 的 min=max 来切档：不用 CPU_FREQ_MAX 锁，因为 PM 给每个核
// 都常驻了一对 rtos 锁，只要任务在跑 CPU 就会被钉在最高档 —— 锁方案在
// 这种常驻轮询的固件里等于永远 240 MHz。
// 注：S3 在 80/160/240 三档下 APB 都由硬件固定为 80 MHz，日志里
// "APB_MAX: 240" 只是 PM 的内部模式命名，不影响 GPS 串口/I2C/SPI/Flash。
static void cpuApply(uint8_t mhz) {
  if (!pmReady || mhz == cpuMhzNow) return;
  esp_pm_config_t cfg = {};
  cfg.max_freq_mhz = mhz;
  cfg.min_freq_mhz = mhz;
  cfg.light_sleep_enable = false;
  esp_err_t err = esp_pm_configure(&cfg);
  if (err == ESP_OK) {
    cpuMhzNow = mhz;
    Serial.printf("[PWR] cpu -> %u MHz\n", (unsigned)mhz);
  } else {
    Serial.printf("[PWR] cpu %u MHz failed (%d)\n", (unsigned)mhz, (int)err);
  }
}
static void cpuBoost(uint32_t ms) {
  cpuBoostUntil = millis() + ms;
  cpuApply(CPU_MHZ_BOOST);
}
static void cpuService(void) {
  if (cpuMhzNow != CPU_MHZ_IDLE && (int32_t)(millis() - cpuBoostUntil) >= 0) {
    cpuApply(CPU_MHZ_IDLE);
  }
}
static void cpuInit(void) {
  esp_log_level_set("pm", ESP_LOG_WARN);   // 每次切档不再刷屏
  esp_pm_config_t cfg = {};
  cfg.max_freq_mhz = CPU_MHZ_IDLE;
  cfg.min_freq_mhz = CPU_MHZ_IDLE;
  cfg.light_sleep_enable = false;
  pmReady = (esp_pm_configure(&cfg) == ESP_OK);
  cpuMhzNow = CPU_MHZ_IDLE;
  Serial.printf("[PWR] cpu plan %s (idle %u / boost %u MHz, %lus)\n",
                pmReady ? "on" : "off", (unsigned)CPU_MHZ_IDLE,
                (unsigned)CPU_MHZ_BOOST, (unsigned long)(CPU_BOOST_MS / 1000));
}

static void noteActivity() {
  lastActivityMs = millis();
  cpuBoost(CPU_BOOST_MS);   // 有操作就提速，空闲自动回落
  if (dimmed) {
    dimmed = false;
    M5.Display.setBrightness(brightHigh);
    Serial.println("[LCD] bright");
  }
}
static void manageIdle() {
  uint32_t now = millis();
  // Keep the screen fully awake for a few seconds after a wake-only press so
  // the user has time to read "WAKE - PRESS AGAIN" and actually press again.
  if (wakeGraceActive(now)) return;
  // Keep the OUTPUT ready page legible while waiting for the PC.
  if (!dimmed && !exportMode && now - lastActivityMs >= dimAfterMs) {
    dimmed = true;
    M5.Display.setBrightness(dimLevel);
    Serial.printf("[LCD] dim (%lus idle) -> level %d\n",
                  (unsigned long)(dimAfterMs / 1000), dimLevel);
  }
  // 2.0.1: never auto power off from inside a sub page. The page itself
  // returns to the main screen after SUBPAGE_TIMEOUT_MS, and powering off on
  // USB would have ended up as an esp_restart() (a "phantom reboot").
  if (!recording && !exportMode && !usbPlugged() && uiMode == UI_MAIN
      && !FurbCam::isActive()  // 相机连接中禁止自动关机
      && autoOffMs > 0 && now - lastActivityMs >= autoOffMs) {
    Serial.printf("[PWR] auto off: %lus idle\n", (unsigned long)(autoOffMs / 1000));
    autoOffReason = true;
    powerOffNow();
  }
}
// Name tracks by GPS LOCAL start time, so the file list directly matches
// photo capture times: /20261120_100730.GPX (UTC+8). Collisions get _01..
static void nextTrackName() {
  int yy, mm, dd, hh, mi, ss;
  localParts(yy, mm, dd, hh, mi, ss);
  char base[24];
  snprintf(base, sizeof(base), "/%04d%02d%02d_%02d%02d%02d", yy, mm, dd, hh, mi, ss);
  for (int i = 0; i < 100; i++) {
    if (i == 0) snprintf(trackName, sizeof(trackName), "%s.GPX", base);
    else        snprintf(trackName, sizeof(trackName), "%s_%02d.GPX", base, i);
    if (!LittleFS.exists(trackName)) return;
  }
}
static bool startRecording() {
  if (!fsOk) return false;
  // Recording requires a valid TIME (GPS has delivered the date/clock) but NOT
  // a position fix: the track file opens immediately so a slow cold-start
  // search never blocks you, and trackpoints are appended once a fix arrives.
  if (!gps.timeValid) return false;
  nextTrackName();
  trackFile = LittleFS.open(trackName, FILE_WRITE);
  if (!trackFile) return false;
  int yy, mm, dd, hh, mi, ss;
  localParts(yy, mm, dd, hh, mi, ss);
  char human[40];
  snprintf(human, sizeof(human), "%04d-%02d-%02d %02d:%02d:%02d UTC%+d",
           yy, mm, dd, hh, mi, ss, TZ_OFFSET_MIN / 60);
  // GPX 1.1 namespace is required by Lightroom / GeoSetter / most XML readers.
  trackFile.print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<gpx version=\"1.1\" creator=\"M5StickS3-Logger\" xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n<trk><name>");
  trackFile.print(human);
  trackFile.print("</name><trkseg>\n");
  trackFile.flush();
  trackPoints = 0;
  recStartSecs = gpsEpochSecs(gps);
  recording = true;
  return true;
}
static void stopRecording() {
  if (!trackFile) { recording = false; return; }
  trackFile.print("</trkseg></trk></gpx>\n");
  trackFile.flush();
  trackFile.close();
  recording = false;
}
static void writeTrackpoint() {
  if (!recording || !trackFile || !gps.valid) return;
  char t[32];
  fmtUTC(gps, t, sizeof(t));
  char lat[16], lon[16], alt[12];
  dtostrf(gps.lat, 2, 6, lat);
  dtostrf(gps.lon, 3, 6, lon);
  dtostrf(gps.alt, 1, 1, alt);
  trackFile.print("<trkpt lat=\"");
  trackFile.print(lat);
  trackFile.print("\" lon=\"");
  trackFile.print(lon);
  trackFile.print("\"><ele>");
  trackFile.print(alt);
  trackFile.print("</ele><time>");
  trackFile.print(t);
  trackFile.print("</time></trkpt>\n");
  trackFile.flush();
  trackPoints++;
  if (recStartSecs < 0) recStartSecs = gpsEpochSecs(gps);
}

// ---------------- USB serial export ----------------
static void setStatus(const char *m);
static void drawOutputUI();
static void drawMainUI();
static void drawMainPortrait();
static void drawBottomRow(int row);
static void drawPortraitChrome();
static int  spdKmh();
static int  countFiles();
static void uiReset();
static void clearRowCache();
static void setUiMode(UiMode m);
// Line-oriented protocol, easy to parse from any serial terminal:
//   #GPX_BEGIN|/20261120_100730.GPX
//   ... raw GPX lines ...
//   #GPX_END
//   #GPX_DONE|3
static bool dumping = false;   // mute heartbeat logs while streaming GPX
static void dumpAllGpx() {
  if (!fsOk) { Serial.println("#GPX_ERROR|FS not ready"); return; }
  dumping = true;
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  int n = 0;
  while (f) {
    String nm = f.name();
    if (!nm.startsWith("/")) nm = "/" + nm;
    if (nm.endsWith(".GPX") || nm.endsWith(".gpx")) {
      Serial.printf("#GPX_BEGIN|%s\n", nm.c_str());
      uint8_t buf[256];
      int got;
      while ((got = f.read(buf, sizeof(buf))) > 0) Serial.write(buf, got);
      Serial.write('\n');
      Serial.println("#GPX_END");
      n++;
    }
    f = root.openNextFile();
  }
  Serial.printf("#GPX_DONE|%d\n", n);
  Serial.flush();
  exportDoneCount = n;
  dumping = false;
  exportLastCount = n;          // bookkeeping only, no display work here
}
static void listGpx() {
  if (!fsOk) { Serial.println("#GPX_ERROR|FS not ready"); return; }
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  while (f) {
    String nm = f.name();
    if (!nm.startsWith("/")) nm = "/" + nm;
    if (nm.endsWith(".GPX") || nm.endsWith(".gpx"))
      Serial.printf("#GPX_ITEM|%s|%u\n", nm.c_str(), (unsigned)f.size());
    f = root.openNextFile();
  }
  Serial.println("#GPX_LIST_END");
}
static void makeTestGpx() {
  if (!fsOk) return;
  File t = LittleFS.open("/TEST_SELFTEST.GPX", FILE_WRITE);
  if (!t) return;
  t.print("<?xml version=\"1.0\"?>\n<gpx version=\"1.1\"><trk><trkseg>\n");
  t.print("<trkpt lat=\"31.23000\" lon=\"121.47000\"><time>2026-11-20T02:00:00.00Z</time></trkpt>\n");
  t.print("</trkseg></trk></gpx>\n");
  t.close();
  Serial.println("#TEST_CREATED");
}
static void cleanTestGpx() {
  if (LittleFS.exists("/TEST_SELFTEST.GPX")) LittleFS.remove("/TEST_SELFTEST.GPX");
  Serial.println("#TEST_REMOVED");
}
static void exitExportScreen() {
  exportMode = false;
  uiMode = UI_MAIN;
  uiReset();
  drawMainUI();               // redraw now, avoid a 1 s black gap
}

// ---------------- Partial-redraw UI (no full screen clear in loop) ----------------
// 4.5.0: 横屏代码已全部删除，UI 固定为面板原生的 135x240 竖屏布局。
struct UiRow { int y; uint8_t size; uint16_t color; char last[40]; uint8_t x; };

// Portrait row indices (135 px wide = 22 size1 chars / 11 size2 chars).
// v3.0.2: AVG and MAX share one size1 line (was size2 - too large); the space
// dedicated constellation line while LAT / LON / ALT keep their own rows.
enum : int {
  P_CLOCK = 0, P_STATE, P_TIME, P_SAT, P_GNSS, P_LAT, P_LON, P_ALT,
  P_SIG, P_BAT, P_BATRUN, P_FREE, P_FILES, P_LINK1, P_LINK2, P_HINT, P_ROW_COUNT
};
// Portrait rows. `x` is the text origin: the merged size2 signal line needs
// x=2, because "AVG47 MAX52" is 11 x 12 px = 132 px of the 135 px width.
static UiRow rowsPort[P_ROW_COUNT] = {
  {  2, 2, 0, "", 4},   // P_CLOCK
  { 22, 2, 0, "", 4},   // P_STATE
  { 42, 1, 0, "", 4},   // P_TIME
  { 58, 1, 0, "", 4},   // P_SAT
  { 70, 1, 0, "", 4},   // P_GNSS  (constellation detail)
  { 84, 1, 0, "", 4},   // P_LAT
  { 96, 1, 0, "", 4},   // P_LON
  {108, 1, 0, "", 4},   // P_ALT   (altitude + speed)
  {128, 1, 0, "", 4},   // P_SIG   (AVG + MAX merged, size1 in 3.0.2)
  {146, 1, 0, "", 4},   // P_BAT   (小字：BAT xx% + 右侧 CPU 频率)
  {158, 1, 0, "", 4},   // P_BATRUN 剩余可用时间估算
  {174, 1, 0, "", 4},   // P_FREE
  {186, 1, 0, "", 4},   // P_FILES
  {200, 1, 0, "", 4},   // P_LINK1  相机连接
  {212, 1, 0, "", 4},   // P_LINK2  GPS 推送状态
  {228, 1, 0, "", 4},   // P_HINT
};
static bool   portChromeReady = false;
// CPU 频率小数字的缓存（80 绿 / 160 黄 / 240 红）
static char     cpuBadgeTxt[6] = "";
static uint16_t cpuBadgeCol = 0;
static void cpuBadgeInvalidate(void) { cpuBadgeTxt[0] = 0; }
static void chgTextInvalidate(void);   // 定义在充电文字绘制处
static const char *statusMsg = "";
static uint32_t statusMsgMs = 0;
static void setStatus(const char *m) { statusMsg = m; statusMsgMs = millis(); }
// A real key action must retire the transient wake banner immediately.
// Otherwise the 1 Hz drawMainUI() keeps repainting "WAKE - PRESS AGAIN" over
// the menu bar for the rest of its 2.5 s lifetime, so it looks like it
// appeared a second time right after the user pressed again.
static void clearStatus() { statusMsg = ""; statusMsgMs = 0; }

static void uiReset() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.waitDMA();         // full-screen fill is async on this panel
  for (int i = 0; i < P_ROW_COUNT; i++) { rowsPort[i].last[0] = 0; rowsPort[i].color = 0; }
  portChromeReady = false;      // dividers were wiped by the full fill
  cpuBadgeInvalidate();         // 全屏清过，频率角标要重画
  chgTextInvalidate();          // 充电文字同样要重画
}

// ---------------- sub-page text helpers (ASCII Font0 ONLY) ----------------
// CJK fonts and full-screen sprites are unusable on this Arduino 2.0.7 +
// qio_opi build (TG1WDT_SYS_RST). All labels are plain English now, drawn
// with the built-in Font0. The LCD font is NEVER switched.
static void useAsciiFont() { M5.Display.setFont(&fonts::Font0); }

// sz=1 -> 6x8 glyphs vertically centred in a 16 px row; sz=2 -> 12x16 title.
static void uiText(int x, int y, const char *str, uint16_t fg,
                   uint16_t bg = TFT_BLACK, uint8_t sz = 1) {
  (void)sz;  // 统一 16px 中文字体
  auto &d = M5.Display;
  d.setFont(&fonts::efontCN_16);
  d.setTextSize(1);
  d.setTextColor(fg, bg);
  d.setCursor(x, y);
  d.print(str);
}
// 按像素宽度自动换行绘制（UTF-8 安全）：优先在空格处断开，中文按字符断开，
// 保证没有字形被 135 px 的屏幕右边缘裁掉。返回最后一行下方一行的 y 坐标。
static int uiTextWrap(int x, int y, int w, const char *str, uint16_t fg,
                      uint16_t bg = TFT_BLACK, int lineH = 18) {
  auto &d = M5.Display;
  d.setFont(&fonts::efontCN_16);
  d.setTextSize(1);
  d.setTextColor(fg, bg);
  if (str == nullptr) return y;

  const char *p = str;
  do {
    const char *lineStart = p;
    const char *lineEnd = p;
    const char *spaceAfter = nullptr;
    int used = 0;
    while (*p && *p != '\n') {
      unsigned char c = (unsigned char)*p;
      int len = (c < 0x80) ? 1 : ((c & 0xE0) == 0xC0) ? 2
                : ((c & 0xF0) == 0xE0) ? 3 : 4;
      char one[5] = {0};
      memcpy(one, p, len);
      int cw = d.textWidth(one);
      if (cw <= 0) cw = (c < 0x80) ? 8 : 16;  // 字体度量不可用时的兜底
      if (used + cw > w && lineEnd != lineStart) break;
      used += cw;
      p += len;
      lineEnd = p;
      if (c == ' ') spaceAfter = p;
    }
    // 放不下时优先回退到最近的空格，避免把英文单词切成两半
    if (*p && *p != '\n' && spaceAfter != nullptr && spaceAfter - lineStart >= 2) {
      lineEnd = spaceAfter;
      p = lineEnd;
    }
    char buf[80];
    size_t n = (size_t)(lineEnd - lineStart);
    if (n >= sizeof buf) n = sizeof buf - 1;
    memcpy(buf, lineStart, n);
    buf[n] = 0;
    while (n > 0 && buf[n - 1] == ' ') buf[--n] = 0;
    d.setCursor(x, y);
    d.print(buf);
    y += lineH;
    if (*p == '\n') p++;
    while (*p == ' ') p++;
  } while (*p);
  return y;
}

// A selectable row: yellow reverse-video when active, plain otherwise.
static void uiSelectRow(int x, int y, int w, const char *str, bool active) {
  auto &d = M5.Display;
  if (active) d.fillRect(x, y, w, 16, TFT_YELLOW);
  uiText(x + 2, y, str, active ? TFT_BLACK : TFT_WHITE,
         active ? TFT_YELLOW : TFT_BLACK);
}

// Portrait (size2) selectable row: taller highlight, 22 px box for a 16 px
// glyph so the yellow band has 3 px of padding above and below.
static void uiSelectRowP(int x, int y, const char *str, bool active) {
  auto &d = M5.Display;
  if (active) d.fillRect(x, y - 3, d.width() - x - 2, 22, TFT_YELLOW);
  uiText(x + 4, y, str, active ? TFT_BLACK : TFT_WHITE,
         active ? TFT_YELLOW : TFT_BLACK, 2);
}

// Compact portrait slider: 4 px track + 9x10 knob, so a label + slider block
// fits in 26 px and all 7 SETUP rows fit a 240 px tall canvas.
static void drawSliderP(int x, int y, int w, int idx, int count, bool active) {
  auto &d = M5.Display;
  d.fillRect(x, y, w, 4, TFT_DARKGREY);
  if (count > 1) {
    int fill = (w - 9) * idx / (count - 1);
    d.fillRect(x, y, fill + 4, 4, active ? TFT_CYAN : TFT_SILVER);
    d.fillRect(x + fill, y - 3, 9, 10, active ? TFT_YELLOW : TFT_WHITE);
  }
}
// Rewrite one row ONLY when its text/colour changed. Equal-length text is
// overdrawn with an opaque background, so there is never a black flash.
static void putRow(int i, uint16_t color, const char *text) {
  if (i < 0 || i >= P_ROW_COUNT) return;   // guard: bad row index
  UiRow &r = rowsPort[i];
  if (r.color == color && !strcmp(r.last, text)) return;
  bool sameLen = (r.last[0] && strlen(r.last) == strlen(text));
  r.color = color;
  strncpy(r.last, text, sizeof(r.last) - 1);
  r.last[sizeof(r.last) - 1] = 0;
  auto &d = M5.Display;
  int h = r.size == 3 ? 24 : (r.size == 2 ? 16 : 8);
  if (!sameLen) d.fillRect(0, r.y, d.width(), h, TFT_BLACK);
  useAsciiFont();                 // guards against a leftover CN font
  d.setTextSize(r.size);
  d.setTextColor(color, TFT_BLACK);
  d.setCursor(r.x, r.y);
  d.print(text);
}

static int countFilesCache = -1;
static uint32_t countFilesAt = 0;
static int countFiles() {
  if (!fsOk) return 0;
  if (countFilesCache >= 0 && millis() - countFilesAt < 5000) return countFilesCache;
  int cnt = 0;
  File root = LittleFS.open("/");
  File ff = root.openNextFile();
  while (ff) {
    String nm = ff.name();
    if (nm.endsWith(".GPX") || nm.endsWith(".gpx")) cnt++;
    ff = root.openNextFile();
  }
  countFilesCache = cnt;
  countFilesAt = millis();
  return cnt;
}

// ---------------- MENU sub page (v1.1.0) ----------------
// The old bottom horizontal menu bar is gone. Pressing MID on the main
// screen now opens a full-screen vertical MENU page; FRONT confirms the
// highlighted entry, the trailing EXIT returns to the main screen.
static const char *MENU_LABELS[] = {"文件", "相机", "设置", "导出", "关机", "退出"};
static const int   MENU_COUNT = 6;

static void drawMenuUI() {
  auto &d = M5.Display;
  useAsciiFont();
  d.fillScreen(TFT_BLACK);
  uiText(4, 4, "菜单", TFT_CYAN, TFT_BLACK, 2);
  for (int i = 0; i < MENU_COUNT; i++) {
    uiSelectRowP(2, 44 + i * 32, MENU_LABELS[i], i == menuIndex);
  }
  clearRowCache();
  d.waitDMA();
  Serial.printf("[MENU] draw idx=%d\n", menuIndex);
}

// Speed over ground in whole km/h (RMC field 7 arrives in knots).
static int spdKmh() {
  return (int)(gps.spdKt * 1.852f + 0.5f);
}

// 充电文字指示：与 BAT 同字号同一行（未满黄 / 充满绿），拔电隐藏。
static int8_t chgShown = -1;   // -1 未画, 0 隐藏, 1 黄, 2 绿
static void chgTextInvalidate(void) { chgShown = -1; }
static void drawChargingText(bool plugged, bool full) {
  int state = plugged ? (full ? 2 : 1) : 0;
  if (state == chgShown) return;
  chgShown = (int8_t)state;
  auto &d = M5.Display;
  useAsciiFont();
  d.setTextSize(1);
  d.fillRect(54, 144, 26, 10, TFT_BLACK);
  if (state != 0) {
    d.setTextColor(state == 2 ? TFT_GREEN : TFT_YELLOW, TFT_BLACK);
    d.setCursor(54, 146);
    d.print("CHG");
  }
}

// 右上角 CPU 频率小数字：80 绿 / 160 黄 / 240 红（无单位）。
static void drawCpuBadge(void) {
  unsigned mhz = (unsigned)(esp_clk_cpu_freq() / 1000000);
  char t[6];
  snprintf(t, sizeof t, "%u", mhz);
  uint16_t col = (mhz <= 80) ? TFT_GREEN
               : (mhz <= 160) ? TFT_YELLOW
                              : TFT_RED;
  if (!strcmp(t, cpuBadgeTxt) && col == cpuBadgeCol) return;
  strlcpy(cpuBadgeTxt, t, sizeof cpuBadgeTxt);
  cpuBadgeCol = col;
  auto &d = M5.Display;
  useAsciiFont();
  d.setTextSize(1);
  // 固定清 3 位数宽的区域，避免 240 -> 80 时残留左边的 "2"
  d.fillRect(110, 144, 25, 10, TFT_BLACK);
  int w = (int)strlen(t) * 6;
  d.setTextColor(col, TFT_BLACK);
  d.setCursor(132 - w, 146);
  d.print(t);
}

// 电量剩余可用时间：电压计无法测电流，按 250 mAh 容量 / 典型 95 mA 折算，
// 仅作量级参考。
#define BAT_CAPACITY_MAH 250
#define BAT_AVG_MA       95
static void batteryRuntimeText(char *out, size_t n) {
  if (batteryLevel <= 0) { snprintf(out, n, "LEFT --"); return; }
  if (M5.Power.getVBUSVoltage() > 4000) { snprintf(out, n, "LEFT CHG"); return; }
  int mins = (int)((float)BAT_CAPACITY_MAH * batteryLevel / 100.0f * 60.0f / BAT_AVG_MA);
  if (mins >= 60) snprintf(out, n, "LEFT %dh%02dm", mins / 60, mins % 60);
  else            snprintf(out, n, "LEFT %dm", mins);
}

// Bottom line shared by both layouts: status banner > auto-off countdown >
// the MID:MENU hint. The row index differs per layout, so it is a parameter.
static void drawBottomRow(int row) {
  char b[40];
  if (statusMsg[0] && millis() - statusMsgMs < 2500) {
    putRow(row, TFT_YELLOW, statusMsg);
  } else if (!recording && !exportMode && !usbPlugged() && autoOffMs > 60000u
             && millis() - lastActivityMs > autoOffMs - 60000u) {
    uint32_t left = (autoOffMs - (millis() - lastActivityMs)) / 1000;
    if (left >= 60) snprintf(b, sizeof(b), "AUTO OFF %lum", (unsigned long)((left + 59) / 60));
    else            snprintf(b, sizeof(b), "AUTO OFF %lus", (unsigned long)left);
    putRow(row, TFT_ORANGE, b);
  } else {
    putRow(row, TFT_DARKGREY, "MID:MENU");
  }
}

// Thin section dividers. Drawn once per full repaint (portChromeReady flag),
// because putRow() only clears its own row and never touches these lines.
static void drawPortraitChrome() {
  if (portChromeReady) return;
  auto &d = M5.Display;
  const uint16_t div = d.color565(48, 48, 48);
  d.drawFastHLine(6, 54,  d.width() - 12, div);   // after elapsed time
  d.drawFastHLine(6, 120, d.width() - 12, div);   // after ALT + speed
  d.drawFastHLine(6, 168, d.width() - 12, div);   // after battery
  portChromeReady = true;
}

// Portrait 135x240: 13 rows. Narrow canvas forces LAT / LON / ALT onto
// separate lines and lets AVG / MAX / BAT use size2 for readability.
static void drawMainPortrait() {
  char b[40];
  drawPortraitChrome();

  // P_CLOCK: time only (8 size2 chars = 96 px of the 135 px width).
  if (gps.timeValid) {
    int yy, mm, dd, hh, mi, ss;
    localParts(yy, mm, dd, hh, mi, ss);
    snprintf(b, sizeof(b), "%02d:%02d:%02d", hh, mi, ss);
  } else {
    snprintf(b, sizeof(b), "--:--:--");
  }
  uint16_t c = recording ? TFT_RED : (gps.valid ? TFT_CYAN : TFT_ORANGE);
  putRow(P_CLOCK, c, b);

  // P_STATE: REC / FIX / --- on its own line.
  putRow(P_STATE, c, recording ? "REC" : (gps.valid ? "FIX" : "---"));

  // P_TIME: elapsed recording time.
  if (recording && recStartSecs >= 0 && gps.timeValid) {
    char e[16]; fmtElapsed(gpsEpochSecs(gps) - recStartSecs, e, sizeof(e));
    snprintf(b, sizeof(b), "TIME %s", e);
  } else {
    snprintf(b, sizeof(b), "TIME --:--:--");
  }
  putRow(P_TIME, recording ? TFT_RED : TFT_DARKGREY, b);

  // P_SAT: satellites in use / visible, always yellow.
  int visTotal = gps.snrVis > 0 ? gps.snrVis : gps.satTotal;
  snprintf(b, sizeof(b), "SAT %02d/%02d", gps.sats, visTotal);
  putRow(P_SAT, TFT_YELLOW, b);

  // P_GNSS: constellation detail on its own line (2.0.0 freed the space by
  // merging AVG + MAX). Falls back to the short code when it would overflow.
  {
    const char *det = GNSS_DETAIL[gnssMode];
    snprintf(b, sizeof(b), "%s", det);
    putRow(P_GNSS, TFT_YELLOW, b);
  }

  // P_LAT / P_LON / P_ALT: one line each; speed rides along with altitude.
  if (gps.valid) {
    char lat[14], lon[14];
    dtostrf(gps.lat, 2, 5, lat); dtostrf(gps.lon, 3, 5, lon);
    snprintf(b, sizeof(b), "LAT %s", lat);
    putRow(P_LAT, TFT_GREEN, b);
    snprintf(b, sizeof(b), "LON %s", lon);
    putRow(P_LON, TFT_GREEN, b);
    snprintf(b, sizeof(b), "ALT %.0fm  SPD %dkm/h", gps.alt, spdKmh());
    putRow(P_ALT, TFT_GREEN, b);
  } else {
    putRow(P_LAT, TFT_DARKGREY, "LAT --.-----");
    putRow(P_LON, TFT_DARKGREY, "LON ---.-----");
    putRow(P_ALT, TFT_DARKGREY, "ALT --m  SPD --");
  }

  // P_SIG: AVG + MAX merged onto a single size1 line (green), P_BAT below.
  if (gps.snrVis > 0 && gps.snrCount > 0) {
    snprintf(b, sizeof(b), "AVG%d MAX%d", gps.snrAvg, gps.snrMax);
    putRow(P_SIG, TFT_WHITE, b);
  } else {
    putRow(P_SIG, TFT_DARKGREY, "AVG-- MAX--");
  }
  {
    // 小字电量：BAT 80%（绿）｜同行的 CHARGING｜右侧小数字 CPU 频率
    static int lastBatLen = -1;
    snprintf(b, sizeof(b), "BAT %d%%", batteryLevel);
    uint16_t bc = (batteryLevel >= 80) ? TFT_GREEN
                : (batteryLevel < 10)  ? TFT_RED
                                       : TFT_YELLOW;
    putRow(P_BAT, bc, b);
    // putRow 在文字长度变化时会整行清屏，角标/CHARGING 需要跟着重画
    if ((int)strlen(b) != lastBatLen) {
      lastBatLen = (int)strlen(b);
      cpuBadgeInvalidate();
      chgTextInvalidate();
    }
    // 充电指示：与 BAT 同字号同一行（未满=黄，充满=绿）；拔电隐藏
    bool plugged = (M5.Power.getVBUSVoltage() > 4000);
    bool full = (batteryLevel >= 100);
    drawChargingText(plugged, full);
    drawCpuBadge();
    // 下一行：剩余可用时间估算
    batteryRuntimeText(b, sizeof(b));
    putRow(P_BATRUN, TFT_DARKGREY, b);
  }

  // P_FREE / P_FILES: storage stats split over two lines.
  if (fsOk) {
    int freeKB = (int)((LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024);
    snprintf(b, sizeof(b), "FREE %dKB", freeKB);
    putRow(P_FREE, TFT_SILVER, b);
    if (recording) {
      snprintf(b, sizeof(b), "%upt   %us/pt",
               (unsigned)trackPoints, (unsigned)recIntervalS);
    } else {
      snprintf(b, sizeof(b), "FILES %d  %us/pt",
               countFiles(), (unsigned)recIntervalS);
    }
    putRow(P_FILES, TFT_SILVER, b);
  } else {
    putRow(P_FREE, TFT_SILVER, "FS ERROR");
    putRow(P_FILES, TFT_SILVER, "");
  }

  // P_LINK1 / P_LINK2：BLE 相机连接与 GPS 推送状态（两行小字）
  {
    FurbCam::LinkInfo li;
    FurbCam::getLinkInfo(li);
    if (li.active) {
      if (li.count > 1) snprintf(b, sizeof(b), "BT %s +%d", li.name, li.count - 1);
      else              snprintf(b, sizeof(b), "BT %s", li.name);
      putRow(P_LINK1, TFT_GREEN, b);
      if (li.gpsOk) {
        snprintf(b, sizeof(b), "GPS %02dSAT SENT%lu", li.sats, (unsigned long)li.sent);
        putRow(P_LINK2, TFT_LIGHTGREY, b);
      } else {
        putRow(P_LINK2, TFT_BLACK, "");  // 无定位：不显示
      }
    } else {
      putRow(P_LINK1, TFT_BLACK, "");
      putRow(P_LINK2, TFT_BLACK, "");
    }
  }

  drawBottomRow(P_HINT);
}

// Main-screen entry point: settle the SNR window once per second, then paint
// the fixed portrait layout.
static void drawMainUI() {
  settleSnrWindow();
  drawMainPortrait();
}

// OUTPUT chooser: 3 host-side clients + EXIT, same key model as FILES/SETUP.
static void drawOutputUI() {
  auto &d = M5.Display;
  useAsciiFont();
  d.fillScreen(TFT_BLACK);
  char b[64];
  const char *tool = outputIndex == 0 ? "export_gpx.html"
                   : outputIndex == 1 ? "export_gpx.py"
                   : outputIndex == 2 ? "export_gpx.ps1 / .sh"
                                      : "BACK TO MAIN";
  uiText(4, 2, "导出", TFT_CYAN, TFT_BLACK, 2);
  snprintf(b, sizeof(b), "TRACKS %d", countFiles());
  uiText(4, 24, b, TFT_SILVER);
  for (int i = 0; i < OUTPUT_ROWS; i++) {
    uiSelectRowP(2, 44 + i * 30, OUTPUT_LABELS[i], i == outputIndex);
  }
  uiTextWrap(4, 172, d.width() - 8, tool, TFT_CYAN);
  clearRowCache();
  d.waitDMA();
  Serial.printf("[OUT] draw idx=%d choice=%d\n", outputIndex, (int)outputChoice);
}

// ---------------- full-screen sub pages ----------------
static void clearRowCache() {
  for (int i = 0; i < P_ROW_COUNT; i++) { rowsPort[i].last[0] = 0; rowsPort[i].color = 0; }
  portChromeReady = false;
}

// Rebuild the .GPX list. Names are GPS timestamps, so sorting by name also
// gives chronological order. A trailing virtual "EXIT" row is NOT stored
// here: row index == fileCount means EXIT (see drawFilesUI).
static void collectFiles() {
  fileCount = 0;
  if (fsOk) {
    File root = LittleFS.open("/");
    File f = root.openNextFile();
    while (f) {
      String nm = f.name();
      if (!nm.startsWith("/")) nm = "/" + nm;
      if ((nm.endsWith(".GPX") || nm.endsWith(".gpx"))
          && fileCount < MAX_FILE_ITEMS) {
        snprintf(fileNames[fileCount], sizeof(fileNames[fileCount]),
                 "%s", nm.c_str());
        fileCount++;
      }
      f = root.openNextFile();
    }
  }
  // insertion sort (small list, no extra RAM)
  for (int i = 1; i < fileCount; i++) {
    char tmp[32];
    memcpy(tmp, fileNames[i], sizeof(tmp));
    int j = i - 1;
    while (j >= 0 && strcmp(fileNames[j], tmp) > 0) {
      memcpy(fileNames[j + 1], fileNames[j], sizeof(tmp));
      j--;
    }
    memcpy(fileNames[j + 1], tmp, sizeof(tmp));
  }
  int rows = fileCount + 1;              // files + EXIT
  if (fileIndex >= rows) fileIndex = rows - 1;
  if (fileIndex < 0)     fileIndex = 0;
  fileRowsVisible = FILE_ROWS_P;
  if (fileIndex < fileTop) fileTop = fileIndex;
  if (fileIndex >= fileTop + fileRowsVisible) fileTop = fileIndex - fileRowsVisible + 1;
  int maxTop = rows - fileRowsVisible;
  if (maxTop < 0) maxTop = 0;
  if (fileTop > maxTop) fileTop = maxTop;
}

static void drawFilesUI() {
  uint32_t t0 = millis();
  auto &d = M5.Display;
  useAsciiFont();
  d.fillScreen(TFT_BLACK);
  char b[64];
  snprintf(b, sizeof(b), "TRACKS %d", fileCount);
  uiText(4, 2, b, TFT_CYAN, TFT_BLACK, 2);
  if (fsOk) {
    int freeKB = (int)((LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024);
    snprintf(b, sizeof(b), "FREE %dKB", freeKB);
    uiText(4, 24, b, TFT_SILVER);
  }
  int rows = fileCount + 1;
  const int pitch = 26;
  const int y0    = 42;
  for (int i = 0; i < fileRowsVisible; i++) {
    int row = fileTop + i;
    if (row >= rows) break;
    int y = y0 + i * pitch;
    if (row < fileCount) {
      const char *q = fileNames[row];
      if (*q == '/') q++;
      // 文件名去掉 .GPX 后缀再显示：完整名 19 字符 = 152 px，会超出 135 px 屏宽
      char nm[32];
      strlcpy(nm, q, sizeof nm);
      size_t L = strlen(nm);
      if (L > 4 && nm[L - 4] == '.' &&
          (nm[L - 3] == 'g' || nm[L - 3] == 'G') &&
          (nm[L - 2] == 'p' || nm[L - 2] == 'P') &&
          (nm[L - 1] == 'x' || nm[L - 1] == 'X')) {
        nm[L - 4] = 0;
      }
      snprintf(b, sizeof(b), "%s", nm);
    } else {
      snprintf(b, sizeof(b), "退出");     // last row = back to parent menu
    }
    uiSelectRow(2, y, 131, b, row == fileIndex);
  }
  clearRowCache();
  Serial.printf("[FILE] draw count=%d idx=%d top=%d ms=%lu\n",
                fileCount, fileIndex, fileTop,
                (unsigned long)(millis() - t0));
}

static void drawDeleteUI() {
  auto &d = M5.Display;
  useAsciiFont();
  d.fillScreen(TFT_BLACK);
  const char *p = (fileIndex < fileCount) ? fileNames[fileIndex] : "";
  if (*p == '/') p++;
  uiText(4, 2, "删除轨迹?", TFT_YELLOW, TFT_BLACK, 2);
  uiText(4, 30, p, TFT_WHITE);
  uiSelectRowP(4,  70, "删除", deleteChoice == 0);
  uiSelectRowP(4, 106, "取消", deleteChoice == 1);
  uiText(4, 150, delArmed ? "再按一次确认!" : "不可恢复", TFT_RED);
  clearRowCache();
  Serial.printf("[DEL] draw choice=%d file=%s\n", deleteChoice, p);
}

static void doDelete() {
  if (fileIndex >= fileCount) { setUiMode(UI_FILES); return; }
  char path[32];
  snprintf(path, sizeof(path), "%s", fileNames[fileIndex]);
  bool ok = LittleFS.remove(path);
  Serial.printf("[DEL] remove %s -> %d\n", path, ok);
  countFilesCache = -1;
  collectFiles();
  setUiMode(UI_FILES);          // back to the (refreshed) list
}

// ---------------- SETUP page: English labels + graphical sliders ----------------
static const char *SETUP_LABELS[] = {
  "亮度", "熄屏延时", "记录间隔", "自动关机", "GNSS系统", "退出"
};
static const int SETUP_COUNT = 6;
// GNSS is an enumerated (not numeric) slider: the step value IS the mode
// index, so the existing slider machinery can be reused unchanged.
static const uint16_t GNSS_STEPS[] = {0, 1, 2, 3, 4, 5, 6};
static const int      GNSS_STEPS_N = GNSS_COUNT;

static int stepIndexOf(uint32_t v, const uint16_t *steps, int n) {
  for (int i = 0; i < n; i++) if (v == steps[i]) return i;
  return 0;
}
static uint16_t snapToStep(const uint16_t *steps, int n, int v) {
  int best = 0, bd = abs(v - (int)steps[0]);
  for (int i = 1; i < n; i++) {
    int dd = abs(v - (int)steps[i]);
    if (dd < bd) { bd = dd; best = i; }
  }
  return steps[best];
}
static const uint16_t *setupSteps(int i, int &n) {
  switch (i) {
    case 0: n = BRIGHT_STEPS_N;   return BRIGHT_STEPS;
    case 1: n = DIMAFTER_STEPS_N; return DIMAFTER_STEPS;
    case 2: n = REC_STEPS_N;      return REC_STEPS;
    case 3: n = AUTOOFF_STEPS_N;  return AUTOOFF_STEPS;
    case 4: n = GNSS_STEPS_N;     return GNSS_STEPS;
    default: n = 0; return nullptr;
  }
}
static uint32_t setupCurValue(int i) {
  switch (i) {
    case 0: return brightHigh;
    case 1: return dimAfterMs / 1000u;
    case 2: return recIntervalS;
    case 3: return autoOffMs / 1000u;
    case 4: return gnssMode;
    default: return 0;
  }
}
// Format an auto-off seconds value for the SETUP row: "30s" for sub-minute,
// "1min" / "5min" ... for whole minutes.
static void fmtAutoOff(uint32_t secs, char *out, size_t n) {
  if (secs < 60) snprintf(out, n, "%lus", (unsigned long)secs);
  else if (secs % 60 == 0) snprintf(out, n, "%lumin", (unsigned long)(secs / 60));
  else snprintf(out, n, "%lus", (unsigned long)secs);
}
static void settingValue(int i, char *out, size_t n) {
  switch (i) {
    case 0: snprintf(out, n, "%u", (unsigned)brightHigh); break;
    case 1: snprintf(out, n, "%lus", (unsigned long)(dimAfterMs / 1000)); break;
    case 2: snprintf(out, n, "%us/pt", (unsigned)recIntervalS); break;
    case 3: fmtAutoOff(autoOffMs / 1000u, out, n); break;
    case 4: snprintf(out, n, "%s", GNSS_LABELS[gnssMode]); break;
    default: snprintf(out, n, "--"); break;
  }
}
static void drawSettingsUI() {
  uint32_t t0 = millis();
  auto &d = M5.Display;
  useAsciiFont();
  d.fillScreen(TFT_BLACK);
  char b[64];
  uiText(4, 2, "设置", TFT_CYAN, TFT_BLACK, 2);
  d.setTextSize(1);
  d.setTextColor(TFT_SILVER, TFT_BLACK);
  d.setCursor(d.width() - 48, 6);
  d.print("v" FIRMWARE_VERSION);
  // Two-line blocks: label + value on line 1, full-width slider on line 2.
  for (int i = 0; i < SETUP_COUNT; i++) {
    bool sel = (i == settingsIndex);
    int y = 30 + i * 32;
    if (sel) d.fillRect(0, y - 2, 4, 20, TFT_YELLOW);   // cursor marker
    uiText(8, y, SETUP_LABELS[i], sel ? TFT_YELLOW : TFT_WHITE);
    settingValue(i, b, sizeof(b));
    int vw = (int)strlen(b) * 8;
    uiText(d.width() - vw - 8, y, b, sel ? TFT_YELLOW : TFT_SILVER);
    int cnt = 0;
    const uint16_t *steps = setupSteps(i, cnt);
    if (steps) {
      drawSliderP(8, y + 19, d.width() - 16,
                  stepIndexOf(setupCurValue(i), steps, cnt), cnt, sel);
    }
  }
  clearRowCache();
  char v[16]; settingValue(settingsIndex, v, sizeof(v));
  Serial.printf("[SET] draw row=%d val=%s ms=%lu\n", settingsIndex, v,
                (unsigned long)(millis() - t0));
}

static void drawExportPage() {
  auto &d = M5.Display;
  d.fillScreen(TFT_BLACK);
  uiText(4, 2, "导出", TFT_CYAN, TFT_BLACK, 2);

  const int w = d.width() - 12;
  char b[48];

  // 纵向流式排版：每个块按实际行数推进 y，长文本自动换行不会互相压盖
  int y = 34;
  snprintf(b, sizeof(b), "方式: %s", OUTPUT_LABELS[outputChoice]);
  y = uiTextWrap(6, y, w, b, TFT_YELLOW) + 6;
  d.fillRect(6, y, w, 1, TFT_DARKGREY);
  y += 8;

  y = uiTextWrap(6, y, w, "电脑端运行:", TFT_SILVER) + 4;
  const char *tool = outputChoice == 0 ? "export_gpx.html"
                   : outputChoice == 1 ? "export_gpx.py"
                                       : "export_gpx.ps1 / .sh";
  y = uiTextWrap(6, y, w, tool, TFT_WHITE) + 8;
  y = uiTextWrap(6, y, w, "发送 D 指令开始导出", TFT_DARKGREY) + 8;

  snprintf(b, sizeof(b), "上次导出: %d 个", exportDoneCount);
  uiTextWrap(6, y, w, b, TFT_SILVER);

  uiSelectRowP(4, 204, "退出", true);
}

static void setUiMode(UiMode m) {
  if (uiMode == UI_CAM && m != UI_CAM) FurbCam::leave();
  uiMode = m;
  switch (m) {
    case UI_MAIN:     useAsciiFont(); uiReset(); drawMainUI(); break;
    case UI_MENU:     menuIndex = 0; drawMenuUI();            break;
    case UI_FILES:    collectFiles(); drawFilesUI();     break;
    case UI_SETTINGS: drawSettingsUI();                  break;
    case UI_DELETE:   drawDeleteUI();                    break;
    case UI_OUTPUT:   outputIndex = outputChoice; drawOutputUI(); break;
    case UI_CAM:      FurbCam::enter(); FurbCam::draw();       break;
    case UI_EXPORT:   drawExportPage();                        break;
  }
}

// 4.5.0: 旋转选择器 / 横屏布局已移除，方向固定竖屏。
static void loadSettings();
static void saveSettings();
static void changeSetting(int i) {
  int cnt = 0;
  const uint16_t *steps = setupSteps(i, cnt);
  if (!steps) {                        // last row = EXIT -> 返回上一级菜单
    setUiMode(UI_MENU);
    return;
  }
  int next = (stepIndexOf(setupCurValue(i), steps, cnt) + 1) % cnt;
  uint16_t v = steps[next];
  switch (i) {
    case 0:
      brightHigh = (uint8_t)v;
      M5.Display.setBrightness(dimmed ? dimLevel : brightHigh);
      break;
    case 1: dimAfterMs   = (uint32_t)v * 1000u; break;
    case 2: recIntervalS = v;                   break;  // linked to main UI
    case 3: autoOffMs    = (uint32_t)v * 1000u; break;
    case 4:                                     // GNSS SYS
      gnssMode = (uint8_t)v;
      sendGnssMode(gnssMode);                  // takes effect immediately
      break;
    default: return;
  }
  saveSettings();
  drawSettingsUI();
  Serial.printf("[SET] rec=%us dim=%u after=%lus off=%lus bright=%u prompt=%d gnss=%s\n",
                (unsigned)recIntervalS, (unsigned)dimLevel,
                (unsigned long)(dimAfterMs / 1000),
                (unsigned long)(autoOffMs / 1000),
                (unsigned)brightHigh, promptOn ? 1 : 0,
                GNSS_LABELS[gnssMode]);
}

// One ASCII line:
// "<rec_s> <dim_after_s> <auto_off_s> <prompt> <dim_level> <bright> <gnss> <rotate> <output>"
// Field 9 (last OUTPUT method) is from v13. Field 8 (rotation) is still
// parsed so old files load unchanged, but 4.5.0 fixed the UI to portrait and
// ignores it. Legacy 5..8-field files load with BROWSER as the output.
static void loadSettings() {
  if (!fsOk) return;
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  while (f) {
    const char *nm = f.name();
    if (!strcmp(nm, "settings.cfg") || !strcmp(nm, "/settings.cfg")) {
      char b[64];
      int n = (int)f.readBytesUntil('\n', b, sizeof(b) - 1);
      b[n > 0 ? n : 0] = 0;
      int r = 10, d = 30, a = 300, bp = 1, dl = 12, bh = BRIGHT_HIGH;
      int gm = 0, rm = 1, oc = 0, rv = 0;
      int got = sscanf(b, "%d %d %d %d %d %d %d %d %d %d",
                       &r, &d, &a, &bp, &dl, &bh, &gm, &rm, &oc, &rv);
      if (got >= 5) {
        // Snap to the slider steps so a legacy value (e.g. 300 s auto-off)
        // can never leave the knob between two positions.
        recIntervalS = snapToStep(REC_STEPS, REC_STEPS_N, r);
        dimAfterMs   = (uint32_t)snapToStep(DIMAFTER_STEPS, DIMAFTER_STEPS_N, d) * 1000u;
        autoOffMs    = (uint32_t)snapToStep(AUTOOFF_STEPS, AUTOOFF_STEPS_N, a) * 1000u;
        promptOn     = (bp != 0);
        dimLevel     = constrain(dl, 1, 255);
        brightHigh   = (got >= 6) ? (uint8_t)constrain(bh, 20, 255)
                                  : (uint8_t)BRIGHT_HIGH;
      }
      if (got >= 7) gnssMode   = (uint8_t)constrain(gm, 0, GNSS_COUNT - 1);
      if (got >= 9) outputChoice = (uint8_t)constrain(oc, 0, OUTPUT_METHODS - 1);
      (void)rm;  // field 8 = 旧旋转设置，横屏移除后忽略
      (void)rv;
      f.close();
      Serial.printf("[SET] loaded rec=%us dim=%u after=%lus off=%lus bright=%u prompt=%d gnss=%s out=%s\n",
                    (unsigned)recIntervalS, (unsigned)dimLevel,
                    (unsigned long)(dimAfterMs / 1000),
                    (unsigned long)(autoOffMs / 1000),
                    (unsigned)brightHigh, promptOn ? 1 : 0,
                    GNSS_LABELS[gnssMode], OUTPUT_LABELS[outputChoice]);
      return;
    }
    f = root.openNextFile();
  }
}
static void saveSettings() {
  if (!fsOk) return;
  File f = LittleFS.open("/settings.cfg", FILE_WRITE);
  if (!f) return;
  char b[96];
  // Field 8 (rotation) is written as 0 for backward-compatible parsing;
  // 4.5.0 fixed the UI to portrait so it is no longer meaningful.
  snprintf(b, sizeof(b), "%u %lu %lu %d %u %u %u %u %u 2\n", (unsigned)recIntervalS,
           (unsigned long)(dimAfterMs / 1000), (unsigned long)(autoOffMs / 1000),
           promptOn ? 1 : 0, (unsigned)dimLevel, (unsigned)brightHigh,
           (unsigned)gnssMode, 0u, (unsigned)outputChoice);
  f.print(b);
  f.close();
}

// ---------------- recording feedback / low-battery guard ----------------
// No I2S audio (TG1WDT_SYS_RST on this toolchain): distinct backlight pulse
// patterns mark REC start / stop / low battery. Non-blocking scheduling.
struct Pulse { uint16_t onMs; uint16_t offMs; };
static const Pulse PROMPT_START[]  = {{ 90, 90}, { 90, 90}};
static const Pulse PROMPT_STOP[]   = {{180, 90}, {180, 90}, {180, 90}};
static const Pulse PROMPT_LOWBAT[] = {{ 60, 60}, { 60, 60}, { 60, 60}, { 60, 60}, {60, 60}};
static const Pulse *pulseSeq = nullptr;
static int      pulseLen = 0, pulseIdx = 0;
static uint32_t pulseNextMs = 0;
static bool     pulseLit = false;

static void playPrompt(const Pulse *seq, int n) {
  if (!promptOn) return;
  pulseSeq = seq; pulseLen = n; pulseIdx = 0;
  pulseNextMs = millis(); pulseLit = false;
}
static void servicePrompt() {
  if (!pulseSeq) return;
  uint32_t now = millis();
  if ((int32_t)(now - pulseNextMs) < 0) return;
  if (!pulseLit) {
    M5.Display.setBrightness(brightHigh);
    pulseLit = true;
    pulseNextMs = now + pulseSeq[pulseIdx].onMs;
  } else {
    M5.Display.setBrightness(dimLevel);
    pulseLit = false;
    pulseNextMs = now + pulseSeq[pulseIdx].offMs;
    if (++pulseIdx >= pulseLen) {
      pulseSeq = nullptr;
      M5.Display.setBrightness(dimmed ? dimLevel : brightHigh);
    }
  }
}
// 锂电池电压 -> 百分比（轻载曲线，线性插值）
static int batteryPercentFromMv(int mv) {
  static const struct { int mv; int pct; } tbl[] = {
    {3300, 0}, {3450, 5}, {3550, 10}, {3650, 20}, {3750, 40},
    {3850, 60}, {3950, 80}, {4050, 90}, {4200, 100},
  };
  const int n = (int)(sizeof tbl / sizeof tbl[0]);
  if (mv <= tbl[0].mv) return 0;
  if (mv >= tbl[n - 1].mv) return 100;
  for (int i = 1; i < n; i++) {
    if (mv <= tbl[i].mv) {
      int span = tbl[i].mv - tbl[i - 1].mv;
      int d = mv - tbl[i - 1].mv;
      return tbl[i - 1].pct + (tbl[i].pct - tbl[i - 1].pct) * d / span;
    }
  }
  return 100;
}

static int batteryPercentSmooth(int target) {
  static int cur = -1;
  if (cur < 0) cur = target;
  else cur = (cur * 3 + target) / 4;   // 75% 旧值平滑，抑制跳变
  return cur;
}

static void updateBatteryLevel(void) {
  int mv = M5.Power.getBatteryVoltage();
  bool chg = M5.Power.isCharging();
  if (mv > 100) {
    int adj = mv - (chg ? 60 : 0);     // 充电时电压虚高，约 60mV 补偿
    batteryLevel = batteryPercentSmooth(batteryPercentFromMv(adj));
  } else {
    batteryLevel = M5.Power.getBatteryLevel();
  }
}

static void checkBattery() {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < BAT_CHECK_MS) return;
  lastCheck = millis();
  updateBatteryLevel();
  if (batteryLevel > BAT_RECOVER_PERCENT) lowBatLock = false;
  if (!recording) { lowBatHits = 0; return; }
  if (batteryLevel >= 0 && batteryLevel <= BAT_LOW_PERCENT) {
    if (++lowBatHits >= 2) {                 // 2 checks x 10 s, avoid spikes
      lowBatHits = 0;
      lowBatLock = true;
      stopRecording();
      countFilesCache = -1;
      setStatus("LOW BAT - REC STOPPED");
      playPrompt(PROMPT_LOWBAT, 5);
      if (uiMode != UI_MAIN) {
        setUiMode(UI_MAIN);
      }
      Serial.printf("[BAT] %d%% -> auto stop recording\n", batteryLevel);
    }
  } else {
    lowBatHits = 0;
  }
}

// ---------------- buttons: direct GPIO scan on FRONT(G11) / MID(G12) ----------------
// M5Unified's Button_Class has a 500 ms hold threshold and a double-click
// state machine that delays clicks - it caused unresponsive keys. We read
// GPIO11/GPIO12 ourselves: active LOW, 25 ms debounce, click vs 1.5 s hold.
// wakeOnly marks a key that was pressed while the screen was dimmed: that
// entire down->up cycle only restores brightness, no click and no hold.
struct KeyState {
  uint8_t pin; bool last; bool armed; uint32_t downMs; bool holdFired; bool wakeOnly;
};
static KeyState keys[2] = {
  {11, false, false, 0, false, false}, {12, false, false, 0, false, false}
};

static void powerOffNow();
static void toggleRecording(const char *keyName);
static void onKeyClick(int idx);
static void onKeyHold(int idx);
static bool keyIsWakeOnly(uint32_t now);

// USB-serial commands (export protocol D/E/L/R/T/X/K UNCHANGED):
//   M=manual waypoint, S=swap FRONT/MID keys, P=idle/power diagnostics.
static void handleSerialCmd() {
  // Bounded loop: USB CDC can report available()>0 while read() returns -1,
  // and an unbounded while() starves the idle task -> TG1WDT reset.
  for (int guard = 0; guard < 8 && Serial.available() > 0; guard++) {
    int v = Serial.read();
    if (v < 0) break;
    char c = (char)v;
    noteActivity();                     // PC activity keeps the screen on
    if (c == 'R')      { toggleRecording("SERIAL"); }
    else if (c == 'D' || c == 'E') { dumpAllGpx(); }
    else if (c == 'G') {
      // 导出 /debug.log 原文
      fileLogSuspend(true);
      File lf = LittleFS.open("/debug.log");
      Serial.println("#LOGFILE_BEGIN");
      if (lf) {
        uint8_t buf[256];
        int got;
        while ((got = lf.read(buf, sizeof(buf))) > 0) Serial.write(buf, got);
        lf.close();
      }
      Serial.println("#LOGFILE_END");
      fileLogSuspend(false);
    }
    else if (c == 'L') { listGpx(); }
    else if (c == 'T') { makeTestGpx(); }
    else if (c == 'X') { cleanTestGpx(); }
    else if (c == 'M') {
      if (recording && gps.valid) { writeTrackpoint(); Serial.println("#MARK_OK"); }
    }
    else if (c == 'S') {
      keySwap = !keySwap; saveKeySwap();
      setStatus(keySwap ? "KEYS SWAPPED" : "KEYS DEFAULT");
      setUiMode(UI_MAIN);            // always return to the ASCII main screen
      Serial.printf("[CMD] keySwap=%d\n", keySwap);
    }
    else if (c == 'P') {
      Serial.printf("[CMD] mode=%d exp=%d menu=%d set=%d files=%d fsel=%d del=%d dim=%d usb=%d bat=%d rec=%d idle=%lu selAge=%lu vbus=%d ovr=%d recInt=%us dimA=%lus off=%lus bright=%u prompt=%d\n",
                    (int)uiMode, exportMode, menuIndex, settingsIndex,
                    fileCount, fileIndex, deleteChoice, dimmed,
                    usbPlugged() ? 1 : 0, batteryLevel, recording,
                    (unsigned long)(millis() - lastActivityMs),
                    (unsigned long)(millis() - selectEnterMs),
                    (int)M5.Power.getVBUSVoltage(), usbOverride,
                    (unsigned)recIntervalS, (unsigned long)(dimAfterMs / 1000),
                    (unsigned long)(autoOffMs / 1000), (unsigned)brightHigh,
                    promptOn ? 1 : 0);
      Serial.printf("[CFG] gnss=%s(%d) portrait=1 imu=%d spd=%d\n",
                    GNSS_LABELS[gnssMode], (int)gnssMode,
                    M5.Imu.isEnabled() ? 1 : 0, spdKmh());
      Serial.printf("[WAKE] graceLeft=%ld status=\"%s\" idle=%lu dimA=%lus\n",
                    (long)(int32_t)(wakeGraceUntilMs - millis()), statusMsg,
                    (unsigned long)(millis() - lastActivityMs),
                    (unsigned long)(dimAfterMs / 1000));
      Serial.printf("[OUT] choice=%s(%d) idx=%d phase=%d sent=%d/%d last=%d\n",
                    OUTPUT_LABELS[outputChoice], (int)outputChoice, outputIndex,
                    (int)exportPhase, exportSent, exportTotal, exportLastCount);
    }
    else if (c == 'A') { onKeyClick(0); }   // headless test: FRONT short press
    else if (c == 'B') { onKeyClick(1); }   // headless test: MID short press
    else if (c == 'J') {                   // PM 锁与当前频率诊断
      rtc_cpu_freq_config_t rc = {};
      rtc_clk_cpu_freq_get_config(&rc);
      Serial.printf("[PM] cpu=%u MHz (regs %u, src %d, div %u) target=%u pm=%d\n",
                    (unsigned)(esp_clk_cpu_freq() / 1000000), (unsigned)rc.freq_mhz,
                    (int)rc.source, (unsigned)rc.div, (unsigned)cpuMhzNow,
                    pmReady ? 1 : 0);
      FILE *f = fopen("/littlefs/pm.txt", "w");
      if (f) {
        esp_pm_dump_locks(f);
        fclose(f);
      }
      File pf = LittleFS.open("/pm.txt");
      if (pf) {
        Serial.println("#PM_BEGIN");
        uint8_t pb[256];
        int got;
        while ((got = pf.read(pb, sizeof(pb))) > 0) Serial.write(pb, got);
        Serial.println("#PM_END");
        pf.close();
      }
    }
    else if (c == 'U') {                   // force the 10 s select timeout
      selectEnterMs = millis() - (SELECT_TIMEOUT_MS - 2000u);
      Serial.println("[TEST] forced select timeout in 2s");
    }
    else if (c == 'Z') {                   // restore factory defaults
      recIntervalS = 10; dimAfterMs = 30000u; autoOffMs = 90000u;
      brightHigh = 200; dimLevel = 12; promptOn = true;
      gnssMode = 0; outputChoice = 0;
      saveSettings();
      sendGnssMode(gnssMode);
      M5.Display.setBrightness(dimmed ? dimLevel : brightHigh);
      setUiMode(UI_MAIN);
      Serial.println("[SET] defaults restored 10s/pt dimAfter=30s off=90s bright=200 gnss=AUTO out=BROWSER");
    }
    else if (c == 'G') {                   // IMU diagnostics
      float ax = 0, ay = 0, az = 0;
      bool ok = M5.Imu.isEnabled() && M5.Imu.getAccel(&ax, &ay, &az);
      Serial.printf("[IMU] enabled=%d type=%d ok=%d a=(%.3f,%.3f,%.3f) %dx%d\n",
                    M5.Imu.isEnabled() ? 1 : 0, (int)M5.Imu.getType(), ok ? 1 : 0,
                    ax, ay, az, (int)M5.Display.width(), (int)M5.Display.height());
    }
    else if (c == 'N') {                   // cycle GNSS mode headlessly
      gnssMode = (gnssMode + 1) % GNSS_COUNT;
      sendGnssMode(gnssMode);
      saveSettings();
      Serial.printf("[GNSS] now %s\n", GNSS_LABELS[gnssMode]);
    }
    else if (c == 'W') {                   // 横屏已移除，固定竖屏
      Serial.println("[ROT] portrait only (landscape removed)");
    }
    else if (c == 'V') {                   // font / UI sanity
      Serial.printf("[UI] font=Font0 ascii-only mode=%d\n", (int)uiMode);
    }
    else if (c == 'F') {          // headless regression for the wake banner
      dimmed = true;
      M5.Display.setBrightness(dimLevel);
      lastActivityMs = millis() - dimAfterMs;
      Serial.printf("[TEST] F1 press on dimmed screen -> wakeOnly=%d (want 1)\n",
                    keyIsWakeOnly(millis()) ? 1 : 0);
      noteActivity();                     // exactly what the down edge does
      wakeGraceUntilMs = millis() + WAKE_GRACE_MS;   // what the release does
      Serial.printf("[TEST] F2 press again during grace -> wakeOnly=%d (want 0)\n",
                    keyIsWakeOnly(millis()) ? 1 : 0);
      setStatus("WAKE - PRESS AGAIN");
      clearStatus();                     // what any real key action does
      Serial.printf("[TEST] F3 banner cleared by action -> empty=%d (want 1)\n",
                    statusMsg[0] ? 0 : 1);
      wakeGraceUntilMs = 0;
      dimmed = true;
      lastActivityMs = millis() - dimAfterMs;
      Serial.printf("[TEST] F4 after grace, dimmed again -> wakeOnly=%d (want 1)\n",
                    keyIsWakeOnly(millis()) ? 1 : 0);
      wakeGraceUntilMs = 0;
      clearStatus();
      noteActivity();                    // restore bright + fresh idle timer
      M5.Display.setBrightness(brightHigh);
      Serial.println("[TEST] F done - state restored");
    }
    else if (c == 'Q') {                   // headless test: start prompt
      playPrompt(PROMPT_START, 2);
      Serial.println("[UI] start prompt queued");
    }
    else if (c == 'I') {          // force the auto-off path in 3 s (diagnostics)
      usbOverride = true;
      uint32_t off = autoOffMs ? autoOffMs : 300000u;
      lastActivityMs = millis() - (off - 3000u);
      Serial.println("[TEST] forced auto-off in 3s");
    }
    else if (c == 'O') {          // restore normal USB detection
      usbOverride = false;
      lastActivityMs = millis();
      Serial.println("[TEST] USB detection restored");
    }
    else if (c == 'K') { Serial.printf("[CMD] G11=%d G12=%d\n", digitalRead(11), digitalRead(12)); }
  }
}

// Single source of truth for "this press only wakes the screen".
// The grace window overrides everything: right after a wake the user is
// still reacting to the banner, so a second press must NEVER be a wake.
// dimmed is the real state; the idle-time check only covers the ~2 ms race
// where the idle threshold was crossed but manageIdle() has not run yet.
static bool keyIsWakeOnly(uint32_t now) {
  if (wakeGraceActive(now)) return false;
  return dimmed || (now - lastActivityMs >= dimAfterMs);
}

static void scanKeys() {
  uint32_t now = millis();
  for (int i = 0; i < 2; i++) {
    KeyState &k = keys[i];
    bool down = (digitalRead(k.pin) == LOW);
    int logical = keySwap ? (1 - i) : i;   // serial 'S' swaps FRONT/MID
    if (down != k.last) {
      if (down) {
        k.downMs = now;
        k.holdFired = false;
        // A press on a dimmed screen is WAKE-ONLY. Disarm it so neither the
        // release click nor the 1.5 s hold can run any action.
        k.wakeOnly = keyIsWakeOnly(now);
        k.armed = !k.wakeOnly;
        if (k.wakeOnly) {
          Serial.printf("[KEY] %s wake-only (action ignored)\n", logical == 0 ? "FRONT" : "MID");
          setStatus("WAKE - PRESS AGAIN");
        }
        noteActivity();              // restores normal brightness + resets idle
      } else {
        if (k.wakeOnly) {
          k.wakeOnly = false;        // swallow the release of the wake press
          // Grace starts when the wake press is fully released: from now on
          // the screen stays bright and the next press performs its action.
          wakeGraceUntilMs = now + WAKE_GRACE_MS;
          Serial.printf("[WAKE] grace %lus - next press acts\n",
                        (unsigned long)(WAKE_GRACE_MS / 1000));
        } else if (k.armed && !k.holdFired && now - k.downMs < HOLD_MS) {
          onKeyClick(logical);
        }
        k.armed = false;
        noteActivity();
      }
      k.last = down;
      delay(25);                    // simple debounce
    }
    // Only a HIGH->LOW edge arms the hold timer; an idle-low pin can never
    // fire a phantom hold (which would call powerOff and reboot-loop).
    if (down && k.armed && !k.holdFired && now - k.downMs >= HOLD_MS) {
      k.holdFired = true;
      k.armed = false;
      onKeyHold(logical);
    }
  }
}

static void toggleRecording(const char *keyName) {
  if (!recording) {
    if (lowBatLock)                        { setStatus("BAT TOO LOW - CHARGE"); }
    else if (!gps.timeValid)               { setStatus("NO TIME - WAIT FOR GPS"); }
    else if (!fsOk)                        { setStatus("FS ERROR"); }
    else if (startRecording()) {
      setStatus(gps.valid ? "REC STARTED" : "REC NO FIX");
      countFilesCache = -1;
      playPrompt(PROMPT_START, 2);        // two short flashes
    }
    else                                  { setStatus("FILE CREATE FAILED"); }
  } else {
    stopRecording();
    countFilesCache = -1;
    setStatus("REC SAVED");
    playPrompt(PROMPT_STOP, 3);           // three long flashes
  }
  Serial.printf("[KEY] %s toggle -> recording=%d\n", keyName, recording);
  // A running track always takes priority over any menu / sub page.
}
static void onKeyClick(int idx) {
  const char *nm = idx == 0 ? "FRONT" : "MID";
  Serial.printf("[KEY] %s click mode=%d\n", nm, (int)uiMode);
  clearStatus();                 // retire the wake banner before any action
  wakeGraceUntilMs = 0;          // the wake grace has served its purpose
  selectEnterMs = millis();      // ANY real key press restarts the 10 s timer
  if (exportMode) { exitExportScreen(); setStatus("EXPORT OFF"); return; }

  if (uiMode == UI_EXPORT) {
    setUiMode(UI_MENU);
    return;
  }
  if (uiMode == UI_CAM) {
    if (idx == 1) FurbCam::mid();
    else          FurbCam::front();
    return;
  }

  if (idx == 1) {                       // MID = SELECT / next row
    if (uiMode == UI_MAIN) {
      setUiMode(UI_MENU);              // open the full-screen menu page
    } else if (uiMode == UI_MENU) {
      menuIndex = (menuIndex + 1) % MENU_COUNT;
      drawMenuUI();
    } else if (uiMode == UI_FILES) {
      int rows = fileCount + 1;                 // files + trailing EXIT
      fileIndex = (fileIndex + 1) % rows;
      if (fileIndex < fileTop) fileTop = fileIndex;
      if (fileIndex >= fileTop + FILE_ROWS_P) fileTop = fileIndex - FILE_ROWS_P + 1;
      drawFilesUI();
    } else if (uiMode == UI_SETTINGS) {
      settingsIndex = (settingsIndex + 1) % SETUP_COUNT;
      drawSettingsUI();
    } else if (uiMode == UI_DELETE) {
      deleteChoice = 1 - deleteChoice;         // DELETE <-> CANCEL
      delArmed = false;
      drawDeleteUI();
    } else if (uiMode == UI_OUTPUT) {
      outputIndex = (outputIndex + 1) % OUTPUT_ROWS;
      drawOutputUI();
    }
    return;
  }

  // FRONT = CONFIRM / primary action
  if (uiMode == UI_MAIN) {
    toggleRecording(nm);
    return;
  }
  if (uiMode == UI_MENU) {
    switch (menuIndex) {
      case 0: fileIndex = 0; fileTop = 0; setUiMode(UI_FILES);    break; // FILES
      case 1: setUiMode(UI_CAM);                                  break; // CAMERA
      case 2: settingsIndex = 0;          setUiMode(UI_SETTINGS); break; // SETUP
      case 3: setUiMode(UI_OUTPUT);                              break; // OUTPUT
      case 4: powerOffNow();                                      break; // POWEROFF
      case 5: setUiMode(UI_MAIN);                                break; // EXIT
    }
    return;
  }
  if (uiMode == UI_OUTPUT) {
    if (outputIndex >= OUTPUT_METHODS) { setUiMode(UI_MENU); return; }  // EXIT -> 菜单
    outputChoice = (uint8_t)outputIndex;
    saveSettings();
    setUiMode(UI_EXPORT);
    Serial.printf("[OUT] chosen %s(%d) saved\n",
                  OUTPUT_LABELS[outputChoice], (int)outputChoice);
    return;
  }
  if (uiMode == UI_FILES) {
    if (fileIndex >= fileCount) setUiMode(UI_MENU);   // EXIT -> 返回菜单
    else { deleteChoice = 1; delArmed = false; setUiMode(UI_DELETE); }  // 默认 CANCEL，防误删
    return;
  }
  if (uiMode == UI_DELETE) {
    if (deleteChoice == 0) {
      if (!delArmed) {              // 第一次：解锁警告
        delArmed = true;
        delArmedMs = millis();
        drawDeleteUI();
      } else {                      // 第二次：真正删除
        delArmed = false;
        doDelete();
      }
    } else {                        // CANCEL
      delArmed = false;
      setUiMode(UI_FILES);
    }
    return;
  }
  if (uiMode == UI_SETTINGS) { changeSetting(settingsIndex); return; }
}
static void onKeyHold(int idx) {
  const char *nm = idx == 0 ? "FRONT" : "MID";
  Serial.printf("[KEY] %s HOLD mode=%d\n", nm, (int)uiMode);
  clearStatus();
  wakeGraceUntilMs = 0;
  if (exportMode) { exitExportScreen(); setStatus("EXPORT OFF"); return; }
  if (uiMode == UI_EXPORT) {
    setUiMode(UI_MENU);
    return;
  }
  if (uiMode == UI_CAM) {
    if (idx == 1) FurbCam::back();         // MID hold = 返回上一级
    else          FurbCam::requestClear(); // FRONT hold = 清除配对(需确认)
    return;
  }
  if (uiMode != UI_MAIN) {
    Serial.println("[KEY] long hold ignored (use POWEROFF in the menu)");
    return;
  }
  if (idx == 0) powerOffNow();
  else          { setUiMode(UI_OUTPUT); }
}

// Every non-main page auto-returns to the main screen after SUBPAGE_TIMEOUT_MS
// (30 s) with no REAL key press, so an abandoned page can neither stay on
// forever nor hold the backlight up. MENU keeps its shorter 10 s timeout
// because it is only a transient hop.
// Uses its own timestamp so USB serial traffic / dim / auto-off cannot
// disturb the countdown.
static void manageSelectTimeout() {
  if (uiMode == UI_DELETE && delArmed && millis() - delArmedMs > 3000) {
    delArmed = false;
    drawDeleteUI();
  }
  if (uiMode == UI_MAIN || uiMode == UI_CAM) return;
  uint32_t limit = (uiMode == UI_MENU) ? SELECT_TIMEOUT_MS : SUBPAGE_TIMEOUT_MS;
  if (millis() - selectEnterMs < limit) return;
  UiMode from = uiMode;
  setUiMode(from == UI_MENU ? UI_MAIN : UI_MENU);   // 子页超时回菜单
  Serial.printf("[UI] page %d idle %lus -> %s\n", (int)from,
                (unsigned long)(limit / 1000), from == UI_MENU ? "main" : "menu");
}

// Graceful shutdown: close GPX, then ask M5PM1 to cut L2/L3 power.
static void powerOffNow() {
  if (recording) stopRecording();
  if (exportMode) exitExportScreen();
  auto &d = M5.Display;
  useAsciiFont();                // shutdown message is English
  d.setBrightness(brightHigh);        // visible even if auto-dimmed
  d.fillScreen(TFT_BLACK);
  d.setTextSize(2);
  d.setTextColor(TFT_YELLOW, TFT_BLACK);
  d.setCursor(20, 40); d.print("SAVED.");
  d.setCursor(20, 66); d.print(autoOffReason ? "AUTO POWER OFF" : "POWER OFF");
  d.setTextSize(1);
  d.setTextColor(TFT_SILVER, TFT_BLACK);
  d.setCursor(20, 98); d.print("Press POWER key to restart");
  Serial.println("[PWR] shutdown");
  delay(900);
  M5.Power.powerOff();              // M5PM1_Class::powerOff -> reg 0x0C = 0xA1
  // Fallback in case M5Unified's PMIC handle was not initialised.
  M5.In_I2C.writeRegister8(0x6E, 0x0C, 0xA1, 100000);
  delay(1500);
  // 2.0.1: if 5 V is still present the PMIC refused to cut power (typical when
  // powered over USB). esp_restart() here would look like an unexplained
  // REBOOT, which is exactly what a wake-up key press used to trigger. Stay on
  // and say why instead; esp_restart() is only used on battery, where power
  // off genuinely failed.
  int16_t vbus = M5.Power.getVBUSVoltage();
  if (vbus > 3500) {
    autoOffReason = false;
    Serial.printf("[PWR] powerOff refused (VBUS=%dmV) - staying on\n", (int)vbus);
    uiMode = UI_MAIN;
    uiReset();
    drawMainUI();
    setStatus("USB POWER - CANNOT OFF");
    return;
  }
  esp_restart();
}

// ---------------- setup / loop ----------------
static void grove5vOn() {
  if (pmicGrove5vOn()) Serial.println("[PWR] Grove 5V enabled");
  else                 Serial.println("[PWR] M5PM1 not found");
}
static uint32_t probeBaud(int rxPin, int txPin, uint32_t baud, uint32_t ms) {
  GPSSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
  uint32_t dollars = 0, t0 = millis();
  while (millis() - t0 < ms) {
    while (GPSSerial.available()) {
      if (GPSSerial.read() == '$') dollars++;
    }
    delay(2);
  }
  GPSSerial.end();
  return dollars;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[LOG] boot");
  Serial.printf("[LOG] %s %s | %s\n", GPScam_NAME, GPScam_VERSION, GPScam_BASE);
  // 2.0.1: capture WHY the last boot happened. Read it here (the value is
  // latched) but PRINT it after M5.begin(), because the native-USB CDC link
  // is not enumerated yet this early and the first lines are lost.
  esp_reset_reason_t bootReason = esp_reset_reason();

  grove5vOn();
  delay(300);

  // Probe pin orientation x baud. ALL UART begin/end BEFORE M5.begin.
  struct { int rx, tx; uint32_t baud; uint32_t d; } best = {9, 10, 0, 0};
  const uint32_t bauds[2] = {115200, 9600};
  for (int swap = 0; swap <= 1; swap++) {
    int rx = swap ? GPS_TX_PIN : GPS_RX_PIN;
    int tx = swap ? GPS_RX_PIN : GPS_TX_PIN;
    for (int b = 0; b < 2; b++) {
      uint32_t d = probeBaud(rx, tx, bauds[b], 1200);
      Serial.printf("[GPS] probe rx=%d tx=%d @%lu -> %lu '$'\n",
                    rx, tx, (unsigned long)bauds[b], (unsigned long)d);
      if (d > best.d) best = {rx, tx, bauds[b], d};
    }
  }
  GPSSerial.setRxBufferSize(2048);
  GPSSerial.begin(best.baud, SERIAL_8N1, best.rx, best.tx);
  Serial.printf("[GPS] lock rx=%d tx=%d @%lu (%lu '$')\n",
                best.rx, best.tx, (unsigned long)best.baud, (unsigned long)best.d);

  auto cfg = M5.config();
  cfg.internal_mic = false;
  // Any I2S / speaker playback trips TG1WDT_SYS_RST on this Arduino 2.0.7 +
  // qio_opi build, so the internal speaker stays off. Recording feedback is
  // given with distinct backlight pulse patterns instead.
  cfg.internal_spk = false;
  M5.begin(cfg);
  M5.Display.setRotation(0);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextWrap(false);
  Serial.println("[LOG] M5 up");
  {
    const char *rn = "OTHER";
    switch (bootReason) {
      case ESP_RST_POWERON:   rn = "POWERON";   break;
      case ESP_RST_EXT:       rn = "EXT_PIN";   break;
      case ESP_RST_SW:        rn = "SOFTWARE";  break;
      case ESP_RST_PANIC:     rn = "PANIC";     break;
      case ESP_RST_INT_WDT:   rn = "INT_WDT";   break;
      case ESP_RST_TASK_WDT:  rn = "TASK_WDT";  break;
      case ESP_RST_WDT:       rn = "WDT";       break;
      case ESP_RST_DEEPSLEEP: rn = "DEEPSLEEP"; break;
      case ESP_RST_BROWNOUT:  rn = "BROWNOUT";  break;
      case ESP_RST_SDIO:      rn = "SDIO";      break;
      default: break;
    }
    Serial.printf("[LOG] reset_reason=%d (%s)\n", (int)bootReason, rn);
  }

  M5.Power.setExtOutput(true, m5::ext_PA);

  // The two SIDE keys are plain GPIOs (active LOW). Add internal pull-ups so
  // the scan is reliable even if the PMIC pull-up rail is off.
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  int k0 = digitalRead(11), k1 = digitalRead(12);
  Serial.printf("[KEY] idle G11=%d G12=%d\n", k0, k1);
  keys[0].last = (k0 == LOW);
  keys[1].last = (k1 == LOW);
  keys[0].downMs = keys[1].downMs = millis();

  initFS();
  fileLogInstall();                    // 日志同时写入 /debug.log（离线排障用）
  Serial.printf("[LOG] FS %s, free=%u bytes\n",
                fsOk ? "OK" : "FAIL",
                fsOk ? (unsigned)(LittleFS.totalBytes() - LittleFS.usedBytes()) : 0u);
  loadKeySwap();
  loadSettings();
  cpuInit();                           // 电源计划：默认 80 MHz，操作时提速

  // ---- BLE: furble Sony 协议栈（含 A7R4 bit2 补丁） ----
  {
    esp_err_t nv = nvs_flash_init();
    if (nv == ESP_ERR_NVS_NO_FREE_PAGES || nv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      nvs_flash_erase();
      nvs_flash_init();
    }
    Furble::Device::init(ESP_PWR_LVL_P9);
    {
      // 稳定的随机身份地址：避免相机侧残留旧 bond（旧 IRK）导致"无法配对"
      uint8_t mac[6] = {0};
      esp_efuse_mac_get_default(mac);
      uint8_t id[6];
      for (int i = 0; i < 6; i++) id[i] = mac[i] ^ 0x5A;
      id[5] |= 0xC0;  // 静态随机地址标志位
      ble_hs_id_set_rnd(id);
      NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RPA_RANDOM_DEFAULT);
      Serial.printf("[BLE] identity %02X:%02X:%02X:%02X:%02X:%02X\n", id[5], id[4], id[3],
                    id[2], id[1], id[0]);
    }
    xTaskCreate(control_task, "control", 8192, &Furble::Control::getInstance(), 4, NULL);
    if (Furble::CameraList::getSaveCount() > 0) {
      Furble::CameraList::clear();
      Furble::CameraList::load();
      int n = (int)Furble::CameraList::size();
      for (int i = 0; i < n; i++) {
        Furble::CameraList::get(i)->setActive(true);
        Furble::Control::getInstance().addActive(Furble::CameraList::get(i));
      }
      if (n > 0) {
        Furble::Control::getInstance().connectAll(false);
        cpuBoost(30000);               // 连接/配对加密期间提速
        Serial.printf("[CAM] auto-connect %d saved camera(s)\n", n);
      }
    }
    Serial.println("[BLE] furble Sony stack up");
  }
  // 固定竖屏；并把 GNSS 星座在每次开机时重发一遍（模块可能不保留）。
  M5.Display.setRotation(0);
  M5.Display.setTextWrap(false);
  sendGnssMode(gnssMode);
  Serial.printf("[DSP] portrait %dx%d | [IMU] enabled=%d type=%d (unused)\n",
                (int)M5.Display.width(), (int)M5.Display.height(),
                M5.Imu.isEnabled() ? 1 : 0, (int)M5.Imu.getType());
  updateBatteryLevel();
  M5.Display.setBrightness(brightHigh);
  lastActivityMs = millis();
  selectEnterMs  = millis();
  Serial.printf("[PWR] VBUS=%d mV | keySwap=%d\n",
                (int)M5.Power.getVBUSVoltage(), keySwap);
  uiReset();
  setStatus("WAIT GPS FIX");
}

void loop() {
  M5.update();
  readGPS();
  scanKeys();
  handleSerialCmd();
  manageIdle();
  manageSelectTimeout();
  servicePrompt();
  cpuService();                        // 提速窗口结束则回落到 80 MHz
  checkBattery();

  // BLE 状态切换（连接/配对/写入）临时提速，完成后自然回落
  {
    static auto lastCtl = Furble::Control::STATE_IDLE;
    auto ctl = Furble::Control::getInstance().getState();
    if (ctl != lastCtl) {
      if (ctl == Furble::Control::STATE_CONNECTING ||
          ctl == Furble::Control::STATE_CONNECT ||
          ctl == Furble::Control::STATE_ACTIVE) {
        cpuBoost(20000);
      }
      lastCtl = ctl;
    }
  }

  if (uiMode == UI_CAM && FurbCam::tick(millis())) setUiMode(UI_MENU);
  static uint32_t lastCamGpsMs = 0;
  if (millis() - lastCamGpsMs >= 1000) {
    lastCamGpsMs = millis();
    FurbCam::relayGPS(gps.valid && gps.timeValid, gps.lat, gps.lon, gps.alt, gps.sats,
                      gps.year, gps.month, gps.day, gps.hour, gps.minute, gps.second);
  }

  if (exportMode) {
    if (millis() - exportIdleMs > 300000) { exitExportScreen(); setStatus("EXPORT TIMEOUT"); }
  } else {
    if (recording && gps.valid && gps.timeValid
        && millis() - lastPointMs >= (uint32_t)recIntervalS * 1000u) {
      writeTrackpoint();
      lastPointMs = millis();
    }
    // Main screen: 1 Hz partial redraw. Menu pages are static and repainted
    // only on a key press -> no flicker.
    static uint32_t lastDraw = 0;
    if (uiMode == UI_MAIN && millis() - lastDraw >= 1000) {
      lastDraw = millis();
      drawMainUI();
    }
  }
  static uint32_t lastBeat = 0;
  if (!dumping && millis() - lastBeat >= 5000) {
    lastBeat = millis();
    Serial.printf("[HB] t=%lu rec=%d exp=%d fix=%d sats=%d dim=%d mode=%d bat=%d cpu=%u\n",
                  (unsigned long)millis(), recording, exportMode, gps.valid,
                  gps.sats, dimmed, (int)uiMode, batteryLevel,
                  (unsigned)(esp_clk_cpu_freq() / 1000000));
  }
  delay(2);
}
