#ifndef CAMERA_PAGE_H
#define CAMERA_PAGE_H

#include <stdint.h>

namespace FurbCam {

/** 进入 CAMERA 菜单。 */
void enter(void);

/** 离开 CAMERA（停止扫描，连接保持）。 */
void leave(void);

/** 重绘当前页。 */
void draw(void);

/** 主循环定期调用。返回 true 表示请求退回主界面。 */
bool tick(uint32_t now);

/** FRONT 点击：当前页的动作/选择。 */
void front(void);

/** MID 点击：光标下移。 */
void mid(void);

/** MID 长按：返回上一级（根菜单则退出）。 */
void back(void);

/** 是否已连接相机。 */
bool isActive(void);

/** 请求清除所有配对（二次确认，FRONT=确认 / MID=取消）。 */
void requestClear(void);

/** 立即清除所有已保存配对（NVS 记录 + BLE bond）。 */
void clearPairings(void);

/** 主界面显示所需的连接信息。 */
struct LinkInfo {
  bool active;
  int count;
  char name[24];
  bool gpsOk;
  int sats;
  uint32_t sent;
};
void getLinkInfo(LinkInfo &out);

/** 向已连接相机转发 GPS/时间。 */
void relayGPS(bool fix, double lat, double lon, float alt, int sats, int year, int month, int day,
              int hour, int minute, int second);

}  // namespace FurbCam

#endif
