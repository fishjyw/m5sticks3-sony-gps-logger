# M5StickS3 Sony Camera GPS Logger

M5StickS3 + M5 Unit GPS 1.1 二合一固件：**GPX 轨迹记录** + **索尼相机 BLE 实时定位**（照片写入 GPS EXIF）。

基于 [furble](https://github.com/gkoh/furble) 的索尼 BLE 协议栈（MIT），移植到 ESP-IDF 5.4.2，
并集成了独立的 GPS 轨迹记录器。已在 **Sony A7R4 / A7M5** 上实测：相机可通过 BLE 获取定位并写入照片。

[English](#english) | 中文

---

## 功能

- **TRACK**：GPX 轨迹记录到内部 LittleFS（约 4.9 MB），USB 串口 `D` 指令一键导出
- **LOCATION LINK**：BLE 连接索尼相机，每秒推送 GPS + 时间，照片自动写入位置
  - A7R4（2019 代）与 A7M5（新代）均实测可用
  - 支持多台相机同时连接
  - 开机自动连接已保存的相机
- **相机管理**：扫描添加 / 设备管理（连接、忘记）/ 已配对列表持久化（NVS）
- **UI**：135×240 竖屏中文界面；菜单全中文；所有 EXIT 返回上一级
- **电源计划**：空闲/记录 80 MHz，按键与 BLE 操作临时 240 MHz，20 秒回落
- **电量**：电压曲线换算 + 平滑 + 充电补偿；显示百分比、CHG 状态、剩余可用时间估算

## 硬件

| 部件 | 说明 |
|---|---|
| 主控 | M5StickS3（ESP32-S3-PICO-1-N8R8，8 MB Flash / 8 MB PSRAM，250 mAh） |
| GPS | M5 Unit GPS 1.1（AT6668），Grove 口，UART 115200（rx=10 / tx=9） |
| 连接 | USB-C 供电与导出；BLE 直连相机 |

## 快速开始

### 编译

需要 [PlatformIO](https://platformio.org/)（首次构建会自动下载 ESP-IDF 5.4.2 工具链）：

```bash
pio run -e sticks3
```

### 刷机

**必须分区写入**（合并镜像会覆盖 NVS，导致配对信息丢失）：

```bash
esptool --chip esp32s3 --port <PORT> write-flash \
  0x0   bootloader.bin \
  0x8000 partitions.bin \
  0x10000 firmware.bin
```

也可以 `pio run -e sticks3 -t upload`（PlatformIO 同样按分区写入）。

### 首次配对

1. 相机菜单 → 添加设备 → 扫描，选中相机按 FRONT 连接
2. 相机端确认配对（部分机型需在相机蓝牙设置里允许）
3. 配对成功后自动保存，下次开机自动连接

## 按键

| 操作 | 主界面 | 菜单内 |
|---|---|---|
| FRONT 短按 | 开始 / 停止记录 | 确定 |
| FRONT 长按 | 关机（先保存 GPX） | 相机页：清除全部配对 |
| MID 短按 | 打开菜单 | 下一项 |
| MID 长按 | 导出页捷径 | 返回上一级 |

## GPX 导出

PC 通过 USB 串口发送 `D`，设备输出：

```
#GPX_BEGIN|/20261124_135204.GPX
... GPX 原文 ...
#GPX_END
#GPX_DONE|3
```

配套的浏览器 / Python / shell 客户端可自行编写，协议就是以上行格式。

## 索尼 BLE 协议要点（A7R4 踩坑记录）

1. **扫描不到 A7R4**：广播 mode22 的遥控位在旧代是 bit2（新代 bit1），
   匹配条件放宽为 `bit1 | bit2 | bit5`
2. **无法配对**：相机侧残留旧 bond 会导致失败；设备使用固定随机身份地址
   （EFUSE MAC 异或 + 静态随机位），并保持 NVS 不被覆盖
3. **位置写不进**：A7R4 不支持 dd30/dd31 使能流程，需**直接写 dd11**
   95 字节坐标包，且连接前 **MTU 提到 158**
4. 协议参考：
   - [anoulis/sony_camera_bluetooth_external_gps](https://github.com/anoulis/sony_camera_bluetooth_external_gps)
   - [Saschl/alpha-gps](https://github.com/Saschl/alpha-gps)
   - [gkoh/furble](https://github.com/gkoh/furble)

## 电源计划

ESP32-S3 的 CPU 有 80/160/240 MHz 三档，APB 在三档下均固定 80 MHz
（见 IDF `rtc_clk.c`），因此切换频率不影响 GPS 串口、I2C、SPI 与 Flash 时序。

- 空闲 / 记录：**80 MHz**
- 按键、菜单操作：**240 MHz**（20 秒无操作回落）
- BLE 连接 / 配对：**240 MHz**（20 秒）
- 开机自动连接：**240 MHz**（30 秒）

主界面右上角实时显示当前频率（80 绿 / 160 黄 / 240 红）。串口发送 `J` 可查看 PM 锁与寄存器频率。

## 项目结构

```
src/main.cpp          记录器 + 主界面 + 菜单 + 导出（兼容层之上）
src/CameraPage.cpp    相机管理页（扫描 / 设备管理 / 连接）
src/compat.cpp        Arduino API 兼容层（USB-Serial-JTAG、LittleFS、UART、PMIC）
lib/furble/           furble 索尼协议栈（含 A7R4 补丁）
include/              对外头文件
patches/              NimBLE ble_gap 补丁（扫描匹配放宽）
sdkconfig.sticks3     完整 IDF 配置快照
platformio.ini        PlatformIO 工程配置
```

## 已知限制

- 只做了索尼协议的实际测试；富士/佳能/尼康/理光代码虽包含但未验证
- 剩余可用时间按 250 mAh / 95 mA 折算，属量级估算，待实测校准
- 记录间隔固定按设置值写点，Flash 写入策略未做批量优化

## 致谢

- [furble](https://github.com/gkoh/furble) — 索尼 BLE 协议栈与 M5StickS3 驱动适配（MIT）
- [M5Unified / M5GFX](https://github.com/m5stack/M5Unified) — 显示与电源管理
- [anoulis/sony_camera_bluetooth_external_gps](https://github.com/anoulis/sony_camera_bluetooth_external_gps) — dd11 坐标包格式

## License

MIT，详见 [LICENSE](LICENSE) 与 [THIRD_PARTY.md](THIRD_PARTY.md)。

---

<a id="english"></a>

## English

A combined firmware for **M5StickS3 + M5 Unit GPS 1.1**: GPX track logging plus
Bluetooth LE geotagging for Sony cameras (GPS written into photo EXIF).

- Sony A7R4 and A7M5 verified on hardware
- A7R4 support requires: relaxed advertising match (mode22 bit2/bit5),
  a stable random BLE identity, direct dd11 location writes, and MTU 158
- Multi-camera support, auto-connect on boot, on-device camera management
- 135×240 portrait Chinese UI; CPU runs at 80 MHz idle and boosts to 240 MHz
  for key/BLE activity (falls back after 20 s)
- Build with PlatformIO (`pio run -e sticks3`); flash in three regions
  (0x0 / 0x8000 / 0x10000) to preserve NVS bonding data

MIT licensed. Built on top of [furble](https://github.com/gkoh/furble).
