# Sony Camera GPS Logger v4.0.0

M5StickS3 GPS 记录器 + 索尼相机实时定位（A7R4/A7M5 实测通过）

版本: v4.9.1
日期: 2026-11-24
底座: ESP-IDF 5.4.2 (espressif32@6.12.0)

## 功能

- **TRACK**: GPX 轨迹记录到 LittleFS (~4.9MB)，USB 'D' 导出（原 PC 工具兼容）
- **LOCATION LINK**: BLE 连接相机，GPS+时间每秒推送，写入照片 EXIF
  - 索尼 A7R4（2019 代）与 A7M5（新代）均实测写入成功
  - 开机自动连接已配对相机；离开页面连接保持
  - 页内: FRONT=连接 / MID=下一台(已连接=断开) / MID长按=退出 / **FRONT长按=清除配对**
- 其他品牌（富士/佳能/尼康/理光）协议栈已包含

## 关键技术点（踩坑记录）

1. **A7R4 扫描不可见**: 广播 mode22 的遥控位在旧代是 bit2（新代 bit1），已补丁兼容
2. **A7R4 配对失败**: 相机侧残留旧 bond 导致，设备侧用固定随机身份地址 + 清除配对解决
3. **A7R4 位置写入**: 该代机型**不支持** dd30/dd31 使能流程（读空、写拒），
   必须**直接写 dd11** 95 字节坐标包，且连接前 MTU 需提到 158。
   协议参考: anoulis/sony_camera_bluetooth_external_gps（AlphaGPS 源头）
4. 刷机必须**分区写入**（0x0 / 0x8000 / 0x10000），合并镜像会覆盖 NVS 清掉配对

## 刷机

    esptool --chip esp32s3 --port <PORT> write-flash \
      0x0 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin

## 编译

    cd gpscam && pio run -e sticks3

## 源码

- src/main.cpp          记录器（逻辑未改，经 compat 层适配 IDF）
- src/compat.cpp/.h     Arduino API 兼容层（Serial/USB-JTAG、File/LittleFS、UART、PMIC）
- src/CameraPage.cpp    LOCATION LINK 页
- lib/furble/*          furble 协议栈（Sony.cpp 含 A7R4 bit2 + 直接 dd11 + MTU158 修改）

## 版本记录

- v4.9.1: AUTO 只在开机时已开启的情况下自动启动；本次开机内在菜单切换只
  保存设置、下次开机生效。修复 AUTO 角标被时钟行整行清屏擦掉的问题
- v4.9.0: 新增「自动」模式（菜单手动开启）：每次开机拿到 GPS 时间后自动
  开始记录（手动停止后本次开机不再重启）；自动模式空闲关机固定 30 分钟；
  主页时钟右上角显示 AUTO（待启动黄 / 已启动绿），开关写入 settings.cfg
- v4.8.0: 修复进入「添加设备 / 设备管理」卡死重启：NimBLE client 连接失败后
  自删除，Camera 仍保留悬空指针，断开时踩空；改为统一 closeClient() 管理。
  进入这些页面/清配对前先等 Control 断开完成再销毁列表。新增固定系统
  日志区 /syslog.txt + /syslog.1.txt（32 KB 滚动，共 64 KB，串口 K 导出）
- v4.7.3: 修复录制约 4 分钟自动断电：M5PM1 看门狗 255s 未喂狗会切断整机，
  现每 10 秒喂一次；修复 countFiles 目录扫描泄漏第一个 FILE 句柄（约 878 B/次）
- v4.7.2: 增加持久化事件日志 events.log（开机原因/录制起停/BLE 状态/堆内存），
  串口 K 导出；控制任务栈 8192->12288；心跳增加 heap
- v4.7.1: 开机恢复电源键双击关机（PMIC 0x4A bit0）。该设置会被以前的
  固件写入并保存在 PMIC 中，ESP32 刷机不会清除
- v4.7.0: 充电时电量时间行显示预计充满时间（FULL xhxxm / FULL DONE），
  放电时仍显示剩余可用时间
- v4.6.3: 充电文字改为 CHG（避免 100% 时与 BAT 文本挤在一起）
- v4.6.2: 频率角标 240->80 残留数字修复；充电闪电改为与 BAT 同字号的
  黄色 "CHARGING" 文字（充满变绿）
- v4.6.1: 主界面电量改为小字，右侧显示 CPU 频率（80 绿/160 黄/240 红），
  电池下方新增剩余可用时间估算（250 mAh / 约 95 mA）
- v4.6.0: CPU 电源计划：空闲/记录固定 80 MHz，按键/菜单/BLE 连接时
  临时 240 MHz（20 s 无操作回落）。S3 三档 80/160/240 下 APB 恒为 80 MHz，
  GPS 串口/I2C/SPI/Flash 不受影响
- v4.5.0: 删除全部横屏/旋转死代码（约 300 行）；进入设备管理不再忙等 800 ms
  （改为非阻塞状态机）；版本号合并为单一来源；清零编译警告
- v4.4.6: 导出页纵向流式排版；长文本按像素宽度自动换行（UTF-8 安全）；
  文件列表去掉 .GPX 后缀、相机页长设备名截断，文字不再超出 135 px 屏宽
- v4.4.5: 导出页排版；子页面超时/退出统一返回上一级菜单
