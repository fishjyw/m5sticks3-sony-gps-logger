#!/usr/bin/env bash
# Sony GPS Logger 4.9.4 单文件刷机（macOS / Linux）
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$DIR/sony-gps-logger-4.9.4.bin"
PORT="${1:-}"
if [ -z "$PORT" ]; then
  PORT="$(ls /dev/cu.usbmodem* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n1 || true)"
fi
if [ -z "$PORT" ]; then
  echo "未找到串口。请插好 USB，或手动指定：$0 /dev/cu.usbmodemXXXX"
  exit 1
fi
echo "使用串口: $PORT"
if command -v esptool.py >/dev/null 2>&1; then
  ESP=(esptool.py)
elif python3 -m esptool version >/dev/null 2>&1; then
  ESP=(python3 -m esptool)
else
  echo "缺少 esptool，请先安装： python3 -m pip install esptool"
  exit 1
fi
"${ESP[@]}" --chip esp32s3 --port "$PORT" write-flash 0x0 "$BIN"
echo
echo "刷机完成。单文件刷机会清空已保存的相机配对，请重新在相机菜单里添加设备。"
