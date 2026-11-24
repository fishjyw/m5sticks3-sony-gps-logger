@echo off
setlocal
set BIN=%~dp0sony-gps-logger-4.9.3.bin
set PORT=%~1
if "%PORT%"=="" set PORT=COM3
echo Port: %PORT%
python -m esptool version >nul 2>nul
if errorlevel 1 (
  echo esptool not found. Install with: python -m pip install esptool
  pause
  exit /b 1
)
python -m esptool --chip esp32s3 --port %PORT% write-flash 0x0 "%BIN%"
if errorlevel 1 (
  echo Flash failed.
  pause
  exit /b 1
)
echo.
echo Done. Saved camera pairings are cleared - re-add cameras on the device.
pause
