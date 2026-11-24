#include "compat.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <esp_littlefs.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/usb_serial_jtag.h>

#define LFS_MOUNT "/littlefs"

// ---------------------------------------------------------------- misc

unsigned long millis(void) {
  return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void delay(uint32_t ms) {
  if (ms == 0) {
    vTaskDelay(1);
    return;
  }
  vTaskDelay(pdMS_TO_TICKS(ms));
}

void pinMode(uint8_t pin, uint8_t mode) {
  gpio_config_t io = {};
  io.pin_bit_mask = 1ULL << pin;
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = (mode == INPUT_PULLUP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io);
}

int digitalRead(uint8_t pin) {
  return gpio_get_level((gpio_num_t)pin) ? HIGH : LOW;
}

char *dtostrf(double val, signed char width, unsigned char prec, char *sout) {
  char fmt[16];
  snprintf(fmt, sizeof fmt, "%%%d.%df", (int)width, (int)prec);
  sprintf(sout, fmt, val);
  return sout;
}

// ---------------------------------------------------------------- Serial (USB Serial JTAG)

static uint8_t s_rx[512];
static size_t s_rxHead = 0;
static size_t s_rxTail = 0;

static bool s_usbReady = false;

static void usbEnsure(void) {
  if (s_usbReady) return;
  usb_serial_jtag_driver_config_t cfg = {};
  cfg.tx_buffer_size = 4096;
  cfg.rx_buffer_size = 4096;
  esp_err_t err = usb_serial_jtag_driver_install(&cfg);
  s_usbReady = (err == ESP_OK) || (err == ESP_ERR_INVALID_STATE);
}

void SerialClass::begin(unsigned long baud) {
  (void)baud;
  usbEnsure();
}

static size_t rxCount(void) {
  return (s_rxHead + sizeof s_rx - s_rxTail) % sizeof s_rx;
}

static void rxRefill(void) {
  if (!s_usbReady) return;
  uint8_t tmp[128];
  while (rxCount() < sizeof s_rx - sizeof tmp) {
    int n = usb_serial_jtag_read_bytes(tmp, sizeof tmp, 0);
    if (n <= 0) break;
    for (int i = 0; i < n; i++) {
      s_rx[s_rxHead] = tmp[i];
      s_rxHead = (s_rxHead + 1) % sizeof s_rx;
    }
  }
}

int SerialClass::available(void) {
  rxRefill();
  return (int)rxCount();
}

int SerialClass::read(void) {
  rxRefill();
  if (rxCount() == 0) return -1;
  uint8_t c = s_rx[s_rxTail];
  s_rxTail = (s_rxTail + 1) % sizeof s_rx;
  return c;
}

size_t SerialClass::write(const uint8_t *buf, size_t n) {
  if (!s_usbReady) return 0;
  size_t done = 0;
  while (done < n) {
    int w = usb_serial_jtag_write_bytes(buf + done, n - done, pdMS_TO_TICKS(200));
    if (w <= 0) break;
    done += w;
  }
  return done;
}

size_t SerialClass::println(const char *s) {
  size_t n = print(s);
  n += print("\n");
  return n;
}

int SerialClass::printf(const char *fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n > 0) write((const uint8_t *)buf, (size_t)((n < (int)sizeof buf) ? n : (int)sizeof buf - 1));
  return n;
}

SerialClass Serial;

// ---------------------------------------------------------------- File / FS

void File::close(void) {
  if (m_fp != nullptr) {
    fclose(m_fp);
    m_fp = nullptr;
  }
  if (m_dir != nullptr) {
    closedir(m_dir);
    m_dir = nullptr;
  }
  m_name[0] = '\0';
}

static std::string lfsPath(const char *path) {
  if (path == nullptr) return LFS_MOUNT;
  if (path[0] == '/') return std::string(LFS_MOUNT) + path;
  return std::string(LFS_MOUNT) + "/" + path;
}

void File::openDir(const char *path) {
  close();
  m_dir = opendir(lfsPath(path).c_str());
  if (m_dir != nullptr && !nextEntry()) {
    closedir(m_dir);
    m_dir = nullptr;
  }
}

bool File::nextEntry(void) {
  if (m_dir == nullptr) return false;
  struct dirent *e;
  while ((e = readdir(m_dir)) != nullptr) {
    if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
    strlcpy(m_name, e->d_name, sizeof m_name);
    // open regular files for reading; subdirectories are skipped (none used)
    std::string p = lfsPath(m_name);
    struct stat st;
    if (stat(p.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) continue;
    m_fp = fopen(p.c_str(), "r");
    return true;
  }
  return false;
}

File File::openNextFile(void) {
  File f;
  if (m_dir == nullptr) return f;
  // openDir() already opened the FIRST entry into m_fp; return that one
  // instead of calling nextEntry() again -- otherwise the first FILE handle
  // is silently overwritten and leaked (~880 B per directory scan).
  if (m_fp == nullptr) {
    if (!nextEntry()) return f;
  }
  f.m_fp = m_fp;
  m_fp = nullptr;  // ownership moved to the returned file
  memcpy(f.m_name, m_name, sizeof m_name);
  return f;
}

bool FSClass::begin(bool formatOnFail) {
  esp_vfs_littlefs_conf_t conf = {};
  conf.base_path = LFS_MOUNT;
  conf.partition_label = "spiffs";
  conf.format_if_mount_failed = formatOnFail;
  conf.dont_mount = false;
  esp_err_t err = esp_vfs_littlefs_register(&conf);
  if (err == ESP_ERR_INVALID_STATE) return true;  // already mounted
  return err == ESP_OK;
}

File FSClass::open(const char *path) {
  File f;
  if (path == nullptr) return f;
  if (!strcmp(path, "/") || !strcmp(path, "")) {
    f.openDir("/");
    return f;
  }
  f.m_fp = fopen(lfsPath(path).c_str(), "r");
  if (f.m_fp != nullptr) strlcpy(f.m_name, path, sizeof f.m_name);
  return f;
}

File FSClass::open(const char *path, const char *mode) {
  File f;
  if (path == nullptr || mode == nullptr) return f;
  f.m_fp = fopen(lfsPath(path).c_str(), mode);
  if (f.m_fp != nullptr) strlcpy(f.m_name, path, sizeof f.m_name);
  return f;
}

bool FSClass::exists(const char *path) {
  struct stat st;
  return stat(lfsPath(path).c_str(), &st) == 0;
}

bool FSClass::remove(const char *path) {
  return unlink(lfsPath(path).c_str()) == 0;
}

uint64_t FSClass::totalBytes(void) {
  size_t total = 0, used = 0;
  if (esp_littlefs_info("spiffs", &total, &used) != ESP_OK) return 0;
  return total;
}

uint64_t FSClass::usedBytes(void) {
  size_t total = 0, used = 0;
  if (esp_littlefs_info("spiffs", &total, &used) != ESP_OK) return 0;
  return used;
}

FSClass LittleFS;

// ---------------------------------------------------------------- HardwareSerial (UART)

static uint8_t s_uartRx[2048];
static size_t s_uartHead = 0;
static size_t s_uartTail = 0;

static size_t urxCount(void) {
  return (s_uartHead + sizeof s_uartRx - s_uartTail) % sizeof s_uartRx;
}

void HardwareSerial::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin) {
  (void)config;
  end();
  const uart_config_t cfg = {
      .baud_rate = (int)baud,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_DEFAULT,
      .flags = {},
  };
  uart_driver_install((uart_port_t)m_uart, (int)m_rxSize, 0, 0, nullptr, 0);
  uart_param_config((uart_port_t)m_uart, &cfg);
  uart_set_pin((uart_port_t)m_uart, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_flush((uart_port_t)m_uart);
  s_uartHead = s_uartTail = 0;
  m_installed = true;
}

void HardwareSerial::end(void) {
  if (m_installed) {
    uart_driver_delete((uart_port_t)m_uart);
    m_installed = false;
  }
  s_uartHead = s_uartTail = 0;
}

int HardwareSerial::refill(void) {
  if (!m_installed) return 0;
  uint8_t tmp[128];
  while (urxCount() < sizeof s_uartRx - sizeof tmp) {
    int n = uart_read_bytes((uart_port_t)m_uart, tmp, sizeof tmp, 0);
    if (n <= 0) break;
    for (int i = 0; i < n; i++) {
      s_uartRx[s_uartHead] = tmp[i];
      s_uartHead = (s_uartHead + 1) % sizeof s_uartRx;
    }
  }
  return (int)urxCount();
}

int HardwareSerial::available(void) {
  return refill();
}

int HardwareSerial::read(void) {
  if (refill() == 0) return -1;
  uint8_t c = s_uartRx[s_uartTail];
  s_uartTail = (s_uartTail + 1) % sizeof s_uartRx;
  return c;
}

size_t HardwareSerial::print(const char *s) {
  if (!m_installed) return 0;
  return (size_t)uart_write_bytes((uart_port_t)m_uart, s, strlen(s));
}

// ---------------------------------------------------------------- log to file

static FILE *s_logF = nullptr;
static bool s_logSuspended = false;

static int fileLogVprintf(const char *fmt, va_list ap) {
  char buf[256];
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  if (n < 0) return 0;
  Serial.write((const uint8_t *)buf, (size_t)((n < (int)sizeof buf) ? n : (int)sizeof buf - 1));
  if (s_logF != nullptr && !s_logSuspended) {
    fwrite(buf, 1, (size_t)((n < (int)sizeof buf) ? n : (int)sizeof buf - 1), s_logF);
    fflush(s_logF);
  }
  return n;
}

// 默认关闭：写入 flash 有磨损与延迟代价，排障时再打开
#define FILELOG_ENABLED 0

void fileLogInstall(void) {
#if FILELOG_ENABLED
  if (s_logF == nullptr) s_logF = fopen("/littlefs/debug.log", "w");
  esp_log_set_vprintf(fileLogVprintf);
#else
  (void)fileLogVprintf;
  (void)s_logF;
#endif
}

void fileLogSuspend(bool suspend) {
  s_logSuspended = suspend;
}

// ---------------------------------------------------------------- Arduino entry point

extern void setup(void);
extern void loop(void);

extern "C" void app_main(void) {
  setup();
  while (true) {
    loop();
  }
}

// ---------------------------------------------------------------- PMIC Grove 5V

bool pmicGrove5vOn(void) {
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_NUM_1;
  bus_cfg.sda_io_num = (gpio_num_t)47;
  bus_cfg.scl_io_num = (gpio_num_t)48;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;

  i2c_master_bus_handle_t bus = nullptr;
  if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) return false;

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = 0x6E;
  dev_cfg.scl_speed_hz = 100000;

  i2c_master_dev_handle_t dev = nullptr;
  if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) != ESP_OK) {
    i2c_del_master_bus(bus);
    return false;
  }

  bool ok = false;
  uint8_t reg = 0x06, val = 0;
  if (i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 100) == ESP_OK) {
    if (val & 0x08) {
      ok = true;
    } else {
      uint8_t buf[2] = {0x06, (uint8_t)(val | 0x08)};
      ok = (i2c_master_transmit(dev, buf, 2, 100) == ESP_OK);
    }
  }

  i2c_master_bus_rm_device(dev);
  i2c_del_master_bus(bus);
  return ok;
}
