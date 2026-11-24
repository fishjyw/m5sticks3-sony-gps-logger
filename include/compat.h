/**
 * Arduino-API 兼容层：让用户 v3.2.0 的 main.cpp 在 ESP-IDF 5.4.2 上原样运行。
 * 覆盖: millis/delay/pinMode/digitalRead/dtostrf/String(子集)/Serial(USB-Serial-JTAG)/
 *       File/FS(LittleFS via VFS)/HardwareSerial(UART driver)/PMIC Grove 5V
 */
#ifndef COMPAT_H
#define COMPAT_H

#include <dirent.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include <string>

#define LOW 0
#define HIGH 1
#define INPUT 0x01
#define OUTPUT 0x02
#define INPUT_PULLUP 0x05
#define SERIAL_8N1 0x800001c
#define FILE_READ "r"
#define FILE_WRITE "w"
#define FILE_APPEND "a"

unsigned long millis(void);
void delay(uint32_t ms);
void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
char *dtostrf(double val, signed char width, unsigned char prec, char *sout);

template <class T>
inline T constrain(T v, T lo, T hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

/** Arduino String 子集（该工程仅用到 startsWith/endsWith/+/c_str）。 */
class String {
 public:
  String() {}
  String(const char *s) : m_s(s ? s : "") {}
  String &operator=(const char *s) {
    m_s = s ? s : "";
    return *this;
  }
  bool startsWith(const char *p) const { return m_s.rfind(p, 0) == 0; }
  bool endsWith(const char *p) const {
    size_t lp = strlen(p), ls = m_s.size();
    return ls >= lp && m_s.compare(ls - lp, lp, p) == 0;
  }
  const char *c_str() const { return m_s.c_str(); }
  String operator+(const char *s) const {
    String r(*this);
    r.m_s += s;
    return r;
  }
  friend String operator+(const char *a, const String &b) {
    String r(a);
    r.m_s += b.m_s;
    return r;
  }

 private:
  std::string m_s;
};

/** USB-Serial-JTAG 串口（替代 Arduino Serial）。 */
class SerialClass {
 public:
  void begin(unsigned long baud);
  int available(void);
  int read(void);
  size_t write(const uint8_t *buf, size_t n);
  size_t write(uint8_t c) { return write(&c, 1); }
  size_t write(const char *s) { return write((const uint8_t *)s, strlen(s)); }
  size_t print(const char *s) { return write(s); }
  size_t println(const char *s = "");
  int printf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
  void flush(void) {}
};
extern SerialClass Serial;

/** Arduino File（基于 POSIX + VFS）。 */
class File {
 public:
  File() {}
  File(File &&o) noexcept { steal(o); }
  File &operator=(File &&o) noexcept {
    if (this != &o) {
      close();
      steal(o);
    }
    return *this;
  }
  File(const File &) = delete;
  File &operator=(const File &) = delete;
  ~File() { close(); }

  explicit operator bool() const { return m_fp != nullptr || m_dir != nullptr; }
  bool isDirectory() const { return m_dir != nullptr; }

  const char *name(void) const { return m_name; }
  File openNextFile(void);

  int read(void) { return m_fp ? fgetc(m_fp) : -1; }
  size_t read(uint8_t *buf, size_t n) { return m_fp ? fread(buf, 1, n, m_fp) : 0; }
  size_t readBytesUntil(char terminator, char *buf, size_t len) {
    size_t i = 0;
    if (m_fp == nullptr) return 0;
    while (i < len) {
      int c = fgetc(m_fp);
      if (c < 0 || c == terminator) break;
      buf[i++] = (char)c;
    }
    return i;
  }
  size_t write(const uint8_t *buf, size_t n) { return m_fp ? fwrite(buf, 1, n, m_fp) : 0; }
  size_t write(uint8_t c) { return write(&c, 1); }
  size_t print(const char *s) { return write((const uint8_t *)s, strlen(s)); }
  size_t println(const char *s = "") {
    size_t n = print(s);
    n += print("\n");
    return n;
  }
  void flush(void) {
    if (m_fp) {
      fflush(m_fp);
      fsync(fileno(m_fp));
    }
  }
  size_t size(void) {
    if (m_fp == nullptr) return 0;
    struct stat st;
    if (fstat(fileno(m_fp), &st) != 0) return 0;
    return (size_t)st.st_size;
  }
  void close(void);

  friend class FSClass;

 private:
  FILE *m_fp = nullptr;
  DIR *m_dir = nullptr;
  char m_name[64] = {0};

  void steal(File &o) {
    m_fp = o.m_fp;
    m_dir = o.m_dir;
    memcpy(m_name, o.m_name, sizeof m_name);
    o.m_fp = nullptr;
    o.m_dir = nullptr;
  }
  void openDir(const char *path);
  bool nextEntry(void);
};

/** Arduino LittleFS 子集（挂载点 /littlefs，对上层透明）。 */
class FSClass {
 public:
  bool begin(bool formatOnFail = false);
  File open(const char *path);
  File open(const char *path, const char *mode);
  bool exists(const char *path);
  bool remove(const char *path);
  uint64_t totalBytes(void);
  uint64_t usedBytes(void);
};
extern FSClass LittleFS;

/** Arduino HardwareSerial（基于 IDF UART driver，支持 begin/end 反复探测）。 */
class HardwareSerial {
 public:
  explicit HardwareSerial(int uartNum) : m_uart(uartNum) {}
  void begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin);
  void end(void);
  void setRxBufferSize(size_t n) { m_rxSize = n; }
  int available(void);
  int read(void);
  size_t print(const char *s);

 private:
  int m_uart;
  bool m_installed = false;
  size_t m_rxSize = 2048;
  int refill(void);
};

/** 通过 M5PM1 (I2C1, 0x6E, reg0x06 bit3) 打开 Grove/PORT.A 5V —— M5.begin 之前调用可用。 */
bool pmicGrove5vOn(void);

/** 将 esp_log 输出同时写入 /debug.log（LittleFS），供无电脑时记录日志。 */
void fileLogInstall(void);
void fileLogSuspend(bool suspend);

#endif
