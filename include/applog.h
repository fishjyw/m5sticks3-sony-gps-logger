#ifndef APPLOG_H
#define APPLOG_H

// 持久化系统日志：写入 /syslog.txt（32 KB 一段，保留 /syslog.1.txt），
// 与 GPX 数据分开、总量固定不超过 64 KB，掉电不丢。串口 K 可导出。
void appLog(const char *fmt, ...);

#endif
