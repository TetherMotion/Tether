/**
 * @file esp_log.h
 * @brief Mock ESP-IDF logging header for host tests
 */
#ifndef ESP_LOG_H
#define ESP_LOG_H

#include <stdio.h>
#ifdef __cplusplus
#include <string>
#include <vector>
#include <algorithm>
#include <stdarg.h>
extern "C" {
#else
#include <stdarg.h>
#endif

typedef enum {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE
} esp_log_level_t;

// Colored logging with TTY detection. Colors are enabled when stdout is a TTY and NO_COLOR is not set.
#include <stdlib.h>
#include <unistd.h>

static inline int tether_log_use_color(void) {
    static int inited = 0;
    static int enabled = 0;
    if (!inited) {
        inited = 1;
        // Force color if TETHER_FORCE_COLOR is set, otherwise require a TTY and no NO_COLOR.
        enabled = ((isatty(fileno(stdout)) || (getenv("TETHER_FORCE_COLOR") != NULL)) && (getenv("NO_COLOR") == NULL));
    }
    return enabled;
}

#define TETHER_COLOR_ERR   "\x1b[1;31m"
#define TETHER_COLOR_WARN  "\x1b[1;33m"
#define TETHER_COLOR_INFO  "\x1b[1;32m"
#define TETHER_COLOR_DEBUG "\x1b[1;36m"
#define TETHER_COLOR_VERB  "\x1b[1;35m"
#define TETHER_COLOR_RESET "\x1b[0m"

static inline void tether_log_printf(int level, const char* tag, const char* format, va_list ap) {
    char buf[1024];
    int needed = vsnprintf(buf, sizeof(buf), format, ap);
    if (needed < 0) return;

    // Split lines and drop a trailing empty segment caused by trailing '\n'
    size_t start = 0;
    std::string s(buf, (size_t)std::min(needed, (int)sizeof(buf)-1));
    std::vector<std::string> lines;
    while (start <= s.size()) {
        size_t pos = s.find('\n', start);
        if (pos == std::string::npos) pos = s.size();
        lines.emplace_back(s.substr(start, pos - start));
        if (pos == s.size()) break;
        start = pos + 1;
    }
    if (!lines.empty() && !s.empty() && s.back() == '\n' && lines.back().empty()) lines.pop_back();

    const char* color = "";
    const char* levelChar = "?";
    switch (level) {
        case ESP_LOG_ERROR:   color = TETHER_COLOR_ERR; levelChar = "E"; break;
        case ESP_LOG_WARN:    color = TETHER_COLOR_WARN; levelChar = "W"; break;
        case ESP_LOG_INFO:    color = TETHER_COLOR_INFO; levelChar = "I"; break;
        case ESP_LOG_DEBUG:   color = TETHER_COLOR_DEBUG; levelChar = "D"; break;
        case ESP_LOG_VERBOSE: color = TETHER_COLOR_VERB; levelChar = "V"; break;
        default: break;
    }

    for (const auto& line : lines) {
        if (tether_log_use_color()) printf("%s[%s]" TETHER_COLOR_RESET " %s: %s\n", color, levelChar, tag, line.c_str());
        else printf("[%s] %s: %s\n", levelChar, tag, line.c_str());
    }
}

#define TETHER_LOGE(tag, format, ...) do { \
    va_list ap; va_start(ap, format); tether_log_printf(ESP_LOG_ERROR, tag, format, ap); va_end(ap); \
} while(0)

#define TETHER_LOGW(tag, format, ...) do { \
    va_list ap; va_start(ap, format); tether_log_printf(ESP_LOG_WARN, tag, format, ap); va_end(ap); \
} while(0)

#define TETHER_LOGI(tag, format, ...) do { \
    va_list ap; va_start(ap, format); tether_log_printf(ESP_LOG_INFO, tag, format, ap); va_end(ap); \
} while(0)

#define TETHER_LOGD(tag, format, ...) do { \
    va_list ap; va_start(ap, format); tether_log_printf(ESP_LOG_DEBUG, tag, format, ap); va_end(ap); \
} while(0)

#define TETHER_LOGV(tag, format, ...) do { \
    va_list ap; va_start(ap, format); tether_log_printf(ESP_LOG_VERBOSE, tag, format, ap); va_end(ap); \
} while(0)

#define ESP_LOG_BUFFER_HEX_LEVEL(tag, buffer, buff_len, level) ((void)0)
#define ESP_LOG_BUFFER_CHAR_LEVEL(tag, buffer, buff_len, level) ((void)0)
#define ESP_LOG_BUFFER_HEXDUMP(tag, buffer, buff_len, level) ((void)0)
#define ESP_LOG_BUFFER_HEX(tag, buffer, buff_len) ((void)0)
#define ESP_LOG_BUFFER_CHAR(tag, buffer, buff_len) ((void)0)

static inline void esp_log_level_set(const char* tag, esp_log_level_t level) {
    (void)tag;
    (void)level;
}

#ifdef __cplusplus
}
#endif

#endif // ESP_LOG_H
