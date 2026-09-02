/**
 * @file esp_log.h
 * @brief Mock ESP-IDF logging header for host tests
 */
#ifndef ESP_LOG_H
#define ESP_LOG_H

#include <stdio.h>
#ifdef __cplusplus
#include <format>
#include <string>
#include <string_view>
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

static inline void tether_log_formatted(int level, const char* tag, std::string_view message) {
    std::string msg(message);

    // Split lines and drop a trailing empty segment caused by trailing '\n'
    size_t start = 0;
    std::vector<std::string> lines;
    while (start <= msg.size()) {
        size_t pos = msg.find('\n', start);
        if (pos == std::string::npos) pos = msg.size();
        lines.emplace_back(msg.substr(start, pos - start));
        if (pos == msg.size()) break;
        start = pos + 1;
    }
    if (!lines.empty() && !msg.empty() && msg.back() == '\n' && lines.back().empty()) lines.pop_back();

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

template <typename... Args>
static inline std::string tether_log_format(const char* fmt, Args... args) {
    return std::vformat(fmt, std::make_format_args(args...));
}

#define TETHER_LOGE(tag, fmt, ...) tether_log_formatted(ESP_LOG_ERROR, tag, tether_log_format(fmt __VA_OPT__(,) __VA_ARGS__))
#define TETHER_LOGW(tag, fmt, ...) tether_log_formatted(ESP_LOG_WARN, tag, tether_log_format(fmt __VA_OPT__(,) __VA_ARGS__))
#define TETHER_LOGI(tag, fmt, ...) tether_log_formatted(ESP_LOG_INFO, tag, tether_log_format(fmt __VA_OPT__(,) __VA_ARGS__))
#define TETHER_LOGD(tag, fmt, ...) tether_log_formatted(ESP_LOG_DEBUG, tag, tether_log_format(fmt __VA_OPT__(,) __VA_ARGS__))
#define TETHER_LOGV(tag, fmt, ...) tether_log_formatted(ESP_LOG_VERBOSE, tag, tether_log_format(fmt __VA_OPT__(,) __VA_ARGS__))

// Compile-time validated std::format variants (see Logger.hpp).
#define TETHER_LOGE_FMT(tag, fmt, ...) tether_log_formatted(ESP_LOG_ERROR, tag, std::format(fmt __VA_OPT__(,) __VA_ARGS__))
#define TETHER_LOGW_FMT(tag, fmt, ...) tether_log_formatted(ESP_LOG_WARN, tag, std::format(fmt __VA_OPT__(,) __VA_ARGS__))
#define TETHER_LOGI_FMT(tag, fmt, ...) tether_log_formatted(ESP_LOG_INFO, tag, std::format(fmt __VA_OPT__(,) __VA_ARGS__))
#define TETHER_LOGD_FMT(tag, fmt, ...) tether_log_formatted(ESP_LOG_DEBUG, tag, std::format(fmt __VA_OPT__(,) __VA_ARGS__))
#define TETHER_LOGV_FMT(tag, fmt, ...) tether_log_formatted(ESP_LOG_VERBOSE, tag, std::format(fmt __VA_OPT__(,) __VA_ARGS__))

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
