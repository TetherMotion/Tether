/**
 * @file esp_timer.h
 * @brief Mock ESP-IDF timer header for host tests
 */
#ifndef ESP_TIMER_H
#define ESP_TIMER_H

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int esp_err_t;

#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NOT_FOUND 0x105

/**
 * @brief Get time since boot in microseconds
 */
static inline int64_t esp_timer_get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000LL + (int64_t)ts.tv_nsec / 1000LL;
}

// Timer handle type
typedef struct esp_timer* esp_timer_handle_t;

// Timer callback type
typedef void (*esp_timer_cb_t)(void* arg);

// Timer dispatch method
typedef enum {
    ESP_TIMER_TASK,
    ESP_TIMER_ISR,
    ESP_TIMER_MAX
} esp_timer_dispatch_t;

// Timer create args
typedef struct {
    esp_timer_cb_t callback;
    void* arg;
    esp_timer_dispatch_t dispatch_method;
    const char* name;
    bool skip_unhandled_events;
} esp_timer_create_args_t;

// Stub implementations
static inline esp_err_t esp_timer_create(const esp_timer_create_args_t* create_args, 
                                         esp_timer_handle_t* out_handle) {
    (void)create_args;
    *out_handle = NULL;
    return ESP_OK;
}

static inline esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us) {
    (void)timer;
    (void)timeout_us;
    return ESP_OK;
}

static inline esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period) {
    (void)timer;
    (void)period;
    return ESP_OK;
}

static inline esp_err_t esp_timer_stop(esp_timer_handle_t timer) {
    (void)timer;
    return ESP_OK;
}

static inline esp_err_t esp_timer_delete(esp_timer_handle_t timer) {
    (void)timer;
    return ESP_OK;
}

static inline bool esp_timer_is_active(esp_timer_handle_t timer) {
    (void)timer;
    return false;
}

#ifdef __cplusplus
}
#endif

#endif // ESP_TIMER_H
