/**
 * @file task.h
 * @brief Mock FreeRTOS task header for host tests
 */
#ifndef FREERTOS_TASK_H
#define FREERTOS_TASK_H

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

// Additional task macros
#define tskIDLE_PRIORITY 0

typedef void (*TaskFunction_t)(void*);

static inline TaskHandle_t xTaskGetCurrentTaskHandle(void) { return NULL; }
static inline void vTaskDelay(TickType_t xTicksToDelay) { (void)xTicksToDelay; }
static inline TickType_t xTaskGetTickCount(void) { return 0; }

#ifdef __cplusplus
}
#endif

#endif // FREERTOS_TASK_H
