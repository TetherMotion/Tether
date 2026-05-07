/**
 * @file FreeRTOS.h
 * @brief Mock FreeRTOS header for host tests
 */
#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Basic types
typedef uint32_t TickType_t;
typedef int32_t BaseType_t;
typedef uint32_t UBaseType_t;
typedef void* TaskHandle_t;
typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* EventGroupHandle_t;
typedef void* TimerHandle_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS pdTRUE
#define pdFAIL pdFALSE

#define portMAX_DELAY 0xFFFFFFFFUL
#define portTICK_PERIOD_MS 1

#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 25

// Tick to milliseconds conversion
#define pdMS_TO_TICKS(xTimeInMs) ((TickType_t)(xTimeInMs))
#define pdTICKS_TO_MS(xTicks) ((uint32_t)(xTicks))

// Task creation
#define xTaskCreate(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask) pdPASS
#define xTaskCreatePinnedToCore(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask, xCoreID) pdPASS
#define vTaskDelete(xTask) ((void)0)
#define vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement) ((void)0)

// Task control
#define vTaskSuspend(xTask) ((void)0)
#define vTaskResume(xTask) ((void)0)

// Critical sections
#define taskENTER_CRITICAL(x) ((void)0)
#define taskEXIT_CRITICAL(x) ((void)0)
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)

#define portDISABLE_INTERRUPTS() ((void)0)
#define portENABLE_INTERRUPTS() ((void)0)

// Mutexes and semaphores
#define xSemaphoreCreateMutex() ((SemaphoreHandle_t)1)
#define xSemaphoreCreateBinary() ((SemaphoreHandle_t)1)
#define xSemaphoreGive(x) pdTRUE
#define xSemaphoreTake(x, blockTime) pdTRUE
#define xSemaphoreGiveFromISR(x, pxHigherPriorityTaskWoken) pdTRUE
#define xSemaphoreTakeFromISR(x, pxHigherPriorityTaskWoken) pdTRUE
#define vSemaphoreDelete(x) ((void)0)

// Queues
#define xQueueCreate(length, itemSize) ((QueueHandle_t)1)
#define xQueueSend(xQueue, pvItemToQueue, xTicksToWait) pdTRUE
#define xQueueReceive(xQueue, pvBuffer, xTicksToWait) pdTRUE
#define xQueueSendFromISR(xQueue, pvItemToQueue, pxHigherPriorityTaskWoken) pdTRUE
#define xQueueReceiveFromISR(xQueue, pvBuffer, pxHigherPriorityTaskWoken) pdTRUE
#define vQueueDelete(xQueue) ((void)0)
#define uxQueueMessagesWaiting(xQueue) 0
#define xQueueReset(xQueue) pdTRUE

// Event groups
#define xEventGroupCreate() ((EventGroupHandle_t)1)
#define xEventGroupSetBits(xEventGroup, uxBitsToSet) 0
#define xEventGroupClearBits(xEventGroup, uxBitsToClear) 0
#define xEventGroupWaitBits(xEventGroup, uxBitsToWaitFor, xClearOnExit, xWaitForAllBits, xTicksToWait) 0
#define xEventGroupGetBits(xEventGroup) 0
#define vEventGroupDelete(xEventGroup) ((void)0)

// Memory
#define pvPortMalloc(xSize) malloc(xSize)
#define vPortFree(pv) free(pv)

// Task notifications
#define xTaskNotifyGive(xTaskToNotify) pdTRUE
#define ulTaskNotifyTake(xClearCountOnExit, xTicksToWait) 1
#define xTaskNotify(xTaskToNotify, ulValue, eAction) pdTRUE
#define xTaskNotifyWait(ulBitsToClearOnEntry, ulBitsToClearOnExit, pulNotificationValue, xTicksToWait) pdTRUE

// Spinlocks
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define spinlock_initialize(x) ((void)0)
#define vPortCPUInitializeMutex(x) ((void)0)
#define portENTER_CRITICAL_SAFE(x) ((void)0)
#define portEXIT_CRITICAL_SAFE(x) ((void)0)

#ifdef __cplusplus
}
#endif

#endif // FREERTOS_H
