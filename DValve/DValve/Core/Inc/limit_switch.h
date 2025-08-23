#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Event flag definitions */
#define TOP_LIMIT_EVENT    (1U << 0)
#define BOTTOM_LIMIT_EVENT (1U << 1)

/* Limit switch status */
typedef struct {
    bool top_limit_active;
    bool bottom_limit_active;
    bool limit_switch_enabled;
} LimitSwitchStatus_t;

/* Function prototypes */
void limit_switch_init(void);
void limit_switch_task(void *argument);
void limit_switch_gpio_callback(uint16_t GPIO_Pin);
bool limit_switch_is_top_active(void);
bool limit_switch_is_bottom_active(void);
void limit_switch_enable(bool enable);
LimitSwitchStatus_t limit_switch_get_status(void);

/* External declarations for RTOS objects */
extern osThreadId_t limitSwitchTaskHandle;
extern const osThreadAttr_t limitSwitchTask_attributes;
extern osEventFlagsId_t limitSwitchEventsHandle;
extern const osEventFlagsAttr_t limitSwitchEvents_attributes;

#ifdef __cplusplus
}
#endif

#endif /* LIMIT_SWITCH_H */
