#ifndef ISD04_DRIVER_CONFIG_H
#define ISD04_DRIVER_CONFIG_H

#include <stdint.h>

/*
 * Configuration header for the ISD04 motor driver.
 *
 * This file centralises all user-tunable build options. Projects select the
 * desired behaviour by setting the macros below to 1 or 0. No automatic
 * detection is performed.
 */

/* Set to 1 when building for a CMSIS-RTOS v2 environment. */
#ifndef ISD04_USE_CMSIS
#define ISD04_USE_CMSIS 1
#endif

/* Set to 1 when using the STM32 HAL. */
#ifndef ISD04_USE_HAL
#define ISD04_USE_HAL 1
#endif

#if ISD04_USE_CMSIS  /* CMSIS-RTOS v2 environment */
#include "cmsis_os.h"
#define ISD04_DELAY_MS(ms)            osDelay(ms)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (osKernelGetSysTimerCount())
#define ISD04_DELAY_ELAPSED(start, ms) ((uint32_t)(osKernelGetSysTimerCount() - (start)) >= (ms))
#elif ISD04_USE_HAL  /* Bare-metal STM32 HAL */
#define ISD04_DELAY_MS(ms)            HAL_Delay(ms)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (HAL_GetTick())
#define ISD04_DELAY_ELAPSED(start, ms) ((uint32_t)(HAL_GetTick() - (start)) >= (ms))
#else  /* Host-side build or tests */
#define ISD04_DELAY_MS(ms)            do { (void)(ms); } while (0)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (0U)
#define ISD04_DELAY_ELAPSED(start, ms) ((void)(start), (void)(ms), true)
#endif

#ifndef ISD04_DELAY_US
#if ISD04_USE_CMSIS
static inline void ISD04_DELAY_US(uint32_t us)
{
    uint32_t ms = us / 1000U;
    if (ms > 0U) {
        osDelay(ms);
    }
    uint32_t start = osKernelGetSysTimerCount();
    uint32_t ticks = (us % 1000U) * osKernelGetSysTimerFreq() / 1000000U;
    while ((osKernelGetSysTimerCount() - start) < ticks) {
    }
}
#elif ISD04_USE_HAL
static inline void ISD04_DELAY_US(uint32_t us)
{
    uint32_t ms = us / 1000U;
    if (ms > 0U) {
        HAL_Delay(ms);
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (us % 1000U) * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < cycles) {
    }
}
#else  /* Host-side build */
static inline void ISD04_DELAY_US(uint32_t us)
{
    (void)us;
    /* Stub delay; precise microsecond timing is not available on host builds */
}
#endif
#endif

#ifndef ISD04_STEP_MIN_INTERVAL_US
#define ISD04_STEP_MIN_INTERVAL_US 4U  /* Minimum low-level pulse width */
#endif

#ifndef ISD04_ENABLE_WAKE_DELAY_MS
#define ISD04_ENABLE_WAKE_DELAY_MS 1U  /* Delay after enabling driver */
#endif

#ifndef ISD04_ENA_ACTIVE_LEVEL
#define ISD04_ENA_ACTIVE_LEVEL GPIO_PIN_RESET  /* Active-low disable */
#endif

#ifndef ISD04_STEP_PULSE_DELAY_MS
#define ISD04_STEP_PULSE_DELAY_MS 0U
#endif

/*
 * Enable hardware-timer based STEP pulse generation.
 *
 * When set to a non-zero value the driver expects a timer to be bound via
 * isd04_driver_bind_step_timer() and will trigger that timer instead of
 * toggling the STEP GPIO directly.
 */
#ifndef ISD04_STEP_CONTROL_TIMER
#define ISD04_STEP_CONTROL_TIMER 1U
#endif

#ifndef ISD04_GPIO_PIN_COUNT
#define ISD04_GPIO_PIN_COUNT 16U
#endif

#endif /* ISD04_DRIVER_CONFIG_H */
