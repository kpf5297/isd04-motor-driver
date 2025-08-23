#ifndef ISD04_DRIVER_CONFIG_H
#define ISD04_DRIVER_CONFIG_H

#include <stdint.h>

/*
 * Configuration header for the ISD04 motor driver.
 *
 * This file centralises all user-tunable build options.  Projects may override
 * any of the macros below at compile time to adapt the driver to their
 * platform.  Delay helpers automatically integrate with either CMSIS-RTOS v2 or
 * the STM32 HAL depending on which environment macros are defined.  When neither
 * environment is detected, the delay helpers become no-ops so that the driver
 * can still be built for host-side tests.
 */

#ifdef CMSIS_OS_VERSION  /* CMSIS-RTOS v2 environment */
#include "cmsis_os2.h"
#define ISD04_DELAY_MS(ms)            osDelay(ms)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (osKernelSysTick())
#define ISD04_DELAY_ELAPSED(start, ms) ((uint32_t)(osKernelSysTick() - (start)) >= (ms))
#elif defined(USE_HAL_DRIVER)  /* bare-metal STM32 HAL */
#define ISD04_DELAY_MS(ms)            HAL_Delay(ms)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (HAL_GetTick())
#define ISD04_DELAY_ELAPSED(start, ms) ((uint32_t)(HAL_GetTick() - (start)) >= (ms))
#else  /* host-side build or tests */
#define ISD04_DELAY_MS(ms)            do { (void)(ms); } while (0)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (0U)
#define ISD04_DELAY_ELAPSED(start, ms) ((void)(start), (void)(ms), true)
#endif

#ifndef ISD04_DELAY_US
#ifdef CMSIS_OS_VERSION
static inline void ISD04_DELAY_US(uint32_t us)
{
    uint32_t ms = us / 1000U;
    if (ms > 0U) {
        osDelay(ms);
    }
    uint32_t start = osKernelSysTick();
    uint32_t ticks = (us % 1000U) * osKernelGetTickFreq() / 1000000U;
    while ((osKernelSysTick() - start) < ticks) {
    }
}
#elif defined(USE_HAL_DRIVER)
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
#else  /* host-side build */
static inline void ISD04_DELAY_US(uint32_t us)
{
    (void)us;
    /* Stub delay; precise microsecond timing is not available on host builds. */
}
#endif
#endif

#ifndef ISD04_STEP_MIN_INTERVAL_US
#define ISD04_STEP_MIN_INTERVAL_US 4U  /* min low-level pulse width */
#endif

#ifndef ISD04_ENABLE_WAKE_DELAY_MS
#define ISD04_ENABLE_WAKE_DELAY_MS 1U  /* delay after enabling driver */
#endif

#ifndef ISD04_ENA_ACTIVE_LEVEL
#define ISD04_ENA_ACTIVE_LEVEL GPIO_PIN_RESET  /* active-low disable */
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
#define ISD04_STEP_CONTROL_TIMER 0U
#endif

#ifndef ISD04_GPIO_PIN_COUNT
#define ISD04_GPIO_PIN_COUNT 16U
#endif

#endif /* ISD04_DRIVER_CONFIG_H */
