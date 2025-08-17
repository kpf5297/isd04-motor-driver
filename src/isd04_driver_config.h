#ifndef ISD04_DRIVER_CONFIG_H
#define ISD04_DRIVER_CONFIG_H

#include <stdint.h>

/*
 * Configuration header for the ISD04 motor driver.
 *
 * Applications may override any of the macros in this file at compile time to
 * tune the driver's behaviour. Delay helpers integrate with either CMSIS-RTOS
 * v2 or the STM32 HAL depending on which environment macros are defined. When
 * neither environment is present, stub implementations are provided so that the
 * driver can be built for host-side tests.
 */

#ifdef CMSIS_OS_VERSION
#include "cmsis_os.h"
#define ISD04_DELAY_MS(ms)            osDelay(ms)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (osKernelSysTick())
#define ISD04_DELAY_ELAPSED(start, ms) ((uint32_t)(osKernelSysTick() - (start)) >= (ms))
#elif defined(USE_HAL_DRIVER)
#define ISD04_DELAY_MS(ms)            HAL_Delay(ms)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (HAL_GetTick())
#define ISD04_DELAY_ELAPSED(start, ms) ((uint32_t)(HAL_GetTick() - (start)) >= (ms))
#else
#define ISD04_DELAY_MS(ms)            do { (void)(ms); } while (0)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (0U)
#define ISD04_DELAY_ELAPSED(start, ms) ((void)(start), (void)(ms), true)
#endif

#ifndef ISD04_DELAY_US
#define ISD04_DELAY_US(us)      /* platform-specific microsecond delay */
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

#ifndef ISD04_GPIO_PIN_COUNT
#define ISD04_GPIO_PIN_COUNT 16U
#endif

#endif /* ISD04_DRIVER_CONFIG_H */
