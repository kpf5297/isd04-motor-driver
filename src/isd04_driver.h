#ifndef ISD04_DRIVER_H
#define ISD04_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#if __has_include("stm32f4xx_hal.h")
#include "stm32f4xx_hal.h"
#else
typedef struct GPIO_TypeDef GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0U, GPIO_PIN_SET } GPIO_PinState;
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    (void)port; (void)pin; (void)state;
}
#endif

/**
 * Delay helper macros.
 *
 * Depending on the build environment these expand to the appropriate HAL or
 * CMSIS-RTOS primitives.  When neither `USE_HAL_DRIVER` nor `CMSIS_OS_VERSION`
 * is defined they fall back to no-ops which allows the driver to be built for
 * host side tests.
 */
#ifdef CMSIS_OS_VERSION
#include "cmsis_os.h"
/** Delay for the specified number of milliseconds. */
#define ISD04_DELAY_MS(ms)            osDelay(ms)
typedef uint32_t Isd04DelayTick;
/** Capture the current system tick for later elapsed checks. */
#define ISD04_DELAY_START()           (osKernelSysTick())
/** Check whether @p ms milliseconds have elapsed since @p start. */
#define ISD04_DELAY_ELAPSED(start, ms) ((uint32_t)(osKernelSysTick() - (start)) >= (ms))
#elif defined(USE_HAL_DRIVER)
/** Delay for the specified number of milliseconds using the HAL. */
#define ISD04_DELAY_MS(ms)            HAL_Delay(ms)
typedef uint32_t Isd04DelayTick;
/** Capture the current HAL tick for later elapsed checks. */
#define ISD04_DELAY_START()           (HAL_GetTick())
/** Check whether @p ms milliseconds have elapsed since @p start. */
#define ISD04_DELAY_ELAPSED(start, ms) ((uint32_t)(HAL_GetTick() - (start)) >= (ms))
#else
/** No-op delay used when neither HAL nor CMSIS-RTOS is available. */
#define ISD04_DELAY_MS(ms)            do { (void)(ms); } while (0)
typedef uint32_t Isd04DelayTick;
#define ISD04_DELAY_START()           (0U)
#define ISD04_DELAY_ELAPSED(start, ms) ((void)(start), (void)(ms), true)
#endif

#define ISD04_DRIVER_VERSION_MAJOR 1
#define ISD04_DRIVER_VERSION_MINOR 0
#define ISD04_DRIVER_VERSION_PATCH 0
#define ISD04_DRIVER_VERSION_STRING "1.0.0"

#ifdef __cplusplus
extern "C" {
#endif

/** Supported microstep resolutions. */
typedef enum {
    ISD04_MICROSTEP_200 = 200,
    ISD04_MICROSTEP_400 = 400,
    ISD04_MICROSTEP_800 = 800,
    ISD04_MICROSTEP_1600 = 1600,
    ISD04_MICROSTEP_3200 = 3200,
} Isd04Microstep;

/**
 * Static configuration for the ISD04 driver.
 */
typedef struct {
    /** PWM frequency in Hz. */
    uint32_t pwm_frequency_hz;
    /** Maximum speed value permitted. */
    int32_t max_speed;
    /** Default microstepping resolution. */
    Isd04Microstep microstep;
    /** Phase current in milliamps. */
    uint16_t phase_current_ma;
} Isd04Config;

/** Hardware definition for ISD04 control pins. */
typedef struct {
    GPIO_TypeDef *stp_port;
    uint16_t stp_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *ena_port;
    uint16_t ena_pin;
} Isd04Hardware;

/**
 * Events that can be emitted by the driver.
 */
typedef enum {
    /** Motor has started running. */
    ISD04_EVENT_STARTED,
    /** Motor has stopped running. */
    ISD04_EVENT_STOPPED,
    /** Motor speed has been updated. */
    ISD04_EVENT_SPEED_CHANGED,
    /** Microstep resolution has changed. */
    ISD04_EVENT_MICROSTEP_CHANGED,
    /** Motor position has changed. */
    ISD04_EVENT_POSITION_CHANGED,
    /** A hardware access failure occurred. */
    ISD04_EVENT_ERROR,
} Isd04Event;

/** DIP switch patterns for microstep modes. */
#define ISD04_MICROSTEP_200_BITS   0x0
#define ISD04_MICROSTEP_400_BITS   0x1
#define ISD04_MICROSTEP_800_BITS   0x2
#define ISD04_MICROSTEP_1600_BITS  0x3
#define ISD04_MICROSTEP_3200_BITS  0x7

/** Map a microstep mode to its DIP switch bit pattern. */
#define ISD04_MICROSTEP_TO_BITS(mode) \
    ((mode) == ISD04_MICROSTEP_200 ? ISD04_MICROSTEP_200_BITS : \
     (mode) == ISD04_MICROSTEP_400 ? ISD04_MICROSTEP_400_BITS : \
     (mode) == ISD04_MICROSTEP_800 ? ISD04_MICROSTEP_800_BITS : \
     (mode) == ISD04_MICROSTEP_1600 ? ISD04_MICROSTEP_1600_BITS : \
     ISD04_MICROSTEP_3200_BITS)

/** Apply a DIP switch pattern to hardware (stub). */
#define ISD04_APPLY_MICROSTEP(bits) \
    do { (void)(bits); } while (0)

/**
 * Callback invoked when a driver event occurs.
 *
 * The callback receives one of the following events:
 *
 * - ::ISD04_EVENT_STARTED when the driver transitions to the running state.
 * - ::ISD04_EVENT_STOPPED when the driver halts the motor.
 * - ::ISD04_EVENT_SPEED_CHANGED when the commanded speed is updated.
 * - ::ISD04_EVENT_MICROSTEP_CHANGED after a new microstepping mode is applied.
 * - ::ISD04_EVENT_POSITION_CHANGED whenever the tracked position is modified.
 * - ::ISD04_EVENT_ERROR if a hardware write operation fails.
 */
typedef void (*Isd04EventCallback)(Isd04Event event, void *context);

/** Identifier for the internal driver state. */
typedef enum {
    /** Driver is stopped and motor is not running. */
    ISD04_STATE_STOPPED,
    /** Driver is running and controlling the motor. */
    ISD04_STATE_RUNNING,
} Isd04StateId;

struct Isd04State; /* Forward declaration of state structure. */

/**
 * Runtime state for an ISD04 motor driver instance.
 */
typedef struct {
    /** Static configuration for the device. */
    Isd04Config config;
    /** Hardware pins controlling the driver. */
    Isd04Hardware hw;
    /** Current motor speed. */
    int32_t current_speed;
    /** Current motor position in steps. */
    int32_t current_position;
    /** Indicates whether the motor is running. */
    bool running;
    /** Registered event callback. */
    Isd04EventCallback callback;
    /** User supplied context passed to the callback. */
    void *callback_context;
    /** Current microstep resolution. */
    Isd04Microstep microstep;
    /** Current behavioral state of the driver. */
    const struct Isd04State *state;
    /** Set when a hardware error has been detected. */
    bool error;
} Isd04Driver;

const char *isd04_driver_get_version(void);
void isd04_driver_get_default_config(Isd04Config *config);
void isd04_driver_init(Isd04Driver *driver, const Isd04Config *config, const Isd04Hardware *hw);
void isd04_driver_start(Isd04Driver *driver);
void isd04_driver_stop(Isd04Driver *driver);
void isd04_driver_set_speed(Isd04Driver *driver, int32_t speed);
void isd04_driver_set_microstep(Isd04Driver *driver, Isd04Microstep mode);
Isd04Microstep isd04_driver_get_microstep(const Isd04Driver *driver);
void isd04_driver_set_direction(Isd04Driver *driver, bool forward);
void isd04_driver_enable(Isd04Driver *driver, bool enable);
void isd04_driver_pulse(Isd04Driver *driver);
void isd04_driver_register_callback(Isd04Driver *driver, Isd04EventCallback callback, void *context);
Isd04Driver *isd04_driver_get_instance(void);
Isd04StateId isd04_driver_get_state(const Isd04Driver *driver);

/**
 * Retrieve the driver's notion of the current motor position in steps.
 */
int32_t isd04_driver_get_position(const Isd04Driver *driver);

/**
 * Set the driver's current motor position counter.
 *
 * Emits ::ISD04_EVENT_POSITION_CHANGED if the value differs.
 */
void isd04_driver_set_position(Isd04Driver *driver, int32_t position);

/**
 * Advance the driver's internal position counter.
 *
 * Call this periodically (e.g., from a timer interrupt) with the number of
 * steps that have been issued to the motor since the last call to keep the
 * position in sync with actual movement.
 */
void isd04_driver_step(Isd04Driver *driver, int32_t steps);

#ifdef __cplusplus
}
#endif

#endif /* ISD04_DRIVER_H */
