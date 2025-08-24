#ifndef ISD04_DRIVER_H
#define ISD04_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "isd04_driver_config.h"

#if ISD04_USE_CMSIS
#include "cmsis_os2.h"
#else
typedef void *osMutexId_t;
static inline osMutexId_t osMutexNew(const void *attr) { (void)attr; return NULL; }
static inline void osMutexAcquire(osMutexId_t id, uint32_t timeout) { (void)id; (void)timeout; }
static inline void osMutexRelease(osMutexId_t id) { (void)id; }
typedef void *osThreadId_t;
static inline osThreadId_t osThreadNew(void (*func)(void *), void *arg, const void *attr)
{ (void)func; (void)arg; (void)attr; return NULL; }
static inline int osThreadTerminate(osThreadId_t id) { (void)id; return 0; }
static inline uint32_t osKernelGetTickFreq(void) { return 1000U; }
static inline void osDelay(uint32_t ms) { (void)ms; }
#ifndef osWaitForever
#define osWaitForever 0xFFFFFFFFU
#endif
#endif

#if ISD04_USE_HAL
#include "stm32f4xx_hal.h"
#else
typedef struct GPIO_TypeDef GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0U, GPIO_PIN_SET } GPIO_PinState;
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    (void)port; (void)pin; (void)state;
}
#endif


#define ISD04_DRIVER_VERSION_MAJOR 1
#define ISD04_DRIVER_VERSION_MINOR 0
#define ISD04_DRIVER_VERSION_PATCH 1
#define ISD04_DRIVER_VERSION_STRING "1.0.1"

/**
 * Validate that a GPIO pin definition does not exceed the MCU's pin count.
 *
 * This macro evaluates to @c true when @p pin falls within the permitted
 * range of single-bit pin masks. It may be used in static assertions or at
 * run time to guard against invalid configuration values.
 */
#define ISD04_VALIDATE_PIN(pin) \
    ((pin) > 0U && (pin) < (1U << ISD04_GPIO_PIN_COUNT))

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
    /** Maximum step rate in steps per second. */
    int32_t max_speed;
    /** Default microstepping resolution. */
    Isd04Microstep microstep;
    /** Phase current in milliamps. */
    uint16_t phase_current_ma;
} Isd04Config;

/** Hardware definition for ISD04 control pins.
 *
 * Each @c *_pin field must specify a single-bit mask that falls within the
 * target MCU's GPIO pin count (see ::ISD04_GPIO_PIN_COUNT). Values outside
 * this range will cause ::isd04_driver_init to set the driver's error flag.
 */
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
    /** Set when the driver outputs are enabled. */
    bool enabled;
    /** Tick recorded when the last step pulse was issued. */
    Isd04DelayTick last_step_tick;
    /** Thread handle for RTOS-based stepping. */
    osThreadId_t step_thread;
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
    /** Mutex protecting driver state. */
    osMutexId_t mutex;
} Isd04Driver;

/**
 * Obtain the semantic version string of the driver.
 *
 * @return NUL-terminated version string in the form "major.minor.patch".
 */
const char *isd04_driver_get_version(void);

/**
 * Populate a configuration structure with library defaults.
 *
 * @param[out] config Configuration structure to initialise. Must not be NULL.
 */
void isd04_driver_get_default_config(Isd04Config *config);

/**
 * Initialise a driver instance.
 *
 * Configures hardware pins and applies the provided configuration. The
 * driver's error flag is set and the operation aborted if any required
 * pointers are NULL, if any @c *_pin exceeds ::ISD04_GPIO_PIN_COUNT, or if
 * initial hardware writes fail.
 *
 * @param[out] driver Driver instance to initialise.
 * @param[in]  config Static configuration values.
 * @param[in]  hw     Hardware pin definitions.
 */
void isd04_driver_init(Isd04Driver *driver, const Isd04Config *config, const Isd04Hardware *hw);
void isd04_driver_rtos_init(Isd04Driver *driver);

/**
 * Transition the driver to the running state.
 *
 * Emits ::ISD04_EVENT_STARTED on success. Does nothing if @p driver is NULL.
 *
 * @param driver Driver instance to start.
 */
void isd04_driver_start(Isd04Driver *driver);

/**
 * Halt the motor and transition to the stopped state.
 *
 * Emits ::ISD04_EVENT_STOPPED on success. Does nothing if @p driver is NULL.
 *
 * @param driver Driver instance to stop.
 */
void isd04_driver_stop(Isd04Driver *driver);

/**
 * Update the commanded motor speed.
 *
 * The value is clamped to the range configured in ::Isd04Config. Generates
 * ::ISD04_EVENT_SPEED_CHANGED when the speed actually changes.
 *
 * @param driver Driver instance.
 * @param speed  New speed in steps per second.
 */
void isd04_driver_set_speed(Isd04Driver *driver, int32_t speed);

/**
 * Obtain the current motor speed in steps per second.
 *
 * @param driver Driver instance.
 * @return Current speed value; 0 if @p driver is NULL.
 */
int32_t isd04_driver_get_speed(Isd04Driver *driver);

/**
 * Configure the microstepping resolution.
 *
 * Applies the DIP switch pattern corresponding to @p mode and emits
 * ::ISD04_EVENT_MICROSTEP_CHANGED when the mode changes.
 *
 * @param driver Driver instance.
 * @param mode   Desired microstepping mode.
 */
void isd04_driver_set_microstep(Isd04Driver *driver, Isd04Microstep mode);

/**
 * Retrieve the currently configured microstepping mode.
 *
 * @param driver Driver instance or NULL.
 * @return Current microstep setting, or ::ISD04_MICROSTEP_200 if @p driver is
 *         NULL.
 */
Isd04Microstep isd04_driver_get_microstep(Isd04Driver *driver);

/**
 * Set the motor rotation direction.
 *
 * Writes to the direction GPIO pin. If the write fails the driver's error flag
 * is set and ::ISD04_EVENT_ERROR is emitted via the registered callback.
 *
 * @param driver  Driver instance.
 * @param forward @c true for forward rotation, @c false for reverse.
 */
void isd04_driver_set_direction(Isd04Driver *driver, bool forward);

/**
 * Enable or disable the motor driver.
 *
 * Writes to the enable GPIO pin. On failure the driver's error flag is set and
 * ::ISD04_EVENT_ERROR is emitted.
 *
 * @param driver Driver instance.
 * @param enable @c true to enable, @c false to disable.
 */
void isd04_driver_enable(Isd04Driver *driver, bool enable);

/**
 * Generate a single step pulse on the STEP pin.
 *
 * The pin is driven high then low. If @c ISD04_STEP_PULSE_DELAY_MS is non-zero
 * a delay of at least that many milliseconds is inserted between edges using
 * ::ISD04_DELAY_MS to satisfy the driver's minimum pulse width requirement.
 * Any GPIO failure sets the driver's error flag and emits
 * ::ISD04_EVENT_ERROR.
 *
 * @param driver Driver instance.
 */
void isd04_driver_pulse(Isd04Driver *driver);

/**
 * Register a callback for driver events.
 *
 * The callback will be invoked from various API calls when notable events
 * occur, including ::ISD04_EVENT_ERROR when hardware access fails.
 *
 * @param driver   Driver instance.
 * @param callback Callback function pointer (may be NULL to disable).
 * @param context  User-provided pointer passed to @p callback.
 */
void isd04_driver_register_callback(Isd04Driver *driver, Isd04EventCallback callback, void *context);

/**
 * Obtain the singleton driver instance.
 *
 * Allocates the instance on first use. Returns NULL if memory allocation
 * fails.
 *
 * @return Pointer to the driver instance or NULL on allocation failure.
 */
Isd04Driver *isd04_driver_get_instance(void);

/**
 * Retrieve the driver's current state identifier.
 *
 * @param driver Driver instance or NULL.
 * @return Current ::Isd04StateId, or ::ISD04_STATE_STOPPED if @p driver is
 *         NULL.
 */
Isd04StateId isd04_driver_get_state(Isd04Driver *driver);

/**
 * Retrieve the driver's notion of the current motor position in steps.
 *
 * @param driver Driver instance or NULL.
 * @return Current position, or 0 if @p driver is NULL.
 */
int32_t isd04_driver_get_position(Isd04Driver *driver);

/**
 * Set the driver's current motor position counter.
 *
 * Emits ::ISD04_EVENT_POSITION_CHANGED if the value differs from the previous
 * position.
 *
 * @param driver   Driver instance.
 * @param position New absolute position in steps.
 */
void isd04_driver_set_position(Isd04Driver *driver, int32_t position);

/**
 * Advance the driver's internal position counter.
 *
 * Call this periodically with the number of
 * steps that have been issued to the motor since the last call to keep the
 * position in sync with actual movement. Emits ::ISD04_EVENT_POSITION_CHANGED
 * on every invocation.
 *
 * @param driver Driver instance.
 * @param steps  Number of steps taken since the previous call (may be
 *               negative).
 */
void isd04_driver_step(Isd04Driver *driver, int32_t steps);

#ifdef __cplusplus
}
#endif

#endif /* ISD04_DRIVER_H */
