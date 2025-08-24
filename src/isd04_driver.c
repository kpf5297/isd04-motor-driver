#include "isd04_driver.h"
#include <stddef.h>
#include <stdlib.h>

static inline void isd04_lock(Isd04Driver *driver) {
    if (driver && driver->mutex) {
        osMutexAcquire(driver->mutex, osWaitForever);
    }
}

static inline void isd04_unlock(Isd04Driver *driver) {
    if (driver && driver->mutex) {
        osMutexRelease(driver->mutex);
    }
}

/** Singleton driver instance. */
static Isd04Driver *instance = NULL;
static osMutexId_t instance_mutex = NULL;

/**
 * Wrapper around HAL_GPIO_WritePin that returns success/failure.
 *
 * The underlying HAL call does not report status, so this wrapper primarily
 * validates the port pointer and assumes the HAL operation succeeds when the
 * pointer is valid.
 */
static inline bool isd04_gpio_write_pin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (!port) {
        return false;
    }
    HAL_GPIO_WritePin(port, pin, state);
    return true;
}

/** Clamp speed to the allowable range. */
static int32_t clamp_speed(const Isd04Driver *driver, int32_t speed);
/** Clamp microstep mode to the allowable range. */
static Isd04Microstep clamp_microstep(Isd04Microstep mode);
/** Update position counter without locking. */
static void step_unlocked(Isd04Driver *driver, int32_t steps);
/** RTOS task driving step pulses while motor runs. */
static void isd04_step_task(void *argument);

const char *isd04_driver_get_version(void)
{
    return ISD04_DRIVER_VERSION_STRING;
}

/** State object defining behaviour for driver operations. */
typedef struct Isd04State {
    Isd04StateId id;
    void (*enter)(Isd04Driver *driver);
    void (*start)(Isd04Driver *driver);
    void (*stop)(Isd04Driver *driver);
    void (*set_speed)(Isd04Driver *driver, int32_t speed);
} Isd04State;

static void change_state(Isd04Driver *driver, const Isd04State *state);

/* Forward declarations for concrete state behaviours. */
static void stopped_enter(Isd04Driver *driver);
static void stopped_start(Isd04Driver *driver);
static void stopped_stop(Isd04Driver *driver);
static void stopped_set_speed(Isd04Driver *driver, int32_t speed);

static void running_enter(Isd04Driver *driver);
static void running_start(Isd04Driver *driver);
static void running_stop(Isd04Driver *driver);
static void running_set_speed(Isd04Driver *driver, int32_t speed);

static const Isd04State stopped_state = {
    .id = ISD04_STATE_STOPPED,
    .enter = stopped_enter,
    .start = stopped_start,
    .stop = stopped_stop,
    .set_speed = stopped_set_speed,
};

static const Isd04State running_state = {
    .id = ISD04_STATE_RUNNING,
    .enter = running_enter,
    .start = running_start,
    .stop = running_stop,
    .set_speed = running_set_speed,
};

static void change_state(Isd04Driver *driver, const Isd04State *state)
{
    if (!driver || !state) {
        return;
    }
    driver->state = state;
    if (driver->state->enter) {
        driver->state->enter(driver);
    }
}

/* --- Stopped state behaviour --- */
static void stopped_enter(Isd04Driver *driver)
{
    driver->running = false;
    driver->current_speed = 0;
}

static void stopped_start(Isd04Driver *driver)
{
    change_state(driver, &running_state);
    if (driver->callback) {
        driver->callback(ISD04_EVENT_STARTED, driver->callback_context);
    }
}

static void stopped_stop(Isd04Driver *driver)
{
    (void)driver; /* Already in stopped state */
}

static void stopped_set_speed(Isd04Driver *driver, int32_t speed)
{
    int32_t new_speed = clamp_speed(driver, speed);
    if (new_speed != driver->current_speed) {
        driver->current_speed = new_speed;
        if (driver->callback) {
            driver->callback(ISD04_EVENT_SPEED_CHANGED, driver->callback_context);
        }
    }
}

/* --- Running state behaviour --- */
static void running_enter(Isd04Driver *driver)
{
    driver->running = true;
}

static void running_start(Isd04Driver *driver)
{
    (void)driver;
}

static void running_stop(Isd04Driver *driver)
{
    change_state(driver, &stopped_state);
    if (driver->callback) {
        driver->callback(ISD04_EVENT_STOPPED, driver->callback_context);
    }
}

static void running_set_speed(Isd04Driver *driver, int32_t speed)
{
    int32_t new_speed = clamp_speed(driver, speed);
    if (new_speed != driver->current_speed) {
        driver->current_speed = new_speed;
        if (driver->callback) {
            driver->callback(ISD04_EVENT_SPEED_CHANGED, driver->callback_context);
        }
    }
}

static void isd04_step_task(void *argument)
{
    Isd04Driver *driver = (Isd04Driver *)argument;
    while (driver) {
        if (driver->running && driver->current_speed != 0) {
            int32_t direction = driver->current_speed > 0 ? 1 : -1;
            uint32_t step_hz = (uint32_t)abs(driver->current_speed);
            if (step_hz == 0U) {
                osDelay(1U);
                continue;
            }
            uint32_t delay_ticks = osKernelGetTickFreq() / step_hz;
            if (delay_ticks == 0U) {
                delay_ticks = 1U;
            }
            isd04_driver_pulse(driver);
            isd04_driver_step(driver, direction);
            osDelay(delay_ticks);
        } else {
            osDelay(1U);
        }
    }
}

Isd04Driver *isd04_driver_get_instance(void)
{
    if (!instance_mutex) {
        instance_mutex = osMutexNew(NULL);
    }

    osMutexAcquire(instance_mutex, osWaitForever);
    if (!instance) {
        instance = calloc(1, sizeof(Isd04Driver));
    }
    osMutexRelease(instance_mutex);

    return instance;
}

void isd04_driver_get_default_config(Isd04Config *config)
{
    if (!config) {
        return;
    }
    config->max_speed = 1000;
    config->microstep = ISD04_MICROSTEP_3200;
    config->phase_current_ma = 2800U;
}

void isd04_driver_init(Isd04Driver *driver, const Isd04Config *config, const Isd04Hardware *hw)
{
    if (!driver || !config || !hw) {
        return;
    }

    driver->mutex = osMutexNew(NULL);
    driver->error = false;

    if (!hw->stp_port || !hw->dir_port || !hw->ena_port) {
        driver->error = true;
        return;
    }

    if (!ISD04_VALIDATE_PIN(hw->stp_pin) ||
        !ISD04_VALIDATE_PIN(hw->dir_pin) ||
        !ISD04_VALIDATE_PIN(hw->ena_pin)) {
        driver->error = true;
        return;
    }

    driver->config = *config;
    driver->hw = *hw;
    driver->current_speed = 0;
    driver->current_position = 0;
    driver->running = false;
    GPIO_PinState ena_level = (ISD04_ENA_ACTIVE_LEVEL == GPIO_PIN_SET)
        ? GPIO_PIN_RESET
        : GPIO_PIN_SET;
    driver->enabled = (ena_level != ISD04_ENA_ACTIVE_LEVEL);
    driver->last_step_tick = 0U;
    driver->step_thread = NULL;
    driver->callback = NULL;
    driver->callback_context = NULL;
    driver->microstep = config->microstep;
    ISD04_APPLY_MICROSTEP(ISD04_MICROSTEP_TO_BITS(driver->microstep));
    if (!isd04_gpio_write_pin(driver->hw.ena_port, driver->hw.ena_pin, ena_level) ||
        !isd04_gpio_write_pin(driver->hw.dir_port, driver->hw.dir_pin, GPIO_PIN_RESET)) {
        driver->error = true;
        return;
    }
    change_state(driver, &stopped_state);
}

void isd04_driver_rtos_init(Isd04Driver *driver)
{
    if (driver) {
        driver->step_thread = NULL;
    }
}

void isd04_driver_start(Isd04Driver *driver)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
    const Isd04State *prev = driver->state;
    if (prev && prev->start) {
        prev->start(driver);
    }
    if (driver->state != prev && driver->state && driver->state->start) {
        driver->state->start(driver);
    }
    isd04_unlock(driver);
    if (!driver->step_thread) {
        driver->step_thread = osThreadNew(isd04_step_task, driver, NULL);
    }
}

void isd04_driver_stop(Isd04Driver *driver)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
    const Isd04State *prev = driver->state;
    if (prev && prev->stop) {
        prev->stop(driver);
    }
    if (driver->state != prev && driver->state && driver->state->stop) {
        driver->state->stop(driver);
    }
    isd04_unlock(driver);
    if (driver->step_thread) {
        osThreadTerminate(driver->step_thread);
        driver->step_thread = NULL;
    }
    isd04_lock(driver);
    driver->running = false;
    driver->current_speed = 0;
    isd04_unlock(driver);
}

void isd04_driver_set_speed(Isd04Driver *driver, int32_t speed)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
    if (driver->state) {
        driver->state->set_speed(driver, speed);
    }
    isd04_unlock(driver);
}

int32_t isd04_driver_get_speed(Isd04Driver *driver)
{
    if (!driver) {
        return 0;
    }
    isd04_lock(driver);
    int32_t speed = driver->current_speed;
    isd04_unlock(driver);
    return speed;
}

void isd04_driver_set_microstep(Isd04Driver *driver, Isd04Microstep mode)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
    Isd04Microstep new_mode = clamp_microstep(mode);
    if (new_mode != driver->microstep) {
        ISD04_APPLY_MICROSTEP(ISD04_MICROSTEP_TO_BITS(new_mode));
        driver->microstep = new_mode;
        if (driver->callback) {
            /* Inform user code that the microstepping mode was updated */
            driver->callback(ISD04_EVENT_MICROSTEP_CHANGED, driver->callback_context);
        }
    }
    isd04_unlock(driver);
}

Isd04Microstep isd04_driver_get_microstep(Isd04Driver *driver)
{
    if (!driver) {
        return ISD04_MICROSTEP_200;
    }

    isd04_lock(driver);
    Isd04Microstep mode = driver->microstep;
    isd04_unlock(driver);
    return mode;
}

void isd04_driver_set_direction(Isd04Driver *driver, bool forward)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
    if (!isd04_gpio_write_pin(driver->hw.dir_port, driver->hw.dir_pin,
                               forward ? GPIO_PIN_SET : GPIO_PIN_RESET)) {
        driver->error = true;
        if (driver->callback) {
            driver->callback(ISD04_EVENT_ERROR, driver->callback_context);
        }
    }
    isd04_unlock(driver);
}

void isd04_driver_enable(Isd04Driver *driver, bool enable)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
    bool was_enabled = driver->enabled;
    GPIO_PinState level = enable
        ? (ISD04_ENA_ACTIVE_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET)
        : ISD04_ENA_ACTIVE_LEVEL;
    if (!isd04_gpio_write_pin(driver->hw.ena_port, driver->hw.ena_pin, level)) {
        driver->error = true;
        if (driver->callback) {
            driver->callback(ISD04_EVENT_ERROR, driver->callback_context);
        }
    }
    if (enable && !was_enabled) {
        ISD04_DELAY_MS(ISD04_ENABLE_WAKE_DELAY_MS);
    }
    driver->enabled = enable;
    isd04_unlock(driver);
}

void isd04_driver_pulse(Isd04Driver *driver)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
#if ISD04_STEP_MIN_INTERVAL_US > 0U
    uint32_t min_ms = ISD04_STEP_MIN_INTERVAL_US / 1000U;
    if (driver->last_step_tick != 0U) {
        if (min_ms > 0U) {
            while (!ISD04_DELAY_ELAPSED(driver->last_step_tick, min_ms)) {
                ISD04_DELAY_MS(1U);
            }
        } else {
            ISD04_DELAY_US(ISD04_STEP_MIN_INTERVAL_US);
        }
    }
#endif

    if (!isd04_gpio_write_pin(driver->hw.stp_port, driver->hw.stp_pin, GPIO_PIN_SET)) {
        driver->error = true;
        if (driver->callback) {
            driver->callback(ISD04_EVENT_ERROR, driver->callback_context);
        }
        isd04_unlock(driver);
        return;
    }
#if ISD04_STEP_PULSE_DELAY_MS > 0U
    /* Ensure minimum pulse width using the delay helpers */
    ISD04_DELAY_MS(ISD04_STEP_PULSE_DELAY_MS);
#endif
    if (!isd04_gpio_write_pin(driver->hw.stp_port, driver->hw.stp_pin, GPIO_PIN_RESET)) {
        driver->error = true;
        if (driver->callback) {
            driver->callback(ISD04_EVENT_ERROR, driver->callback_context);
        }
        isd04_unlock(driver);
        return;
    }
    driver->last_step_tick = ISD04_DELAY_START();
    isd04_unlock(driver);
}

void isd04_driver_register_callback(Isd04Driver *driver, Isd04EventCallback callback, void *context)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
    driver->callback = callback;
    driver->callback_context = context;
    isd04_unlock(driver);
}

Isd04StateId isd04_driver_get_state(Isd04Driver *driver)
{
    if (!driver) {
        return ISD04_STATE_STOPPED;
    }
    isd04_lock(driver);
    Isd04StateId id = driver->state ? driver->state->id : ISD04_STATE_STOPPED;
    isd04_unlock(driver);
    return id;
}

int32_t isd04_driver_get_position(Isd04Driver *driver)
{
    if (!driver) {
        return 0;
    }
    isd04_lock(driver);
    int32_t pos = driver->current_position;
    isd04_unlock(driver);
    return pos;
}

void isd04_driver_set_position(Isd04Driver *driver, int32_t position)
{
    if (!driver) {
        return;
    }
    isd04_lock(driver);
    if (driver->current_position != position) {
        driver->current_position = position;
        if (driver->callback) {
            /* Notify listeners of the new absolute position */
            driver->callback(ISD04_EVENT_POSITION_CHANGED, driver->callback_context);
        }
    }
    isd04_unlock(driver);
}

void isd04_driver_step(Isd04Driver *driver, int32_t steps)
{
    if (!driver || steps == 0) {
        return;
    }
    isd04_lock(driver);
    step_unlocked(driver, steps);
    isd04_unlock(driver);
}

static void step_unlocked(Isd04Driver *driver, int32_t steps)
{
    if (steps == 0) {
        return;
    }
    driver->current_position += steps;
    if (driver->callback) {
        /* Emit event each time position moves */
        driver->callback(ISD04_EVENT_POSITION_CHANGED, driver->callback_context);
    }
}

static int32_t clamp_speed(const Isd04Driver *driver, int32_t speed)
{
    if (speed > driver->config.max_speed) {
        return driver->config.max_speed;
    }

    if (speed < -driver->config.max_speed) {
        return -driver->config.max_speed;
    }

    return speed;
}

static Isd04Microstep clamp_microstep(Isd04Microstep mode)
{
    switch (mode) {
    case ISD04_MICROSTEP_200:
    case ISD04_MICROSTEP_400:
    case ISD04_MICROSTEP_800:
    case ISD04_MICROSTEP_1600:
    case ISD04_MICROSTEP_3200:
        return mode;
    default:
        if (mode < ISD04_MICROSTEP_200) {
            return ISD04_MICROSTEP_200;
        }
        return ISD04_MICROSTEP_3200;
    }
}

