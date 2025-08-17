#include "isd04_driver.h"
#include <stddef.h>
#include <stdlib.h>

/** Singleton driver instance. */
static Isd04Driver *instance = NULL;

/** Clamp speed to the allowable range. */
static int32_t clamp_speed(const Isd04Driver *driver, int32_t speed);
/** Clamp microstep mode to the allowable range. */
static Isd04Microstep clamp_microstep(Isd04Microstep mode);

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
    (void)driver; /* already stopped */
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
    (void)driver; /* already running */
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

Isd04Driver *isd04_driver_get_instance(void)
{
    if (instance) {
        return instance;
    }

    instance = calloc(1, sizeof(Isd04Driver));
    return instance;
}

void isd04_driver_get_default_config(Isd04Config *config)
{
    if (!config) {
        return;
    }

    config->pwm_frequency_hz = 20000U;
    config->max_speed = 1000;
    config->microstep = ISD04_MICROSTEP_3200;
    config->phase_current_ma = 2800U;
}

void isd04_driver_init(Isd04Driver *driver, const Isd04Config *config, const Isd04Hardware *hw)
{
    if (!driver || !config || !hw) {
        return;
    }

    driver->config = *config;
    driver->hw = *hw;
    driver->current_speed = 0;
    driver->current_position = 0;
    driver->running = false;
    driver->callback = NULL;
    driver->callback_context = NULL;
    driver->microstep = config->microstep;
    ISD04_APPLY_MICROSTEP(ISD04_MICROSTEP_TO_BITS(driver->microstep));
    HAL_GPIO_WritePin(driver->hw.ena_port, driver->hw.ena_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(driver->hw.dir_port, driver->hw.dir_pin, GPIO_PIN_RESET);
    change_state(driver, &stopped_state);
}

void isd04_driver_start(Isd04Driver *driver)
{
    if (!driver || !driver->state) {
        return;
    }

    driver->state->start(driver);
}

void isd04_driver_stop(Isd04Driver *driver)
{
    if (!driver || !driver->state) {
        return;
    }

    driver->state->stop(driver);
}

void isd04_driver_set_speed(Isd04Driver *driver, int32_t speed)
{
    if (!driver || !driver->state) {
        return;
    }

    driver->state->set_speed(driver, speed);
}

void isd04_driver_set_microstep(Isd04Driver *driver, Isd04Microstep mode)
{
    if (!driver) {
        return;
    }

    Isd04Microstep new_mode = clamp_microstep(mode);
    if (new_mode != driver->microstep) {
        ISD04_APPLY_MICROSTEP(ISD04_MICROSTEP_TO_BITS(new_mode));
        driver->microstep = new_mode;
        if (driver->callback) {
            /* inform user code that the microstepping mode was updated */
            driver->callback(ISD04_EVENT_MICROSTEP_CHANGED, driver->callback_context);
        }
    }
}

Isd04Microstep isd04_driver_get_microstep(const Isd04Driver *driver)
{
    if (!driver) {
        return ISD04_MICROSTEP_200;
    }

    return driver->microstep;
}

void isd04_driver_set_direction(Isd04Driver *driver, bool forward)
{
    if (!driver) {
        return;
    }
    HAL_GPIO_WritePin(driver->hw.dir_port, driver->hw.dir_pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void isd04_driver_enable(Isd04Driver *driver, bool enable)
{
    if (!driver) {
        return;
    }
    HAL_GPIO_WritePin(driver->hw.ena_port, driver->hw.ena_pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void isd04_driver_pulse(Isd04Driver *driver)
{
    if (!driver) {
        return;
    }
    HAL_GPIO_WritePin(driver->hw.stp_port, driver->hw.stp_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(driver->hw.stp_port, driver->hw.stp_pin, GPIO_PIN_RESET);
    isd04_driver_step(driver, 1);
}

void isd04_driver_register_callback(Isd04Driver *driver, Isd04EventCallback callback, void *context)
{
    if (!driver) {
        return;
    }

    driver->callback = callback;
    driver->callback_context = context;
}

Isd04StateId isd04_driver_get_state(const Isd04Driver *driver)
{
    if (!driver || !driver->state) {
        return ISD04_STATE_STOPPED;
    }
    return driver->state->id;
}

int32_t isd04_driver_get_position(const Isd04Driver *driver)
{
    if (!driver) {
        return 0;
    }
    return driver->current_position;
}

void isd04_driver_set_position(Isd04Driver *driver, int32_t position)
{
    if (!driver) {
        return;
    }
    if (driver->current_position != position) {
        driver->current_position = position;
        if (driver->callback) {
            /* notify listeners of the new absolute position */
            driver->callback(ISD04_EVENT_POSITION_CHANGED, driver->callback_context);
        }
    }
}

void isd04_driver_step(Isd04Driver *driver, int32_t steps)
{
    if (!driver || steps == 0) {
        return;
    }
    driver->current_position += steps;
    if (driver->callback) {
        /* emit event each time position moves */
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

