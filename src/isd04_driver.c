#include "isd04_driver.h"
#include <stddef.h>

/** Clamp speed to the allowable range. */
static int32_t clamp_speed(const Isd04Driver *driver, int32_t speed);

void isd04_driver_init(Isd04Driver *driver, const Isd04Config *config)
{
    if (!driver || !config) {
        return;
    }

    driver->config = *config;
    driver->current_speed = 0;
    driver->running = false;
    driver->callback = NULL;
    driver->callback_context = NULL;
}

void isd04_driver_start(Isd04Driver *driver)
{
    if (!driver) {
        return;
    }

    driver->running = true;
    if (driver->callback) {
        driver->callback(ISD04_EVENT_STARTED, driver->callback_context);
    }
}

void isd04_driver_stop(Isd04Driver *driver)
{
    if (!driver) {
        return;
    }

    driver->running = false;
    driver->current_speed = 0;
    if (driver->callback) {
        driver->callback(ISD04_EVENT_STOPPED, driver->callback_context);
    }
}

void isd04_driver_set_speed(Isd04Driver *driver, int32_t speed)
{
    if (!driver) {
        return;
    }
    int32_t new_speed = clamp_speed(driver, speed);
    if (new_speed != driver->current_speed) {
        driver->current_speed = new_speed;
        if (driver->callback) {
            driver->callback(ISD04_EVENT_SPEED_CHANGED, driver->callback_context);
        }
    }
}

void isd04_driver_register_callback(Isd04Driver *driver, Isd04EventCallback callback, void *context)
{
    if (!driver) {
        return;
    }

    driver->callback = callback;
    driver->callback_context = context;
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

