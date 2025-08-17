#include "isd04_driver.h"

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
}

void isd04_driver_start(Isd04Driver *driver)
{
    if (!driver) {
        return;
    }

    driver->running = true;
}

void isd04_driver_stop(Isd04Driver *driver)
{
    if (!driver) {
        return;
    }

    driver->running = false;
    driver->current_speed = 0;
}

void isd04_driver_set_speed(Isd04Driver *driver, int32_t speed)
{
    if (!driver) {
        return;
    }

    driver->current_speed = clamp_speed(driver, speed);
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

