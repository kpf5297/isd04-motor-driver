#ifndef ISD04_DRIVER_H
#define ISD04_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Configuration parameters for the ISD04 driver.
 */
typedef struct {
    /** PWM frequency in Hz. */
    uint32_t pwm_frequency_hz;
    /** Maximum speed value permitted. */
    int32_t max_speed;
} Isd04Config;

/**
 * Runtime state for an ISD04 motor driver instance.
 */
typedef struct {
    /** Static configuration for the device. */
    Isd04Config config;
    /** Current motor speed. */
    int32_t current_speed;
    /** Indicates whether the motor is running. */
    bool running;
} Isd04Driver;

void isd04_driver_init(Isd04Driver *driver, const Isd04Config *config);
void isd04_driver_start(Isd04Driver *driver);
void isd04_driver_stop(Isd04Driver *driver);
void isd04_driver_set_speed(Isd04Driver *driver, int32_t speed);

#ifdef __cplusplus
}
#endif

#endif /* ISD04_DRIVER_H */
