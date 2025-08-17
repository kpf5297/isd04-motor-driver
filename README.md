# isd04-motor-driver
Portable C driver for the ISD04 motor driver IC with STM32 HAL reference port, example projects, and Python-based telemetry visualization.

## Usage

```c
#include "isd04_driver.h"

static void on_event(Isd04Event event, void *context) {
    (void)context;
    /* react to driver events */
}

int main(void) {
    Isd04Config config = {
        .pwm_frequency_hz = 1000,
        .max_speed = 100,
    };

    Isd04Driver *driver = isd04_driver_get_instance();
    isd04_driver_init(driver, &config);
    isd04_driver_register_callback(driver, on_event, NULL);

    /* configure microstepping */
    isd04_driver_set_microstep(driver, ISD04_MICROSTEP_1600);
    Isd04Microstep ms = isd04_driver_get_microstep(driver);
    (void)ms;

    isd04_driver_start(driver);
    isd04_driver_set_speed(driver, 50);

    /* periodically advance position and read it */
    isd04_driver_step(driver, 1); /* call from a timer/interrupt */
    int32_t pos = isd04_driver_get_position(driver);
    (void)pos;

    /* ... */

    isd04_driver_stop(driver);
    return 0;
}
```

To keep the driver's `current_position` in sync with actual motor motion, call
`isd04_driver_step` from a periodic timer or interrupt whenever step pulses are
issued. This allows real-time position queries via `isd04_driver_get_position`.
