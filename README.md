# isd04-motor-driver
Portable C driver for the ISD04 motor driver IC with STM32 HAL reference port, example projects, and Python-based telemetry visualization.

## Usage
Call `isd04_driver_get_version()` to retrieve the driver's version string.

```c
#include "isd04_driver.h"

static void on_event(Isd04Event event, void *context) {
    Isd04Driver *driver = (Isd04Driver *)context;

    switch (event) {
    case ISD04_EVENT_MICROSTEP_CHANGED:
        /* react to microstep changes */
        (void)isd04_driver_get_microstep(driver);
        break;
    case ISD04_EVENT_POSITION_CHANGED:
        /* respond to position updates */
        (void)isd04_driver_get_position(driver);
        break;
    default:
        break;
    }
}

int main(void) {
    const char *version = isd04_driver_get_version();
    (void)version;

    Isd04Config config;
    isd04_driver_get_default_config(&config);

    Isd04Hardware hw = {
        .stp_port = GPIOA,
        .stp_pin  = GPIO_PIN_0,
        .dir_port = GPIOA,
        .dir_pin  = GPIO_PIN_1,
        .ena_port = GPIOA,
        .ena_pin  = GPIO_PIN_2,
    };

    Isd04Driver *driver = isd04_driver_get_instance();
    isd04_driver_init(driver, &config, &hw);
    /* pass driver as context so the callback can query state */
    isd04_driver_register_callback(driver, on_event, driver);

    isd04_driver_enable(driver, true);
    isd04_driver_set_direction(driver, true);

    isd04_driver_start(driver);
    isd04_driver_set_speed(driver, 50);

    /* periodically issue a step pulse */
    isd04_driver_pulse(driver); /* call from a timer/interrupt */

    /* ... */

    isd04_driver_stop(driver);
    return 0;
}
```

`isd04_driver_pulse` toggles the step pin and updates the driver's internal
position counter. If step pulses are produced elsewhere, call
`isd04_driver_step` to keep the tracked position synchronized.
