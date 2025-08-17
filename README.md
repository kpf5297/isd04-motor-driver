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

    isd04_driver_start(driver);
    isd04_driver_set_speed(driver, 50);

    /* query current behavioral state */
    Isd04StateId state = isd04_driver_get_state(driver);
    (void)state;

    /* ... */

    isd04_driver_stop(driver);
    return 0;
}
```
