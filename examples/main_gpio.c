/**
 * Example application toggling the STEP pin directly via GPIO.
 *
 * Hardware connections:
 *   - STEP -> PA0 (GPIO output)
 *   - DIR  -> PA1 (GPIO output)
 *   - ENA  -> PA2 (GPIO output, active low)
 *
 * Build notes:
 *   - Set ISD04_STEP_CONTROL_TIMER to 0 in isd04_driver_config.h.
 *   - Configure the listed GPIO pins as push-pull outputs.
 */

#include "isd04_driver.h"

int main(void)
{
    HAL_Init();
    // GPIO initialization should configure PA0, PA1 and PA2 as outputs.

    Isd04Config config;
    isd04_driver_get_default_config(&config);

    Isd04Hardware hw = {
        .stp_port = GPIOA, .stp_pin = GPIO_PIN_0,
        .dir_port = GPIOA, .dir_pin = GPIO_PIN_1,
        .ena_port = GPIOA, .ena_pin = GPIO_PIN_2,
    };

    Isd04Driver *driver = isd04_driver_get_instance();
    isd04_driver_init(driver, &config, &hw);

    // Enable outputs and set initial direction
    isd04_driver_enable(driver, true);
    isd04_driver_set_direction(driver, true); // forward

    while (1) {
        // Generate one step pulse manually
        isd04_driver_pulse(driver);
        // Delay controls step frequency (adjust as needed)
        ISD04_DELAY_US(500);
    }
}

