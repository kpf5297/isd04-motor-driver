/**
 * Example application demonstrating hardware timer control of the STEP pin.
 *
 * Hardware connections:
 *   - STEP -> PA0 (TIM2_CH1 configured for PWM)
 *   - DIR  -> PA1 (GPIO output)
 *   - ENA  -> PA2 (GPIO output, active low)
 *
 * Timer setup:
 *   - TIM2 base clock set as needed for desired step frequency.
 *   - Channel 1 configured in PWM mode 1 with 50% duty cycle.
 *   - Update interrupt enabled if the application needs to react to each pulse.
 *
 * Build notes:
 *   - Ensure ISD04_STEP_CONTROL_TIMER is set to 1 in isd04_driver_config.h.
 *   - Provide and initialize TIM2 before running this example.
 */

#include "isd04_driver.h"

// Timer handle configured elsewhere (e.g. CubeMX) for PWM output on STEP
extern TIM_HandleTypeDef htim2;

// Optional timer interrupt callback. The driver does not require it but
// applications can hook here to monitor pulses or update motion profiles.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2) {
        // Timer period elapsed: place application-specific code here.
    }
}

int main(void)
{
    HAL_Init();
    // System clock and GPIO initialization should occur here.

    Isd04Config config;
    isd04_driver_get_default_config(&config);

    Isd04Hardware hw = {
        .dir_port = GPIOA, .dir_pin = GPIO_PIN_1,
        .ena_port = GPIOA, .ena_pin = GPIO_PIN_2,
    };

    Isd04Driver *driver = isd04_driver_get_instance();
    isd04_driver_init(driver, &config, &hw);
    isd04_driver_bind_step_timer(driver, &htim2);

    // Enable outputs and start motion at 50% of maximum speed
    isd04_driver_enable(driver, true);
    isd04_driver_set_direction(driver, true); // forward
    isd04_driver_set_speed(driver, config.max_speed / 2);
    isd04_driver_start(driver);

    while (1) {
        // Main loop can perform other tasks while timer drives the motor.
    }
}

