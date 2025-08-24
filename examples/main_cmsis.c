/**
 * Example application controlling the ISD04 driver using CMSIS-RTOS v2.
 *
 * The driver manages step pulses in a background thread started via
 * isd04_driver_start(), while this example task adjusts speed and direction.
 *
 * Hardware connections:
 *   - STEP -> PA0 (GPIO output)
 *   - DIR  -> PA1 (GPIO output)
 *   - ENA  -> PA2 (GPIO output, active low)
 *
 * Build notes:
 *   - Configure the listed GPIO pins as push-pull outputs.
 *   - Ensure ISD04_USE_CMSIS is enabled in isd04_driver_config.h.
 */

#include "cmsis_os2.h"
#include "isd04_driver.h"

static void motor_task(void *argument)
{
    Isd04Driver *driver = (Isd04Driver *)argument;

    /* Start background step generation */
    isd04_driver_start(driver);

    /* Run forward at 100 RPM for 1 second */
    isd04_driver_set_direction(driver, true);
    isd04_driver_set_speed(driver, 100);
    osDelay(1000);

    /* Reverse and slow down to 50 RPM */
    isd04_driver_set_direction(driver, false);
    isd04_driver_set_speed(driver, 50);
    osDelay(1000);

    /* Stop motor */
    isd04_driver_stop(driver);

    for (;;) {
        osDelay(1000);
    }
}

int main(void)
{
    HAL_Init();
    /* GPIO initialization for PA0, PA1 and PA2 should be performed here. */

    Isd04Config config;
    isd04_driver_get_default_config(&config);

    Isd04Hardware hw = {
        .stp_port = GPIOA, .stp_pin = GPIO_PIN_0,
        .dir_port = GPIOA, .dir_pin = GPIO_PIN_1,
        .ena_port = GPIOA, .ena_pin = GPIO_PIN_2,
    };

    Isd04Driver *driver = isd04_driver_get_instance();
    isd04_driver_init(driver, &config, &hw);

    osKernelInitialize();
    osThreadNew(motor_task, driver, NULL);
    osKernelStart();

    for (;;) {
    }
}

