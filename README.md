# ISD04 Motor Driver

Portable C driver library for the ISD04 stepper motor driver IC with optional STM32 HAL or CMSIS-RTOS integration configured via macros.

## Hardware Setup

Connect your microcontroller GPIO pins to the ISD04 control inputs:
- **STEP**: Pulse high-to-low to advance one microstep
- **DIR**: High = forward, Low = reverse  
- **ENA**: Low = enabled, High = disabled (configurable via `ISD04_ENA_ACTIVE_LEVEL`)

## Usage
Generate STEP pulses directly via GPIO:

```c
#include "isd04_driver.h"

Isd04Config config;
isd04_driver_get_default_config(&config);

Isd04Hardware hw = {
    .stp_port = GPIOA, .stp_pin = GPIO_PIN_0,
    .dir_port = GPIOA, .dir_pin = GPIO_PIN_1,
    .ena_port = GPIOA, .ena_pin = GPIO_PIN_2,
};

Isd04Driver *driver = isd04_driver_get_instance();
isd04_driver_init(driver, &config, &hw);

// Enable outputs and set direction
isd04_driver_enable(driver, true);
isd04_driver_set_direction(driver, true); // forward

// Generate pulses manually
for (int i = 0; i < 200; ++i) {
    isd04_driver_pulse(driver);
    ISD04_DELAY_MS(1);
}
```

`max_speed` is the largest step rate the driver accepts (in steps per second).
`isd04_driver_set_speed()` specifies a signed target rate within
`[-max_speed, max_speed]`, and the driver generates steps at this rate while
running.

### CMSIS-RTOS task

When `ISD04_USE_CMSIS` is enabled the driver can generate step pulses from a
background thread. Call `isd04_driver_start()` once the RTOS is running to
launch the internal task and use `isd04_driver_set_speed()` to update the step
rate while the motor is running. `isd04_driver_stop()` halts the task.

```c
#include "cmsis_os2.h"
#include "isd04_driver.h"

static void motor_task(void *argument) {
    Isd04Driver *driver = argument;

    isd04_driver_start(driver);            // start internal step thread
    isd04_driver_set_speed(driver, 100);   // run forward
    osDelay(1000);

    isd04_driver_set_speed(driver, -100);  // reverse direction
    osDelay(1000);

    isd04_driver_stop(driver);             // stop motor
}

int main(void) {
    HAL_Init();
    // GPIO configuration omitted

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

    for (;;) { /* Application code */ }
}
```

## Key Features

- **Thread-safe**: All public APIs are mutex-protected for multi-threaded applications
- **Platform support**: Optional STM32 HAL or CMSIS-RTOS v2 integration selected via config macros with fallback implementations for testing
- **Error handling**: Built-in GPIO validation and error reporting via event callbacks
- **Configurable timing**: Adjustable step intervals and wake-up delays

## Configuration

All compile-time options live in `src/isd04_driver_config.h`. Enable features by
setting the macros below to `1` or disable them with `0`; no automatic detection
is performed.

Key options include:
- `ISD04_USE_CMSIS`: integrate with CMSIS-RTOS v2 (default: 0)
- `ISD04_USE_HAL`: use STM32 HAL for GPIO/delay support (default: 0)
- `ISD04_ENA_ACTIVE_LEVEL`: enable pin polarity (default: active-low)
- `ISD04_STEP_MIN_INTERVAL_US`: minimum step pulse interval (default: 4μs)
- `ISD04_ENABLE_WAKE_DELAY_MS`: driver wake-up delay (default: 1ms)
- `ISD04_GPIO_PIN_COUNT`: GPIO pin validation range (default: 16)

## Examples

A reference application is provided in the `examples` directory:

- `examples/main_gpio.c` demonstrates manual STEP pulses via GPIO.
- `examples/main_cmsis.c` runs the driver from a CMSIS-RTOS thread and updates speed and direction.

## API Reference

**Core Functions:**
- `isd04_driver_get_instance()` - Get singleton driver instance
- `isd04_driver_init(driver, config, hw)` - Initialize with hardware configuration
- `isd04_driver_enable(driver, enable)` - Enable/disable driver outputs
- `isd04_driver_pulse(driver)` - Generate single step pulse
- `isd04_driver_set_speed(driver, speed)` - Set target speed
- `isd04_driver_get_version()` - Get library version string

## License

MIT License - see LICENSE file for details.
