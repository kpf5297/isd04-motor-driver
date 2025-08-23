# ISD04 Motor Driver

Portable C driver library for the ISD04 stepper motor driver IC with optional STM32 HAL or CMSIS-RTOS integration configured via macros.

## Hardware Setup

Connect your microcontroller GPIO pins to the ISD04 control inputs:
- **STEP**: Pulse high-to-low to advance one microstep
- **DIR**: High = forward, Low = reverse  
- **ENA**: Low = enabled, High = disabled (configurable via `ISD04_ENA_ACTIVE_LEVEL`)

## Usage

```c
#include "isd04_driver.h"

// Initialize driver
Isd04Config config;
isd04_driver_get_default_config(&config);

Isd04Hardware hw = {
    .stp_port = GPIOA, .stp_pin = GPIO_PIN_0,
    .dir_port = GPIOA, .dir_pin = GPIO_PIN_1,
    .ena_port = GPIOA, .ena_pin = GPIO_PIN_2,
};

Isd04Driver *driver = isd04_driver_get_instance();
isd04_driver_init(driver, &config, &hw);

// Control motor
isd04_driver_enable(driver, true);
isd04_driver_set_direction(driver, true);  // forward
isd04_driver_start(driver);
isd04_driver_set_speed(driver, 100);

// Generate step pulses (call from timer interrupt)
isd04_driver_pulse(driver);
```

## Key Features

- **Thread-safe**: All public APIs are mutex-protected for multi-threaded applications
- **Platform support**: Optional STM32 HAL or CMSIS-RTOS v2 integration selected via config macros with fallback implementations for testing
- **Timer integration**: Hardware timer support for precise step pulse generation
- **Error handling**: Built-in GPIO validation and error reporting via event callbacks
- **Configurable timing**: Adjustable step intervals and wake-up delays

## Configuration

All compile-time options live in `src/isd04_driver_config.h`. Enable features by
setting the macros below to `1` or disable them with `0`; no automatic detection
is performed.

Key options include:
- `ISD04_USE_CMSIS`: integrate with CMSIS-RTOS v2 (default: 0)
- `ISD04_USE_HAL`: use STM32 HAL for GPIO/delay support (default: 0)
- `ISD04_STEP_CONTROL_TIMER`: drive STEP via a bound hardware timer instead of toggling the GPIO directly (default: 0)
- `ISD04_ENA_ACTIVE_LEVEL`: enable pin polarity (default: active-low)
- `ISD04_STEP_MIN_INTERVAL_US`: minimum step pulse interval (default: 4μs)
- `ISD04_ENABLE_WAKE_DELAY_MS`: driver wake-up delay (default: 1ms)
- `ISD04_GPIO_PIN_COUNT`: GPIO pin validation range (default: 16)

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
