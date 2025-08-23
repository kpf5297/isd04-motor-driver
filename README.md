# ISD04 Motor Driver

Portable C driver library for the ISD04 stepper motor driver IC with STM32 HAL integration.

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
- **Platform support**: STM32 HAL and CMSIS-RTOS v2 integration with fallback for testing  
- **Timer integration**: Hardware timer support for precise step pulse generation
- **Error handling**: Built-in GPIO validation and error reporting via event callbacks
- **Configurable timing**: Adjustable step intervals and wake-up delays

## Configuration

Key compile-time options:
- `ISD04_ENA_ACTIVE_LEVEL`: Enable pin polarity (default: active-low)
- `ISD04_STEP_MIN_INTERVAL_US`: Minimum step pulse interval (default: 4μs)
- `ISD04_ENABLE_WAKE_DELAY_MS`: Driver wake-up delay (default: 1ms)
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