# isd04-motor-driver
Portable C driver for the ISD04 motor driver IC with STM32 HAL reference port, example projects, and Python-based telemetry visualization.

## Versioning
Call `isd04_driver_get_version()` to query the driver's semantic version string:

```c
#include "isd04_driver.h"

const char *version = isd04_driver_get_version();
```

## Hardware Setup

Connect the microcontroller's STEP, DIR, and ENA signals to the ISD04 control
inputs on connector P2 as described in the datasheet. P2 exposes each signal as
a differential pair; tie the negative pins to ground and drive the positive
pins from the MCU's GPIO outputs.

All three logic inputs default high when left floating. Drive STEP high then
low to advance a microstep, set DIR high for forward rotation and low for
reverse, and pull ENA low to enable the outputs (high disables the driver).

## Usage

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

## Timer Integration

The driver can be stepped from a periodic timer interrupt. The example below
configures a 1 kHz update timer using the STM32 HAL and emits a pulse from the
callback:

```c
static TIM_HandleTypeDef htim2;

void isd04_timer_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000U) - 1U; // 1 MHz base
    htim2.Init.Period = 1000U - 1U; // 1 kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    HAL_TIM_Base_Init(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        isd04_driver_pulse(driver);
        /* optionally send telemetry */
        // isd04_driver_send_telemetry(driver);
    }
}
```

`isd04_driver_pulse` honours the `ISD04_STEP_MIN_INTERVAL_US` macro, ensuring
successive pulses respect the datasheet's minimum step period. When enabling the
driver, `ISD04_ENABLE_WAKE_DELAY_MS` inserts the mandated wake delay before the
first pulse. These guards keep the system within the timing limits specified by
the ISD04 datasheet.

## Hardware validation and error handling

The driver validates that the provided GPIO pins fall within the target MCU's
pin count. Override `ISD04_GPIO_PIN_COUNT` at compile time if your platform
exposes a different number of pins per port. Applications may use the
`ISD04_VALIDATE_PIN(pin)` macro for their own static assertions.

When built against the STM32 HAL the library assumes `HAL_GPIO_WritePin`
succeeds for valid ports. The driver wraps this call to check port pointers and
sets an internal error flag while emitting `ISD04_EVENT_ERROR` if a validation
step fails or a write cannot be performed. Applications should supply valid
hardware definitions and handle the error event to detect faults.

`isd04_driver_pulse` toggles the step pin and updates the driver's internal
position counter. If step pulses are produced elsewhere, call
`isd04_driver_step` to keep the tracked position synchronized.

## Timing helpers

The driver exposes macros for integrating with platform-specific delay mechanisms:

* `ISD04_DELAY_MS(ms)` – blocking delay for a number of milliseconds.
* `ISD04_DELAY_US(us)` – blocking delay for a number of microseconds.
* `ISD04_DELAY_START()` / `ISD04_DELAY_ELAPSED(start, ms)` – capture a tick
  count and test whether a duration has passed without blocking. For example:

```c
Isd04DelayTick start = ISD04_DELAY_START();
while (!ISD04_DELAY_ELAPSED(start, 10)) {
    /* perform other tasks */
}
```

These macros map to `HAL_Delay`/`HAL_GetTick` when `USE_HAL_DRIVER` is defined
and to `osDelay`/`osKernelSysTick` when `CMSIS_OS_VERSION` is defined. When
neither symbol is present they become no-ops so the driver can be built for host
tests. Projects may also set `ISD04_STEP_PULSE_DELAY_MS` to enforce a minimum
step pulse width using the delay helpers. Additional tuning is available via
`ISD04_STEP_MIN_INTERVAL_US` to specify a low-level pulse width and
`ISD04_ENABLE_WAKE_DELAY_MS` to insert a delay after enabling the driver.

## Version history

* **1.0.1** – Added pin range validation and `ISD04_VALIDATE_PIN` macro.
