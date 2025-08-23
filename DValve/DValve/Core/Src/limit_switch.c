#include "limit_switch.h"
#include "main.h"
#include "cmsis_os.h"

/* RTOS objects for limit switch handling */
osThreadId_t limitSwitchTaskHandle;
const osThreadAttr_t limitSwitchTask_attributes = {
  .name = "limitSwitchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osEventFlagsId_t limitSwitchEventsHandle;
const osEventFlagsAttr_t limitSwitchEvents_attributes = {
  .name = "limitSwitchEvents"
};

/* Private variables */
static LimitSwitchStatus_t limit_switch_status = {
    .top_limit_active = false,
    .bottom_limit_active = false,
    .limit_switch_enabled = true
};

static osMutexId_t limit_switch_mutex = NULL;
static const osMutexAttr_t limit_switch_mutex_attr = {
    .name = "limitSwitchMutex"
};

/**
 * @brief Initialize the limit switch module
 */
void limit_switch_init(void)
{
    /* Create mutex for thread-safe access to status */
    limit_switch_mutex = osMutexNew(&limit_switch_mutex_attr);
    
    /* Initialize status */
    osMutexAcquire(limit_switch_mutex, osWaitForever);
    limit_switch_status.top_limit_active = false;
    limit_switch_status.bottom_limit_active = false;
    limit_switch_status.limit_switch_enabled = true;
    osMutexRelease(limit_switch_mutex);
}

/**
 * @brief Limit switch task implementation
 * @param argument: Not used
 */
void limit_switch_task(void *argument)
{
    UNUSED(argument);
    
    /* Infinite loop */
    for(;;)
    {
        /* Wait for limit switch events */
        uint32_t events = osEventFlagsWait(limitSwitchEventsHandle, 
                                         TOP_LIMIT_EVENT | BOTTOM_LIMIT_EVENT,
                                         osFlagsWaitAny,
                                         osWaitForever);
        
        if (!limit_switch_status.limit_switch_enabled) {
            continue; /* Skip processing if limit switches are disabled */
        }
        
        osMutexAcquire(limit_switch_mutex, osWaitForever);
        
        if (events & TOP_LIMIT_EVENT) {
            /* Top limit switch triggered */
            limit_switch_status.top_limit_active = true;
            
            /* Add your limit switch handling code here */
            /* For example: stop motor, reverse direction, etc. */
            
            /* Clear the flag after processing */
            limit_switch_status.top_limit_active = false;
        }
        
        if (events & BOTTOM_LIMIT_EVENT) {
            /* Bottom limit switch triggered */
            limit_switch_status.bottom_limit_active = true;
            
            /* Add your limit switch handling code here */
            /* For example: stop motor, reverse direction, etc. */
            
            /* Clear the flag after processing */
            limit_switch_status.bottom_limit_active = false;
        }
        
        osMutexRelease(limit_switch_mutex);
        
        /* Small delay to prevent task hogging */
        osDelay(10);
    }
}

/**
 * @brief GPIO interrupt callback for limit switches
 * @param GPIO_Pin: Pin that triggered the interrupt
 */
void limit_switch_gpio_callback(uint16_t GPIO_Pin)
{
    if (!limit_switch_status.limit_switch_enabled) {
        return; /* Ignore if limit switches are disabled */
    }
    
    if (GPIO_Pin == TOP_LIMIT_Pin) {
        /* Top limit switch triggered (falling edge) */
        osEventFlagsSet(limitSwitchEventsHandle, TOP_LIMIT_EVENT);
    } 
    else if (GPIO_Pin == BOTTOM_LIMIT_Pin) {
        /* Bottom limit switch triggered (falling edge) */
        osEventFlagsSet(limitSwitchEventsHandle, BOTTOM_LIMIT_EVENT);
    }
}

/**
 * @brief Check if top limit switch is active
 * @return true if top limit is active, false otherwise
 */
bool limit_switch_is_top_active(void)
{
    bool status = false;
    if (limit_switch_mutex != NULL) {
        osMutexAcquire(limit_switch_mutex, osWaitForever);
        status = limit_switch_status.top_limit_active;
        osMutexRelease(limit_switch_mutex);
    }
    return status;
}

/**
 * @brief Check if bottom limit switch is active
 * @return true if bottom limit is active, false otherwise
 */
bool limit_switch_is_bottom_active(void)
{
    bool status = false;
    if (limit_switch_mutex != NULL) {
        osMutexAcquire(limit_switch_mutex, osWaitForever);
        status = limit_switch_status.bottom_limit_active;
        osMutexRelease(limit_switch_mutex);
    }
    return status;
}

/**
 * @brief Enable or disable limit switch functionality
 * @param enable: true to enable, false to disable
 */
void limit_switch_enable(bool enable)
{
    if (limit_switch_mutex != NULL) {
        osMutexAcquire(limit_switch_mutex, osWaitForever);
        limit_switch_status.limit_switch_enabled = enable;
        osMutexRelease(limit_switch_mutex);
    }
}

/**
 * @brief Get the current limit switch status
 * @return LimitSwitchStatus_t containing current status
 */
LimitSwitchStatus_t limit_switch_get_status(void)
{
    LimitSwitchStatus_t status = {0};
    if (limit_switch_mutex != NULL) {
        osMutexAcquire(limit_switch_mutex, osWaitForever);
        status = limit_switch_status;
        osMutexRelease(limit_switch_mutex);
    }
    return status;
}
