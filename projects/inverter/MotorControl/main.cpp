#define __MAIN_CPP__
#include "odrive_main.h"
#include "nvm_config.hpp"
#include <sketch.hpp>

#include "freertos_vars.h"
#include <communication/interface_usb.h>
//#include <communication/interface_uart.h>
#include <communication/interface_i2c.h>

communication communicable;

// Structs that store the configuration data for the controller
BoardConfig_t board_config;
SensorlessEstimator::Config_t sensorless_configs;
Controller::Config_t controller_configs;
Motor::Config_t motor_configs;
Axis::Config_t axis_configs;
bool user_config_loaded_;

SystemStats_t system_stats_ = { 0 };

// Motor & Controller Objects 
Axis *axes[AXIS_COUNT];

#ifdef __MIDI_HPP
Midi *midi_;
#endif

// Define what structs can be stored in NVM 
typedef Config<
    BoardConfig_t,
    Controller::Config_t,
    Motor::Config_t,
    Axis::Config_t
> ConfigFormat;

// Function to save configuration "ConfigFormat" to NVM
void save_configuration(void) {

    if (ConfigFormat::safe_store_config(
            &board_config,
            &controller_configs,
            &motor_configs,
            &axis_configs
            )) {
        //printf("saving configuration failed\r\n"); osDelay(5);
    } 
    else {
        user_config_loaded_ = true;
    }
}

// Function to load configuration "ConfigFormat" to NVM
extern "C" int load_configuration(void) {

    volatile bool init_failed = NVM_init();
    volatile bool load_failed = ConfigFormat::safe_load_config(
        &board_config,
        &controller_configs,
        &motor_configs,
        &axis_configs
    );

    // Try to load configs
    if (init_failed || load_failed) {
        
        // If loading failed, restore defaults
        board_config = BoardConfig_t();

        controller_configs = Controller::Config_t();
        motor_configs = Motor::Config_t();
        axis_configs = Axis::Config_t();
    } 
    else {
        // Loaded config from NVM succesfully
        user_config_loaded_ = true;
    }
    return user_config_loaded_;
}

// Function to erase configuration "ConfigFormat" from NVM
void erase_configuration(void) {
    NVM_erase();
}

// Function for device firmware update over USB
void enter_dfu_mode() {
    if ((hw_version_major == 3) && (hw_version_minor >= 5)) {
        __asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        _reboot_cookie = 0xDEADBEEF;
        NVIC_SystemReset();
    } 
    else {
        /*
        * DFU mode is only allowed on board version >= 3.5 because it can burn
        * the brake resistor FETs on older boards.
        * If you really want to use it on an older board, add 3.3k pull-down resistors
        * to the AUX_L and AUX_H signals and _only then_ uncomment these lines.
        */
        //__asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        //_reboot_cookie = 0xDEADFE75;
        //NVIC_SystemReset();
    }
}

extern "C" {
    int inverter_main(void);
    void vApplicationStackOverflowHook(void) {
        for (;;); // TODO: safe action
    }
    void vApplicationIdleHook(void) {
        if (system_stats_.fully_booted) {
            system_stats_.uptime = xTaskGetTickCount();
            system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();
            system_stats_.min_stack_space_comms = uxTaskGetStackHighWaterMark(comm_thread) * sizeof(StackType_t);
            system_stats_.min_stack_space_axis0 = uxTaskGetStackHighWaterMark(axes[0]->thread_id_) * sizeof(StackType_t);
            system_stats_.min_stack_space_usb = uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
            //system_stats_.min_stack_space_uart = uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
            system_stats_.min_stack_space_usb_irq = uxTaskGetStackHighWaterMark(usb_irq_thread) * sizeof(StackType_t);
            system_stats_.min_stack_space_startup = uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
        }
    }
}

int inverter_main(void) {

    //MX_CAN1_Init();

    // Init general user ADC on some GPIOs.
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_1_Pin;
    HAL_GPIO_Init(GPIO_1_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_2_Pin;
    HAL_GPIO_Init(GPIO_2_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_3_Pin;
    HAL_GPIO_Init(GPIO_3_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_4_Pin;
    HAL_GPIO_Init(GPIO_4_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_5_Pin;
    HAL_GPIO_Init(GPIO_5_GPIO_Port, &GPIO_InitStruct);

    // Construct all objects.
    #ifdef __MIDI_HPP
    midi_ = new Midi(midi_config);
    #endif

    SensorlessEstimator *sensorless_estimator = new SensorlessEstimator(sensorless_configs);
    
    Controller *controller = new Controller(controller_configs);
    Motor *motor = new Motor(hw_configs[0].motor_config,
                                hw_configs[0].gate_driver_config,
                                motor_configs);

    axes[0] = new Axis(hw_configs[0].axis_config, axis_configs,
            *sensorless_estimator, *controller, *motor);
    
    // Start ADC for temperature measurements and user measurements
    start_general_purpose_adc();

    // TODO: make dynamically reconfigurable
    if (board_config.enable_uart) {
        SetGPIO12toUART();
    }

    osDelay(100);

    // Init communications (this requires the axis objects to be constructed)
    init_communication();

    // Start pwm-in compare modules
    // must happen after communication is initialized
    start_pwm_input_capture();

    // Setup hardware for all components
    axes[0]->setup();

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    // This delay serves two purposes:
    //  - Let the current sense calibration converge (the current
    //    sense interrupts are firing in background by now)
    //  - Allow a user to interrupt the code, e.g. by flashing a new code,
    //    before it does anything crazy
    // TODO make timing a function of calibration filter tau
    osDelay(1500);

    // Start state machine threads. Each thread will go through various calibration
    // procedures and then run the actual controller loops.
    axes[0]->start_thread();

    // Start thread to sample analog input pins
    start_analog_thread();

    #ifdef __MIDI_HPP
    midi_->start_thread(); 
    #endif

    system_stats_.fully_booted = true;
    return 0;
}
