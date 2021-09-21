
#include <stdlib.h>
#include <functional>
#include "gpio.h"

#include "utils.h"
#include "odrive_main.h"


// =========================================================================
Axis::Axis(const AxisHardwareConfig_t& hw_config,
           Config_t& config,
           SensorlessEstimator& sensorless_estimator,
           Controller& controller,
           Motor& motor)
    : hw_config_(hw_config),
      config_(config),
      sensorless_estimator_(sensorless_estimator),
      controller_(controller),
      motor_(motor)
{
    controller_.axis_ = this;
    motor_.axis_ = this;

    update_watchdog_settings();
}

// =========================================================================
// @brief Sets up all components of the axis,
// such as gate driver and encoder hardware.
void Axis::setup() {
    //encoder_.setup();
    motor_.setup();
}


// =========================================================================
static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->run_state_machine_loop();
    reinterpret_cast<Axis*>(ctx)->thread_id_valid_ = false;
}


// =========================================================================
// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread() {
    osThreadDef(axis_thread, run_state_machine_loop_wrapper, hw_config_.thread_priority, 0, 5*512 + 256);
    thread_id_ = osThreadCreate(osThread(axis_thread), this);
    thread_id_valid_ = true;
}


// =========================================================================
// @brief Unblocks the control loop thread.
// This is called from the current sense interrupt handler.
void Axis::signal_current_meas() {
    if (thread_id_valid_)
        osSignalSet(thread_id_, M_SIGNAL_PH_CURRENT_MEAS);
}


// =========================================================================
// @brief Blocks until a current measurement is completed
// @returns True on success, false otherwise
bool Axis::wait_for_current_meas() {
    return osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status == osEventSignal;
}


// =========================================================================
// @brief: Setup the watchdog reset value from the configuration watchdog timeout interval. 
void Axis::update_watchdog_settings() {

    if(config_.watchdog_timeout <= 0.0f) { // watchdog disabled 
        watchdog_reset_value_ = 0;
    } 
    
    else if(config_.watchdog_timeout >= UINT32_MAX / (current_meas_hz+1)) { //overflow! 
        watchdog_reset_value_ = UINT32_MAX;
    } 
    
    else {
        watchdog_reset_value_ = static_cast<uint32_t>(config_.watchdog_timeout * current_meas_hz);
    }

    // Do a feed to avoid instant timeout
    watchdog_feed();
}


// =========================================================================
// @brief Do axis level checks and call subcomponent do_checks
// Returns true if everything is ok.
bool Axis::do_checks() {

    if (!brake_resistor_armed) {
        error_ |= ERROR_BRAKE_RESISTOR_DISARMED;
    }

    if ((current_state_ != AXIS_STATE_IDLE) && (motor_.armed_state_ == Motor::ARMED_STATE_DISARMED)) {
        // motor got disarmed in something other than the idle loop
        error_ |= ERROR_MOTOR_DISARMED;
    }
    
    /*
    if (!(vbus_voltage >= board_config.dc_bus_undervoltage_trip_level)) {
        error_ |= ERROR_DC_BUS_UNDER_VOLTAGE;
    }
    */
    
    /*
    if (!(vbus_voltage <= board_config.dc_bus_overvoltage_trip_level)) {
        error_ |= ERROR_DC_BUS_OVER_VOLTAGE;
    }
    */

    // Sub-components should use set_error which will propegate to this error_
    motor_.do_checks();

    //encoder_.do_checks();
    // controller_.do_checks();

    return check_for_errors();
}


// =========================================================================
// @brief Update all esitmators
bool Axis::do_updates() {
    // Sub-components should use set_error which will propegate to this error_
    //encoder_.update();

    return check_for_errors();
}


// =========================================================================
// @brief Feed the watchdog to prevent watchdog timeouts.
void Axis::watchdog_feed() {
    watchdog_current_value_ = watchdog_reset_value_;
}


// =========================================================================
// @brief Check the watchdog timer for expiration. Also sets the watchdog error bit if expired. 
bool Axis::watchdog_check() {
    // reset value = 0 means watchdog disabled. 
    if(watchdog_reset_value_ == 0) return true;

    // explicit check here to ensure that we don't underflow back to UINT32_MAX
    if(watchdog_current_value_ > 0) {
        watchdog_current_value_--;
        return true;
    } 
    
    else {
        error_ |= ERROR_WATCHDOG_TIMER_EXPIRED;
        return false;
    }
}


// =========================================================================
// Note run_inverter_loop and run_closed_loop_control_loop are very similar 
// and differ only in where we get the estimate from.
bool Axis::run_inverter_loop() {

    run_control_loop([this]() {

    
        float phase_velocity = base_frequency_ * M_PI * 2.0f;

        // Advance our phase angle by dt
        phase_angle_ = wrap_pm_pi(phase_angle_ + phase_velocity * current_meas_period);

        float current_output;
        float voltage_modulation_magnitude;

        // Update our controller
        if (!motor_.update_controller(
                phase_velocity, 
                phase_angle_, 
                &current_output, 
                &voltage_modulation_magnitude)
            ) {
            // Return if error
            return error_ |= ERROR_CONTROLLER_FAILED, false;
        }

        // Now try to update our motor with new parameters as set by the controller
        if (!motor_.update_modulations(
                voltage_modulation_magnitude,
                phase_angle_, 
                phase_velocity)
            ) {

            // Return if error
            return false; // set_error should update axis.error_
        }
        
        return true;
    });

    return check_for_errors();
}


// =========================================================================
//bool Axis::run_closed_loop_control_loop() {
    /*
    // To avoid any transient on startup, we intialize the setpoint to be the current position
    controller_.pos_setpoint_ = encoder_.pos_estimate_;

    run_control_loop([this]() {

        // Note that all estimators are updated in the loop prefix in run_control_loop
        float d_current_setpoint;
        float q_current_setpoint;

        float phase_vel  = 2.0f * M_PI * encoder_.vel_estimate_ / (float)encoder_.config_.cpr * motor_.config_.pole_pairs;
        
        if (!controller_.update(encoder_.pos_estimate_, encoder_.vel_estimate_, encoder_.phase_, &d_current_setpoint, &q_current_setpoint)) {
            return error_ |= ERROR_CONTROLLER_FAILED, false; //TODO: Make controller.set_error
        }

        if (!motor_.update(d_current_setpoint, q_current_setpoint, encoder_.phase_, phase_vel)) {
            return false; // set_error should update axis.error_
        }

        return true;
    });
    */

//    return check_for_errors();
//}


// =========================================================================
bool Axis::run_idle_loop() {
    // run_control_loop ignores missed modulation timing updates
    // if and only if we're in AXIS_STATE_IDLE
    safety_critical_disarm_motor_pwm(motor_);
    run_control_loop([this](){
        return true;
    });
    return check_for_errors();
}


// =========================================================================
// Infinite loop that does calibration and enters main control loop as appropriate
void Axis::run_state_machine_loop() {

    // Allocate the map for anti-cogging algorithm and initialize all values to 0.0f
    // TODO: Move this somewhere else
    // TODO: respect changes of CPR
    /*
    int encoder_cpr = encoder_.config_.cpr;
    controller_.anticogging_.cogging_map = (float*)malloc(encoder_cpr * sizeof(float));
    if (controller_.anticogging_.cogging_map != NULL) {
        for (int i = 0; i < encoder_cpr; i++) {
            controller_.anticogging_.cogging_map[i] = 0.0f;
        }
    }
    */

    // arm!
    motor_.arm();
    
    for (;;) {
        // Load the task chain if a specific request is pending
        if (requested_state_ != AXIS_STATE_UNDEFINED) {

            __asm__ volatile ("nop");

            // Clear errors
            if(requested_state_ != AXIS_STATE_IDLE && requested_state_ != AXIS_STATE_UNDEFINED) {
                motor_.error_ = Motor::ERROR_NONE;
                error_ = ERROR_NONE;
                controller_.error_ = Controller::ERROR_NONE;
            };

            size_t pos = 0;

            if (requested_state_ == AXIS_STATE_STARTUP_SEQUENCE) {

                if (config_.startup_inverter_automatically) {
                    task_chain_[pos++] = AXIS_STATE_ACTIVE;
                }
                
                task_chain_[pos++] = AXIS_STATE_IDLE;
            } 
            else if (requested_state_ == AXIS_STATE_CALIBRATION) {
                
                /*
                task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                if (encoder_.config_.use_index) {
                    task_chain_[pos++] = AXIS_STATE_ENCODER_INDEX_SEARCH;
                }
                task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                */

                task_chain_[pos++] = AXIS_STATE_IDLE;
            } 
            else if (requested_state_ != AXIS_STATE_UNDEFINED) {
                task_chain_[pos++] = requested_state_;
                task_chain_[pos++] = AXIS_STATE_IDLE;

                __asm__ volatile ("nop");
            }

            // TODO: bounds checking
            task_chain_[pos++] = AXIS_STATE_UNDEFINED; 
            requested_state_ = AXIS_STATE_UNDEFINED;

            // Auto-clear any invalid state error
            error_ &= ~ERROR_INVALID_STATE;
        }

        // Note that current_state is a reference to task_chain_[0]

        // Run the specified state
        // Handlers should exit if requested_state != AXIS_STATE_UNDEFINED
        bool status = 1;

        __asm__ volatile ("nop");

        switch (current_state_) {

            // Calibration state is for analyzing the circuit system
            case AXIS_STATE_CALIBRATION: {
                //status = motor_.run_calibration();
            } break;

            // Active state is for running the inverter
            case AXIS_STATE_ACTIVE: {
                status = run_inverter_loop();
            } break;

            // Idle state stands by and does    
            case AXIS_STATE_IDLE: {
                run_idle_loop();
                status = motor_.arm(); // done with idling - try to arm the motor
            } break;

            default:
                error_ |= ERROR_INVALID_STATE;
                status = false; // this will set the state to idle
                break;
        }

        // If the state failed, go to idle, else advance task chain
        if (!status)
            current_state_ = AXIS_STATE_IDLE;
        else
            memcpy(task_chain_, task_chain_ + 1, sizeof(task_chain_) - sizeof(task_chain_[0]));
    }
}
