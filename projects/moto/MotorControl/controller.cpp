
#include "odrive_main.h"


Controller::Controller(Config_t& config) :
    config_(config)
{}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_current_ = 0.0f;
    current_setpoint_ = 0.0f;
}

void Controller::set_error(Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_CONTROLLER_FAILED;
}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    pos_setpoint_ = pos_setpoint;
    vel_setpoint_ = vel_feed_forward;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", pos_setpoint, vel_setpoint_, current_setpoint_);
#endif
}

void Controller::set_vel_setpoint(float vel_setpoint, float current_feed_forward) {
    vel_setpoint_ = vel_setpoint;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", vel_setpoint_, motor->current_setpoint_);
#endif
}

void Controller::set_current_setpoint(float current_setpoint) {
    current_setpoint_ = current_setpoint;
    config_.control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", current_setpoint_);
#endif
}

void Controller::move_to_pos(float goal_point) {
    axis_->trap_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_.config_.vel_limit,
                                 axis_->trap_.config_.accel_limit,
                                 axis_->trap_.config_.decel_limit);
    traj_start_loop_count_ = axis_->loop_counter_;
    config_.control_mode = CTRL_MODE_TRAJECTORY_CONTROL;
    goal_point_ = goal_point;
}

void Controller::move_incremental(float displacement, bool from_goal_point = true){
    if(from_goal_point){
        move_to_pos(goal_point_ + displacement);
    } else{
        move_to_pos(pos_setpoint_ + displacement);
    }
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (anticogging_.cogging_map != NULL && axis_->error_ == Axis::ERROR_NONE) {
        anticogging_.calib_anticogging = true;
    }
}

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    if (anticogging_.calib_anticogging && anticogging_.cogging_map != NULL) {
        float pos_err = anticogging_.index - pos_estimate;
        if (fabsf(pos_err) <= anticogging_.calib_pos_threshold &&
            fabsf(vel_estimate) < anticogging_.calib_vel_threshold) {
            anticogging_.cogging_map[anticogging_.index++] = vel_integrator_current_;
        }
        if (anticogging_.index < axis_->encoder_.config_.cpr) { // TODO: remove the dependency on encoder CPR
            set_pos_setpoint(anticogging_.index, 0.0f, 0.0f);
            return false;
        } else {
            anticogging_.index = 0;
            set_pos_setpoint(0.0f, 0.0f, 0.0f);  // Send the motor home
            anticogging_.use_anticogging = true;  // We're good to go, enable anti-cogging
            anticogging_.calib_anticogging = false;
            return true;
        }
    }
    return false;
}

bool Controller::update(float pos_estimate, float vel_estimate, float phase, float*d_current_output, float* q_current_output) {
    // Buffer control mode to prevent race conditions if control_mode changes
    ControlMode_t buffered_control_mode = config_.control_mode;

    // Current limit baseline
    effective_current_limit_ = axis_->motor_.effective_current_lim();

    // Only runs if anticogging_.calib_anticogging is true; non-blocking
    anticogging_calibration(pos_estimate, vel_estimate);
    float anticogging_pos = pos_estimate;

    // Throttle control ---------------------------------------------------------------------------------
    float buffered_brake_setpoint_   = brake_setpoint_;
    float buffered_throttle_setpoint = throttle_setpoint_;
    
    if (buffered_control_mode == CTRL_MODE_THROTTLE_CONTROL) {
        // If brake is engaged
        if(buffered_brake_setpoint_ >= 0.001f) {   
            // Constrain brake setpoint to sensible value
            buffered_brake_setpoint_ = MACRO_CONSTRAIN(buffered_brake_setpoint_, 0.0f, 1.0f);

            // Set brake current to some fraction of user-set max brake current
            effective_current_limit_ = std::min(effective_current_limit_, buffered_brake_setpoint_ * config_.throttle_brake_current_limit);

            // Set target velocity to zero
            vel_setpoint_ = 0.0f;
        }
        else if(abs(buffered_throttle_setpoint) >= 0.001f) {
            // Constrain throttle setpoint to sensible value
            buffered_throttle_setpoint = MACRO_CONSTRAIN(buffered_throttle_setpoint, -1.0f, 1.0f);

            // Set velocity to the sign of throttle
            vel_setpoint_ = copysignf(config_.vel_limit, buffered_throttle_setpoint);
            
            // Set torque as a function of max current
            // EXCEPT in the case where vel > vel_setpoint in where we use throttle_regen_deceleration_current
            if(vel_estimate < vel_setpoint_) {
                effective_current_limit_ = std::min(effective_current_limit_, abs(buffered_throttle_setpoint) * config_.throttle_acceleration_current_limit);
            }
            else {
                effective_current_limit_ = std::min(effective_current_limit_, config_.throttle_regen_deceleration_current);
            }
        }
        else {
            vel_setpoint_ = 0.0f;
            effective_current_limit_ = std::min(effective_current_limit_, config_.throttle_regen_deceleration_current);
        }
    }

    // Trajectory control -----------------------------------------------------------------------------    
    if (buffered_control_mode == CTRL_MODE_TRAJECTORY_CONTROL) {
        // Note: uint32_t loop count delta is OK across overflow
        // Beware of negative deltas, as they will not be well behaved due to uint!
        float t = (axis_->loop_counter_ - traj_start_loop_count_) * current_meas_period;

        if (t > axis_->trap_.Tf_) {
            // Drop into position control mode when done to avoid problems on loop counter delta overflow
            buffered_control_mode = CTRL_MODE_POSITION_CONTROL;
            // pos_setpoint already set by trajectory
            vel_setpoint_ = 0.0f;
            current_setpoint_ = 0.0f;
        }
        else {
            TrapezoidalTrajectory::Step_t traj_step = axis_->trap_.eval(t);
            pos_setpoint_ = traj_step.Y;
            vel_setpoint_ = traj_step.Yd;
            current_setpoint_ = traj_step.Ydd * axis_->trap_.config_.A_per_css;
        }
        anticogging_pos = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
    }

    // Ramp rate limited velocity setpoint
    if ((buffered_control_mode == CTRL_MODE_VELOCITY_CONTROL ||
        buffered_control_mode == CTRL_MODE_THROTTLE_CONTROL) 
        && vel_ramp_enable_) {

        float max_step_size = current_meas_period * config_.vel_ramp_rate;
        float full_step = vel_ramp_target_ - vel_setpoint_;
        float step;

        if (fabsf(full_step) > max_step_size) {
            step = std::copysignf(max_step_size, full_step);
        } 
        else {
            step = full_step;
        }

        vel_setpoint_ += step;
    }

    // Position control -----------------------------------------------------------------------------   
    // TODO Decide if we want to use encoder or pll position here
    float vel_target = vel_setpoint_;
    float pos_err;

    if (buffered_control_mode == CTRL_MODE_POSITION_CONTROL || 
        buffered_control_mode == CTRL_MODE_TRAJECTORY_CONTROL)
    {
        if (config_.setpoints_in_cpr)
        {
            // TODO this breaks the semantics that estimates come in on the arguments.
            // It's probably better to call a get_estimate that will arbitrate (enc vs sensorless) instead.
            float cpr = (float)(axis_->encoder_.config_.cpr);
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, cpr);
            // Circular delta
            pos_err = pos_setpoint_ - axis_->encoder_.pos_cpr_;
            pos_err = wrap_pm(pos_err, 0.5f * cpr);
        } 
        else
        {
            pos_err = pos_setpoint_ - pos_estimate;
        }

        vel_target += config_.pos_gain * pos_err;
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    vel_target = MACRO_CONSTRAIN(vel_target, -vel_lim, vel_lim);

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    // 0.0f to disable
    if (config_.vel_limit_tolerance > 0.0f && config_.vel_limit_error_enabled)
    { 
        if (fabsf(vel_estimate) > config_.vel_limit_tolerance * vel_lim)
        {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // Velocity control is acheived by measuring velocity error and then modifying the 
    // q current of the motor accordinly.
    float Iq = current_setpoint_;
    float Id = 0.0f;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_.use_anticogging)
    {
        Iq += anticogging_.cogging_map[mod(static_cast<int>(anticogging_pos), axis_->encoder_.config_.cpr)];
    }

    // Calculate new velocity error, save old for derivative term
    vel_error_last_value_  = vel_error_;
    vel_error_             = vel_target - vel_estimate;

    // Add (Kp_v * vel_error_or) to Iq if we are not in a velocity-control mode
    if (buffered_control_mode == CTRL_MODE_THROTTLE_CONTROL ||
        buffered_control_mode == CTRL_MODE_POSITION_CONTROL ||
        buffered_control_mode == CTRL_MODE_VELOCITY_CONTROL )
    {
        Iq += config_.vel_gain * vel_error_;
    }

    // Add velocity integrator & derivative calculated in previous iteration 
    // of this loop to the controller output
    Iq += vel_integrator_current_;
    //Iq += vel_derivative_current_;

    // Current limiting
    bool we_hit_current_limit = false;
    if (Iq > effective_current_limit_)
    {
        we_hit_current_limit = true;
        Iq = effective_current_limit_;
    }
    else if (Iq < -effective_current_limit_)
    {
        we_hit_current_limit = true;
        Iq = -effective_current_limit_;
    }

    // Reset integral & derivative to zero if we are not using the velocity controller
    if (buffered_control_mode == CTRL_MODE_CURRENT_CONTROL || 
        buffered_control_mode == CTRL_MODE_VOLTAGE_CONTROL) 
    {
        vel_integrator_current_ = 0.0f;
        vel_derivative_current_ = 0.0f;
    } 

    // Velocity integrator (behaviour dependent on limiting)
    else if (   buffered_control_mode == CTRL_MODE_VELOCITY_CONTROL || 
                buffered_control_mode == CTRL_MODE_POSITION_CONTROL || 
                buffered_control_mode == CTRL_MODE_TRAJECTORY_CONTROL || 
                buffered_control_mode == CTRL_MODE_THROTTLE_CONTROL )
    {
        // Check if we hit the current limit
        if (we_hit_current_limit)
        {
            // If current limit has been hit then bleed some current from the veloicity integrator
            // TODO make decayfactor configurable
            vel_integrator_current_ *= 0.95f;
            vel_derivative_current_ *= 0.95f;
        }

        // If not then calculate I term
        else 
        {
            // Calculate integral of velocity error kI * vel_error_ * T
            // Integrator gets added to Iq on the next cycle of this loop
            vel_integrator_current_ += config_.vel_integrator_gain * current_meas_period * vel_error_;

            // Calculate derivative of velocity error Kd * (vel_error_ - vel_error_last_value_)
            // Derivative gets added to Iq on the next cycle of this loop
                        
            vel_derivative_current_ = config_.vel_derivative_gain * (vel_error_ - vel_error_last_value_);                        
            
            //vel_derivative_current_ = config_.vel_derivative_gain * (vel_setpoint_ - vel_setpoint_last_value_);                        
        }
    }

    // Set DC bias current (if enabled). DC bias current increases cogging, 
    // can sometimes be useful for position hold or braking
    float current_bias_d = 0.0f;
    float current_bias_q = 0.0f;
    
    if(abs(dc_bias_current_) > 0.001f)
    {
        float pwm_phase = phase + 1.5f * current_meas_period * vel_estimate;
        
        current_bias_d = dc_bias_current_ * our_arm_cos_f32(pwm_phase);
        current_bias_q = -dc_bias_current_ * our_arm_sin_f32(pwm_phase);
    }
    
    // Set controller outputs 
    if (q_current_output)
    {
        *q_current_output = Iq + current_bias_q;
    }

    if (d_current_output)
    {
        *d_current_output = Id + current_bias_d;
    }

    return true;
}
