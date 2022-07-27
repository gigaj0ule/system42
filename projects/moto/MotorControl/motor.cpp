
#include <algorithm>

#include "drv8301.hpp"
#include "odrive_main.h"



// Define constantexpr's for quick calculations
static constexpr const int adc_full_scale = ADC_FULL_SCALE;
static constexpr const int adc_half_scale = ADC_HALF_SCALE;
static constexpr const float adc_volts_per_count = ADC_VOLTS_PER_COUNT;


Motor::Motor(const MotorHardwareConfig_t& hw_config,
             const GateDriverHardwareConfig_t& gate_driver_config,
             Config_t& config) :
        hw_config_(hw_config),
        gate_driver_config_(gate_driver_config),
        config_(config),
        gate_driver_({
            .spiHandle = gate_driver_config_.spi,
            .EngpioHandle = gate_driver_config_.enable_port,
            .EngpioNumber = gate_driver_config_.enable_pin,
            .nCSgpioHandle = gate_driver_config_.nCS_port,
            .nCSgpioNumber = gate_driver_config_.nCS_pin,
        }) {

    // Calculate current controller gain values
    update_current_controller_gains();

    // Intialize the timings stack to sensible values
    reset_timings_stacks();
}


// @brief This function initializes the timings stacks with sensible values
void Motor::reset_timings_stacks() {
    for(uint8_t state_vector = 0; state_vector < 6; state_vector++) {
        for(uint8_t phase = 0; phase < 3; phase++) {
            next_timings_stack_[state_vector][phase]   = TIM_1_8_PERIOD_CLOCKS / 2; 
            active_timings_stack_[state_vector][phase] = TIM_1_8_PERIOD_CLOCKS / 2; 
        }
    }
}

// @brief Arms the PWM outputs that belong to this motor.
//
// Note that this does not yet activate the PWM outputs, it just unlocks them.
//
// While the motor is armed, the control loop must set new modulation timings
// between any two interrupts (that is, enqueue_modulation_timings must be executed).
// If the control loop fails to do so, the next interrupt handler floats the
// phases. Once this happens, missed_control_deadline is set to true and
// the motor can be considered disarmed.
//
// @returns: True on success, false otherwise
bool Motor::arm() {

    // Reset controller states, integrators, setpoints, etc.
    axis_->controller_.reset();
    reset_current_control();

    // Reset sensorless estimator
    axis_->sensorless_estimator_.reset_state();

    // Wait until the interrupt handler triggers twice. This gives
    // the control loop the correct time quota to set up modulation timings.
    if (!axis_->wait_for_current_meas()) {
        return axis_->error_ |= Axis::ERROR_CURRENT_MEASUREMENT_TIMEOUT, false;
    }

    // Invalidate any pre-exisiting motor timings
    next_timings_valid_ = false;

    axis_->sensorless_estimator_.hfi_closed_feedback_loop_ = true;

    // Arm the motor
    safety_critical_arm_motor_pwm(*this);

    // Success
    return true;
}

void Motor::reset_current_control() {
    current_control_.v_current_control_integral_d = 0.0f;
    current_control_.v_current_control_integral_q = 0.0f;
}

// @brief Tune the current controller based on phase resistance and inductance
// This should be invoked whenever one of these values changes.
// TODO: allow update on user-request or update automatically via hooks
void Motor::update_current_controller_gains() {

    // Obviously the phase inductance can never be zero here or we will 
    // proliferate NaNs in our code. If it is zero, let us make the assumption it
    // is 0.001 to keep the space shuttle flying.
    if(config_.phase_inductance == 0.0f) {
        config_.phase_inductance = 0.001f;
    }

    // Pole of filter
    volatile float plant_pole = config_.phase_resistance / config_.phase_inductance;

    // Calculate current control gains
    current_control_.p_gain = config_.current_control_bandwidth * config_.phase_inductance;
    current_control_.i_gain = plant_pole * current_control_.p_gain;

    //current_control_.p_gain_max = current_meas_hz * config_.phase_inductance;
    //current_control_.i_gain_max = plant_pole * current_control_.p_gain_max;
}

// @brief Set up the gate drivers
void Motor::DRV8301_setup() {
    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    // Solve for exact gain, then snap down to have equal or larger range as requested
    // or largest possible range otherwise
    static const float kMargin = 0.90f;
    static const float kTripMargin = 1.0f; // Trip level is at edge of linear range of amplifer
    static const float max_output_swing = 1.35f; // [V] out of amplifier
    float max_unity_gain_current = kMargin * max_output_swing * hw_config_.shunt_conductance; // [A]
    float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]

    // Decoding array for snapping gain
    std::array<std::pair<float, DRV8301_ShuntAmpGain_e>, 4> gain_choices = { 
        std::make_pair(10.0f, DRV8301_ShuntAmpGain_10VpV),
        std::make_pair(20.0f, DRV8301_ShuntAmpGain_20VpV),
        std::make_pair(40.0f, DRV8301_ShuntAmpGain_40VpV),
        std::make_pair(80.0f, DRV8301_ShuntAmpGain_80VpV)
    };

    // We use lower_bound in reverse because it snaps up by default, we want to snap down.
    auto gain_snap_down = std::lower_bound(gain_choices.crbegin(), gain_choices.crend(), requested_gain, 
    [](std::pair<float, DRV8301_ShuntAmpGain_e> pair, float val){
        return pair.first > val;
    });

    // If we snap to outside the array, clip to smallest val
    if(gain_snap_down == gain_choices.crend())
       --gain_snap_down;

    // Set variable to scale current readings from controller
    phase_current_rev_gain_ = 1.0f / gain_snap_down->first;

    // Clip all current control to actual usable range
    current_control_.max_allowed_current = max_unity_gain_current * phase_current_rev_gain_;
    // Set trip level
    current_control_.overcurrent_trip_level = (kTripMargin / kMargin) * current_control_.max_allowed_current;

    // We now have the gain settings we want to use, lets set up DRV chip
    DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
    DRV8301_enable(&gate_driver_);
    DRV8301_setupSpi(&gate_driver_, local_regs);

    local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    local_regs->Ctrl_Reg_2.GAIN = gain_snap_down->second;

    local_regs->SndCmd = true;
    DRV8301_writeData(&gate_driver_, local_regs);
    local_regs->RcvCmd = true;
    DRV8301_readData(&gate_driver_, local_regs);
}

// @brief Checks if the gate driver is in operational state.
// @returns: true if the gate driver is OK (no fault), false otherwise
bool Motor::check_DRV_fault() {
    //TODO: make this pin configurable per motor ch
    GPIO_PinState nFAULT_state = HAL_GPIO_ReadPin(gate_driver_config_.nFAULT_port, gate_driver_config_.nFAULT_pin);
    if (nFAULT_state == GPIO_PIN_RESET) {
        // Update DRV Fault Code
        drv_fault_ = DRV8301_getFaultType(&gate_driver_);
        // Update/Cache all SPI device registers
        // DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
        // local_regs->RcvCmd = true;
        // DRV8301_readData(&gate_driver_, local_regs);
        return false;
    };
   return true;
}

void Motor::set_error(Motor::Error_t error){
    error_ |= error;
    axis_->error_ |= Axis::ERROR_MOTOR_FAILED;
    safety_critical_disarm_motor_pwm(*this);
    update_brake_current();
}

float Motor::get_inverter_temp() {
    float adc = adc_measurements_[hw_config_.inverter_thermistor_adc_ch];
    float normalized_voltage = adc / adc_full_scale;
    return horner_fma(normalized_voltage, thermistor_poly_coeffs, thermistor_num_coeffs);
}

bool Motor::update_thermal_limits() {
    float fet_temp = get_inverter_temp();
    float temp_margin = config_.inverter_temp_limit_upper - fet_temp;
    float derating_range = config_.inverter_temp_limit_upper - config_.inverter_temp_limit_lower;
    thermal_current_lim_ = config_.current_lim * (temp_margin / derating_range);
    if (!(thermal_current_lim_ >= 0.0f)) { //Funny polarity to also catch NaN
        thermal_current_lim_ = 0.0f;
    }
    if (fet_temp > config_.inverter_temp_limit_upper + 5) {
        set_error(ERROR_INVERTER_OVER_TEMP);
        return false;
    }
    return true;
}

bool Motor::do_checks() {
    if (!check_DRV_fault()) {
        set_error(ERROR_DRV_FAULT);
        return false;
    }
    if (!update_thermal_limits()) {
        //error already set in function
        return false;
    }
    return true;
}

float Motor::effective_current_lim() {
    // Configured limit
    float current_lim = config_.current_lim;
    // Hardware limit
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        current_lim = std::min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage);
    } else {
        current_lim = std::min(current_lim, axis_->motor_.current_control_.max_allowed_current);
    }
    // Thermal limit
    current_lim = std::min(current_lim, thermal_current_lim_);

    return current_lim;
}

void Motor::log_timing(TimingLog_t log_idx) {
    static const uint16_t clocks_per_cnt = (uint16_t)((float)TIM_1_8_CLOCK_HZ / (float)TIM_APB1_CLOCK_HZ);
    uint16_t timing = clocks_per_cnt * htim13.Instance->CNT; // TODO: Use a hw_config

    if (log_idx < TIMING_LOG_NUM_SLOTS) {
        timing_log_[log_idx] = timing;
    }
}


// This function calculates the phase current from the ADC value
float Motor::phase_current_from_adcval(uint32_t ADCValue) {

    // Subtract 2048 since 0 current == ADC half-scale
    int adcval_balanced = ADCValue - /*2048*/adc_half_scale;

    // Calculate shunt voltage
    float shunt_voltage = /*.000805*/(adc_volts_per_count * /*4096*/(float)adcval_balanced) * /*0.05*/phase_current_rev_gain_;
    
    // Calculate current from shunt voltage
    float current = shunt_voltage * /*1999.99*/hw_config_.shunt_conductance;

    // Return current
    return current;
}


// TODO check Ibeta balance to verify good motor connection
bool Motor::measure_phase_resistance(float test_current, float max_voltage) {

    static const float kI = 10.0f;  // [(V/s)/A]
    static const int num_test_cycles = current_meas_hz * 3; // Test runs for 3s
    float test_voltage = 0.0f;

    size_t i = 0;

    axis_->run_control_loop([&](){

        float Ialpha = -(current_meas_[5].phB + current_meas_[5].phC);
        test_voltage += (kI * current_meas_period) * (test_current - Ialpha);

        if (test_voltage > max_voltage || test_voltage < -max_voltage) {
            return set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE), false;
        }

        float v_alphas[6]; 
        float v_betas[6]; 
        
        for(int z = 0; z < 6; z++) {
            v_alphas[z] = test_voltage;
            v_betas[z] = 0.0f;
        }

        // Test voltage along phase A
        if (!enqueue_timings(v_alphas, v_betas, VOLTAGE_TIMINGS)) {
            // If we failed to enqueue timings, error set inside enqueue_voltage_timings
            return false;
        }

        log_timing(TIMING_LOG_MEAS_R);

        return ++i < num_test_cycles;
    });

    if (axis_->error_ != Axis::ERROR_NONE) {
        return false;
    }

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    float R = test_voltage / test_current;
    config_.phase_resistance = R;

    // if we ran to completion that means success
    return true; 
}


//--------------------------------
// Measurement and calibration
//--------------------------------

bool Motor::measure_phase_inductance(float voltage_low, float voltage_high, float saturating_current, AplhaBetaType_t measurement_axis) {
    
    float test_voltages[2] = {voltage_low, voltage_high};

    float Isamples[2] = {0.0f};
    static const int num_cycles = 3000;

    size_t t = 0;

    axis_->run_control_loop([&](){
        int i = t & 1;

        // Which axis are we measuring on?
        if(measurement_axis == TYPE_ALPHA) {
            // Measuring alpha
            Isamples[i] += -current_meas_[5].phB - current_meas_[5].phC;
        }
        else {
            // Measuring beta
            Isamples[i] +=  one_by_sqrt3 * (current_meas_[i].phB - current_meas_[i].phC);
        }

        // Stores voltages for the timings
        float v_alphas[6];
        float v_betas[6];

        // Calculate six new voltages for the timings
        for(int j = 0; j < 6; j++){
            if(measurement_axis == TYPE_ALPHA) {
                v_alphas[j] = test_voltages[i] + (saturating_current * config_.phase_resistance);
                v_betas[j] = 0.0f;
            }
            else {
                v_alphas[j] = 0.0f;
                v_betas[j] = test_voltages[i] + (saturating_current * config_.phase_resistance);
            }
        }

        // Shuffle voltages into timers to run the test
        if (!enqueue_timings(v_alphas, v_betas, VOLTAGE_TIMINGS)) {
            // If we failed to enqueue timings, error set inside enqueue_voltage_timings
            return false;
        }

        log_timing(TIMING_LOG_MEAS_L);

        return ++t < (num_cycles << 1);
    });
    
    if (axis_->error_ != Axis::ERROR_NONE) {
        return false;
    }

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    float v_L = 0.5f * (voltage_high - voltage_low);
    
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Isamples[1] - Isamples[0]) / (current_meas_period * (float)num_cycles);
    float L = v_L / dI_by_dt;

    config_.phase_inductance = L;

    // TODO arbitrary values set for now
    if (L < 2e-6f || L > 4000e-6f) {
        return set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE), false;
    }

    return true;
}

bool Motor::measure_motor_dynamics(bool disarm_after) {

    // Cache old controller values
    bool cached_hfi_engaged = axis_->sensorless_estimator_.hfi_engaged;
    float cached_spin_up_target_vel = axis_->config_.lockin.vel;
    float cached_spin_up_target_accel = axis_->config_.lockin.accel;
    float cached_spin_up_current = axis_->config_.lockin.current;

    // Set sensible controller values for detection
    axis_->sensorless_estimator_.hfi_engaged = false;
    axis_->config_.lockin.current = axis_->motor_.config_.calibration_current;
    axis_->config_.lockin.vel = config_.motor_calibration_velocity; //100.0f; //config_.motor_calibration_velocity;
    axis_->config_.lockin.accel = config_.motor_calibration_velocity / 2.0f;

    // Set up variables for calculation
    float Id = 0;
    float Iq = axis_->config_.lockin.current;
    float Rs = config_.phase_resistance;
    float Ld = config_.phase_inductance; // Wrong but close enough for now
    float Lq = config_.phase_inductance; // Wrong but close enough for now
    float omega = config_.motor_calibration_velocity; //axis_->config_.lockin.vel;

    // Spin up and run motor constant velocity
    axis_->config_.lockin.finish_distance = 1.0f * config_.motor_calibration_velocity;
    axis_->config_.lockin.finish_on_distance = true;
    axis_->config_.lockin.finish_on_vel = false;
    axis_->config_.lockin.finish_on_enc_idx = false;
    axis_->config_.lockin.finish_on_flag = false;

    axis_->run_lockin_spin(false);

    // Calculate flux linkage
    //  Vq = Rs Iq + ω Ld Id + ω λm
    //  Vd = Rs Id - ω Lq Iq   
    //  V^2 = Vq^2 + Vd^2
    //  V^2 = (Rs Iq + ω Ld Id + ω λ)^2 + (Rs Id - ω Lq Iq)^2
    //  V^2 - (Rs Id - ω Lq Iq)^2 = (Rs Iq + ω Ld Id + ω λm)^2
    //  sqrt[ V^2 - (Rs Id - ω Lq Iq)^2 ] = Rs Iq + ω Ld Id + ω λm
    //  sqrt[ V^2 - (Rs Id - ω Lq Iq)^2 ] - Rs Iq - ω Ld Id / ω = λm
    volatile float V_squared = SQ(current_control_.v_current_control_integral_d) + SQ(current_control_.v_current_control_integral_q);
    volatile float lambda_m = (sqrt( V_squared - SQ(Rs * Id - omega * Lq * Iq)) - Rs * Iq - omega * Ld * Id) / omega;

    // Check if value is reasonable, error if out of bounds
    if(lambda_m < 0.0f || lambda_m > 1.0f) {
        // Error, coast motor
        safety_critical_disarm_motor_pwm(axis_->motor_);
        return set_error(ERROR_PM_FLUX_LINKAGE_OUT_OF_RANGE), false;
    }
    else {
        // If values are good, save values
        axis_->sensorless_estimator_.config_.pm_flux_linkage = lambda_m;
    }

    // Now measure inertia! ----------------------------------------------------------------------
    // (incomplete)
    /*
    size_t t = 0;

    axis_->controller_.vel_setpoint_ = axis_->config_.lockin.vel;
    axis_->controller_.config_.control_mode = axis_->controller_.CTRL_MODE_VELOCITY_CONTROL;
    axis_->controller_.vel_setpoint_ = axis_->config_.lockin.vel;

    axis_->run_control_loop([&]() {

        // Note that all estimators are updated in the loop prefix in run_control_loop
        float d_current_setpoint, q_current_setpoint;

        if (!axis_->controller_.update(axis_->sensorless_estimator_.pll_pos_, axis_->sensorless_estimator_.vel_estimate_, 
            axis_->sensorless_estimator_.phase_, &d_current_setpoint, &q_current_setpoint)) {
            return false;
        }

        if (!update(d_current_setpoint, q_current_setpoint, axis_->sensorless_estimator_.phase_, axis_->sensorless_estimator_.vel_estimate_)) {
            return false; 
        }
        
        if(t == 2 * current_meas_hz) {
            axis_->controller_.vel_setpoint_ = 2 * axis_->config_.lockin.vel;// + 0.5f * axis_->config_.lockin.vel;
        }

        return ++t < 3 * current_meas_hz;
    });
    */

    safety_critical_disarm_motor_pwm(axis_->motor_);    
   
    // Restore cached values to controller
    axis_->sensorless_estimator_.hfi_engaged = cached_hfi_engaged;
    axis_->config_.lockin.current = cached_spin_up_current; 
    axis_->config_.lockin.vel = cached_spin_up_target_vel;  
    axis_->config_.lockin.accel = cached_spin_up_target_accel;  
    axis_->config_.lockin.finish_on_distance = false;
    
    return true;
}


bool Motor::run_calibration() {

    float R_calib_max_voltage = config_.resistance_calib_max_voltage;

    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT) {

        is_calibrated_ = false;
        
        if (!measure_phase_resistance(config_.calibration_current, R_calib_max_voltage)) {
            return false;
        }
        if (!measure_phase_inductance(-R_calib_max_voltage, R_calib_max_voltage, 0.0f, TYPE_ALPHA)) {
            return false;
        }

        __asm__ volatile ("nop");

        // Update P & I gain
        update_current_controller_gains();

        __asm__ volatile ("nop");

        // Measure HFI intrinsics
        if(!axis_->sensorless_estimator_.measure_hfi_intrinsics()) {
            return false;
        }

        __asm__ volatile ("nop");

        // Spin the motor to measure PM flux linkage
        if (!measure_motor_dynamics(false)) { 
            return false;
        }

        __asm__ volatile ("nop");
    } 
    else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        // no calibration needed
    } 
    else {
        return false;
    }

    /*
    if(!measure_dynamics()) {
        return false;
    }*/

    is_calibrated_ = true;

    return true;
}


bool Motor::enqueue_timings(float alphas[6], float betas[6], TimingsType_t timings_type) {
    
    // If our timings are voltage timings then we first must scale them
    if(timings_type == VOLTAGE_TIMINGS) {

        // Find the scale ratio
        float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);

        // Then scale the timings to modulation values
        for(int i = 0; i < 6; i++) {
            alphas[i] = vfactor * alphas[i];
            betas[i] = vfactor * betas[i];
        }
    }

    // These are used by the SVM state machine to calculate the correct 
    // pwm values for our triple H bridge
    float tA, tB, tC;

    // Fill the timings queue stack
    for(int i = 0; i < 6; i ++) {

        // First transform from ABC into PWM values
        if (SVM(alphas[i], betas[i], &tA, &tB, &tC) != 0) {   

            __asm__ volatile ("nop");

            return set_error(ERROR_MODULATION_MAGNITUDE), false;
        }

        // Then pop the values into the stack
        next_timings_stack_[i][0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
        next_timings_stack_[i][1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
        next_timings_stack_[i][2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
    }

    // Mark next timings to be valid
    next_timings_valid_ = true;
    
    return true;
}


// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool Motor::FOC_voltage(float v_d, float v_q, float phase, float phase_vel) {
    // Make a stack of modulation timings
    float v_alphas[6];
    float v_betas[6];

    // We use these to calculate the PWM phase angles for the next 6 timings
    float phase_per_period  = phase_vel * current_meas_period;
    float first_pwm_phase   = phase + (1.0f + (1.0f / 12.0f)) * phase_per_period;
    float pwm_phase_spacing = (1.0f / 6.0f) * phase_per_period ;
    float pwm_phases[6];
    float cos_p[6];
    float sin_p[6];

    // Calculate next six modualtion timings for the PWM timers
    for (int i = 0; i < 6; i++){

        // Calculate PWM phase angle
        pwm_phases[i] = first_pwm_phase + i * pwm_phase_spacing;
  
        // Calculate sin, cos for our park transform
        cos_p[i] = our_arm_cos_f32(pwm_phases[i]);
        sin_p[i] = our_arm_sin_f32(pwm_phases[i]);

        // Reverse park transform
        v_alphas[i] = cos_p[i] * v_d - sin_p[i] * v_q;
        v_betas[i]  = cos_p[i] * v_q + sin_p[i] * v_d;
    }   

    // Push timings to the timers
    return enqueue_timings(v_alphas, v_betas, VOLTAGE_TIMINGS);
}


bool Motor::FOC_current(float Id_setpoint, float Iq_setpoint, float I_phase, float phase_vel) {

    // Three currents from electrical machine
    float I_alpha[3], I_beta[3];

    // Forward clarke transform to estimate three currents from two measurements
    for(int i = 0; i < 3; i+=2) {
        I_alpha[i/2] = -current_meas_[i].phB - current_meas_[i].phC;
        I_beta[i/2]  = one_by_sqrt3 * (current_meas_[i].phB - current_meas_[i].phC);
    }

    if(axis_->sensorless_estimator_.hfi_engaged) {
        axis_->sensorless_estimator_.sample_hfi(&I_phase, I_alpha, I_beta);
    }
    
    // For Reporting
    current_control_.Iq_setpoint = Iq_setpoint;
    current_control_.Id_setpoint = Id_setpoint;

    // Check for current sense saturation
    if (fabsf(current_meas_[5].phB) > current_control_.overcurrent_trip_level
     || fabsf(current_meas_[5].phC) > current_control_.overcurrent_trip_level) {
        set_error(ERROR_CURRENT_SENSE_SATURATION);
    }

    // Calculate sin and cos of phase for forward dq0 transform
    volatile float c_I = our_arm_cos_f32(I_phase);
    volatile float s_I = our_arm_sin_f32(I_phase);

    // Forward dq0 transform
    float Id = c_I * I_alpha[2] + s_I * I_beta[2];
    float Iq = c_I * I_beta[2] - s_I * I_alpha[2];

    if(axis_->sensorless_estimator_.hfi_engaged) {
        axis_->sensorless_estimator_.update_hfi(&Id);
    }

    // Debugging
    current_control_.Iq_measured += current_control_.I_measured_report_filter_k * (Iq - current_control_.Iq_measured);
    current_control_.Id_measured += current_control_.I_measured_report_filter_k * (Id - current_control_.Id_measured);

    // Current error
    volatile float Ierr_d = Id_setpoint - Id;
    volatile float Ierr_q = Iq_setpoint - Iq;

    // Calculate scale factors to change voltage to modulation amount
    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    volatile float Vd = current_control_.v_current_control_integral_d + Ierr_d * current_control_.p_gain;
    volatile float Vq = current_control_.v_current_control_integral_q + Ierr_q * current_control_.p_gain;

    // Calculate vector modulations (0 = no pwm, 1 = max pwm)
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    // High-Frequency Signal Injection voltage carrier
    float hfi_alphas[6] = {0};
    float hfi_betas[6] = {0};

    // Maximum modulation for the HFI
    float hfi_modulation_index = 0;

    // Generate new HFI modulations
    if(axis_->sensorless_estimator_.hfi_engaged == true) {
        axis_->sensorless_estimator_.generate_new_hfi_modulations(&I_phase, &V_to_mod, &hfi_modulation_index, hfi_alphas, hfi_betas);
    }

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable
    float mod_scalefactor = (0.80f - hfi_modulation_index) * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);

    // If modulation is saturated...
    if (mod_scalefactor < 1.0f) {

        // Then decay it a little bit
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;

        // TODO make decayfactor configurable
        current_control_.v_current_control_integral_d *= 0.99f;
        current_control_.v_current_control_integral_q *= 0.99f;
    } 
    else {
        // Otherwise forward the current control error
        current_control_.v_current_control_integral_d += Ierr_d * (current_control_.i_gain * current_meas_period);
        current_control_.v_current_control_integral_q += Ierr_q * (current_control_.i_gain * current_meas_period);
    }

    // Compute estimated bus current
    current_control_.Ibus = mod_d * Id + mod_q * Iq;

    #ifdef __MIDI_HPP
    /*
    // Modulation table for music!    
    if(midi_->music_enabled_) {
        midi_->voltage_modulation_index_ += 6;

        // Check to see if we have over-scanned our modulation table
        if (midi_->voltage_modulation_index_ >= midi_modulation_table_size_ - 5) {
            midi_->voltage_modulation_index_ = midi_modulation_table_underscan_;
        }
    } 
    */
    #endif

    // We use these to calculate the PWM phase angles for the next 6 timings
    float phase_per_period   = phase_vel * current_meas_period;
    float first_pwm_phase    = I_phase + (1.0f + (1.0f / 12.0f)) * phase_per_period;
    float pwm_phase_spacing  = (1.0f / 6.0f) * phase_per_period;
    float pwm_phases[6];

    // Modulations that get pushed to the timers
    float mod_alphas[6] = {0};
    float mod_betas[6] = {0};

    // Calculate next six modulation timings for the PWM timers
    for (int i = 0; i < 6; i++) {

        // Calculate PWM phase angle
        pwm_phases[i] = first_pwm_phase + i * pwm_phase_spacing;
  
        // Calculate sin, cos for our inverse dq0 transform
        float arc_dq0_cos = our_arm_cos_f32(pwm_phases[i]);
        float arc_dq0_sin = our_arm_sin_f32(pwm_phases[i]);

        mod_alphas[i] = arc_dq0_cos * mod_d - arc_dq0_sin * mod_q;
        mod_betas[i]  = arc_dq0_cos * mod_q + arc_dq0_sin * mod_d;

        #ifdef __MIDI_HPP
        /*
        if(midi_->music_enabled_) {
            // Check if we should music-modulate by checking our mod table
            if(midi_->midi_modulation_table_[voltage_modulation_index_ + i]) {

                // Modulate the q current by music!
                float music_modulated_q = 0.6f;
                float music_modulated_d = 0.0f;
            
                // Reverse Park transform
                mod_alphas[i] = arc_dq0_cos[i] * music_modulated_d - arc_dq0_sin[i] * music_modulated_q;
                mod_betas[i]  = arc_dq0_cos[i] * music_modulated_q + arc_dq0_sin[i] * music_modulated_d;          
            } 
        }
        */
        #endif

        if(axis_->sensorless_estimator_.hfi_engaged) {
            // Add HFI signal if enabled
            mod_alphas[i] += hfi_alphas[i];
            mod_betas[i] += hfi_betas[i];
        }
    }

    // Make sin/cos for fourier series compensation
    /*
    if(axis_->sensorless_estimator_.hfi_run_DFT_integration_) {
        for(int i = 0; i < axis_->sensorless_estimator_.hfi_DFT_orders; i++) {
            float harmonic_phase = wrap_pm_pi((i + 1) * first_pwm_phase);

            axis_->sensorless_estimator_.hfi_DFT_cos_[i] = our_arm_cos_f32(harmonic_phase);
            axis_->sensorless_estimator_.hfi_DFT_sin_[i] = our_arm_sin_f32(harmonic_phase);
        }
    }
    */

    // Report final applied voltage in stationary frame (for sensorles estimator)
    current_control_.final_v_alpha = mod_to_V * mod_alphas[0];
    current_control_.final_v_beta = mod_to_V * mod_betas[0];

    // Apply SVM
    if (!enqueue_timings(mod_alphas, mod_betas, MODULATION_TIMINGS)) {

        __asm__ volatile ("nop");

        // If we failed to enqueue timings, error set inside enqueue_voltage_timings
        return false;
    }

    log_timing(TIMING_LOG_FOC_CURRENT);

    return true;
}


bool Motor::update(float d_current_setpoint, float q_current_setpoint, float phase, float phase_vel) {
    
    // Set current polarities based on the motor direction
    q_current_setpoint  *= config_.direction;
    d_current_setpoint  *= config_.direction;
    phase               *= config_.direction;
    phase_vel           *= config_.direction;

    // Execute current command
    // TODO: move this into the mot
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT) {
        if(!FOC_current(d_current_setpoint, q_current_setpoint, phase, phase_vel)){
            return false;
        }
    } 
    else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        //In gimbal motor mode, current is reinterptreted as voltage.
        if(!FOC_voltage(d_current_setpoint, q_current_setpoint, phase, phase_vel))
            return false;
    }
    else {
        set_error(ERROR_NOT_IMPLEMENTED_MOTOR_TYPE);
        return false;
    }
    return true;
}