
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
    //update_current_controller_gains();

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
    //axis_->sensorless_estimator_.reset_state();

    // Wait until the interrupt handler triggers twice. This gives
    // the control loop the correct time quota to set up modulation timings.
    if (!axis_->wait_for_current_meas()) {
        return axis_->error_ |= Axis::ERROR_CURRENT_MEASUREMENT_TIMEOUT, false;
    }

    // Invalidate any pre-exisiting motor timings
    next_timings_valid_ = false;

    // Arm the motor
    safety_critical_arm_motor_pwm(*this);

    // Success
    return true;
}

void Motor::reset_current_control() {
    current_control_.I_integral = 0.0f;
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

    /*
    // Pole of filter
    volatile float plant_pole = config_.phase_resistance / config_.phase_inductance;

    // Calculate current control gains
    config_.p_gain = config_.current_control_bandwidth * config_.phase_inductance;
    config_.i_gain = plant_pole * config_.p_gain;

    //current_control_.p_gain_max = current_meas_hz * config_.phase_inductance;
    //current_control_.i_gain_max = plant_pole * current_control_.p_gain_max;
    */
}

// @brief Set up the gate drivers
void Motor::DRV8301_setup() {
   
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
    /*if (!update_thermal_limits()) {
        //error already set in function
        return false;
    }*/
    return true;
}

float Motor::effective_current_lim() {
    // Configured limit
    float current_lim = config_.current_lim;
    
    // Hardware limit
    current_lim = std::min(current_lim, axis_->motor_.current_control_.max_allowed_current);

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


// This function calculates the phase current from the ADC value
static constexpr const float sense_amp_vref_ = (1.225f * adc_volts_per_count);

float Motor::output_voltage_from_adcval(uint32_t ADCValue) {

    // Subtract (supposed) Vref since 0 current == ADC half-scale
    int adcval_balanced = ADCValue - sense_amp_vref_;

    // Calculate voltage
    float voltage = /*.000805*/(adc_volts_per_count * /*4096*/(float)adcval_balanced) * /*0.05*/output_voltage_rev_gain_;

    // Return current
    return voltage;
}



// TODO check Ibeta balance to verify good motor connection
bool Motor::measure_phase_resistance(float test_current, float max_voltage) {
    /*

    static const float kI = 10.0f;  // [(V/s)/A]
    static const int num_test_cycles = current_meas_hz * 3; // Test runs for 3s
    float test_voltage = 0.0f;

    size_t i = 0;

    axis_->run_control_loop([&](){

        float Ialpha = -(current_meas_[2].phB + current_meas_[2].phC);
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

    */

    // if we ran to completion that means success
    return true; 
}


//--------------------------------
// Measurement and calibration
//--------------------------------

bool Motor::measure_phase_inductance(float voltage_low, float voltage_high, float saturating_current, AplhaBetaType_t measurement_axis) {
    
    /*
    float test_voltages[2] = {voltage_low, voltage_high};

    float Isamples[2] = {0.0f};
    static const int num_cycles = 3000;

    size_t t = 0;

    axis_->run_control_loop([&](){
        int i = t & 1;

        // Which axis are we measuring on?
        if(measurement_axis == TYPE_ALPHA) {
            // Measuring alpha
            Isamples[i] += -current_meas_[2].phB - current_meas_[2].phC;
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
    */

    return true;
}

bool Motor::measure_motor_dynamics(bool disarm_after) {

    /*
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
    volatile float V_squared = SQ(current_control_.I_integral_d) + SQ(current_control_.I_integral_q);
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
    */

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

    /*
    safety_critical_disarm_motor_pwm(axis_->motor_);    
   
    // Restore cached values to controller
    axis_->sensorless_estimator_.hfi_engaged = cached_hfi_engaged;
    axis_->config_.lockin.current = cached_spin_up_current; 
    axis_->config_.lockin.vel = cached_spin_up_target_vel;  
    axis_->config_.lockin.accel = cached_spin_up_target_accel;  
    axis_->config_.lockin.finish_on_distance = false;
    */

    return true;
}


bool Motor::run_calibration() {

    /*
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

    //if(!measure_dynamics()) {
    //    return false;
    //}

    is_calibrated_ = true;
    */

    return true;
}

// Enqueue new timings 
//
bool Motor::enqueue_timings(float b_phase[6], float c_phase[6]) {

    // The modulation values for this function, are expected to 
    // be between -1.0f and 1.0f. 0.0f is 50% modulation.

    // Fill the timings queue stack
    for(int i = 0; i < 6; i ++) {

        // First transform from ABC into PWM values
    
        // tB and tC are the modulation values for 
        // phase B and C
        float tB = b_phase[i];
        float tC = c_phase[i];
        
        // Normalize the timings from -1.0 to 1.0, to, 0.0 to 1.0
        float timing_b = (tB + 1.0f) / 2.0f;
        float timing_c = (tC + 1.0f) / 2.0f;

        // Then pop the new timing values into the stack of next timings
        next_timings_stack_[i][0] = 0; //(uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
        next_timings_stack_[i][1] = (uint32_t)(timing_b * (float)TIM_1_8_PERIOD_CLOCKS);
        next_timings_stack_[i][2] = (uint32_t)(timing_c * (float)TIM_1_8_PERIOD_CLOCKS);
    }

    // Mark next timings to be valid
    next_timings_valid_ = true;
    
    return true;
}


/*
bool Motor::FOC_current(float Id_setpoint, float Iq_setpoint, float I_phase, float phase_vel) {

    // Three currents from electrical machine
    float I_alpha[3], I_beta[3];

    // Forward clarke transform to estimate three currents from two measurements
    for(int i = 0; i < 3; i++) {
        I_alpha[i] = -current_meas_[i].phB - current_meas_[i].phC;
        I_beta[i]  = one_by_sqrt3 * (current_meas_[i].phB - current_meas_[i].phC);
    }

    if(axis_->sensorless_estimator_.hfi_engaged) {
        axis_->sensorless_estimator_.sample_hfi(&I_phase, I_alpha, I_beta);
    }
    
    // For Reporting
    current_control_.Iq_setpoint = Iq_setpoint;
    current_control_.Id_setpoint = Id_setpoint;

    // Check for current sense saturation
    if (fabsf(current_meas_[2].phB) > current_control_.overcurrent_trip_level
     || fabsf(current_meas_[2].phC) > current_control_.overcurrent_trip_level) {
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
    volatile float Vd = current_control_.I_integral_d + Ierr_d * current_control_.p_gain;
    volatile float Vq = current_control_.I_integral_q + Ierr_q * current_control_.p_gain;

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
        current_control_.I_integral_d *= 0.99f;
        current_control_.I_integral_q *= 0.99f;
    } 
    else {
        // Otherwise forward the current control error
        current_control_.I_integral_d += Ierr_d * (current_control_.i_gain * current_meas_period);
        current_control_.I_integral_q += Ierr_q * (current_control_.i_gain * current_meas_period);
    }

    // Compute estimated bus current
    current_control_.Ibus = mod_d * Id + mod_q * Iq;

    #ifdef __MIDI_HPP
    // Modulation table for music!    
    if(midi_->music_enabled_) {
        midi_->voltage_modulation_index_ += 6;

        // Check to see if we have over-scanned our modulation table
        if (midi_->voltage_modulation_index_ >= midi_modulation_table_size_ - 5) {
            midi_->voltage_modulation_index_ = midi_modulation_table_underscan_;
        }
    } 
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
        #endif

        if(axis_->sensorless_estimator_.hfi_engaged) {
            // Add HFI signal if enabled
            mod_alphas[i] += hfi_alphas[i];
            mod_betas[i] += hfi_betas[i];
        }
    }

    // Make sin/cos for fourier series compensation

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
*/

#define calib_tau 0.2f  
static const float reference_calib_filter_k = current_meas_period / calib_tau;


float sin_p[6];

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// Update the modulator control loop
//
bool Motor::update_controller(float vel_estimate, float phase, float* current_output, float* voltage_modulation_magnitude) {

    // Update DC references
    // V_out_ref_adc_value_ += reference_calib_filter_k * (get_adc_value(GPIOC, GPIO_PIN_5) - V_out_ref_adc_value_);


    // -- Input Limits ------------------------

    // Set CC bandwidth maximum
    if(config_.current_control_bandwidth > axis_->base_frequency_) {
        config_.current_control_bandwidth = axis_->base_frequency_;
    }

    // Set VC bandwidth maximum 
    if(config_.voltage_control_bandwidth > axis_->base_frequency_) {
        //config_.voltage_control_bandwidth = axis_->base_frequency_;
    }

    // There cannot be negative current setpoint, it is nonsensical
    if(config_.I_setpoint < 0.0f) {
        config_.I_setpoint = 0.0f;
    }
    

    // -- Unrotate voltage ------------------------
    const int number_of_samples = 6;


    // -- Average Samples ------------------------

    // Recent samples
    float I_b = 0.0f;
    float I_c = 0.0f;
    float V_out = 0.0f;

    // What is the mean of the new samples?
    for(int i = 0; i < number_of_samples; i++) {
        I_b += current_meas_[i].phB;
        I_c += current_meas_[i].phC;
        V_out += output_voltages_[i];
    }

    // Normalize samples
    I_b /= number_of_samples;
    I_c /= number_of_samples;
    V_out /= number_of_samples;    

    // -- Debug Samples ------------------------

    // Record measured currents
    current_control_.I_b_measured += current_control_.I_measured_report_filter_k * (I_b - current_control_.I_b_measured);
    current_control_.I_c_measured += current_control_.I_measured_report_filter_k * (I_c - current_control_.I_c_measured);

    // Record measured voltages
    voltage_control_.V_out_measured += voltage_control_.V_out_measured_report_filter_k * (V_out - voltage_control_.V_out_measured);

    // -- Get Magnitudes ------------------------

    // Get the magnitude of the current measurement
    current_control_.I_total = (I_b - I_c) / 2.0f;
    float filtered_I_magnitude = fabsf(current_control_.I_total);

    // Get the magnitude of the voltage measurement
    float filtered_V_magnitude = fabsf(V_out);

    // -- Debug Magnitudes ------------------------

    // Update current control magnitude, low-pass filtered from our current control bandwidth
    current_control_.I_magnitude += config_.current_control_bandwidth * current_meas_period * (filtered_I_magnitude - current_control_.I_magnitude);
    
    // Update voltage control magnitude, low-pass filtered from our voltage control bandwidth
    voltage_control_.V_out_magnitude += config_.voltage_control_bandwidth * current_meas_period * (filtered_V_magnitude - voltage_control_.V_out_magnitude);

    // Voltage samples

    for(int i = 0; i < number_of_samples; i++) {

        if(axis_->loop_counter_ % 1 == 0) {
            oscilloscope_pos++;
            oscilloscope_[0][oscilloscope_pos % OSCILLOSCOPE_SIZE] = mapfloat(V_out, -50, 50, 0, 255);
        }
    }


    // -- Update Errors ------------------------

    // Update current error
    current_control_.I_error = config_.I_setpoint - current_control_.I_magnitude;

    // Update voltage error
    voltage_control_.V_out_error = config_.V_setpoint - voltage_control_.V_out_magnitude;

    // -- Proportional Output ------------------------

    // Calculate the output magnitude from the current integral
    volatile float I_modulation_magnitude = current_control_.I_integral + current_control_.I_error * config_.I_p_gain;

    // Calculate the output magnitude from the voltage integral
    volatile float V_modulation_magnitude = voltage_control_.V_integral + voltage_control_.V_out_error * config_.V_p_gain;

    // --------------------------
    // I am now puzzled
    // --------------------------

    // Output modulations (0 = no pwm, 1 = max pwm)
    float output_modulation_magnitude = 0.0f;

    // Combine PI outputs (didnt work)
    //output_modulation_magnitude = (I_modulation_magnitude + V_modulation_magnitude) / 2.0f;

    // FIXME: Current mode only for now
    //output_modulation_magnitude = I_modulation_magnitude;
    output_modulation_magnitude = I_modulation_magnitude;

    // -- Checks? ------------------------

    // If voltage modulation is saturated (hit max voltage)
    if(voltage_control_.V_out_magnitude > config_.V_setpoint) {
        // Do anything?
        //config_.I_setpoint *= 0.99f;
    } 

    // If current modulation is saturated (hit max current)
    if(current_control_.I_magnitude > config_.I_setpoint) {
        // Do anything?
        //config_.V_setpoint *= 0.99f;
    }

    // -- Overmodulation Check ------------------------

    // Prevent modulation from being saturated
    if (output_modulation_magnitude > 0.99f) {

        // Clamp modulation to safe value
        output_modulation_magnitude = 0.99f;

        // Decay the integrals
        current_control_.I_integral *= 0.99f;
        voltage_control_.V_integral *= 0.99f;
    } 
    else {
        // Modulation not saturated (normal condition)...

        // Feed forward the current control error
        current_control_.I_integral += current_control_.I_error * (config_.I_i_gain * current_meas_period);

        // Feed forward the voltage control error
        voltage_control_.V_integral += voltage_control_.V_out_error * (config_.V_i_gain * current_meas_period);
    }

    // -- Integral Caps ------------------------

    if(fabsf(voltage_control_.V_integral) > 50) {
        voltage_control_.V_integral = copysignf(50, voltage_control_.V_integral);
    }

    if(fabsf(current_control_.I_integral) > 50) {
        current_control_.I_integral = copysignf(50, current_control_.I_integral);
    }

    // -- Final Checks ------------------------

    // Negative current integral is unstable, we cannot let it happen.
    if(current_control_.I_integral < 0.0f) {
        current_control_.I_integral *= -1.0f;
    }

    // -- Controller Output ------------------------

    // Output the modulation magnitudes
    *voltage_modulation_magnitude = output_modulation_magnitude;

    return true;
};


// Update the modulator timings
//
bool Motor::update_modulations(float modulation_magnitude, float phase, float phase_vel) {
    
    // Make a stack of modulations
    float phase_b_modulations[6];
    float phase_c_modulations[6];

    // We use these to calculate the PWM phase angles for the next 6 timings
    float phase_per_period  = phase_vel * current_meas_period;
    float first_pwm_phase   = phase + (1.0f + (1.0f / 12.0f)) * phase_per_period;
    float pwm_phase_spacing = (1.0f / 6.0f) * phase_per_period ;

    volatile float pwm_phases[6];
    float sin_p_n[6];

    // Calculate next six modualtion timings for the PWM timers
    for (int i = 0; i < 6; i++){

        // Calculate PWM phase angle
        pwm_phases[i] = first_pwm_phase + i * pwm_phase_spacing;
  
        // Generate a sine wave from 0 to 1.0f
        sin_p[i] = our_arm_sin_f32(pwm_phases[i]);
    
        // And a sine wave of same amplitude, exactly out of phase
        float anti_phase = wrap_pm_pi(pwm_phases[i] + M_PI);
        sin_p_n[i] = our_arm_sin_f32(anti_phase);

        // Then generate PWM modulation values from -1.0 to 1.0f
        phase_b_modulations[i] = modulation_magnitude * sin_p[i];
        phase_c_modulations[i] = modulation_magnitude * sin_p_n[i];
    }   

    // Push timings to the timers
    return enqueue_timings(phase_b_modulations, phase_c_modulations);

    return true;
}