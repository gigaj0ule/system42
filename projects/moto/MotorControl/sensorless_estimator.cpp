#pragma GCC optimize ("O3")

#include "odrive_main.h"

// Constant expressions used in fast calculations -----------------------------------------------
static constexpr const int pwm_frequency_                         = 3 * (int)CURRENT_MEAS_HZ;
static constexpr float const hfi_angular_frequency_               = 6.28318530717958647692f * HFI_TEST_FREQUENCY;
static constexpr float const hfi_modulation_phase_increment_      = hfi_angular_frequency_ / (2.0f * pwm_frequency_);
static constexpr float const three_halfs_omega_c_                 = 3.0f / (2.0f * hfi_angular_frequency_);
static constexpr float const one_half_omega_c_                    = 1.0f / (2.0f * hfi_angular_frequency_);
static constexpr const int   half_calibration_table_angle_entries = HFI_COMPENSATION_MAP_NUMBER_OF_ANGLES / 2;
static constexpr float const calibration_table_angle_ratio        = HFI_COMPENSATION_MAP_NUMBER_OF_ANGLES / TWO_M_PI;
static constexpr float const calibration_table_angle_inv_ratio    = TWO_M_PI / HFI_COMPENSATION_MAP_NUMBER_OF_ANGLES;

//static constexpr float const calibration_table_current_ratio      = hfi_max_calibration_current / HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS;


// Fast Filters ---------------------------------------------------------------------------------

/*
// Low-pass 2nd order FIR butterworth filter
// Sampling = 24kHz
// Corner = 1kHz
static inline __attribute__((always_inline)) float lp_butterworth_2_1k_24k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (1.440144034651119620e-2f * x)
            + (-0.69059892324149707576f * v[0])
            + (1.63299316185545229096f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];       
}
*/

// Low-pass 2nd order FIR butterworth filter
// Sampling = 24kHz
// Corner = 500Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_500_24k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (3.916126660547358806e-3f * x)
            + (-0.83100558934675783362f * v[0])
            + (1.81534108270456839840f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];       
}

// Low-pass 2nd order FIR butterworth filter
// Sampling = 24kHz
// Corner = 250Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_250_24k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (1.023217638470907964e-3f * x)
            + (-0.91159449659995983595f * v[0])
            + (1.90750162604607620409f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];       
}

/*
// Low-pass 2nd order FIR butterworth filter
// Sampling = 24kHz
// Corner = 500Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_150_24k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (3.750696162969635594e-4f * x)
            + (-0.94597793623228154658f * v[0])
            + (1.94447765776709369234f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];       
}
*/

/*
// Low-pass 2nd order FIR butterworth filter
// Sampling = 24kHz
// Corner = 100Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_100_24k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
        v[2] = (1.682237085914583825e-4f * x)
                + (-0.96365298422370504472f * v[0])
                + (1.96298008938933921108f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];       
}
*/

/*
// Low-pass 2nd order FIR butterworth filter
// Sampling = 24kHz
// Corner = 50Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_50_24k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
        v[2] = (4.244336814021587578e-5f * x)
                + (-0.98165828261713394820f * v[0])
                + (1.98148850914457308470f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];       
}
*/

/*
// Low-pass 2nd order FIR butterworth filter
// Sampling = 24kHz
// Corner = 50Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_25_24k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (1.065983454073826664e-5f * x)
        + (-0.99078669884321146633f * v[0])
        + (1.99074405950504851326f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];       
}
*/

/*
// Low-pass 2nd order FIR butterworth filter
// Sampling = 8kHz
// Cutoff = 50Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_50_8k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (3.750696162969635594e-4f * x)
            + (-0.94597793623228154658f * v[0])
            + (1.94447765776709369234f  * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];
}
*/

// Low-pass 2nd order FIR butterworth filter
// Sampling = 8kHz
// Cutoff = 35Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_35_8k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (3.362255137503988855e-5f * x)
            + (-0.98001437405587832341f * v[0])
            + (1.97987988385037816386f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];
}

/*
// Low-pass 2nd order FIR butterworth filter
// Sampling = 8kHz
// Cutoff = 20Hz
static inline __attribute__((always_inline)) float lp_butterworth_2_20_8k(float* v, float x) {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (6.828594198232674509e-6f * x)
            + (-0.99262254312709463644f * v[0])
            + (1.99259522875030170574f * v[1]);
    return 
        (v[0] + v[2]) + 2 * v[1];
}
*/

// This function is used to linearly interpolate our lookup tables
static inline __attribute__((always_inline)) float linear_interpolate_lookup_table(float const index, float * lut, uint16_t const max_lut_size) {

    float const floor_of_value = floor(index);
    uint32_t const lut_index = (uint32_t) floor_of_value; 

    // If we reach the end of the LUT then assume the function continues linearly
    if(lut_index > max_lut_size) {

        float const y_1 = lut[max_lut_size - 2];
        float const y_2 = lut[max_lut_size - 1];

        return y_2 + (index - (float)(max_lut_size - 1)) * (y_2 - y_1);
    }

    // Otherwise interpolate the values of the function
    float const fractional = index - floor_of_value;

    // Read two nearest values of input value from table
    float const a = lut[lut_index];
    float const b = lut[lut_index + 1];

    // Linear interpolation between the two values
    return (1.0f - fractional) * a + fractional * b;
}

static inline __attribute__((always_inline)) float linear_interpolate(float const index, float const a, float const b) {

    uint32_t const floor_of_value = (uint32_t) floor(index);

    float const fractional = index - floor_of_value;

    // Linear interpolation process
    return (1.0f - fractional) * a + fractional * b;
}

static inline uint16_t hfi_I_phase_to_compensation_angle(float I_phase) {
    return half_calibration_table_angle_entries + (int16_t)(calibration_table_angle_ratio * I_phase);
}

SensorlessEstimator::SensorlessEstimator(Config_t& config) :
    config_(config)
{};


bool SensorlessEstimator::reset_state() {
    error_ = ERROR_NONE;

    flux_state_[0] = 0.0f;
    flux_state_[1] = 0.0f;
    V_alpha_beta_memory_[0] = 0.0f;
    V_alpha_beta_memory_[1] = 0.0f;
    pll_pos_ = 0.0f;
    vel_estimate_filtered_ = 0.0f;

    return 1;
}


bool SensorlessEstimator::update() {
    // Algorithm based on paper: Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
    // http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
    // In particular, equation 8 (and by extension eqn 4 and 6).

    // The V_alpha_beta applied immedietly prior to the current measurement associated with this cycle
    // is the one computed two cycles ago. To get the correct measurement, it was stored twice:
    // once by final_v_alpha/final_v_beta in the current control reporting, and once by V_alpha_beta_memory.

    // Clarke transform
    float I_alpha_beta[2];
    
    if(hfi_engaged == 0) {
        I_alpha_beta[0] = -axis_->motor_.buffered_current_meas_[5].phB - axis_->motor_.buffered_current_meas_[5].phC;
        I_alpha_beta[1] = one_by_sqrt3 * (axis_->motor_.buffered_current_meas_[5].phB - axis_->motor_.buffered_current_meas_[5].phC);
    }
    else {
       I_alpha_beta[0] = hfi_cleaned_I_alpha_; 
       I_alpha_beta[1] = hfi_cleaned_I_beta_; 
    }

    // If there are any DC values in I_alpha_beta then our estimator becomes unstable.
    // For this reason we high pass filter the inputs to the estimator.
    
    // Swap sign of I_beta if motor is reversed
    I_alpha_beta[1] *= axis_->motor_.config_.direction;

    // alpha-beta vector operations
    float eta[2];
    for (int i = 0; i <= 1; ++i) {
        // y is the total flux-driving voltage (see paper eqn 4)
        volatile float y = -axis_->motor_.config_.phase_resistance * I_alpha_beta[i] + V_alpha_beta_memory_[i];
        // flux dynamics (prediction)
        volatile float x_dot = y;
        // integrate prediction to current timestep
        flux_state_[i] += x_dot * current_meas_period;

        // eta is the estimated permanent magnet flux (see paper eqn 6)
        eta[i] = flux_state_[i] - axis_->motor_.config_.phase_inductance * I_alpha_beta[i];
    }

    // Non-linear observer (see paper eqn 8):
    volatile float pm_flux_sqr = config_.pm_flux_linkage * config_.pm_flux_linkage;
    volatile float est_pm_flux_sqr = eta[0] * eta[0] + eta[1] * eta[1];
    volatile float bandwidth_factor = 1.0f / pm_flux_sqr;
    volatile float eta_factor = 0.5f * (config_.observer_gain * bandwidth_factor) * (pm_flux_sqr - est_pm_flux_sqr);

    // If for whatever reason this controller goes unstable, this 
    // constraint prevents -inf & NaN from propogating and causing 
    // unrecoverable faults
    /*
    if(isinf(eta_factor)) {

        __asm__ volatile("nop");

        if(eta_factor < 0.0f) {
            eta_factor = FLT_MIN + 1.0f;
        }
        else {
            eta_factor = FLT_MAX - 1.0f;
        }
    }
    */

    // alpha-beta vector operations
    for (int i = 0; i <= 1; ++i) {

        // add observer action to flux estimate dynamics
        volatile float x_dot = eta_factor * eta[i];
        // convert action to discrete-time
        flux_state_[i] += x_dot * current_meas_period;
        // update new eta
        eta[i] = flux_state_[i] - axis_->motor_.config_.phase_inductance * I_alpha_beta[i];

        // Constrain eta again
        /*
        if(isinf(eta[i])) {
            __asm__ volatile("nop");
            if(eta[i] < 0.0f) {
                eta[i] = FLT_MIN + 1.0f;
            }
            else {
                eta[i] = FLT_MAX - 1.0f;
            }
        */
    }

    // Flux state estimation done, store V_alpha_beta for next timestep
    V_alpha_beta_memory_[0] = axis_->motor_.current_control_.final_v_alpha;
    V_alpha_beta_memory_[1] = axis_->motor_.current_control_.final_v_beta * axis_->motor_.config_.direction;

    __asm__ volatile ("nop");

    // PLL
    // TODO: the PLL part has some code duplication with the encoder PLL
    // Pll gains as a function of bandwidth
    float pll_kp = 2.0f * config_.pll_bandwidth;
    
    // Critically damped
    float pll_ki = 0.25f * (pll_kp * pll_kp);

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp < 1.0f)) {
        error_ |= ERROR_UNSTABLE_GAIN;
        return false;
    }

    // NaN check
    if(isnan(eta[0]) || isnan(eta[1]) || isinf(eta[0]) || isinf(eta[1])) {
        //reset_state();
        error_ |= ERROR_NAN_FLUX_STATE;
        return false; // TODO: FALSE
    }

    // predict PLL phase with velocity
    pll_pos_ = wrap_pm_pi(pll_pos_ + current_meas_period * vel_estimate_);

    // update PLL phase with observer permanent magnet phase
    phase_ = fast_atan2(eta[1], eta[0]);
    float delta_phase = wrap_pm_pi(phase_ - pll_pos_);

    pll_pos_ = wrap_pm_pi(pll_pos_ + current_meas_period * pll_kp * delta_phase);

    // update PLL velocity
    vel_estimate_ += current_meas_period * pll_ki * delta_phase;

    vel_estimate_filtered_ += vel_estimate_filter_constant_ * (vel_estimate_ - vel_estimate_filtered_);  

    #if defined(HFI_DEBUGGING_INSPECT_FUSION)
    if(config_.hfi_calibrated) {
        debug_[0] = pll_pos_;
        debug_[1] = phase_;
        debug_[2] = vel_estimate_filtered_;
        debug_[3] = axis_->controller_.direct_mode_phase_;
        debug_[4] = axis_->controller_.direct_mode_vel_;  
    }      
    #endif

    __asm__ volatile ("nop");

    return true;
};


bool SensorlessEstimator::measure_hfi_intrinsics() {

    config_.hfi_calibrated = false;

    // Cache old controller values
    bool cahced_hfi_enabled = config_.hfi_enabled;    

    // Align to motor "D" axis
    size_t control_loop_counts = 0;
    axis_->run_control_loop([&]() {
        if (!axis_->motor_.update(config_.hfi_max_calibration_current, 0.0f, 0.0f, 0.0f)) {
            return false;
        }

        return ++control_loop_counts < 2 * current_meas_hz;
    }); 

    // Find the hfi test voltage which results in an ideal response signal  -----------------------------
    config_.hfi_enabled = true;
    hfi_engaged = true;
    
    // Find "I_c" DC noise floor by finding min and max values with no disturbance
    control_loop_counts = 0;
    hfi_calibration_.samples = 0;
    hfi_calibration_.I_real_min = 0.0f;
    hfi_calibration_.I_real_max = 0.0f;
    hfi_calibration_.enable_filter = false;

    axis_->run_control_loop([&]() {
        
        if (!axis_->motor_.update(config_.hfi_quiescent_current, 0.0f, 0.0f, 0.0f)) {
            return false;
        }

        // Warp field stabilize
        if(control_loop_counts == 100) {
            hfi_run_calibration_integrator_  = true;
        }

        return ++control_loop_counts < current_meas_hz;
    }); 

    float const hfi_noise_floor = hfi_calibration_.I_real_max - hfi_calibration_.I_real_min;

    uint8_t voltage_loop_counts = 0;
    uint8_t const voltage_loop_limit = 255;

    while(voltage_loop_counts < voltage_loop_limit) {
        config_.test_signal_voltage = voltage_loop_counts * 0.1f;

        float const test_velocity = 8.0f;
        float test_distance = 0.0f;
        float test_phase = 0.0f;
        float const finish_distance = TWO_M_PI;

        hfi_calibration_.samples = 0;
        hfi_calibration_.I_real_min = 0.0f;
        hfi_calibration_.I_real_max = 0.0f;
        hfi_calibration_.enable_filter = true;
        for(int i = 0; i < 3; i++) {
            hfi_calibration_.filter_state[i] = 0.0f;
        }

        hfi_run_calibration_integrator_ = false;

        axis_->run_control_loop([&]() {

            // Break the feedback loop and set the test angle equal to our 
            // lock-in current angle
            hfi_closed_feedback_loop_ = false;
            hfi_test_signal_injection_angle_ = wrap_pm_pi(test_phase + HALF_M_PI);

            if (!axis_->motor_.update(config_.hfi_quiescent_current, 0.0f, 0.0f, 0.0f)) {
                return false;
            }

            test_distance += test_velocity * current_meas_period;
            test_phase = wrap_pm_pi(test_phase + test_velocity * current_meas_period);

            if(test_distance >= finish_distance) {
                hfi_run_calibration_integrator_ = false;
                return false;
            }
            else {
                hfi_run_calibration_integrator_ = true;
                return true;
            }
        }); 

        float const signal_to_noise_ratio = (hfi_calibration_.I_real_max - hfi_calibration_.I_real_min) / hfi_noise_floor;

        if(signal_to_noise_ratio > config_.test_signal_desired_snr) {
            break;
        }
        else if (config_.test_signal_voltage + 0.1f > vbus_voltage / 2.0f) {
            break;
        }
        else {
            voltage_loop_counts ++;
        }
    }

    // Find "I_c" angle-dependent nonlinearity --------------------------------------------

    // Step size for calibration table
    float calibration_current_increment = config_.hfi_max_calibration_current / HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS;

    // Inverse increment
    config_.calibration_current_inv_increment = 1.0f / calibration_current_increment;

    for(    uint8_t test_current_interval = 0; 
            test_current_interval < HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS; 
            test_current_interval++) {

        axis_->controller_.direct_mode_direct_current_ = test_current_interval * calibration_current_increment;
        float const test_current = config_.hfi_quiescent_current + axis_->controller_.direct_mode_direct_current_;
        float const test_velocity = 4.0f;
        float test_distance = 0.0f;
        float test_phase = 0.0f;
        float const finish_distance = TWO_M_PI;

        float I_real_nonlinearity_integrator = 0.0f;
        /*
        float I_imag_nonlinearity_integrator = 0.0f;
        for(int i = 0; i < 3; i++) {
            hfi_I_real_nonlinearity_table_filter_state_[i] = 0.0f;
            hfi_I_imag_nonlinearity_table_filter_state_[i] = 0.0f;
        }
        */

        uint16_t compensation_phase_angle = 0.0f;
        uint16_t compensation_phase_angle_previous = 0.0f;
        uint16_t compensation_phase_samples_taken = 0.0f;

        hfi_closed_feedback_loop_ = true;

        axis_->run_control_loop([&]() {

            if (!axis_->motor_.update(test_current, 0.0f, test_phase, 0.0f)) {
                return false;
            }

            compensation_phase_angle_previous = compensation_phase_angle;
            compensation_phase_angle = hfi_I_phase_to_compensation_angle(test_phase);
            
            if(compensation_phase_angle_previous == compensation_phase_angle){
                compensation_phase_samples_taken ++;
                I_real_nonlinearity_integrator += lp_butterworth_2_35_8k(hfi_I_real_nonlinearity_table_filter_state_, hfi_I_differential_real_average_);
                //I_imag_nonlinearity_integrator += lp_butterworth_2_20_8k(hfi_I_imag_nonlinearity_table_filter_state_, hfi_I_differential_imag_average_);
            } 
            else {
                config_.hfi_I_real_nonlinearity_table[compensation_phase_angle_previous][test_current_interval] 
                    = I_real_nonlinearity_integrator / compensation_phase_samples_taken;
                //config_.hfi_I_imag_nonlinearity_table[phase_angle_previous][test_current_interval] = I_imag_nonlinearity_integrator / phase_samples_taken;

                compensation_phase_samples_taken = 0;
                I_real_nonlinearity_integrator   = 0.0f;
                //I_imag_nonlinearity_integrator = 0.0f;                
            }

            test_distance += test_velocity * current_meas_period;
            test_phase = wrap_pm_pi(test_phase + test_velocity * current_meas_period);

            if(test_distance >= finish_distance) {
                return false;
            }
            else {
                return true;
            }
        }); 
    }

    // Find min/max values of "I_c" signal ----------------------------------------------------------------
    for(    uint8_t test_current_interval = 0; 
            test_current_interval < HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS;
            test_current_interval++) {

        float const test_velocity = 8.0f;
        float test_distance = 0.0f;
        float test_phase = 0.0f;
        float const finish_distance = TWO_M_PI;

        hfi_calibration_.samples = 0;
        hfi_calibration_.I_real_min = 0.0f;
        hfi_calibration_.I_real_max = 0.0f;
        hfi_calibration_.enable_filter  = true;
        for(int i = 0; i < 3; i++) {
            hfi_calibration_.filter_state[i] = 0.0f;
        }

        hfi_run_calibration_integrator_ = false;

        axis_->run_control_loop([&]() {

            // Break the feedback loop and set the test angle equal to our 
            // lock-in current angle
            hfi_closed_feedback_loop_ = false;
            hfi_test_signal_injection_angle_ = wrap_pm_pi(test_phase + HALF_M_PI);

            axis_->controller_.direct_mode_direct_current_ = test_current_interval * calibration_current_increment;

            if (!axis_->motor_.update(config_.hfi_quiescent_current + axis_->controller_.direct_mode_direct_current_, 0.0f, 0.0f, 0.0f)) {
                return false;
            }

            test_distance += test_velocity * current_meas_period;
            test_phase = wrap_pm_pi(test_phase + test_velocity * current_meas_period);

            if(test_distance >= finish_distance) {
                hfi_run_calibration_integrator_ = false;
                return false;
            }
            else {
                hfi_run_calibration_integrator_ = true;
                return true;
            }
        }); 

        // Store min and max values
        config_.hfi_I_real_norm_[0][test_current_interval] = - hfi_calibration_.I_real_min;
        config_.hfi_I_real_norm_[1][test_current_interval] = + hfi_calibration_.I_real_max;

        //config_.hfi_I_imag_norm_[0][test_current_interval] = - hfi_calibration_.I_imag_min;
        //config_.hfi_I_imag_norm_[1][test_current_interval] = + hfi_calibration_.I_imag_max;
    }

    // Close the feedback loop again
    axis_->controller_.direct_mode_direct_current_ = 0.0f;
    hfi_closed_feedback_loop_ = true;

    // Restore cached values
    config_.hfi_enabled = cahced_hfi_enabled;

    // Set flag to OK (@todo we should do some sanity checks here...)
    config_.hfi_calibrated = true;

    return true;    
}


void SensorlessEstimator::sample_hfi(float * I_phase, float * I_alpha, float * I_beta) {

    // These variables will store our HFI signal current
    float hfi_I_alpha[3], hfi_I_beta[3];

    // First we need a high pass filter to remove I_f currents which are contributed from 
    // Iq_setpoint and Id_setpoint of the controller. Filter out the low frequency motor 
    // currents which are below our HFI current frequencies. We do this by tracking the 
    // slow currents and subtracting them from our measurements.
    for(int i = 0; i < 3; i++) {
        hfi_I_alpha[i] = I_alpha[i] - lp_butterworth_2_500_24k(hfi_I_alpha_filter_state_, I_alpha[i]);
        hfi_I_beta[i]  = I_beta[i] - lp_butterworth_2_500_24k(hfi_I_beta_filter_state_, I_beta[i]);
    }

    // Now we must subtract the HFI response from the input currents so that we can 
    // run the rest of the FOC loop without salting our controller with HFI noise
    for(int i = 0; i < 3; i++) {
        I_alpha[i] -= hfi_I_alpha[i];
        I_beta[i] -= hfi_I_beta[i];
    }

    // Pass these values along to the BEMF estimator
    hfi_cleaned_I_alpha_ = (I_alpha[0] + I_alpha[1] + I_alpha[2]) / 3.0f;
    hfi_cleaned_I_beta_ = (I_beta[0] + I_beta[1] + I_beta[2]) / 3.0f;

    // The sensorless estimator on loop N + 1 will be looking for the oldest values made in 
    // generate_new_hfi_modulations(). The *newest* values were stored at offset 6 if 
    // hfi_buffer_ping_pong_ was high for loop N-1, and 0 if it was low.
    bool hfi_trig_values_buffer_offset_;

    if(hfi_buffer_ping_pong_) {
        // Newest values are at index 6 if hfi_buffer_ping_pong_ was TRUE on loop N-1,
        // therefore oldest values are at index 0.
        hfi_trig_values_buffer_offset_ = 6;
    } 
    else {
        // Newest values are at index 0 if hfi_buffer_ping_pong_ was FALSE on loop N-1,
        // therefore oldest values are at index 6.            
        hfi_trig_values_buffer_offset_ = 0;
    }

    // I_c is our current from our test carrier signal injected along our D axis estimate (δ_hat)
    // These variables will hold the direct and quadrature currents
    float hfi_I_c_real[3];
    float hfi_I_c_imag[3]; 

    // Our response takes the form of I_c = I_p * e^i(ω_c * t + δ_hat) + I_n * e^i(-ω_c * t + δ_hat - 2γ_u)

    // Rotate the signal by (-1 * injection_frequency [ω_c] ) to isolate the positive sequence response 
    // (I_p), then later low pass filter the negative sequence response (I_n) away.
    for(int i = 0; i < 3; i++) {

        // Rotation by -ω_c applied to I_c makes I_p a "DC" value 
        // also rotate by -δ_hat since that is a "constant" in both I_p and I_n
        /*
        // Unoptimized (creates new values, easy to understand)
        float inverse_rotation_angle = wrap_pm_pi(
            - (hfi_carrier_phases_[hfi_loop_n_minus_2_references_supposedly_correct_[i] + hfi_trig_values_buffer_offset_] 
            - hfi_test_signal_injection_angle_)
        );

        float I_c_rotation_cos = our_arm_cos_f32(inverse_rotation_angle);
        float I_c_rotation_sin = our_arm_sin_f32(inverse_rotation_angle);

        hfi_I_c_real[i] = I_c_rotation_cos * hfi_I_alpha[i] - I_c_rotation_sin * hfi_I_beta[i];
        hfi_I_c_imag[i] = I_c_rotation_cos * hfi_I_beta[i]  + I_c_rotation_sin * hfi_I_alpha[i];
        */
        
        // This is optimized to be fast by using previously calculated trig functions, but 
        // it is more terse and harder to understand. Note these transforms have opposite sign as 
        // those we used to create them in generate_new_hfi_modulations().
        uint8_t const loop_n_minus_2_reference = 
            hfi_loop_n_minus_2_references_supposedly_correct_[i] + hfi_trig_values_buffer_offset_;

        // We can re-use cos(x) directly as cos(-x) == cos(x)
        // We can re-use sin(x) too as sin(-x) == -sin(x)
        float const hfi_I_a_rotated 
            = hfi_injection_angle_cos_[loop_n_minus_2_reference] * hfi_I_alpha[i] + hfi_injection_angle_sin_[loop_n_minus_2_reference] * hfi_I_beta[i];
        
        float const hfi_I_b_rotated 
            = hfi_injection_angle_cos_[loop_n_minus_2_reference] * hfi_I_beta[i]  - hfi_injection_angle_sin_[loop_n_minus_2_reference] * hfi_I_alpha[i];

        hfi_I_c_real[i] = hfi_carrier_cos_p_[loop_n_minus_2_reference] * hfi_I_b_rotated - hfi_carrier_sin_p_[loop_n_minus_2_reference] * hfi_I_a_rotated;
        hfi_I_c_imag[i] = hfi_carrier_cos_p_[loop_n_minus_2_reference] * hfi_I_a_rotated + hfi_carrier_sin_p_[loop_n_minus_2_reference] * hfi_I_b_rotated;
    }

    // These variables will hold our positive sequence currents (I_p) which we filter from (I_c)
    float hfi_I_p_real_DC_filtered[3], hfi_I_p_imag_DC_filtered[3];

    // These variables will hold our (rotated) negative sequence currents (I_n) which we filter from (I_c)
    float hfi_I_n_real_2_omega_c[3], hfi_I_n_imag_2_omega_c[3];

    // Low pass filter to suppress negative sequence current (now at -2 ω_c frequency because we rotated it)
    for(int i = 0; i < 3; i++) {    

        // Application of this filter isolates I_p from I_c = I_p + I_n * e^i(-2ω_c * t - 2γ_u)
        hfi_I_p_real_DC_filtered[i] = lp_butterworth_2_250_24k(hfi_I_p_real_DC_filter_state_[0], hfi_I_c_real[i]);
        hfi_I_p_imag_DC_filtered[i] = lp_butterworth_2_250_24k(hfi_I_p_imag_DC_filter_state_[0], hfi_I_c_imag[i]);

        // Roll-off of first filter not good enough, and 4th order was unstable so use a second 2-order one
        hfi_I_p_real_DC_filtered[i] = lp_butterworth_2_250_24k(hfi_I_p_real_DC_filter_state_[1], hfi_I_p_real_DC_filtered[i]);
        hfi_I_p_imag_DC_filtered[i] = lp_butterworth_2_250_24k(hfi_I_p_imag_DC_filter_state_[1], hfi_I_p_imag_DC_filtered[i]);

        // We can isolate I_n * e^i(-2ω_c * t - 2γ_u) by subtracting I_p from I_c
        hfi_I_n_real_2_omega_c[i] = hfi_I_c_real[i] - hfi_I_p_real_DC_filtered[i];
        hfi_I_n_imag_2_omega_c[i] = hfi_I_c_imag[i] - hfi_I_p_imag_DC_filtered[i];
    }

    // These variables store our (I_n) response transformed to non-rotating coordinates
    float hfi_I_n_real_DC[3];
    //float hfi_I_n_imag_DC[3];

    // These variables store our (I_n) response, low pass filtered which is apparently 
    // required or the signal is unintelligable. I don't know why yet.
    float hfi_I_n_real_DC_filtered[3];
    //float hfi_I_n_imag_DC_filtered[3];

    // Now rotate I_n * e^i(-2ω_c * t - 2γ_u) by +2ω_c to get a non-rotating (I_n) response
    for(int i = 0; i < 3; i++) {

        // Calculate new trig angles
        float forward_2f_rotation_angle = wrap_pm_pi(
            2 * hfi_carrier_phases_[hfi_loop_n_minus_2_references_supposedly_correct_[i] 
            + hfi_trig_values_buffer_offset_]
        );

        float const I_n_forward_cos = our_arm_cos_f32(forward_2f_rotation_angle);
        float const I_n_forward_sin = our_arm_sin_f32(forward_2f_rotation_angle);

        // Apply the rotation
        hfi_I_n_real_DC[i] = I_n_forward_cos * hfi_I_n_real_2_omega_c[i] + I_n_forward_sin * hfi_I_n_imag_2_omega_c[i];
        //hfi_I_n_imag_DC[i] = I_n_forward_cos * hfi_I_n_imag_2_omega_c[i] - I_n_forward_sin * hfi_I_n_real_2_omega_c[i];

        hfi_I_n_real_DC_filtered[i] = lp_butterworth_2_250_24k(hfi_I_n_real_DC_filter_state_, hfi_I_n_real_DC[i]);
        //hfi_I_n_imag_DC_filtered[i] = lp_butterworth_2_250_24k(hfi_I_n_imag_DC_filter_state_, hfi_I_n_imag_DC[i]);
    }

    // These will hold the mean of all 3 I_c samples 
    float hfi_I_differential_real_average = 0;
    //float hfi_I_differential_imag_average = 0;  

    // Take the mean of all HFI samples 
    for(int i = 0; i < 3; i++) {
        hfi_I_differential_real_average += hfi_I_p_real_DC_filtered[i] - hfi_I_n_real_DC_filtered[i];    
        //hfi_I_differential_imag_average += hfi_I_n_imag_DC_filtered[i] + hfi_I_p_imag_DC_filtered[i];   
    }

    hfi_I_differential_real_average_ = hfi_I_differential_real_average / 3.0f;
    //hfi_I_differential_imag_average_ = hfi_I_differential_real_average / 3.0f;

    #ifdef HFI_DEBUGING_INSPECT_IC_RESPONSE
        debug_[0] = hfi_I_differential_real_average_;
        //debug_[1] = hfi_I_differential_imag_average_;
    #endif
}


void SensorlessEstimator::update_hfi(float * I_d_measured) {

    // States: ---------------------------------------------------------------------------

    // We now must guage the misalingment of our rotor with I_phase. Technically, we can 
    // extract this data directly from the imaginary portion of our test currents I_c, but, 
    // the estimator allows us to use bandwidth limiting and (eventually) a predictive model 
    // to get a more trustworthy measurement.

    // Hidden states:
    // φ_elec 
    // φ_rotor
    // Δω = d/dt(Δφ)
    // Δα = d/dt(Δω)

    // Visible states:
    // Δφ   =   asinf(δ_real / |δ_real|) - f(φ_elec) - f(Is)
    // Is_d =   some_k * δ_real / cos(Δφ) ...maybe
    // Is_q =   some_k * δ_imag / sin(Δφ) ...maybe

    // Output control vector(s):
    // Is = Kt * Δφ

    // Where:
    // φ_elec   = Stator electrical angle            
    // φ_rotor  = Rotor angle
    // Δφ       = Rotor misalignment
    // δ        = HFI measurement signal
    // δ_real   = δ_p_real - δ_n_real
    // Δω       = Rotor misalignment velocity
    // Δα       = Rotor misalignment acceleration
    // Is       = Stator current
    // Is_d     = Stator current aligned with rotor
    // Is_q     = Stator current quadrature to rotor
    // Kt       = Some motor torque constant

    // State vector (X) for this system:
    // [Δφ]
    // [Δω]
    // [α ]
    // [Iq]

    // Recover last stator current phasor as an index for our compensation map
    float const hfi_stashed_I_phase  = hfi_stashed_injection_I_phases_[hfi_buffer_ping_pong_];

    // This line is technically correct but it was unstable in real life 
    //uint16_t const compensation_angle    = hfi_I_phase_to_compensation_angle(wrap_pm_pi(hfi_stashed_I_phase + hfi_delta_phase_estimate_));

    // This line is an OK approximation but can get strange at large load angles
    uint16_t const compensation_angle = hfi_I_phase_to_compensation_angle(hfi_stashed_I_phase);
    
    // Get the last "d" current we used in the controller
    float const hfi_stashed_d_current = hfi_stashed_d_currents_[hfi_buffer_ping_pong_];
    float const compensation_table_current_index = hfi_stashed_d_current * config_.calibration_current_inv_increment;
    
    // Linear interpolate the smallest dimension of our table. @todo, do some experiments
    // to see whether or not using a bi-linear interpolation is better.
    float const hfi_I_differential_real_nonlinearity 
        = linear_interpolate_lookup_table(compensation_table_current_index, config_.hfi_I_real_nonlinearity_table[compensation_angle], HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS);

    //float const hfi_I_differential_imag_nonlinearity 
    //    = linear_interpolate_lookup_table(compensation_table_current_index, config_.hfi_I_imag_nonlinearity_table[compensation_angle]);

//    debug_[3] = hfi_I_differential_real_average_;
//    debug_[5] = hfi_I_differential_real_nonlinearity;

    float const hfi_I_differential_real_compensated = hfi_I_differential_real_average_ - hfi_I_differential_real_nonlinearity;
    //float const hfi_I_differential_imag_compensated = hfi_I_differential_imag_average_ - hfi_I_differential_imag_nonlinearity;

    // Normalize δ_real using values previously found in calibration routine
    float const hfi_delta_phase_measurement_norm
        = linear_interpolate_lookup_table(compensation_table_current_index, config_.hfi_I_real_norm_[(bool)(hfi_I_differential_real_compensated > 0.0f)], HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS);
    
    float hfi_delta_phase_measurement_normalized = hfi_I_differential_real_compensated / hfi_delta_phase_measurement_norm;

    // Constrain so asinf doesn't NaN
    hfi_delta_phase_measurement_normalized = MACRO_CONSTRAIN(hfi_delta_phase_measurement_normalized, -1.3f, 1.3f);

    // Linearize error to represent angle (scaled by some constant)
    float const hfi_delta_phase_measurement_linearized = fast_asinf(hfi_delta_phase_measurement_normalized);

    // Calculate a phase velocity by the difference from the last state
    // I don't think this gives us any more new information?
    float const hfi_delta_vel_measurement_linearized 
        = hfi_delta_phase_measurement_linearized - hfi_delta_phase_measurement_linearized_last_;

    // Save old state
    hfi_delta_phase_measurement_linearized_last_ = hfi_delta_phase_measurement_linearized;

    if(config_.hfi_calibrated) {

        // Prediction =========================================================================
        // A = System (transition) matrix -----------------------------------------------------
        // Calculate new state prediction based on predictions from old state and
        // intrinsic time-invariant dynamics of the system

        // [Δφ_est] += Δω_est * dt
        // [Δω_est] += α_est * dt
        // [α_est ] += 0
        // [Id_est] += 0

        // B = Input matrix -------------------------------------------------------------------
        // Calculate new state prediction based on how our (N-1) control vector u(t) was 
        // anticipated to change the system state.
        //
        // [Δφ_est]    [0  0  0  0]   [Δφ_out](N-1)
        // [Δω_est] += [0  0  0  0] * [Δω_out](N-1)
        // [α_est ]    [0  0  0  0]   [α_out ](N-1)
        // [Id_est]    [0  0  0  1]   [Id_out](N-1)

        // Where:
        // J  = Moment of inertia
        // Kt = Motor torque constant 
        // Iq = Motor current
        // w  = Process noise

        hfi_d_current_estimate_   += hfi_stashed_d_current;
        hfi_phase_accel_estimate_ += 0.0f;
        hfi_delta_vel_estimate_   += hfi_phase_accel_estimate_ * current_meas_period;
        hfi_delta_phase_estimate_ += hfi_delta_vel_estimate_ * current_meas_period;

        // Update =============================================================================
        // C = Output matrix ------------------------------------------------------------------
        // Determine the relationships between the system state and the system output 

        // [L1   0   0   0]  (Δφ)
        // [ 0  L2   0   0]  (Δω)
        // [ 0   0   0   0]  (α )
        // [ 0   0   0  L4]  (Iq)

        float const hfi_L1 = config_.hfi_torque_gain;
        float const hfi_L2 = (hfi_L1 * hfi_L1) * 0.25f;
        float const hfi_L4 = 1.0f;

        // D = Feedforward --------------------------------------------------------------------
        // We have a quiescent current in the motor so we know it must be subtracted

        // [Δφ_D]   [Δφ_est]   [0]
        // [Δω_D] = [Δω_est] - [0]
        // [α_D ]   [α_est ]   [0]  
        // [Id_D]   [Id_est]   [quiescent current]

        // Z (innovation, aka residual): X_est - X measured -----------------------------------
        // Get a measurement of the (presumed) error of our prediction, of how the state has 
        // changed since loop(N-1)

        // [Δφ_z]   [Δφ_est]   [Δφ]
        // [Δω_z] = [Δω_est] - [Δω]
        // [α_z ]   [α_est ]   [0 ]  
        // [Id_z]   [Id_est]   [0 ]

        float const residual_d_current_error   = (* I_d_measured) - hfi_d_current_estimate_; 
        float const residual_delta_vel_error   = hfi_delta_vel_measurement_linearized - hfi_delta_vel_estimate_;
        float const residual_delta_phase_error = hfi_delta_phase_measurement_linearized - hfi_delta_phase_estimate_;

        // Update the state estimate: X_est += C * Z + D u(t) ---------------------------------
        
        // [Δφ_est]    [L1   0  0   0]   [Δφ_z]
        // [Δφ_est] =  [ 0  L2  0   0] * [Δω_z] 
        // [α_est ]    [ 0   0  0   0]   [α_z ]
        // [Id_est]    [ 0   0  0  L4]   [Id_z] 

        // Update states
        hfi_d_current_estimate_   += hfi_L4 * residual_d_current_error - config_.hfi_quiescent_current;
        hfi_delta_vel_estimate_   += hfi_L2 * residual_delta_vel_error * current_meas_period;
        hfi_delta_phase_estimate_ += hfi_L1 * residual_delta_phase_error * current_meas_period;

        // Constrain states
        //hfi_delta_vel_estimate_ = MACRO_CONSTRAIN(hfi_delta_vel_estimate_, -config_.hfi_max_velocity, config_.hfi_max_velocity);
        hfi_delta_phase_estimate_ = MACRO_CONSTRAIN(hfi_delta_phase_estimate_, -HALF_M_PI, HALF_M_PI);

        // Control vector =======================================================================
        // Controller output: u(t) --------------------------------------------------------------

        // Output current on the direct axis. I tried closing the loop using Q current but it was just too unstable.
        // The problem is that, Q current changes our angle estimate in a way that changes the error signal without 
        // physically reducing the real rotor mis-alignment, and as a result, the error signal starts to decouple from
        // reality making the controller unstable. It is not clear to me that any solution exists for this problem, 
        // I don't think it does, unless we observe more information somehow. 

        float drive_current_lim = axis_->controller_.effective_current_limit_ - config_.hfi_quiescent_current - 5.0f;

        // Still not sure how to "properly" calculate this, is it motor Kt?
        float const some_torque_constant = 1.0f * drive_current_lim;

        // Multiplying the output signal by sin(error) reduces the gradient of the correction when the error is close 
        // to zero and encourages the controller to not over-correct for small deflections. 
        float direct_current = our_arm_sin_f32(hfi_delta_phase_estimate_) * some_torque_constant * hfi_delta_phase_estimate_;

        // Constrain the output current so as to not trip the over current limits of the system.
        direct_current = MACRO_CONSTRAIN(direct_current, -drive_current_lim, drive_current_lim);
        
        // It was found by experiment that an asymmetric integrator makes the system more stable than a 
        // proportional output. 
        if(direct_current > hfi_d_current_filter_) {
            // Increase I_D rapidly
            hfi_d_current_filter_ = direct_current;
        } 
        else {
            // Decay I_D slowly
            hfi_d_current_filter_ *= 1.0f - (config_.hfi_torque_decay * current_meas_period);
        }

        // Finally, increase the direct current to correct for a misailgned rotor. 
        axis_->controller_.direct_mode_direct_current_ = hfi_d_current_filter_;

        #if defined(HFI_DEBUGGING_INSPECT_DRIVE_CURRENT)
            debug_[0] = hfi_I_differential_real_compensated;
            //debug_[1] = hfi_I_differential_imag_compensated;
        #endif

        #if defined(HFI_DEBUGGING_INSPECT_ESTIMATOR_DC_CAL_RESULTS)
            /*
            if(axis_->loop_seconds_counter_ % 10 >= 5){
                debug_[0] = hfi_I_real_DC_filtered_average;
                debug_[1] = config_.hfi_I_real_DC_offset;
                debug_[2] = hfi_I_real_DC_filtered_average_compensated;
                debug_[3] = hfi_I_imag_DC_filtered_average;
                debug_[4] = config_.hfi_I_imag_DC_offset;
                debug_[5] = hfi_I_imag_DC_filtered_average_compensated;                   
            }
            else {
            */
                //debug_[0] = axis_->controller_.direct_mode_torque_current_ ;
                //debug_[1] = d_current;//hfi_q_current_filter_;
                //debug_[2] = hfi_delta_phase_estimate_ * some_torque_constant;
                //debug_[3] = hfi_I_differential_imag_compensated * some_torque_constant;//axis_->controller_.direct_mode_torque_current_;//hfi_q_current_filter_;
                //debug_[4] = (hfi_I_differential_real_compensated + hfi_I_differential_imag_compensated) * some_torque_constant;//d_current;
                
                debug_[0] = vel_estimate_filtered_;
                debug_[1] = hfi_stashed_d_current;
                debug_[2] = hfi_I_differential_real_average_;
                //debug_[2] = hfi_I_differential_real_nonlinearity;
                //debug_[3] = hfi_cleaned_I_alpha_;

                //debug_[3] = hfi_stashed_d_current;
                //debug_[4] = hfi_delta_phase_measurement_linearized;

                //debug_[3] = hfi_d_current_estimate_ / 10.0f;

                debug_[4] = pll_pos_;
                //debug_[5] = 0;//hfi_stashed_I_phase;

                //debug_[4] = hfi_I_n_imag_DC_filtered_average;
                //debug_[5] = hfi_I_differential_real_average_bias_removed_;//current / 10.0f;
                //debug_[5] = hfi_delta_phase_estimate_;
                //debug_[5] = hfi_I_imag_DC_filtered_average_compensated;
            //}         
        #elif defined(HFI_DEBUGGING_INSPECT_ESTIMATOR)
            if(config_.hfi_calibrated) {
                debug_[0] = hfi_delta_phase_estimate_;
                debug_[1] = hfi_delta_vel_estimate_;
                debug_[2] = current / 10.0f;
            }
        #endif
    }

    #if defined(HFI_DEBUGGING_INSPECT_ESTIMATOR) || defined(HFI_DEBUGGING_INSPECT_ESTIMATOR_DC_CAL_RESULTS)
        if(!config_.hfi_calibrated) {
            debug_[0] = hfi_I_differential_real_average_;
            debug_[1] = hfi_I_differential_imag_average_;
            debug_[2] = axis_->controller_.direct_mode_direct_current_ / 10.0f;
        }
    #endif         


    // The following code is used to find the norm and possibly other characterisitics
    // about our test signals (I_c). It is run once for every new motor.
    if(hfi_run_calibration_integrator_) {
        hfi_calibration_.samples ++;

        // Filter so we don't get noisy data.
        float hfi_I_differential_real_norm_filtered;

        if(hfi_calibration_.enable_filter) {
            hfi_I_differential_real_norm_filtered 
                = lp_butterworth_2_35_8k(hfi_calibration_.filter_state, hfi_I_differential_real_compensated);
        }
        else {
            hfi_I_differential_real_norm_filtered = hfi_I_differential_real_compensated;
        }

        // Find the norm of I_real.
        if(hfi_calibration_.I_real_max < hfi_I_differential_real_norm_filtered) {
            hfi_calibration_.I_real_max = hfi_I_differential_real_norm_filtered;
        }
        else if(hfi_calibration_.I_real_min > hfi_I_differential_real_norm_filtered) {
            hfi_calibration_.I_real_min = hfi_I_differential_real_norm_filtered;
        }

        // Find the norm of I_imag
        /*
        if(hfi_calibration_.I_imag_max < hfi_I_differential_imag_compensated) {
            hfi_calibration_.I_imag_max = hfi_I_differential_imag_compensated;
        }
        else if(hfi_calibration_.I_imag_min > hfi_I_differential_imag_compensated) {
            hfi_calibration_.I_imag_min = hfi_I_differential_imag_compensated;
        }

        hfi_calibration_.I_real += hfi_I_differential_real_compensated;
        hfi_calibration_.I_imag += hfi_I_differential_imag_compensated;
        */
    }   

    // Toggle ping_pong_
    hfi_buffer_ping_pong_ = !hfi_buffer_ping_pong_;
}


void SensorlessEstimator::generate_new_hfi_modulations(float * I_phase, float * V_to_mod, float * hfi_modulation_index, float * hfi_alphas, float * hfi_betas) {

    // We need to save the old trig functions we calculate on the last iteration of this loop
    // so that we can use them for inverse transformations later. Instead of shuffeling them through
    // memory for no reason we use a ping-pong buffer for speed

    // Store I_phase for later
    if(hfi_closed_feedback_loop_) {
        hfi_test_signal_injection_angle_ = wrap_pm_pi(HALF_M_PI + (* I_phase));
    }

    hfi_stashed_injection_angles_[hfi_buffer_ping_pong_]   = hfi_test_signal_injection_angle_;
    hfi_stashed_injection_I_phases_[hfi_buffer_ping_pong_] = (* I_phase);
    hfi_stashed_d_currents_[hfi_buffer_ping_pong_]         = axis_->controller_.direct_mode_direct_current_;

    uint8_t hfi_buffer_offset;

    // The sensorless estimator on loop N + 1 will be looking for the oldest values. We should
    // store the *newest* values at an index which alternates based on the state of ping_pong_
    // such that the *oldest* values will be at the opposite hfi_buffer_ping_pong_ state.
    if(hfi_buffer_ping_pong_) {
        // Store newest values at index 6 if hfi_buffer_ping_pong_ is TRUE
        hfi_buffer_offset = 6;
    }
    else {
        // Store newest values at index 0 if hfi_buffer_ping_pong_ is FALSE
        hfi_buffer_offset = 0;
    }

    // Calculate six new HFI modulations
    for (uint8_t i = 0; i < 6; i++) {

        // Offset for ping-pong buffer
        uint8_t j = hfi_buffer_offset + i;

        // Cache our injection angle (estimate)
        hfi_injection_angles_[j] = hfi_test_signal_injection_angle_;

        // Generate our HFI carrier signal (u_c)
        // We can share this calculation between axis0 and axis1 by making it 
        // global above the estimator class

        // Calculate our next HFI carrier phase
        hfi_carrier_phase_ -= hfi_modulation_phase_increment_;

        // Wrap if out of bounds
        hfi_carrier_phase_ = wrap_pm_pi(hfi_carrier_phase_);

        // Store the new carrier HFI phase (ω_c)
        hfi_carrier_phases_[j] = hfi_carrier_phase_;         
        hfi_carrier_sin_p_[j] = our_arm_sin_f32(hfi_carrier_phase_);
        hfi_carrier_cos_p_[j] = our_arm_cos_f32(hfi_carrier_phase_);

        // Test vector max modulation
        (*hfi_modulation_index) = config_.test_signal_voltage * (*V_to_mod);        

        // Unoptimized and good for asymmetric vectors ------------------------------
        /*
        // Test vector amplitudes
        (*hfi_modulation_index) = config_.test_signal_voltage * (*V_to_mod);        
        float const hfi_pos_sequence_voltage_vector_magnitude = (*hfi_modulation_index);
        float const hfi_neg_sequence_voltage_vector_magnitude = (*hfi_modulation_index);

        // Trig rotations for two test vectors
        float hfi_pos_sequence_voltage_vector_amplitude_real =   hfi_carrier_sin_p_[j] * hfi_pos_sequence_voltage_vector_magnitude;
        float hfi_neg_sequence_voltage_vector_amplitude_real = - hfi_carrier_sin_p_[j] * hfi_neg_sequence_voltage_vector_magnitude;

        // Calculate superposition of test vectors
        // Imaginary component has no sign change so we can optimize the calculation
        float hfi_asymmetric_test_vector_real = hfi_pos_sequence_voltage_vector_amplitude_real + hfi_neg_sequence_voltage_vector_amplitude_real;
        float hfi_asymmetric_test_vector_imag = hfi_carrier_cos_p_[j] * (hfi_pos_sequence_voltage_vector_magnitude + hfi_neg_sequence_voltage_vector_magnitude);             

        // Find sin, cos for our injected carrier mapping to stator field
        hfi_injection_angle_sin_[j] = our_arm_sin_f32(hfi_test_signal_injection_angle_);
        hfi_injection_angle_cos_[j] = our_arm_cos_f32(hfi_test_signal_injection_angle_);

        // Rotate our test signal to the estimated D axis angle
        hfi_alphas[i] = hfi_injection_angle_cos_[j] * hfi_asymmetric_test_vector_real - hfi_injection_angle_sin_[j] * hfi_asymmetric_test_vector_imag;
        hfi_betas[i]  = hfi_injection_angle_cos_[j] * hfi_asymmetric_test_vector_imag + hfi_injection_angle_sin_[j] * hfi_asymmetric_test_vector_real;
        */

        // This optimization assumes symmetric test signals ---------------------------------
        float hfi_asymmetric_test_vector_imag = hfi_carrier_cos_p_[j] * 2.0f * (*hfi_modulation_index);             

        // Find sin, cos for our injected carrier mapping to stator field
        hfi_injection_angle_sin_[j] = our_arm_sin_f32(hfi_test_signal_injection_angle_);
        hfi_injection_angle_cos_[j] = our_arm_cos_f32(hfi_test_signal_injection_angle_);

        // Rotate our test signal to the estimated D axis angle
        hfi_alphas[i] = - hfi_injection_angle_sin_[j] * hfi_asymmetric_test_vector_imag;
        hfi_betas[i]  =   hfi_injection_angle_cos_[j] * hfi_asymmetric_test_vector_imag;
    }
}