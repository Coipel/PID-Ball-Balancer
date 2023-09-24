#include <Arduino.h>

#pragma once

float fmap(float x, float in_min, float in_max, float out_min, float out_max);

// --- PID Class ---
class PID {
  public:
    float Kp;
    float Ki;
    float Kd;
    float Ts;

    float accumulated_error = 0;
    float prior_error = 0;

    float lower_saturation;
    float zero_actuation;
    float upper_saturation;

    float offset;

    // Ts is needed when intending to use integral and derivative terms
    void set_constants(float Kp, float Ki = 0, float Kd = 0, float Ts = 0);
    // balanced_actuation is the value where one can say that no external energy is being added to the system
    void set_operation_details(float lower_saturation, float upper_saturation);
    void set_operation_details(float lower_saturation, float zero_actuation, float upper_saturation);
    
    float calculate(float error);
    void clear();
}; 

// This Tuner is hardcoded to solve only for four constant system
class Tuner {
  public:
    float Kp, Ki, Kd, Kz;
  
    float Kp_lb, Kp_i, Kp_ub;
    float Ki_lb, Ki_i, Ki_ub;
    float Kd_lb, Kd_i, Kd_ub;
    float Kz_lb, Kz_i, Kz_ub;

    int tuning_index = -1; // -1 for no change, 0 for Kz, 1 for Kp, 2 for Ki, 3 for Kd

    int sample_amount;
    int sample_index = 0;
    float total_cost = 0;

    int trial_number = 1;
    bool new_trial = true;

    void set_sample_amount(int sample_amount);
    // array as follows [Kp_lb, Kp_i, Kp_ub, Ki_lb, Ki_i, Ki_ub, ... etc] where _lb means lower bound and _ub means upper and _i meaning incrementation
    void set_operation_details(float Kp_lb, float Kp_i, float Kp_ub,
                               float Ki_lb, float Ki_i, float Ki_ub,
                               float Kd_lb, float Kd_i, float Kd_ub,
                               float Kz_lb, float Kz_i, float Kz_ub);

    // function returns true when sample_index suggests end of sample_amount (end of sampling a trial)
    void measure_cost(float error);
    bool get_new_trial();
    void update_constants();

};



