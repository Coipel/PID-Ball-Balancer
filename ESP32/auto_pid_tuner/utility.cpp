#include "utility.h"

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- PID Class Methods ---
void PID::set_constants(float Kp, float Ki, float Kd, float Ts) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  PID::Ts = Ts;
}

void PID::set_operation_details(float lower_saturation, float upper_saturation) {
  PID::lower_saturation = lower_saturation;
  PID::upper_saturation = upper_saturation;
  // If no zero_actuation point is provided than the lower_saturation is assumed as the zero_actuation point
  PID::offset = fmap(PID::lower_saturation, PID::lower_saturation, PID::upper_saturation, -100, 100); 
}

void PID::set_operation_details(float lower_saturation, float zero_actuation, float upper_saturation) {
  PID::lower_saturation = lower_saturation;
  PID::upper_saturation = upper_saturation;
  PID::zero_actuation = zero_actuation;
  PID::offset = fmap(PID::zero_actuation, PID::lower_saturation, PID::upper_saturation, -100, 100);
}

float PID::calculate(float error) {  
  float actuation_pct = PID::Kp*error + PID::Ki*accumulated_error*PID::Ts + PID::Kd*(error - prior_error)*(1/PID::Ts) + PID::offset;
  
  PID::accumulated_error += error;
  PID::prior_error = error;
  
  // clamps output to go from -100% to 100% of lower and upper saturation.
  if (actuation_pct > 100)
    actuation_pct = 99.999; // accouting for floating point math 
  if (actuation_pct < -100)
    actuation_pct = -99.999;

  return fmap(actuation_pct, -100, 100, PID::lower_saturation, PID::upper_saturation);
}

void PID::clear() {
  // Remove the lingering effects of a prior use of the PID class. Useful for say chaning PID constants in autotuning.
  PID::prior_error = 0;
  PID::accumulated_error = 0;
}


// --- Tuner Class Methods ---
void Tuner::set_sample_amount(int sample_amount) {
  Tuner::sample_amount = sample_amount; 
}


void Tuner::set_operation_details(float Kp_lb, float Kp_i, float Kp_ub,
                                  float Ki_lb, float Ki_i, float Ki_ub,
                                  float Kd_lb, float Kd_i, float Kd_ub,
                                  float Kz_lb, float Kz_i, float Kz_ub) {
  Tuner::Kp_lb = Kp_lb;
  Tuner::Kp_i =  Kp_i;
  Tuner::Kp_ub = Kp_ub;

  Tuner::Ki_lb = Ki_lb;
  Tuner::Ki_i =  Ki_i;
  Tuner::Ki_ub = Ki_ub;

  Tuner::Kd_lb = Kd_lb;
  Tuner::Kd_i =  Kd_i;
  Tuner::Kd_ub = Kd_ub;

  Tuner::Kz_lb = Kz_lb;
  Tuner::Kz_i =  Kz_i;
  Tuner::Kz_ub = Kz_ub;

  Tuner::Kp = Tuner::Kp_lb;
  Tuner::Ki = Tuner::Ki_lb;
  Tuner::Kd = Tuner::Kd_lb;
  Tuner::Kz = Tuner::Kz_lb;
}

void Tuner::measure_cost(float error) {
  if (sample_index == sample_amount - 1) {
    Serial.print("Trial_Number: ");
    Serial.println(Tuner::trial_number);
    Serial.print("Kp: ");
    Serial.println(Tuner::Kp);
    Serial.print("Ki: ");
    Serial.println(Tuner::Ki);
    Serial.print("Kd: ");
    Serial.println(Tuner::Kd);
    Serial.print("Kz: ");
    Serial.println(Tuner::Kz);
    Serial.print("Cost: ");
    Serial.println(Tuner::total_cost);

    Tuner::new_trial = true;
    Tuner::sample_index = 0;
    Tuner::trial_number += 1;
    Tuner::total_cost = 0;
  }
    
  else {
    Tuner::new_trial = false;
    Tuner::sample_index += 1;
    Tuner::total_cost += fabs(error);
  }
}


bool Tuner::get_new_trial() {
  return Tuner::new_trial;
}

void Tuner::update_constants() {
  Tuner::Kz += Tuner::Kz_i;
  if (Tuner::Kz > Tuner::Kz_ub) {
    Tuner::Kz = Tuner::Kz_lb;
    Tuner::Kp += Tuner::Kp_i;
  }

  if (Tuner::Kp > Tuner::Kp_ub) {
    Tuner::Kp = Tuner::Kp_lb;
    Tuner::Ki += Tuner::Ki_i;
  }
 
  if (Tuner::Ki > Tuner::Ki_ub) {
    Tuner::Ki = Tuner::Ki_lb;
    Tuner::Kd += Tuner::Kd_i;
  }

  if (Tuner::Kd > Tuner::Kd_ub) {
    Serial.println("Completed All Trials");
  }
}
