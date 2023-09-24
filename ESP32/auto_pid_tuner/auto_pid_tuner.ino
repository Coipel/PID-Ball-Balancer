#include <ESP32Servo.h>
#include "ultrasonic.h"
#include "utility.h"

bool status_tx = false;
bool status_tuning = true;

const int servo_pin = 4;
float distance_cm;

UltraSonic ultrasonic;
Servo servo;
PID pid;
Tuner tuner;

const int potentiometer_pin = 34;
long int time_start_ms;

const int average_amount = 5;
float distance_cm_record[average_amount];

const float distance_cm_lowerbound = 5.3;
const float distance_cm_upperbound = 30.5;
const float distance_cm_tolerance = 1;

const float Ts = 0.05;
float reference = 5;
float error = 0;

void setup() {
  ultrasonic.attach(25,33);
  delay(1000);

  servo.attach(servo_pin);
  pinMode(potentiometer_pin, INPUT);

  // Put tuned constants here and set status_tuning to false to test control system with said constants.
  pid.set_constants(0.60, 0.10, 0.60, 0.05); // Best numbers found so far with autotuning 0.60, 0.10, 0.60, 0.05
  pid.set_operation_details(60, 86, 180); // 60, 86, 180

  tuner.set_sample_amount(50); // 400 samples at 0.05 sample rate is 20 seconds worth of testing per trial
  tuner.set_operation_details(0, 0.2, 2,
                              0, 0.1, 0.2,
                              0, 0.2, 2,
                              85, 0.5, 86);

  Serial.begin(57600);
  time_start_ms = millis();

  // Fill the averaging array with inital readings
  for (int i = 0; i != average_amount; i++)
    distance_cm_record[i] = ultrasonic.read_cm();
}

void loop() {
  
  if (status_tuning == true) {
    // --- Start of a new trial ---
    if (tuner.get_new_trial()) {
      servo.write(pid.upper_saturation); // A clean slate where each trial the ball starts all the way to the right
      pid.clear();
      delay(1000);
      tuner.update_constants();
      pid.set_constants(tuner.Kp, tuner.Ki, tuner.Kd, 0.05);
      pid.set_operation_details(pid.lower_saturation, tuner.Kz, pid.upper_saturation);
    }
    tuner.measure_cost(error);
  }
  
  float actuation = pid.calculate(error);
  servo.write(actuation);

  // Pushback a new entry into the averaging array if it is within the reasonable range of possibility
  distance_cm = ultrasonic.read_cm();
  if (distance_cm > distance_cm_lowerbound - distance_cm_tolerance && distance_cm < distance_cm_upperbound + distance_cm_tolerance) {
    for (int i = average_amount-1; i != 0; i--) 
      distance_cm_record[i] = distance_cm_record[i-1];
    distance_cm_record[0] = distance_cm;
  }

  // Compute the average
  float sum = 0;
  for (int i = 0; i != average_amount; i++)
    sum += distance_cm_record[i];

  float distance_cm_avg = sum/average_amount;
  float x = fmap(distance_cm_avg, distance_cm_lowerbound, distance_cm_upperbound, -50, 50); // prior last two arguments: -15.81, 11.31
  long int time_sample_ms = millis();
  error = reference - x;

  if (Serial.available() > 0) {
    String incoming_message = Serial.readStringUntil('\n');
    
    if (incoming_message == "enable_tx") 
      status_tx = true;
    else if (incoming_message == "disable_tx")
      status_tx = false;
  }

  /*
  if (status_tx == true) {
    Serial.print("Actuation: ");
    Serial.println(actuation, 3);
  
    Serial.print("X: ");
    Serial.println(x, 3);
    
    Serial.print("Time_ms: ");
    Serial.println(time_sample_ms - time_start_ms);
    
    Serial.print("Error: ");
    Serial.println(error);  
  }
  */

  delay(50);
}
