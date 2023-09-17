#include <ESP32Servo.h>
#include "ultrasonic.h"
#include "utility.h"

bool status_tx = false;

const int servo_pin = 4;
float distance_cm;
UltraSonic ultrasonic_sensor;

Servo servo;

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
float accumulated_error = 0;
float prior_error = 0;

// 0.35, 0.01, 0.25
// 0.2, 0.01, 0.15
// 0.175, 0.01, 0.170

const float Kp = 0.175;
const float Ki = 0.01;
const float Kd = 0.170;

void setup() {
  ultrasonic_sensor.setPins(25,33);
  ultrasonic_sensor.setupPins();
  delay(5000);

  servo.attach(servo_pin);
  pinMode(potentiometer_pin, INPUT);

  Serial.begin(57600);
  time_start_ms = millis();

  for (int i = 0; i != average_amount; i++)
    distance_cm_record[i] = ultrasonic_sensor.takeDistance_cm();
}

void loop() {
  float actuation = Kp*error + Ki*accumulated_error*Ts + Kd*(error - prior_error)*(1/Ts);

  if (actuation > 78.5) 
    actuation = 78.5;
  else if (actuation < -21.5)
    actuation = -21.5;

  accumulated_error += error;
  prior_error = error;

  float position = fmap(actuation, -21.5, 78.5, 60, 180);
  servo.write(position);

  distance_cm = ultrasonic_sensor.takeDistance_cm();

  if (distance_cm > distance_cm_lowerbound - distance_cm_tolerance && distance_cm < distance_cm_upperbound + distance_cm_tolerance) {
    for (int i = average_amount-1; i != 0; i--) 
      distance_cm_record[i] = distance_cm_record[i-1];
    distance_cm_record[0] = distance_cm;
  }

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
  
  delay(50);
}
