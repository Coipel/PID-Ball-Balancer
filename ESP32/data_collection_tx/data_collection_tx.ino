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
  float potenitometer = analogRead(potentiometer_pin); // goes from 0~4095
  float position = fmap(potenitometer, 0, 4095, 60, 180);

  // Protection because servo.write requires a value that does not exceed 180
  if (position > 180)
    position = 179.9;
  
  float actuation = fmap(position, 60, 180, -21.5, 78.5);
  
  distance_cm = ultrasonic_sensor.takeDistance_cm();
  servo.write(position);
  
  if (distance_cm > distance_cm_lowerbound - distance_cm_tolerance && distance_cm < distance_cm_upperbound + distance_cm_tolerance) {
    for (int i = average_amount-1; i != 0; i--) 
      distance_cm_record[i] = distance_cm_record[i-1];
    
    distance_cm_record[0] = distance_cm;
  }

  float sum = 0;
  for (int i = 0; i != average_amount; i++)
    sum += distance_cm_record[i];

  float distance_cm_avg = sum/average_amount;

  float x = fmap(distance_cm_avg, distance_cm_lowerbound, distance_cm_upperbound, -50, 50);
  long int time_sample_ms = millis();
  
  if (Serial.available() > 0) {
    String incoming_message = Serial.readStringUntil('\n');
    
    if (incoming_message == "enable_tx") 
      status_tx = true;
    
    else if (incoming_message == "disable_tx")
      status_tx = false;
  }

  if (status_tx == true) {
    
    Serial.print("Raw_cm: ");
    Serial.println(distance_cm, 3);

    Serial.print("Avg_cm: ");
    Serial.println(distance_cm_avg, 3);
    
    Serial.print("Actuation: ");
    Serial.println(actuation, 3);

    Serial.print("X: ");
    Serial.println(x, 3);

    Serial.print("Time_ms: ");
    Serial.println(time_sample_ms - time_start_ms);
  }

  delay(50);
}
