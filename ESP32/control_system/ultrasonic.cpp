// Ultrasonic sensor library made by Anthony Coipel

#include <Arduino.h>
#include "ultrasonic.h"

void UltraSonic::setPins(int trig_pin, int echo_pin) {
  UltraSonic::TRIG_PIN = trig_pin;
  UltraSonic::ECHO_PIN = echo_pin;
}

void UltraSonic::setupPins() {
  pinMode(UltraSonic::TRIG_PIN, OUTPUT);
  pinMode(UltraSonic::ECHO_PIN, INPUT);
}

float UltraSonic::takeDistance_cm() {
  digitalWrite(UltraSonic::TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(UltraSonic::TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltraSonic::TRIG_PIN, LOW);

  float duration_us = pulseIn(UltraSonic::ECHO_PIN, HIGH);
  float distance_cm = duration_us/58.309;
  return distance_cm;
}