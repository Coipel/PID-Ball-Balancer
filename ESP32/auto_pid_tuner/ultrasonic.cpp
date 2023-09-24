// Ultrasonic sensor library made by Anthony Coipel

#include <Arduino.h>
#include "ultrasonic.h"

void UltraSonic::attach(int trig_pin, int echo_pin) {
  UltraSonic::trig_pin = trig_pin;
  UltraSonic::echo_pin = echo_pin;
  pinMode(UltraSonic::trig_pin, OUTPUT);
  pinMode(UltraSonic::echo_pin, INPUT);
}

float UltraSonic::read_cm() {
  digitalWrite(UltraSonic::trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltraSonic::trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltraSonic::trig_pin, LOW);

  float duration_us = pulseIn(UltraSonic::echo_pin, HIGH);
  return duration_us/58.309;
}
