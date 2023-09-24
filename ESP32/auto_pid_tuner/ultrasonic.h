// Ultrasonic sensor library made by Anthony Coipel
#pragma once
 
class UltraSonic {
  private:
    // Default pins. Use setPins() method to change the values
    int trig_pin;
    int echo_pin;

  public:
    void attach(int trig_pin, int echo_pin);
    float read_cm();
};