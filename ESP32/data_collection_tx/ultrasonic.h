// Ultrasonic sensor library made by Anthony Coipel

class UltraSonic {
  private:
    // Default pins. Use setPins() method to change the values
    int TRIG_PIN = 9;
    int ECHO_PIN = 10;
  
  public:
    void setPins(int trig_pin, int echo_pin);
    void setupPins();
    float takeDistance_cm();
};