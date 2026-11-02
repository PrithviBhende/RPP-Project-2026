#include <ESP32Servo.h>

// Sesame Distro Board V1 Pinout (ESP32 WROOM-32)
// Order: R1, R2, L1, L2, R4, R3, L3, L4
int servoPins[8] = {15, 2, 23, 19, 4, 16, 17, 18};

Servo servos[8];

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 8; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 500, 2400);
    servos[i].write(90);
    delay(100);   // stagger so all 8 don't yank power at once
  }
}

void loop() {
  // nothing needed here to hold position
}