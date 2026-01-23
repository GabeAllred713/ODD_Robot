#pragma once
#include <Arduino.h>

class MotorControl {
public:
  // speed: -255..255 (negative = reverse, positive = forward, 0 = stop)
  MotorControl(uint8_t pwm1, uint8_t inA1, uint8_t inB1,
               uint8_t pwm2, uint8_t inA2, uint8_t inB2);

  void begin();

  void setMotor1(int16_t speed);
  void setMotor2(int16_t speed);
  void setSpeeds(int16_t m1Speed, int16_t m2Speed);

  void stopMotor1();
  void stopMotor2();
  void stopAll();

private:
  uint8_t _pwm1, _inA1, _inB1;
  uint8_t _pwm2, _inA2, _inB2;

  void drive(uint8_t pwm, uint8_t inA, uint8_t inB, int16_t speed);
  void coast(uint8_t pwm, uint8_t inA, uint8_t inB);
};
