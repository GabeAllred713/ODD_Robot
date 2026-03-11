
#pragma once
#include <Arduino.h>
// This file controls both the motors based on enables A and B as well as the PWM signal for both motors
// Invert will flip the enable so the the motors go in opposite direction 

class MotorControl {
public:
  MotorControl(uint8_t pwm1, uint8_t ina1, uint8_t inb1,
               uint8_t pwm2, uint8_t ina2, uint8_t inb2,
               bool invert1=false, bool invert2=false)
  : _pwm1(pwm1), _ina1(ina1), _inb1(inb1),
    _pwm2(pwm2), _ina2(ina2), _inb2(inb2),
    _inv1(invert1), _inv2(invert2) {}

  void begin() {
    pinMode(_pwm1, OUTPUT); pinMode(_ina1, OUTPUT); pinMode(_inb1, OUTPUT);
    pinMode(_pwm2, OUTPUT); pinMode(_ina2, OUTPUT); pinMode(_inb2, OUTPUT);
    setDirection(true, true);
    applyPWM(0, 0);
  }

  // forward mapping: INA LOW, INB HIGH 
  void setDirection(bool fwd1, bool fwd2) {
    if (_inv1) fwd1 = !fwd1;
    if (_inv2) fwd2 = !fwd2;

    digitalWrite(_ina1, fwd1 ? LOW  : HIGH);
    digitalWrite(_inb1, fwd1 ? HIGH : LOW);

    digitalWrite(_ina2, fwd2 ? LOW  : HIGH);
    digitalWrite(_inb2, fwd2 ? HIGH : LOW);
  }

  // Applies the PWM to the motors 
  void applyPWM(int pwm1, int pwm2) {
    analogWrite(_pwm1, (uint8_t)constrain(pwm1, 0, 255));
    analogWrite(_pwm2, (uint8_t)constrain(pwm2, 0, 255));
  }

private:
  uint8_t _pwm1, _ina1, _inb1;
  uint8_t _pwm2, _ina2, _inb2;
  bool _inv1, _inv2;
};



