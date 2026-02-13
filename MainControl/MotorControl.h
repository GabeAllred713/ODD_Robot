 /*#pragma once
#include <Arduino.h>

class MotorControl {
public:
  MotorControl(uint8_t pwm1, uint8_t ina1, uint8_t inb1,
               uint8_t pwm2, uint8_t ina2, uint8_t inb2,
               float& setRPM1_ref, float& setRPM2_ref)
  : _pwm1(pwm1), _ina1(ina1), _inb1(inb1),
    _pwm2(pwm2), _ina2(ina2), _inb2(inb2),
    _sp1(setRPM1_ref), _sp2(setRPM2_ref) {}

  void begin() {
    pinMode(_pwm1, OUTPUT); pinMode(_ina1, OUTPUT); pinMode(_inb1, OUTPUT);
    pinMode(_pwm2, OUTPUT); pinMode(_ina2, OUTPUT); pinMode(_inb2, OUTPUT);
    applyPWM(0, 0);
    // default: forward
    setDirPins(_pwm1, _ina1, _inb1, true);
    setDirPins(_pwm2, _ina2, _inb2, true);
    _sp1 = 0; _sp2 = 0;
  }

  // +RPM = forward, -RPM = reverse
  // Also updates the PID setpoints to abs(RPM)
  void commandRPM(float rpm1_signed, float rpm2_signed) {
    const bool fwd1 = (rpm1_signed >= 0.0f);
    const bool fwd2 = (rpm2_signed >= 0.0f);

    // If command is ~0, just keep direction as-is and zero setpoint
    if (fabs(rpm1_signed) < 0.01f) _sp1 = 0.0f;
    else {
      setDirPins(_pwm1, _ina1, _inb1, fwd1);
      _sp1 = fabs(rpm1_signed);
    }

    if (fabs(rpm2_signed) < 0.01f) _sp2 = 0.0f;
    else {
      setDirPins(_pwm2, _ina2, _inb2, fwd2);
      _sp2 = fabs(rpm2_signed);
    }
  }

  void applyPWM(int pwm1, int pwm2) {
    analogWrite(_pwm1, (uint8_t)constrain(pwm1, 0, 255));
    analogWrite(_pwm2, (uint8_t)constrain(pwm2, 0, 255));
  }

private:
  uint8_t _pwm1, _ina1, _inb1;
  uint8_t _pwm2, _ina2, _inb2;
  float& _sp1;
  float& _sp2;

  // Your forward mapping: INA LOW, INB HIGH
  static void setDirPins(uint8_t pwm, uint8_t ina, uint8_t inb, bool forward) {
    // drop PWM before flipping direction pins
    analogWrite(pwm, 0);

    if (forward) { digitalWrite(ina, LOW);  digitalWrite(inb, HIGH); }
    else         { digitalWrite(ina, HIGH); digitalWrite(inb, LOW);  }
  }
}; */

#pragma once
#include <Arduino.h>

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

  // forward mapping: INA LOW, INB HIGH (same as your original)
  void setDirection(bool fwd1, bool fwd2) {
    if (_inv1) fwd1 = !fwd1;
    if (_inv2) fwd2 = !fwd2;

    digitalWrite(_ina1, fwd1 ? LOW  : HIGH);
    digitalWrite(_inb1, fwd1 ? HIGH : LOW);

    digitalWrite(_ina2, fwd2 ? LOW  : HIGH);
    digitalWrite(_inb2, fwd2 ? HIGH : LOW);
  }

  void applyPWM(int pwm1, int pwm2) {
    analogWrite(_pwm1, (uint8_t)constrain(pwm1, 0, 255));
    analogWrite(_pwm2, (uint8_t)constrain(pwm2, 0, 255));
  }

private:
  uint8_t _pwm1, _ina1, _inb1;
  uint8_t _pwm2, _ina2, _inb2;
  bool _inv1, _inv2;
};



