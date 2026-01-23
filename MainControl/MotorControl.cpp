#include "MotorControl.h"

MotorControl::MotorControl(uint8_t pwm1, uint8_t inA1, uint8_t inB1,
                           uint8_t pwm2, uint8_t inA2, uint8_t inB2)
  : _pwm1(pwm1), _inA1(inA1), _inB1(inB1),
    _pwm2(pwm2), _inA2(inA2), _inB2(inB2) {}

void MotorControl::begin() {
  pinMode(_pwm1, OUTPUT);
  pinMode(_inA1, OUTPUT);
  pinMode(_inB1, OUTPUT);

  pinMode(_pwm2, OUTPUT);
  pinMode(_inA2, OUTPUT);
  pinMode(_inB2, OUTPUT);

  stopAll();
}

void MotorControl::setMotor1(int16_t speed) {
  drive(_pwm1, _inA1, _inB1, speed);
}

void MotorControl::setMotor2(int16_t speed) {
  drive(_pwm2, _inA2, _inB2, speed);
}

void MotorControl::setSpeeds(int16_t m1Speed, int16_t m2Speed) {
  setMotor1(m1Speed);
  setMotor2(m2Speed);
}

void MotorControl::stopMotor1() {
  coast(_pwm1, _inA1, _inB1);
}

void MotorControl::stopMotor2() {
  coast(_pwm2, _inA2, _inB2);
}

void MotorControl::stopAll() {
  stopMotor1();
  stopMotor2();
}

void MotorControl::drive(uint8_t pwm, uint8_t inA, uint8_t inB, int16_t speed) {
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    coast(pwm, inA, inB);
    return;
  }

  if (speed > 0) {
    // Forward (matches your first case: INA LOW, INB HIGH)
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    analogWrite(pwm, (uint8_t)speed);
  } else {
    // Reverse
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
    analogWrite(pwm, (uint8_t)(-speed));
  }
}

void MotorControl::coast(uint8_t pwm, uint8_t inA, uint8_t inB) {
  // Matches your stop case: INA LOW, INB LOW
  digitalWrite(inA, LOW);
  digitalWrite(inB, LOW);
  analogWrite(pwm, 0);  // actually disable PWM
}
