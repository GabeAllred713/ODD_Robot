// main run file

#include "IMU.h"
#include "MotorControl.h"
#include "BumpSensors.h"

IMU imu;
MotorControl motors(PIN_PWM_L, PIN_DIR_L, PIN_PWM_R, PIN_DIR_R);
BumpSensors bump(PIN_BUMP_L, PIN_BUMP_R);

// Pin Connections

#include "MotorControl.h"

constexpr uint8_t PWM1 = 9;
constexpr uint8_t INA1 = 5;
constexpr uint8_t INB1 = 4;

constexpr uint8_t PWM2 = 10;
constexpr uint8_t INA2 = 6;
constexpr uint8_t INB2 = 7;

MotorControl motors(PWM1, INA1, INB1, PWM2, INA2, INB2);

void setup() {
  motors.begin();
  Serial.begin(115200);

  // IMU setup
  if (!imu.begin(true)) {
    Serial.println("Ooops, no BNO055 detected ... Check wiring or I2C ADDR!");
    while (1) { delay(10); }
  }

  imu.setSamplePeriodMs(100);
  imu.printSensorDetails();

  
}

void loop() {

  // both forward
  motors.setSpeeds(255, 255);
  delay(5000);

  // motor1 forward, motor2 reverse (your 2nd case)
  motors.setSpeeds(255, -255);
  delay(5000);

  // stop both
  motors.stopAll();
  delay(5000);

  // IMU stuff
  if (imu.update()) {
    Serial.print("Roll: ");
    Serial.print(imu.rollDeg(), 2);
    Serial.print("  Pitch: ");
    Serial.print(imu.pitchDeg(), 2);
    Serial.print("  Yaw: ");
    Serial.println(imu.yawDeg(), 2);
  }


}
