//#include <INA226.h>
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <GyverINA.h>
#include "MotorControl.h"
#include "WheelPID.h"
#include "IMU.h" 
#include "BumpSensors.h" 

INA226 ina;  // voltage reader object
IMU_BNO055 bno055(55, 0x28);  // IMU Object 
BumpSensors bumps(A3, A2, A1, A0); // Bump sensor object and pin assignment
//MotorControl motors(PWM1, INA1, INB1, PWM2, INA2, INB2); // Motor control object 
// ================= PID gains =================
WheelPIDGains gains1, gains2;  // PID control gains set up for left and right motor
WheelPIDState state1, state2;  // Currnt State for PID control for left and right motor

// ===== Motor L pins =====
const uint8_t PWM1 = D10; // PWM 1 (left motor)
const uint8_t INA1 = D6;  // Enable A1
const uint8_t INB1 = D7;  // Enable B1
 
// ===== Motor R pins =====
const uint8_t PWM2 = D9;  // PWM2 (Right Motor)
const uint8_t INA2 = D5;  // Enable A2
const uint8_t INB2 = D4;  // Enable B2
 
// ===== Encoder pins ===== // left motor
const uint8_t ENC_A  = D2;  // Motor 2 encoder A
const uint8_t ENC_B  = D3;  // Motor 2 encoder B
 
// Right encoder 
const uint8_t ENC2_A = D8;  // Motor 1 encoder A
const uint8_t ENC2_B = D12; // Motor 1 encoder B
 
// ===== Encoder scaling =====
const float COUNTS_PER_REV = 537.7f;
 
// ===== Encoder counters =====
volatile long encoderCounts  = 0; // Left Motor encoder counts
volatile long encoder2Counts = 0; // Right Motor encoder counts
volatile uint8_t lastAB  = 0;     // Last enable, left motor
volatile uint8_t lastAB2 = 0;     // Last enable, Right motor
MotorControl motors(PWM1, INA1, INB1, PWM2, INA2, INB2);
// Bump Sensors
int b1 = 0;  // Front right
int b2 = 0;  // Front Center
int b3 = 0;  // Front left
int b4 = 0;  // Rear

// x4 quadrature table
const int8_t quadTable[16] = {
  0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
  0, +1, -1,  0
};


// Pin assignments for encoder interupts
inline uint8_t readAB(uint8_t pinA, uint8_t pinB) {
  return (uint8_t)((digitalRead(pinA) << 1) | digitalRead(pinB));
}

// ESP32 ISR attribute
#if defined(ARDUINO_ARCH_ESP32)
  #define ISR_ATTR IRAM_ATTR
#else
  #define ISR_ATTR
#endif
 
// ---------- Encoder 1 ISR ----------
void ISR_ATTR encoderISR() {
  uint8_t ab  = readAB(ENC_A, ENC_B);
  uint8_t idx = (uint8_t)((lastAB << 2) | ab);
  encoderCounts += quadTable[idx];
  lastAB = ab;
}
 
// ---------- Encoder 2 ISR ----------
void ISR_ATTR encoder2ISR() {
  uint8_t ab2  = readAB(ENC2_A, ENC2_B);
  uint8_t idx2 = (uint8_t)((lastAB2 << 2) | ab2);
  encoder2Counts += quadTable[idx2];
  lastAB2 = ab2;
}
 
// ================= Speed setpoints =================
float setRPM1 = 0.0f;  // initial speeds
float setRPM2 = 0.0f;
 
// Signed commands from the user (can be negative)
float cmdRPM1 = 0.0f;
float cmdRPM2 = 0.0f;
 
float Vs = 0.0f;  // Intial input voltage from INA226

// Deadband 
const int PWM_DEADBAND1 = 20; //min PWM input for left and right motors
const int PWM_DEADBAND2 = 20;
 
// Timing control
const unsigned long CTRL_MS  = 300;
const unsigned long PRINT_MS = 300;
 
unsigned long lastCtrlMs  = 0;
unsigned long lastPrintMs = 0;

// Global variables for Control RPM, Actrual RPM of motors and PWM command for the motors 
long lastC1 = 0, lastC2 = 0;
float rpm1 = 0.0f, rpm2 = 0.0f;
int pwmCmd1 = 0, pwmCmd2 = 0;

int  imu_error = 0;
int  ina_error = 0;
// This function is what communicates to the Jetson, reads the serial input
bool readSerialMessage() {
  float m1 = Serial.parseFloat();
  float m2 = Serial.parseFloat();
  float m1_p = Serial.parseFloat();
  float m1_I = Serial.parseFloat();
  float m1_d = Serial.parseFloat();
  float m2_p = Serial.parseFloat();
  float m2_I = Serial.parseFloat();
  float m2_d = Serial.parseFloat();
 
  if (!Serial.find('\n')) return false;
 
  cmdRPM1 = (float)m1;
  cmdRPM2 = (float)m2;
  
  motors.setDirection(cmdRPM1 >= 0.0f, cmdRPM2 >= 0.0f);
 
  setRPM1 = fabs(cmdRPM1);
  setRPM2 = fabs(cmdRPM2);
 
  wheelPID_setGains(gains1, m1_p, m1_I, m1_d);
  wheelPID_setGains(gains2, m2_p, m2_I, m2_d);
 
  return true;
}

// This function writes data for the Jetson in a (,) seperated list
void writeSerialMessage() {
  long c1, c2;
  noInterrupts();
  c1 = encoderCounts;
  c2 = encoder2Counts;
  interrupts();
 
  // rpm1,rpm2,c1,c2,x,y,z,ax,ay,az
  Serial.print(rpm1); Serial.print(",");
  Serial.print(rpm2); Serial.print(",");
  Serial.print(c1);   Serial.print(",");
  Serial.print(c2);   Serial.print(",");
 

  // Orientation (deg): Adafruit Euler -> x=Yaw, y=Roll, z=Pitch
  Serial.print(bno055.xDeg()); Serial.print(",");
  Serial.print(bno055.yDeg()); Serial.print(",");
  Serial.print(bno055.zDeg()); Serial.print(",");
 
  // Linear acceleration (m/s^2)
  Serial.print(bno055.ax()); Serial.print(",");
  Serial.print(bno055.ay()); Serial.print(",");
  Serial.print(bno055.az()); Serial.print(",");

  Serial.print(b1); Serial.print(",");
  Serial.print(b2); Serial.print(",");
  Serial.print(b3); Serial.print(",");
  Serial.print(b4); Serial.print(",");
  Serial.print(ina.getVoltage()); Serial.print(",");
  Serial.println(bno055.temp()); // Deg C
}
 
void setup() {
  Serial.begin(115200);
  bumps.begin();
  Serial.setTimeout(50);
  Wire.setClock(400000);
  // --- INA226 ---
  if (ina.begin());//Serial.println(F("INA226 connected!"));
  else { Serial.println(F("INA226 not found!")); ina_error = 1;}
 
  Serial.println(ina.getCalibration());
  ina.setSampleTime(INA226_VBUS,  INA226_CONV_2116US);
  ina.setSampleTime(INA226_VSHUNT, INA226_CONV_8244US);
  ina.setAveraging(INA226_AVG_X4);
 
  // --- BNO055 ---
  if (bno055.begin(true)) {
    Serial.println(F("BNO055 connected!"));
    bno055.setSamplePeriodMs(50); // 20 Hz update
    bno055.printSensorDetails();
  } else {
    Serial.println(F("BNO055 not found!"));
    imu_error = 1;
  }
 
  // --- PID gains ---
  wheelPID_setGains(gains1, 0.55f, 0.001f, 0.0f);
  wheelPID_setGains(gains2, 0.55f, 0.001f, 0.0f);
 
  // --- Pins ---
  pinMode(PWM1, OUTPUT); pinMode(INA1, OUTPUT); pinMode(INB1, OUTPUT);
  pinMode(PWM2, OUTPUT); pinMode(INA2, OUTPUT); pinMode(INB2, OUTPUT);
 
  pinMode(ENC_A,  INPUT_PULLUP);
  pinMode(ENC_B,  INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
 
  // initialize last states using digitalRead
  lastAB  = readAB(ENC_A,  ENC_B);
  lastAB2 = readAB(ENC2_A, ENC2_B);
 
  // interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A),  encoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B),  encoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), encoder2ISR, CHANGE);
 
  digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH);
 
  noInterrupts();
  lastC1 = encoderCounts;
  lastC2 = encoder2Counts;
  interrupts();
 
  lastCtrlMs  = millis();
  lastPrintMs = millis();

  motors.begin();
//  Wire.setClock(400000);
}
 
void loop() {
  // Update IMU (rate-limited internally)
  bno055.update();
  bumps.update();
 // if (imu_error == 1) {
 //   Serial.println("imu not working");
 // } 

 // if (ina_error == 1) {
 //   Serial.println("ina not working");
//  } 

  b1 = bumps.read(1);
  b2 = bumps.read(2);
  b3 = bumps.read(3);
  b4 = bumps.read(4);

  if (Serial.available() > 0) {
    if (readSerialMessage()) writeSerialMessage();
  }
 
  unsigned long now = millis();
  if (now - lastCtrlMs >= CTRL_MS) {
    unsigned long dtMs = now - lastCtrlMs;
    lastCtrlMs = now;
    float dt = dtMs / 1000.0f;
 
    long c1, c2;
    noInterrupts();
    c1 = encoderCounts;
    c2 = encoder2Counts;
    interrupts();
 
    long d1 = c1 - lastC1;
    long d2 = c2 - lastC2;
    lastC1 = c1;
    lastC2 = c2;
 
    rpm1 = ((float)d1 / COUNTS_PER_REV) / dt * 60.0f;
    rpm2 = ((float)d2 / COUNTS_PER_REV) / dt * 60.0f;
 
    Vs = ina.getVoltage();
    if (!isfinite(Vs)) Vs = 12.0f;
    Vs = constrain(Vs, 5.0f, 16.8f);
 
    if (setRPM1 <= 0.01f) { pwmCmd1 = 0; wheelPID_reset(state1); }
    else pwmCmd1 = wheelPID_step(setRPM1, fabs(rpm1), state1, gains1, dt, PWM_DEADBAND1, Vs);
 
    if (setRPM2 <= 0.01f) { pwmCmd2 = 0; wheelPID_reset(state2); }
    else pwmCmd2 = wheelPID_step(setRPM2, fabs(rpm2), state2, gains2, dt, PWM_DEADBAND2, Vs);
 
    motors.applyPWM(pwmCmd1, pwmCmd2);
  }
}
