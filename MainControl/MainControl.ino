#include <INA226.h>
#include <Arduino.h>
#include <math.h>
#include <GyverINA.h>
#include "MotorControl.h"
#include "wheelPID.h"

INA226 ina; // voltage reader object

// ===== Motor L pins =====
const uint8_t PWM1 = 10; // motor 2
const uint8_t INA1 = 6;
const uint8_t INB1 = 7;

// ===== Motor R pins =====
const uint8_t PWM2 = 9; // motor 1 
const uint8_t INA2 = 5;
const uint8_t INB2 = 4;

// ===== Encoder pins =====
const uint8_t ENC_A  = 2; // Motor 2 encoder A (D2)
const uint8_t ENC_B  = 3; // Motor 2 encoder B (D3)

const uint8_t ENC2_A = 8;   // Motor 1 encoder A (D8  PB0 / PCINT0)
const uint8_t ENC2_B = 12;  // Motor 1 encoder B (D12 PB4 / PCINT4)

// ===== Encoder scaling =====
const float COUNTS_PER_REV = 537.7f;

// ===== Encoder counters =====
volatile long encoderCounts  = 0;  // motor 1
volatile long encoder2Counts = 0;  // motor 2
volatile uint8_t lastAB  = 0;
volatile uint8_t lastAB2 = 0;

// x4 quadrature table
const int8_t quadTable[16] = {
  0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
  0, +1, -1,  0
};

// ---------- Encoder 1 ISR ----------
inline uint8_t readAB() {
  uint8_t a = (uint8_t)digitalRead(ENC_A);
  uint8_t b = (uint8_t)digitalRead(ENC_B);
  return (uint8_t)((a << 1) | b);
}

void encoderISR() {
  uint8_t ab = readAB();
  uint8_t idx = (uint8_t)((lastAB << 2) | ab);
  encoderCounts += quadTable[idx];
  lastAB = ab;
}

// ---------- Encoder 2 ISR ----------
inline uint8_t readAB2_fromPINB(uint8_t pinb) {
  uint8_t a = (pinb >> 0) & 0x01; // D8  = PB0
  uint8_t b = (pinb >> 4) & 0x01; // D12 = PB4
  return (uint8_t)((a << 1) | b);
}

ISR(PCINT0_vect) {
  uint8_t pinb = PINB;
  uint8_t ab2  = readAB2_fromPINB(pinb);
  uint8_t idx2 = (uint8_t)((lastAB2 << 2) | ab2);
  encoder2Counts += quadTable[idx2];
  lastAB2 = ab2;
}

// ================= Speed setpoints =================
float setRPM1 = 50.0f;
float setRPM2 = 50.0f;

// Signed commands from the user (can be negative)
float cmdRPM1 = 50.0f;
float cmdRPM2 = 50.0f;

MotorControl motors(PWM1, INA1, INB1, PWM2, INA2, INB2);

// ================= PID gains (now via struct + function) =================
WheelPIDGains gains1, gains2;
WheelPIDState state1, state2;

// Deadband (same as before)
const int PWM_DEADBAND1 = 20;
const int PWM_DEADBAND2 = 20;

// Timing
const unsigned long CTRL_MS  = 100;  // 10 Hz control
const unsigned long PRINT_MS = 300;  // print a few times a second

unsigned long lastCtrlMs  = 0;
unsigned long lastPrintMs = 0;

long lastC1 = 0, lastC2 = 0;
float rpm1 = 0.0f, rpm2 = 0.0f;
int pwmCmd1 = 0, pwmCmd2 = 0;

// -------- Serial command parser --------
String line;

void handleSerial() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  float a = 0, b = 0;

  int comma = s.indexOf(',');
  int space = s.indexOf(' ');

  if (comma >= 0) {
    a = s.substring(0, comma).toFloat();
    b = s.substring(comma + 1).toFloat();
  } 
  else if (space >= 0) {
    a = s.substring(0, space).toFloat();
    b = s.substring(space + 1).toFloat();
  } 
  else {
    a = s.toFloat();
    b = a;
  }

  cmdRPM1 = a;   // signed command
  cmdRPM2 = b;
  motors.setDirection(cmdRPM1 >= 0.0f, cmdRPM2 >= 0.0f);

  setRPM1 = fabs(cmdRPM1);
  setRPM2 = fabs(cmdRPM2);

  wheelPID_reset(state1);
  wheelPID_reset(state2);
 //  motors.commandRPM(cmdRPM1, cmdRPM2);  // sets direction + writes abs() into setRPM1/setRPM2

  // simple: always reset both on any new command
 // wheelPID_reset(state1);
//  wheelPID_reset(state2);

  Serial.print("CMD: "); Serial.print(cmdRPM1, 1);
  Serial.print(", ");     Serial.print(cmdRPM2, 1);
  Serial.print(" | SP(abs): "); Serial.print(setRPM1, 1);
  Serial.print(", ");          Serial.println(setRPM2, 1);
}

void setup() {
  Serial.begin(115200);

  if (ina.begin()) {
    Serial.println(F("connected!"));
  } else {
    Serial.println(F("not found!"));
    while (1);
  }

  // Voltage reader setup
  Serial.print(F("Calibration value: "));
  Serial.println(ina.getCalibration());
  ina.setSampleTime(INA226_VBUS, INA226_CONV_2116US);
  ina.setSampleTime(INA226_VSHUNT, INA226_CONV_8244US);
  ina.setAveraging(INA226_AVG_X4);

  // Set default gains
  wheelPID_setGains(gains1, 0.55f, 0.001f, 0.0f);
  wheelPID_setGains(gains2, 0.55f, 0.001f, 0.0f);

  // Pin mode set up for motors
  pinMode(PWM1, OUTPUT); pinMode(INA1, OUTPUT); pinMode(INB1, OUTPUT);
  pinMode(PWM2, OUTPUT); pinMode(INA2, OUTPUT); pinMode(INB2, OUTPUT);

  // Incoder pins
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  lastAB  = readAB();
  lastAB2 = readAB2_fromPINB(PINB);

  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT4);

  // Forward direction
  digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH);

  noInterrupts();
  lastC1 = encoderCounts;
  lastC2 = encoder2Counts;
  interrupts();

  lastCtrlMs  = millis();
  lastPrintMs = millis();

  Serial.println("2-Motor PID Speed Control");
  Serial.println("Type: 120  (sets both), 120,110 (sets M1,M2), 1 120 (M1 only), 2 120 (M2 only)");

  motors.begin();
  //motors.commandRPM(+50, +50);   // example default

}

void loop() {
  handleSerial();

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

    // Pass voltage into PID step (NEW)
    float Vs = ina.getVoltage();

  if (setRPM1 <= 0.01f) {
    pwmCmd1 = 0;
    wheelPID_reset(state1);
  } else {
    float speed1 = fabs(rpm1);
    pwmCmd1 = wheelPID_step(setRPM1, speed1, state1, gains1, dt, PWM_DEADBAND1, Vs);
  }

   if (setRPM2 <= 0.01f) {
  pwmCmd2 = 0;
  wheelPID_reset(state2);
  } else {
  float speed2 = fabs(rpm2);
  pwmCmd2 = wheelPID_step(setRPM2, speed2, state2, gains2, dt, PWM_DEADBAND2, Vs);
  }
    motors.applyPWM(pwmCmd1, pwmCmd2);
  }

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;
    Serial.print("CMD1: "); Serial.print(cmdRPM1, 1);
    Serial.print(" SP1: "); Serial.print(setRPM1, 1);
    Serial.print(" RPM1: "); Serial.print(rpm1, 1);
    Serial.print(" PWM1: "); Serial.print(pwmCmd1);

    Serial.print(" | CMD2: "); Serial.print(cmdRPM2, 1);
    Serial.print(" SP2: "); Serial.print(setRPM2, 1);
    Serial.print(" RPM2: "); Serial.print(rpm2, 1);
    Serial.print(" PWM2: "); Serial.println(pwmCmd2);
    // Setting motor direction

    

  }
}
