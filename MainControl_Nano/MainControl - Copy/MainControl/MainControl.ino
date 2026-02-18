#include <Arduino.h>
#include <math.h>
#include <GyverINA.h> // Voltage Reader (INA226)
#include "MotorControl.h"
#include "wheelPID.h"
#include "MotorEncoderSetup.h"
#include "IMU.h"

INA226 ina; // voltage reader object
IMU_BNO055 bno55(55,0x28);
// =======================
// Arduino Nano ESP32 pins
// =======================

// ===== Motor L pins =====
const uint8_t PWM1 = D10; // motor 2
const uint8_t INA1 = D6;
const uint8_t INB1 = D7;

// ===== Motor R pins =====
const uint8_t PWM2 = D9;  // motor 1
const uint8_t INA2 = D5;
const uint8_t INB2 = D4;

// ================= Speed setpoints =================
float setRPM1 = 0.0f;
float setRPM2 = 0.0f;

// Signed commands from the user (can be negative)
float cmdRPM1 = 0.0f;
float cmdRPM2 = 0.0f;

MotorControl motors(PWM1, INA1, INB1, PWM2, INA2, INB2);

int motor1Command = 0;
int motor2Command = 0;

// ================= PID gains  =================
WheelPIDGains gains1, gains2;
WheelPIDState state1, state2;

// Deadband
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

float yaw   = 0.0f;
float roll  = 0.0f;
float pitch = 0.0f;

float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;

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

  motor1Command = (int)a;
  motor2Command = (int)b;

  motors.setDirection(cmdRPM1 >= 0.0f, cmdRPM2 >= 0.0f);

  setRPM1 = fabs(cmdRPM1);
  setRPM2 = fabs(cmdRPM2);

  Serial.print("CMD: "); Serial.print(cmdRPM1, 1);
  Serial.print(", ");     Serial.print(cmdRPM2, 1);
  Serial.print(" | SP(abs): "); Serial.print(setRPM1, 1);
  Serial.print(", ");          Serial.println(setRPM2, 1);
}

bool readSerialMessage() {
  int motor1Serial = Serial.parseInt();
  int motor2Serial = Serial.parseInt();

  if (!Serial.find('\n')) return false;

  motor1Command = motor1Serial;
  motor2Command = motor2Serial;
  return true;
}

void writeSerialMessage() {
  Serial.print(rpm1,1); // rpm
  Serial.print(",");
  Serial.print(rpm2,1); // rpm 
  Serial.print(encoderCounts); // counts
  Serial.print(",");
  Serial.println(encoder2Counts); // counts
  Serial.println(yaw,1); // degs
  Serial.print(",");
  Serial.println(roll,1); // degs
  Serial.print(",");
  Serial.println(pitch,1); // degs
  Serial.print(",");
  Serial.println(ax,1); // m/s^2
  Serial.print(",");
  Serial.println(ay,1); // m/s^2
  Serial.print(",");
  Serial.println(az,1); // m/s^2
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  if (ina.begin()) {
    Serial.println(F("msg: connected!"));
  } else {
    Serial.println(F("e: not found!"));
    while (1) { delay(10); }
  }

  if (!bno55.begin(true)) {
      Serial.println("e: BNO055 not found");
      while (1) { delay(10); }
  }

 bno55.setSamplePeriodMs(20);

  // Voltage reader setup
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

  // Encoder pins
  pinMode(ENC_A,  INPUT_PULLUP);
  pinMode(ENC_B,  INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  // Initialize last states
  lastAB  = readAB();
  lastAB2 = readAB2();

  // Encoder interrupts (ESP32 supports interrupts on all digital GPIOs)
  attachInterrupt(ENC_A,  encoderISR,  CHANGE);
  attachInterrupt(ENC_B,  encoderISR,  CHANGE);
  attachInterrupt(ENC2_A, encoder2ISR, CHANGE);
  attachInterrupt(ENC2_B, encoder2ISR, CHANGE);

  // Forward direction (your MotorControl may override this later)
  digitalWrite(INA1, LOW);  digitalWrite(INB1, HIGH);
  digitalWrite(INA2, LOW);  digitalWrite(INB2, HIGH);

  noInterrupts();
  lastC1 = encoderCounts;
  lastC2 = encoder2Counts;
  interrupts();

  lastCtrlMs  = millis();
  lastPrintMs = millis();

  motors.begin();
  // motors.commandRPM(+50, +50);   // example default
}

void loop() {
  unsigned long now = millis();

  // Optional: if you actually want to use your "a,b" command parser, uncomment:
  // handleSerial();

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

    // Pass voltage into PID step
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

  if (bno55.update()) {
    yaw   = bno55.yawDeg();
    roll  = bno55.rollDeg();
    pitch = bno55.pitchDeg();

    ax = bno55.accelX();
    ay = bno55.accelY();
    az = bno55.accelZ();
  } 

  if (now - lastPrintMs >= PRINT_MS) {
    lastPrintMs = now;

    if (Serial.available() > 0) {
      if (readSerialMessage()) {
        writeSerialMessage();
      }
    }
  }
}
