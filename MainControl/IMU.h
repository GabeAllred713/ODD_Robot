#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU_BNO055 {
public:
  // address: 0x28 or 0x29 depending on ADR pin
  // id: any number you want (Adafruit examples often use 55)
  IMU_BNO055(uint8_t id = 55, uint8_t address = 0x28);

  // Returns true if sensor initialized
  bool begin(bool useExtCrystal = true);

  // Call often in loop(); returns true when it actually updated (rate-limited)
  bool update();

  // Set update period (ms). Default 100ms like your sketch.
  void setSamplePeriodMs(uint16_t ms) { _samplePeriodMs = ms; }

  // Latest values (degrees)
  float rollDeg()  const { return _roll; }
  float pitchDeg() const { return _pitch; }
  float yawDeg()   const { return _yaw; }

  // Optional: print sensor info (like your displaySensorDetails)
  void printSensorDetails(Stream &out = Serial);

  bool isReady() const { return _ready; }

private:
  Adafruit_BNO055 _bno;
  bool _ready = false;

  uint16_t _samplePeriodMs = 100;
  uint32_t _lastUpdateMs = 0;

  float _yaw = 0.0f;
  float _roll = 0.0f;
  float _pitch = 0.0f;
};
