#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU_BNO055 {
public:
  IMU_BNO055(uint8_t id = 55, uint8_t address = 0x28);
  bool begin(bool useExtCrystal = true);
  bool update();  // rate-limited
  void setSamplePeriodMs(uint16_t ms) { _samplePeriodMs = ms; }
  // Orientation (degrees) - Adafruit Euler: x=Yaw/Heading, y=Roll, z=Pitch
  float xDeg() const { return _yaw; }   // x (Yaw)
  float yDeg() const { return _roll; }  // y (Roll)
  float zDeg() const { return _pitch; } // z (Pitch)
  // Linear acceleration (m/s^2) 
  float ax() const { return _ax; }
  float ay() const { return _ay; }
  float az() const { return _az; }
  int temp() const { return _temp; }
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
  float _ax = 0.0f;
  float _ay = 0.0f;
  float _az = 0.0f;
  int _temp = 0; 
};