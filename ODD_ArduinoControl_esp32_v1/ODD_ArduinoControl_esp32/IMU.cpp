#include "IMU.h"
#include <Wire.h>

// This file is the set up for the Adafruit BNO055 9-Axis IMU
IMU_BNO055::IMU_BNO055(uint8_t id, uint8_t address)
  : _bno(id, address) {}
bool IMU_BNO055::begin(bool useExtCrystal) {
  Wire.begin();
  if (!_bno.begin()) {
    _ready = false;
    return false;
  }
  delay(1000);
  _bno.setExtCrystalUse(useExtCrystal);
  _ready = true;
  _lastUpdateMs = 0;
  return true;
}
bool IMU_BNO055::update() {
  if (!_ready) return false;
  const uint32_t now = millis();
  if (now - _lastUpdateMs < _samplePeriodMs) return false;
  _lastUpdateMs = now;
  // Orientation (degrees): x=Yaw, y=Roll, z=Pitch
  imu::Vector<3> euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  _yaw   = euler.x();
  _roll  = euler.y();
  _pitch = euler.z();
  // Linear acceleration (m/s^2)
  imu::Vector<3> la = _bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  _ax = la.x();
  _ay = la.y();
  _az = la.z();

  _temp = _bno.getTemp();
  return true; 

}
void IMU_BNO055::printSensorDetails(Stream &out) {
  if (!_ready) {
    out.println("BNO055 not initialized."); 
    return;
  }
  sensor_t sensor;
  _bno.getSensor(&sensor);
}