#include "IMU.h"

IMU_BNO055::IMU_BNO055(uint8_t id, uint8_t address)
: _bno(id, address) {}

bool IMU_BNO055::begin(bool useExtCrystal, TwoWire &wire) {
  // Ensure I2C is up (on Nano ESP32 this uses the board’s default SDA/SCL)
  wire.begin();

  if (!_bno.begin()) {
    _ready = false;
    return false;
  }

  delay(10);
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

  // Euler angles (degrees)
  imu::Vector<3> e = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  _yaw   = e.x();
  _roll  = e.y();
  _pitch = e.z();

  // Linear acceleration (usually m/s^2)
  imu::Vector<3> a = _bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  _ax = a.x();
  _ay = a.y();
  _az = a.z();

  return true;
}

void IMU_BNO055::printSensorDetails(Stream &out) {
  if (!_ready) {
    out.println(F("BNO055 not initialized"));
    return;
  }

  sensor_t sensor;
  _bno.getSensor(&sensor);


}
