#include "IMU.h"
#include <Wire.h>

IMU_BNO055::IMU_BNO055(uint8_t id, uint8_t address)
  : _bno(id, address) {}

bool IMU_BNO055::begin(bool useExtCrystal) {
  Wire.begin();

  if (!_bno.begin()) {
    _ready = false;
    return false;
  }

  delay(1000);                 // like your sketch
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

  // x=Heading(Yaw), y=Roll, z=Pitch
  imu::Vector<3> euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  _yaw   = euler.x();
  _roll  = euler.y();
  _pitch = euler.z();

  return true;
}

void IMU_BNO055::printSensorDetails(Stream &out) {
  if (!_ready) {
    out.println("BNO055 not initialized.");
    return;
  }

  sensor_t sensor;
  _bno.getSensor(&sensor);

  out.println("------------------------------------");
  out.print  ("Sensor:       "); out.println(sensor.name);
  out.print  ("Driver Ver:   "); out.println(sensor.version);
  out.print  ("Unique ID:    "); out.println(sensor.sensor_id);
  out.print  ("Max Value:    "); out.print(sensor.max_value); out.println(" xxx");
  out.print  ("Min Value:    "); out.print(sensor.min_value); out.println(" xxx");
  out.print  ("Resolution:   "); out.print(sensor.resolution); out.println(" xxx");
  out.println("------------------------------------");
  out.println();
}
