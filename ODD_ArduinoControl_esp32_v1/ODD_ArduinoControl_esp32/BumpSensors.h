// BumpSensors.h
// Simple 4x bump switch reader for Arduino (Nano ESP32 etc.)
// Wiring: one side of each NO switch -> GND, other side -> pin
// Uses INPUT_PULLUP so: pressed = 1, released = 0
#ifndef BUMPSENSORS_H
#define BUMPSENSORS_H

#include <Arduino.h>
 
class BumpSensors {
public:
  // Pass the 4 pins in order (bump1, bump2, bump3, bump4)
  BumpSensors(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
  : _pins{p1, p2, p3, p4} {}
 
  // Call in setup()
  void begin() {
    for (int i = 0; i < 4; i++) {
      pinMode(_pins[i], INPUT_PULLUP);   // switch to GND when pressed
      _state[i] = 0;
    }
  }
 
  // Call in loop() to refresh readings
  // Returns a 4-bit mask: bit0=bump1, bit1=bump2, bit2=bump3, bit3=bump4
  // Example: 0b0101 means bump1 and bump3 pressed
  uint8_t update() {
    uint8_t mask = 0;
    for (int i = 0; i < 4; i++) {
      // INPUT_PULLUP: pressed reads LOW, so invert it
      _state[i] = (digitalRead(_pins[i]) == LOW) ? 1 : 0;
      mask |= (_state[i] << i);
    }
    return mask;
  }
 
  // Read individual bump by number 1..4 (returns 0 or 1)
  uint8_t read(uint8_t bumpNum) const {
    if (bumpNum < 1 || bumpNum > 4) return 0;
    return _state[bumpNum - 1];
  }
 
  //  gets all states into an array  (size 4)
  void readAll(uint8_t out[4]) const {
    for (int i = 0; i < 4; i++) out[i] = _state[i];
  }
 
private:
  uint8_t _pins[4];
  volatile uint8_t _state[4] = {0, 0, 0, 0};
};

#endif