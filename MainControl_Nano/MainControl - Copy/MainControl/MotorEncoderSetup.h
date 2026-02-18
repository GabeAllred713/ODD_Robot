#ifndef MOTORENCODERSETUP_H
#define MOTORENCODERSETUP_H

#include <Arduino.h>

// ===== Encoder pins =====
const uint8_t ENC_A  = D2;  // Motor 2 encoder A
const uint8_t ENC_B  = D3;  // Motor 2 encoder B

const uint8_t ENC2_A = D8;   // Motor 1 encoder A
const uint8_t ENC2_B = D12;  // Motor 1 encoder B

// ===== Encoder scaling =====
const float COUNTS_PER_REV = 537.7f;

// ===== Encoder counters =====
volatile long encoderCounts  = 0;  // motor 1
volatile long encoder2Counts = 0;  // motor 2
volatile uint8_t lastAB  = 0;
volatile uint8_t lastAB2 = 0;


static const int8_t quadTable[16] = {
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

void IRAM_ATTR encoderISR() {
  uint8_t ab = readAB();
  uint8_t idx = (uint8_t)((lastAB << 2) | ab);
  encoderCounts += quadTable[idx];
  lastAB = ab;
}

// ---------- Encoder 2 ISR (ESP32 version; replaces AVR PCINT code) ----------
inline uint8_t readAB2() {
  uint8_t a = (uint8_t)digitalRead(ENC2_A);
  uint8_t b = (uint8_t)digitalRead(ENC2_B);
  return (uint8_t)((a << 1) | b);
}

void IRAM_ATTR encoder2ISR() {
  uint8_t ab2  = readAB2();
  uint8_t idx2 = (uint8_t)((lastAB2 << 2) | ab2);
  encoder2Counts += quadTable[idx2];
  lastAB2 = ab2;
}

#endif
