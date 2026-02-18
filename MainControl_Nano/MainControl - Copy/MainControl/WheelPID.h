
// wheelPID.h
#ifndef WHEELPID_H
#define WHEELPID_H

#include <Arduino.h>
#include <math.h>

// ===== User-configurable constants =====
static const float WHEELPID_I_LIM  = 500.0f;
static const int   WHEELPID_PWM_MAX = 255;

// PID gains container
struct WheelPIDGains {
  float Kp = 0.0f;
  float Ki = 0.0f;
  float Kd = 0.0f;
};

// PID state container (integrator + previous error)
struct WheelPIDState {
  float integ = 0.0f;
  float prevE = 0.0f;
};

// Set gains by function
inline void wheelPID_setGains(WheelPIDGains &g, float kp, float ki, float kd) {
  g.Kp = kp;
  g.Ki = ki;
  g.Kd = kd;
}

// Reset PID state by function
inline void wheelPID_reset(WheelPIDState &s) {
  s.integ = 0.0f;
  s.prevE = 0.0f;
}

// PID step (returns PWM 0..255). Pass Vs in.
inline int wheelPID_step(
  float setRPM,
  float rpm,
  WheelPIDState &st,
  const WheelPIDGains &g,
  float dt,
  int deadband,
  float Vs
) {
  // ---- keep your motor model params exactly like your code ----
  float k_t = 0.254f; // Nm
  float k_e = k_t;
  float R_a = 1.3f;   // ohms
  float T_L = 0.0f;

  // Guard dt / voltage
  if (dt <= 0.0f) return 0;
  if (Vs <= 0.1f) Vs = 0.1f; // prevent divide-by-zero behavior

  float e = setRPM - rpm;

  // candidate integral
  float integNew = st.integ + e * dt;
  integNew = constrain(integNew, -WHEELPID_I_LIM, WHEELPID_I_LIM);

  float dE = (e - st.prevE) / dt;

  float u = g.Kp * e + g.Ki * integNew + g.Kd * dE;

  // ---- keep your duty->PWM math exactly like your code ----
  float D = 0.0f;
  float inc = 0.0f;

  D = ((u + setRPM) / (2.0f * 3.14f) + ((T_L * R_a) / k_e * k_t)) * (k_e / Vs);

  // Clamp duty to sane range
  if (D < 0.0f) D = 0.0f;
  if (D > 1.0f) D = 1.0f;

  inc = 255.0f * D;

  if (inc > 0.0f && inc < (float)deadband) inc = (float)deadband;

  int pwm = (int)lroundf(inc);
  if (pwm > WHEELPID_PWM_MAX) pwm = WHEELPID_PWM_MAX;
  if (pwm < 0) pwm = 0;

  // anti-windup (same logic as yours)
  bool satHigh = (pwm == WHEELPID_PWM_MAX);
  bool satLow  = (pwm == 0);

  if (!((satHigh && e > 0) || (satLow && e < 0))) {
    st.integ = integNew;
  }
  st.prevE = e;


  return pwm;
}

#endif // WHEELPID_H
