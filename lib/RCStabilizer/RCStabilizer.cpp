#include "RCStabilizer.h"
#include <math.h>

RCStabilizer::RCStabilizer() {}

void RCStabilizer::begin(const Pins* pins,
                         const PID* pid,
                         const Limits* limits,
                         const RateToPwm* r2p,
                         const EscProfile* esc)
{
  if (pins)    pins_ = *pins;
  if (pid)     pid_  = *pid;
  if (limits)  lim_  = *limits;
  if (r2p)     r2p_  = *r2p;
  if (esc)     esc_  = *esc;

  pinMode(pins_.motorA, OUTPUT);
  pinMode(pins_.motorB, OUTPUT);
  motor_(0);

  pinMode(pins_.limitCCW, INPUT_PULLUP);
  pinMode(pins_.limitCW,  INPUT_PULLUP);
  ccw_last_raw_ = digitalRead(pins_.limitCCW);
  cw_last_raw_  = digitalRead(pins_.limitCW);
  ccw_last_change_ms_ = cw_last_change_ms_ = millis();
  readLimitsDebounced_();

  escServo_.attach(pins_.escPin);
  escServo_.writeMicroseconds(esc_.pwmMin);

  last_ms_ = millis();
}

void RCStabilizer::armESCSlow() {
  escServo_.writeMicroseconds(esc_.pwmMin);
  delay(esc_.preArmMs);

  int start = esc_.rampStart;
  int end   = esc_.rampEnd;
  int step  = (esc_.stepUs <= 0) ? 1 : esc_.stepUs;
  if (end < start) step = -abs(step);

  for (int us = start; (step > 0 ? us <= end : us >= end); us += step) {
    escServo_.writeMicroseconds(us);
    delay(esc_.stepMs);
  }

  if (esc_.holdAfter >= esc_.pwmMin && esc_.holdAfter <= esc_.pwmMax) {
    escServo_.writeMicroseconds(esc_.holdAfter);
  }
}

void RCStabilizer::escWriteMicroseconds(int us) {
  us = constrain(us, esc_.pwmMin, esc_.pwmMax);
  escServo_.writeMicroseconds(us);
}

void RCStabilizer::escHoldAtConfigured() {
  escWriteMicroseconds(esc_.holdAfter);
}

void RCStabilizer::setPID(float kp, float ki, float kd) {
  pid_.Kp = kp; pid_.Ki = ki; pid_.Kd = kd;
}
void RCStabilizer::setDeadband(float deg) { lim_.roll_deadband = deg; }
void RCStabilizer::setSlewAndMax(float rate_slew, float rate_max) {
  lim_.rate_cmd_slew = rate_slew;
  lim_.rate_cmd_max  = rate_max;
}
void RCStabilizer::setRateToPwm(float k_rate, int pwm_min_start, int pwm_max, bool invert) {
  r2p_.k_rate = k_rate;
  r2p_.pwm_min_start = pwm_min_start;
  r2p_.pwm_max = pwm_max;
  r2p_.invertOutput = invert;
}

void RCStabilizer::update(float roll_deg) {
  unsigned long now = millis();
  float dt = (now - last_ms_) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  last_ms_ = now;

  readLimitsDebounced_();

  // PID on roll -> desired rate
  const float setpoint = 6.0f; // your original target
  float error = setpoint - roll_deg;
  error = deadband_(error, lim_.roll_deadband);

  const float tau_leak = 0.6f;
  if (fabs(error) > 0.0f) {
    I_acc_ += error * dt;
  } else {
    float leak = dt / tau_leak;
    if (leak > 1.0f) leak = 1.0f;
    I_acc_ *= (1.0f - leak);
  }

  float eps = (pid_.Ki != 0.0f) ? pid_.Ki : 1e-6f;
  I_acc_ = clampf_(I_acc_, pid_.I_min / eps, pid_.I_max / eps);

  float D = (error - e_prev_) / dt;
  e_prev_ = error;

  float rate_cmd = pid_.Kp * error + pid_.Ki * I_acc_ + pid_.Kd * D;

  float max_step = lim_.rate_cmd_slew * dt;
  float delta = rate_cmd - rate_cmd_prev_;
  if (delta >  max_step) rate_cmd = rate_cmd_prev_ + max_step;
  if (delta < -max_step) rate_cmd = rate_cmd_prev_ - max_step;
  rate_cmd_prev_ = rate_cmd;

  rate_cmd = clampf_(rate_cmd, -lim_.rate_cmd_max, lim_.rate_cmd_max);

  // Map to signed PWM and enforce limit switches ONLY
  int pwm_cmd = rateToPwm_(rate_cmd);
  if (r2p_.invertOutput) pwm_cmd = -pwm_cmd;
  pwm_cmd = applyLimitConstraints_(pwm_cmd);

  motor_(pwm_cmd);
}

// ===== Helpers =====
void RCStabilizer::motor_(int speed) {
  speed = constrain(speed, -r2p_.pwm_max, r2p_.pwm_max);
  pwm_signed_ = speed;

  if (speed == 0) {
    analogWrite(pins_.motorA, 0);
    analogWrite(pins_.motorB, 0);
    return;
  }

  int mag = abs(speed);
  if (mag > 0 && mag < r2p_.pwm_min_start) mag = r2p_.pwm_min_start;

  if (speed > 0) {
    analogWrite(pins_.motorA, mag);
    analogWrite(pins_.motorB, 0);
  } else {
    analogWrite(pins_.motorA, 0);
    analogWrite(pins_.motorB, mag);
  }
}

int RCStabilizer::rateToPwm_(float rate_cmd) {
  float sign = (rate_cmd >= 0.0f) ? 1.0f : -1.0f;
  float mag  = fabs(rate_cmd);

  float pwm_f = r2p_.k_rate * mag;
  if (pwm_f > 0.0f) pwm_f += r2p_.pwm_min_start;
  if (pwm_f > r2p_.pwm_max) pwm_f = r2p_.pwm_max;

  int pwm = (int)lroundf(pwm_f);
  return (int)(sign * pwm);
}

void RCStabilizer::readLimitsDebounced_() {
  unsigned long now = millis();

  bool ccw_raw = digitalRead(pins_.limitCCW);
  bool cw_raw  = digitalRead(pins_.limitCW);

  if (ccw_raw != ccw_last_raw_) {
    ccw_last_change_ms_ = now;
    ccw_last_raw_ = ccw_raw;
  } else if ((now - ccw_last_change_ms_) >= LIMIT_DEBOUNCE_MS_) {
    ccw_limit_state_ = pins_.limitActiveLow ? (ccw_raw == LOW) : (ccw_raw == HIGH);
  }

  if (cw_raw != cw_last_raw_) {
    cw_last_change_ms_ = now;
    cw_last_raw_ = cw_raw;
  } else if ((now - cw_last_change_ms_) >= LIMIT_DEBOUNCE_MS_) {
    cw_limit_state_ = pins_.limitActiveLow ? (cw_raw == LOW) : (cw_raw == HIGH);
  }
}

int RCStabilizer::applyLimitConstraints_(int pwm_cmd) {
  if (ccw_limit_state_ && cw_limit_state_) return 0; // both → stop
  if (ccw_limit_state_ && pwm_cmd < 0) return 0;     // block CCW
  if (cw_limit_state_  && pwm_cmd > 0) return 0;     // block CW
  return pwm_cmd;
}
