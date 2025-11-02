#include "VelControl.h"
#include <math.h>

VelControl::VelControl() {
  // Move problematic brace initializers here (works across AVR cores)
  vpid_.kp = 0.90f; vpid_.ki = 0.60f; vpid_.kd = 0.05f;
  wpid_.kp = 0.80f; wpid_.ki = 0.40f; wpid_.kd = 0.04f;
}

void VelControl::begin(int escNeutralLeftUs, int escNeutralRightUs) {
  esc_neutral_left_us_  = escNeutralLeftUs;
  esc_neutral_right_us_ = escNeutralRightUs;
  left_pwm_us_  = esc_neutral_left_us_;
  right_pwm_us_ = esc_neutral_right_us_;
  resetTiming();
}

void VelControl::setMaxVel(float max_lin_mps, float max_ang_rads) {
  max_lin_vel_ = max_lin_mps;
  max_ang_vel_ = max_ang_rads;
}

void VelControl::setGainsV(float kp, float ki, float kd) { vpid_.kp = kp; vpid_.ki = ki; vpid_.kd = kd; }
void VelControl::setGainsW(float kp, float ki, float kd) { wpid_.kp = kp; wpid_.ki = ki; wpid_.kd = kd; }
void VelControl::setIntegralLimits(float imin, float imax) { i_min_ = imin; i_max_ = imax; }
void VelControl::setOutputLimits(float omin, float omax) { out_min_ = omin; out_max_ = omax; }
void VelControl::setDerivativeAlpha(float a) { d_alpha_ = constrain(a, 0.0f, 1.0f); }
void VelControl::setNeutralDeadbandUs(int deadband_us) { neutral_db_us_ = max(0, deadband_us); }
void VelControl::setEscBoundsUs(int esc_min_us, int esc_max_us) { esc_min_us_ = esc_min_us; esc_max_us_ = esc_max_us; }
void VelControl::setDtClamp(float dt_min_s, float dt_max_s) { dt_min_ = max(0.0f, dt_min_s); dt_max_ = max(dt_min_, dt_max_s); }
void VelControl::setDtFallback(float dt_fallback_s) { dt_fallback_ = max(0.0005f, dt_fallback_s); }

void VelControl::resetTiming() {
  prev_ms_ = millis();
  first_update_ = true;
  vpid_.i = 0.0f; vpid_.prev_err = 0.0f; vpid_.d_filt = 0.0f;
  wpid_.i = 0.0f; wpid_.prev_err = 0.0f; wpid_.d_filt = 0.0f;
}

float VelControl::fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return out_min + (out_max - out_min) * (x - in_min) / (in_max - in_min);
}

int VelControl::pctToPwmWithNeutral(int pct, int neutral_us) const {
  int span = (pct >= 0) ? (esc_max_us_ - neutral_us) : (neutral_us - esc_min_us_);
  long us  = (long)neutral_us + ((long)pct * span) / 100;
  if (abs((int)us - neutral_us) <= neutral_db_us_) us = neutral_us;
  return (int)constrain(us, esc_min_us_, esc_max_us_);
}

void VelControl::update(int throttle_pct, int steer_pct, float lin_vel, float ang_vel) {
  // ==== self-timed dt ====
  unsigned long now_ms = millis();
  float dt;
  if (first_update_) {
    dt = dt_fallback_;
    first_update_ = false;
  } else {
    dt = (now_ms - prev_ms_) * 0.001f;
    if (dt < dt_min_ || dt > dt_max_) dt = dt_fallback_;
  }
  prev_ms_ = now_ms;

  // RC setpoints → physical
  const float sp_v = fmap((float)throttle_pct, -100.0f, 100.0f, -max_lin_vel_, +max_lin_vel_);
  const float sp_w = fmap((float)steer_pct,    -100.0f, 100.0f, -max_ang_vel_, +max_ang_vel_);

  // ===== PID(v) =====
  float e_v = sp_v - lin_vel;
  vpid_.i += vpid_.ki * e_v * dt;
  vpid_.i = constrain(vpid_.i, i_min_, i_max_);
  float d_v = (e_v - vpid_.prev_err) / (dt > 1e-6f ? dt : 1e-6f);
  vpid_.d_filt = (1.0f - d_alpha_) * vpid_.d_filt + d_alpha_ * d_v;
  float u_v = vpid_.kp * e_v + vpid_.i + vpid_.kd * vpid_.d_filt;
  vpid_.prev_err = e_v;

  // ===== PID(w) =====
  float e_w = sp_w - ang_vel;
  wpid_.i += wpid_.ki * e_w * dt;
  wpid_.i = constrain(wpid_.i, i_min_, i_max_);
  float d_w = (e_w - wpid_.prev_err) / (dt > 1e-6f ? dt : 1e-6f);
  wpid_.d_filt = (1.0f - d_alpha_) * wpid_.d_filt + d_alpha_ * d_w;
  float u_w = wpid_.kp * e_w + wpid_.i + wpid_.kd * wpid_.d_filt;
  wpid_.prev_err = e_w;

  // Mix → percent
  float left_pct  = u_v + u_w;
  float right_pct = u_v - u_w;
  left_pct  = constrain(left_pct,  out_min_, out_max_);
  right_pct = constrain(right_pct, out_min_, out_max_);

  // Percent → µs around measured neutrals
  left_pwm_us_  = pctToPwmWithNeutral((int)left_pct,  esc_neutral_left_us_);
  right_pwm_us_ = pctToPwmWithNeutral((int)right_pct, esc_neutral_right_us_);
}
