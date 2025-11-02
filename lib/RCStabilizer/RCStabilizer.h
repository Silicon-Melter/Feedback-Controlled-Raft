#pragma once
#include <Arduino.h>
#include <Servo.h>

class RCStabilizer {
public:
  // ===== Types =====
  struct Pins {
    uint8_t motorA      = 12;   // H-bridge A
    uint8_t motorB      = 13;   // H-bridge B
    uint8_t escPin      = 8;  // BLDC ESC signal
    uint8_t limitCCW    = 24;   // normally-open to GND → LOW when pressed
    uint8_t limitCW     = 25;   // normally-open to GND → LOW when pressed
    bool    limitActiveLow = true; // INPUT_PULLUP; LOW=pressed
  };

  struct PID {
    float Kp = 0.2f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    // anti-windup as equivalent output clamp (scaled by Ki internally)
    float I_min = -50.0f;
    float I_max =  50.0f;
  };

  struct Limits {
    float roll_deadband   = 2.0f;  // deg
    float rate_cmd_max    = 120.0f; // deg/s
    float rate_cmd_slew   = 800.0f; // deg/s^2
  };

  struct RateToPwm {
    float k_rate        = 1.6f; // PWM per (deg/s)
    int   pwm_min_start = 1;    // overcome stiction
    int   pwm_max       = 255;  // abs max
    bool  invertOutput  = true; // matches your original "-pwm_signed"
  };

  struct EscProfile {
    int    pwmMin     = 1000;
    int    pwmMax     = 2000;
    int    holdAfter  = 1400;   // where to hold after arming
    int    rampStart  = 1200;
    int    rampEnd    = 1400;
    int    stepUs     = 5;
    uint16_t stepMs   = 200;
    uint16_t preArmMs = 3000;
  };

 RCStabilizer();

  void begin(const Pins* pins = nullptr,
             const PID* pid = nullptr,
             const Limits* limits = nullptr,
             const RateToPwm* r2p = nullptr,
             const EscProfile* esc = nullptr);

  void armESCSlow();
  void escWriteMicroseconds(int us);
  void escHoldAtConfigured();

  // pitch_deg is accepted for backward compatibility but ignored
  void update(float roll_deg);

  // Tunables
  void setPID(float kp, float ki, float kd);
  void setDeadband(float deg);
  void setSlewAndMax(float rate_slew, float rate_max);
  void setRateToPwm(float k_rate, int pwm_min_start, int pwm_max, bool invert=false);

  // Deprecated: no-op (kept to avoid breaking existing code)
  void setPitchLimit(float) {}

  int  lastMotorPwm() const { return pwm_signed_; }

private:
  static inline float clampf_(float v, float lo, float hi) {
    if (v < lo) return lo; if (v > hi) return hi; return v;
  }
  static inline float deadband_(float v, float db) {
    return (fabs(v) < db) ? 0.0f : v;
  }

  void motor_(int speed);
  int  rateToPwm_(float rate_cmd);
  void readLimitsDebounced_();
  int  applyLimitConstraints_(int pwm_cmd);

private:
  Pins        pins_;
  PID         pid_;
  Limits      lim_;
  RateToPwm   r2p_;
  EscProfile  esc_;

  Servo       escServo_;
  bool        ccw_limit_state_ = false;
  bool        cw_limit_state_  = false;
  bool        ccw_last_raw_    = true;
  bool        cw_last_raw_     = true;
  unsigned long ccw_last_change_ms_ = 0;
  unsigned long cw_last_change_ms_  = 0;
  const unsigned long LIMIT_DEBOUNCE_MS_ = 8;

  unsigned long last_ms_ = 0;
  float e_prev_          = 0.0f;
  float I_acc_           = 0.0f;
  float rate_cmd_prev_   = 0.0f;

  int   pwm_signed_      = 0;
};