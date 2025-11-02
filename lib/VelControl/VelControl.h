#pragma once
#include <Arduino.h>

/**
 * VelControl (self-timed)
 *  - RC setpoints (%): throttle → v_set [m/s], steer → w_set [rad/s]
 *  - Dual PID → L/R ESC µs, with internal dt measured via millis()
 */
class VelControl {
public:
  VelControl();

  // Call once — pass measured ESC neutrals (µs)
  void begin(int escNeutralLeftUs, int escNeutralRightUs);

  // Optional tunables
  void setMaxVel(float max_lin_mps, float max_ang_rads);
  void setGainsV(float kp, float ki, float kd);
  void setGainsW(float kp, float ki, float kd);
  void setIntegralLimits(float imin, float imax);      // percent·s
  void setOutputLimits(float omin, float omax);        // percent
  void setDerivativeAlpha(float a);                    // 0..1 smoothing
  void setNeutralDeadbandUs(int deadband_us);          // snap zone at neutral
  void setEscBoundsUs(int esc_min_us, int esc_max_us); // default 1000..2000

  // Timing behavior
  void setDtClamp(float dt_min_s, float dt_max_s);     // default 0.001..0.20
  void setDtFallback(float dt_fallback_s);             // default 0.02 (50 Hz)
  void resetTiming();                                   // resets timer & I terms

  // Main step (SELF-TIMED): RC setpoints (%) + feedback (m/s, rad/s)
  void update(int throttle_pct, int steer_pct, float lin_vel, float ang_vel);

  // Outputs (microseconds)
  inline int leftPWM()  const { return left_pwm_us_;  }
  inline int rightPWM() const { return right_pwm_us_; }

private:
  struct PID {
    float kp, ki, kd;
    float i;
    float prev_err;
    float d_filt;
    PID() : kp(0), ki(0), kd(0), i(0), prev_err(0), d_filt(0) {}
  };

  // Helpers
  static float fmap(float x, float in_min, float in_max, float out_min, float out_max);
  int pctToPwmWithNeutral(int pct, int neutral_us) const;

  // Config/state
  int esc_min_us_{1000};
  int esc_max_us_{2000};
  int esc_neutral_left_us_{1500};
  int esc_neutral_right_us_{1500};
  int neutral_db_us_{10};

  float max_lin_vel_{1.20f};   // m/s
  float max_ang_vel_{1.50f};   // rad/s

  float out_min_{-100.0f};
  float out_max_{ 100.0f};
  float i_min_{-80.0f};
  float i_max_{ 80.0f};

  float d_alpha_{0.15f};       // derivative LPF

  PID vpid_;   // defaults applied in .cpp constructor
  PID wpid_;   // defaults applied in .cpp constructor

  // Timing
  unsigned long prev_ms_{0};
  bool first_update_{true};
  float dt_min_{0.001f};
  float dt_max_{0.20f};
  float dt_fallback_{0.02f};

  // Outputs
  int left_pwm_us_{1500};
  int right_pwm_us_{1500};
};
