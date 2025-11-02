#include <RCStabilizer.h>
// #include <RCChannels.h>  // if you need RC input in this sketch
// RCChannels rc;

RCStabilizer stab;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Optional: override defaults
  RCStabilizer::Pins pins;
  pins.motorA = 5;
  pins.motorB = 6;
  pins.escPin = 11;
  pins.limitCCW = 2;
  pins.limitCW  = 3;
  pins.limitActiveLow = true; // INPUT_PULLUP

  RCStabilizer::PID pid;
  pid.Kp = 0.2f; pid.Ki = 0.0f; pid.Kd = 0.0f;
  pid.I_min = -50.0f; pid.I_max = 50.0f;

  RCStabilizer::Limits lim;
  lim.pitch_deg_limit = 60.0f;
  lim.roll_deadband   = 2.0f;
  lim.rate_cmd_max    = 120.0f;
  lim.rate_cmd_slew   = 800.0f;

  RCStabilizer::RateToPwm r2p;
  r2p.k_rate = 1.6f;
  r2p.pwm_min_start = 1;
  r2p.pwm_max = 255;
  r2p.invertOutput = true; // matches your original

  RCStabilizer::EscProfile esc;
  esc.pwmMin = 1000; esc.pwmMax = 2000;
  esc.holdAfter = 1400;
  esc.rampStart = 1200; esc.rampEnd = 1400;
  esc.stepUs = 5; esc.stepMs = 200; esc.preArmMs = 3000;

  // Initialize stabilizer (all structs optional; pass nullptr to keep defaults)
  stab.begin(&pins, &pid, &lim, &r2p, &esc);

  // Slow arm the flywheel ESC (optional; call once after power-up)
  stab.armESCSlow();

  // If you want to hold ESC at a given throttle after arming
  stab.escHoldAtConfigured();

  Serial.println(F("RCStabilizer ready."));
}

void loop() {
  // You get roll (and optionally pitch) from your own sensor pipeline:
  // float roll_deg = <your roll angle>;
  // For demo, simulate a noisy roll around 5–7 degrees:
  static float t = 0.0f;
  t += 0.02f;
  float roll_deg = 6.0f + 1.0f * sinf(t); // pretend IMU

  // Drive gimbal motor based on the roll you provide
  stab.update(roll_deg);

  // If you need to adjust PID at runtime:
  // stab.setPID(0.25f, 0.02f, 0.01f);

  delay(20); // ~50 Hz
}
