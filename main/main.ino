#include <Servo.h>
#include <RCChannels.h>
#include <RCStabilizer.h>
#include <RPilink.h>
#include <VelControl.h>

RCChannels rc;
RCStabilizer stab;
RPilink link;
VelControl vc;

Servo esc1, esc2;  // Left, Right

// ===== ESC timing =====
const int ESC_MIN_US         = 1000;
const int ESC_MAX_US         = 2000;
int       ESC1_NEUTRAL_US    = 1500;  // left
int       ESC2_NEUTRAL_US    = 1460;  // right (your measured neutral)
const int ESC_NEUTRAL_DB_US  = 10;    // snap-to-neutral deadband


bool  armed  = false;   // start disarmed for safety
bool  manual = true;    // CH5 selects MAN/AUTO

float steering_gain = 0.8f;   // scales angular demand into differential mix

// ===== Sensors / setpoints =====
float roll_deg = 0.0f;
float lin_vel  = 0.0f;
float ang_vel  = 0.0f;

// ===== Outputs =====
int leftPWM  = ESC1_NEUTRAL_US;
int rightPWM = ESC2_NEUTRAL_US;
int AUTO_leftPWM  = ESC1_NEUTRAL_US;
int AUTO_rightPWM = ESC2_NEUTRAL_US;

// ===== Controller =====
int8_t steer; // CH1 = steering
int8_t throttle; // CH2 = throttle
int8_t mode; // CH5 = MAN/AUTO
int8_t kill; // CH6 = arm/disarm

// Convert -100..100 (%) to microseconds around a given neutral
static inline int pctToPwmWithNeutral(int pct, int neutral_us) {
  int span = (pct >= 0) ? (ESC_MAX_US - neutral_us) : (neutral_us - ESC_MIN_US);
  long us  = (long)neutral_us + ((long)pct * span) / 100;
  if (abs((int)us - neutral_us) <= ESC_NEUTRAL_DB_US) us = neutral_us;  // snap-to-neutral
  return (int)constrain(us, ESC_MIN_US, ESC_MAX_US);
}

// ===== Stabilizer bring-up =====
void stabStartUp(){
  RCStabilizer::Pins pins;        // use defaults from the library unless you need to override
  RCStabilizer::PID pid;          // Kp,Ki,Kd (your values carried over)
  pid.Kp = 0.2f; pid.Ki = 0.0f; pid.Kd = 0.0f;
  pid.I_min = -50.0f; pid.I_max = 50.0f;

  RCStabilizer::Limits lim;
  lim.roll_deadband   = 2.0f;
  lim.rate_cmd_max    = 120.0f;
  lim.rate_cmd_slew   = 800.0f;

  RCStabilizer::RateToPwm r2p;
  r2p.k_rate = 1.6f;
  r2p.pwm_min_start = 1;
  r2p.pwm_max = 255;
  r2p.invertOutput = true;

  RCStabilizer::EscProfile esc;
  esc.pwmMin = 1000; esc.pwmMax = 2000;
  esc.holdAfter = 1400;
  esc.rampStart = 1200; esc.rampEnd = 1400;
  esc.stepUs = 5; esc.stepMs = 200; esc.preArmMs = 3000;

  stab.begin(&pins, &pid, &lim, &r2p, &esc);
  stab.armESCSlow();           // gentle ESC ramp
  stab.escHoldAtConfigured();  // hold after arming
  Serial.println(F("RCStabilizer ready."));
}

// ===== RPilink ingest =====
void getData(){
  if (link.poll()) {
    const RPilink::Packet& p = link.last();
    // Only accept new data if it’s “fresh” (e.g., <= 300 ms old)
    if (p.fresh(300)) {
      roll_deg = p.roll;
      lin_vel  = p.v;
      ang_vel  = p.w;
      // If the RPi is commanding ESC microseconds directly for AUTO:
      AUTO_leftPWM  = constrain(p.lpwm, ESC_MIN_US, ESC_MAX_US);
      AUTO_rightPWM = constrain(p.rpwm, ESC_MIN_US, ESC_MAX_US);
    }
    // else: keep last known values (fail-safe)
  }
}

// ===== VelControl =====
void velControlSetup(){
  vc.begin(ESC1_NEUTRAL_US, ESC2_NEUTRAL_US);
  vc.setMaxVel(1.20f, 1.50f);        // [m/s], [rad/s]
  vc.setGainsV(0.90f, 0.60f, 0.05f); // linear PID
  vc.setGainsW(0.80f, 0.40f, 0.04f); // angular PID
  vc.setNeutralDeadbandUs(ESC_NEUTRAL_DB_US);
  vc.setEscBoundsUs(ESC_MIN_US, ESC_MAX_US);
  vc.setDerivativeAlpha(0.15f);      // derivative smoothing
  vc.setDtFallback(0.02f);           // 50 Hz loop fallback
  vc.resetTiming();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // wait for USB, if applicable

  Serial.println(F("\n=== Raft Controller ==="));
  Serial.println(F("CH1=steer, CH2=throttle, CH5=mode, CH6=kill"));

  // RC receiver (FlySky iBUS) on Serial1
  rc.begin(Serial1);
  Serial.println(F("FlySky iBUS started."));

  // RPi telemetry/control on USB Serial
  link.begin(Serial);
  Serial.println(F("RPilink started."));

  // ESC signal pins (adjust as needed)
  esc1.attach(2);  // Left motor ESC signal pin
  esc2.attach(3);  // Right motor ESC signal pin
  // Safe startup
  esc1.writeMicroseconds(ESC1_NEUTRAL_US);
  esc2.writeMicroseconds(ESC2_NEUTRAL_US);
  Serial.println(F("ESC startup"));

  // Stabilizer (flywheel) startup
  stabStartUp();
  Serial.println(F("Stabilizer ready."));

  // Vel startup
  velControlSetup();
  Serial.println(F("VelControl initialized."));

  Serial.println(F("🛑 ESCs disarmed. Flip CH6 high to arm..."));
  delay(2000);
}

void loop() {
  // 1) Read inbound data
  getData();

  // update stabilizer
  stab.update(roll_deg);   // consume roll_deg from RPilink

  // 2) Read RC channels (scaled: -100..100 with deadband)
  steer    = rc.readScaled(0); // CH1 = steering
  throttle = rc.readScaled(1); // CH2 = throttle
  mode     = rc.readScaled(4); // CH5 = MAN/AUTO
  kill     = rc.readScaled(5); // CH6 = arm/disarm

  // 3) Arm/disarm logic (CH6 only)
  if (!armed && kill >= 80) {
    armed = true;
    Serial.println(F("✅ ESCs ARMED"));
  } else if (armed && kill <= 20) {
    armed = false;
    Serial.println(F("🛑 ESCs DISARMED"));
  }

  // 4) Mode selection (CH5 only)
  bool want_manual = (mode <= 20);    // low → Manual, high → Auto (tune thresholds if 3-pos switch)
  if (manual != want_manual) {
    manual = want_manual;
    Serial.println(manual ? F("Mode: MANUAL") : F("Mode: AUTO"));
  }

  // 5) If disarmed, hold neutrals and exit quickly
  if (!armed) {
    esc1.writeMicroseconds(ESC1_NEUTRAL_US);
    esc2.writeMicroseconds(ESC2_NEUTRAL_US);
    delay(20);
    return;
  }

  // 6) Control based on mode
  if (!manual) {
    // Auto: use RPilink pwm commands
    leftPWM  = AUTO_leftPWM;
    rightPWM = AUTO_rightPWM;
    Serial.print(F("[AUTO] Using RPi PWM L=")); Serial.print(leftPWM);
    Serial.print(F("  R=")); Serial.println(rightPWM);
  } 
  else {
    // Manual: run velocity control using RC setpoints
    vc.update(throttle, steer, lin_vel, ang_vel);
    leftPWM  = vc.leftPWM();
    rightPWM = vc.rightPWM();

    Serial.print(F("[MANUAL PID] Thr=")); Serial.print(throttle);
    Serial.print(F("  Steer=")); Serial.print(steer);
    Serial.print(F("  v=")); Serial.print(lin_vel, 3);
    Serial.print(F("  w=")); Serial.print(ang_vel, 3);
    Serial.print(F("  L_PWM=")); Serial.print(leftPWM);
    Serial.print(F("  R_PWM=")); Serial.println(rightPWM);
  }

  // 7) Apply to ESCs
  esc1.writeMicroseconds(leftPWM);
  esc2.writeMicroseconds(rightPWM);

  // 8) Debug
  Serial.print(F("[OUT] Armed=")); Serial.print(armed);
  Serial.print(F("  Mode=")); Serial.print(manual ? F("MAN") : F("AUTO"));
  Serial.print(F("  Roll=")); Serial.print(roll_deg, 2);
  Serial.print(F("°  L=")); Serial.print(leftPWM);
  Serial.print(F("  R=")); Serial.println(rightPWM);
  Serial.println(F("--------------------------------"));

  delay(20);  // ~50 Hz loop
}
