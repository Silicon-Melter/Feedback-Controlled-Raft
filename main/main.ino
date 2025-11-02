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
int       ESC1_NEUTRAL_US    = 1430;  // left
int       ESC2_NEUTRAL_US    = 1400;  // right (your measured neutral)
const int ESC_NEUTRAL_DB_US  = 0;    // snap-to-neutral deadband


bool  armed  = false;   // start disarmed for safety
bool  manual = true;    // CH5 selects MAN/AUTO
float Ksteering = 0.8;

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

// ===== Aux DC motor (H-bridge) on pins 10/11, driven by CH3 =====
const uint8_t AUX_CCW_PIN     = 10;   // drive this for CCW
const uint8_t AUX_CW_PIN      = 11;   // drive this for CW
const uint8_t AUX_MIN_START   = 30;   // minimum PWM to overcome stiction
const uint8_t AUX_MAX_PWM     = 255;  // cap
int8_t ch3_aux = 0;                   // CH3 = aux motor command (-100..100)

// Convert -100..100 (%) to microseconds around a given neutral
static inline int pctToPwmWithNeutral(int pct, int neutral_us) {
  int span = (pct >= 0) ? (ESC_MAX_US - neutral_us) : (neutral_us - ESC_MIN_US);
  long us  = (long)neutral_us + ((long)pct * span) / 100;
  if (abs((int)us - neutral_us) <= ESC_NEUTRAL_DB_US) us = neutral_us;  // snap-to-neutral
  return (int)constrain(us, ESC_MIN_US, ESC_MAX_US);
}

// ===== Stabilizer bring-up =====
void stabStartUp(){
  Serial.println(F("Starting Stabilizer"));

  RCStabilizer::Pins pins;        // use defaults from the library unless you need to override
  pins.escPin = 7;
  pins.motorA = 12;
  pins.motorB = 13;
  pins.limitCCW = 24;    // limit switch CCW
  pins.limitCW  = 25;    // limit switch CW
  pins.limitActiveLow = true;

  RCStabilizer::PID pid;          // Kp,Ki,Kd (your values carried over)
  pid.Kp = 2.0f; pid.Ki = 0.0f; pid.Kd = 0.0f;
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
  esc.holdAfter = 1300;
  esc.rampStart = 1000; esc.rampEnd = 1300;
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
  vc.setMaxVel(0.3f, 0.50f);        // [m/s], [rad/s]
  vc.setGainsV(10.0f, 0.0f, 0.0f); // linear PID
  vc.setGainsW(40.0f, 0.0f, 0.0f); // angular PID
  vc.setNeutralDeadbandUs(ESC_NEUTRAL_DB_US);
  vc.setEscBoundsUs(ESC_MIN_US, ESC_MAX_US);
  vc.setDerivativeAlpha(0.15f);      // derivative smoothing
  vc.setDtFallback(0.02f);           // 50 Hz loop fallback
  vc.resetTiming();
}

static inline void driveAuxMotor(int8_t cmdPercent) {
  // cmdPercent: -100..100  (>0 → CCW, <0 → CW)
  int mag = abs(cmdPercent);
  int pwm = map(mag, 0, 100, 0, AUX_MAX_PWM);
  if (pwm > 0 && pwm < AUX_MIN_START) pwm = AUX_MIN_START;

  if (cmdPercent > 0) {
    // CCW
    analogWrite(AUX_CCW_PIN, pwm);
    analogWrite(AUX_CW_PIN,  0);
  } else if (cmdPercent < 0) {
    // CW
    analogWrite(AUX_CCW_PIN, 0);
    analogWrite(AUX_CW_PIN,  pwm);
  } else {
    // stop
    analogWrite(AUX_CCW_PIN, 0);
    analogWrite(AUX_CW_PIN,  0);
  }
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
  //stabStartUp();
  // Serial.println(F("Stabilizer ready."));

  // Vel startup
  velControlSetup();
  Serial.println(F("VelControl initialized."));

  // Aux DC motor pins
  pinMode(AUX_CCW_PIN, OUTPUT);
  pinMode(AUX_CW_PIN,  OUTPUT);
  analogWrite(AUX_CCW_PIN, 0);
  analogWrite(AUX_CW_PIN,  0);

  Serial.println(F("🛑 ESCs disarmed. Flip CH6 high to arm..."));
  delay(2000);
}

void loop() {
  // 1) Read inbound data
  getData();

  // update stabilizer
  //stab.update(roll_deg);   // consume roll_deg from RPilink

  // 2) Read RC channels (scaled: -100..100 with deadband)
  steer    = rc.readScaled(0); // CH1 = steering
  throttle = rc.readScaled(1); // CH2 = throttle
  ch3_aux  = rc.readScaled(2); // CH3 = aux motor
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
    driveAuxMotor(0);
    delay(20);
    return;
  }

  // 6) Control based on mode
  if (!manual) {
    // Auto: use RPilink pwm commands
    //leftPWM  = AUTO_leftPWM;
    //rightPWM = AUTO_rightPWM;
    //Serial.print(F("[AUTO] Using RPi PWM L=")); Serial.print(leftPWM);
    //Serial.print(F("  R=")); Serial.println(rightPWM);

    //raw manual
    int steerMix = steer * Ksteering;  // reduce steering sensitivity
    int leftPct  = constrain(throttle + steerMix, -100, 100);
    int rightPct = constrain(throttle - steerMix, -100, 100);

    leftPWM  = pctToPwmWithNeutral(leftPct,  ESC1_NEUTRAL_US);
    rightPWM = pctToPwmWithNeutral(rightPct, ESC2_NEUTRAL_US);
    Serial.print(F("[RAW] Using RAW PWM L=")); Serial.print(leftPWM);
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

  // 6.5) Drive Aux DC motor from CH3 (only when ARMED)
  driveAuxMotor(ch3_aux);   // >0 CCW, <0 CW
  Serial.print(F("[AUX] CH3=")); Serial.print(ch3_aux);
  Serial.print(F("  PWM="));
  Serial.println(map(abs(ch3_aux), 0, 100, 0, AUX_MAX_PWM));

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

}
