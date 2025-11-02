#include <Servo.h>
#include <RCChannels.h>
#include <RPilink.h>
#include <VelControl.h>

/* ========= ESC pins / limits / neutrals ========= */
Servo escL, escR;
const int PIN_ESC_L = 2;
const int PIN_ESC_R = 3;

const int ESC_MIN_US  = 1000;
const int ESC_MAX_US  = 2000;
int ESC_NEU_L_US      = 1430;
int ESC_NEU_R_US      = 1400;
const int ESC_NEU_DB  = 10;

/* ========= AUX DC motor pins (H-bridge) ========= */
const int AUX_CCW_PIN   = 10;   // PWM
const int AUX_CW_PIN    = 11;   // PWM
const int AUX_MAX_PWM   = 255;  // 8-bit analogWrite
const int AUX_MIN_START = 20;   // overcome stiction; tune as needed

/* ========= RC channel indices ========= */
const uint8_t CH_STEER = 0;  // shared: steer (MANUAL) | yaw-trim rate (AUTO)
const uint8_t CH_THR   = 1;  // throttle
const uint8_t CH_AUX   = 2;  // aux motor command (-100..100)
const uint8_t CH_MODE  = 4;  // MAN/AUTO (<=20 manual)
const uint8_t CH_KILL  = 5;  // arm/disarm (>=80 arm, <=20 disarm)

/* ========= Objects ========= */
RCChannels rc;
RPilink    link;
VelControl vc;

/* ========= State ========= */
bool  armed  = false;
bool  manual = true;
float yaw_set_deg = 0.0f;
bool  yaw_seeded  = false;

int8_t steer = 0, throttle = 0, ch3_aux = 0, mode = 0, kill = 0;
int L_us = 1500, R_us = 1500;
bool piFresh = false;
unsigned long yawTrim_prev_ms = 0;

/* ========= Helpers ========= */
static inline float wrapDeg180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static inline int pctToPwmWithNeutral(int pct, int neutral_us) {
  int span = (pct >= 0) ? (ESC_MAX_US - neutral_us) : (neutral_us - ESC_MIN_US);
  long us  = (long)neutral_us + ((long)pct * span) / 100;
  if (abs((int)us - neutral_us) <= ESC_NEU_DB) us = neutral_us;
  return (int)constrain(us, ESC_MIN_US, ESC_MAX_US);
}
static inline float yawTrimRateFromStick(int8_t stick_pct) { return ((float)stick_pct) / 5.0f; }
float stepDt(unsigned long &t_prev_ms, float fallback = 0.02f) {
  unsigned long now = millis();
  float dt = (t_prev_ms == 0) ? fallback : (now - t_prev_ms) * 0.001f;
  if (dt <= 0.0005f || dt > 0.20f) dt = fallback;
  t_prev_ms = now;
  return dt;
}
void holdNeutral() {
  escL.writeMicroseconds(ESC_NEU_L_US); L_us = ESC_NEU_L_US;
  escR.writeMicroseconds(ESC_NEU_R_US); R_us = ESC_NEU_R_US;
}

/* ========= AUX motor driver (your function) ========= */
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

/* ========= Pi data poll wrapper ========= */
void getData() {
  link.poll();
  piFresh = link.last().fresh(500);
}

/* ===================== setup ===================== */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  rc.begin(Serial1);
  link.begin(Serial);

  escL.attach(PIN_ESC_L);
  escR.attach(PIN_ESC_R);
  holdNeutral();

  // AUX motor pins
  pinMode(AUX_CCW_PIN, OUTPUT);
  pinMode(AUX_CW_PIN,  OUTPUT);
  driveAuxMotor(0);  // make sure it's stopped

  vc.begin(ESC_NEU_L_US, ESC_NEU_R_US);
  vc.setEscBoundsUs(ESC_MIN_US, ESC_MAX_US);
  vc.setNeutralDeadbandUs(ESC_NEU_DB);
  vc.setGainsV(400.0f, 0.0f, 0.0f);
  vc.setGainsW(5.0f, 0.05f, 0.1f);
  vc.setMaxVel(0.3f, 1.5f);
  vc.setDtFallback(0.02f);
  vc.setDtClamp(0.001f, 0.20f);

  Serial.println(F("Ready."));
}

/* ====================== loop ====================== */
void loop() {
  // 1) Read inbound data
  getData();

  // 2) Read RC channels
  steer    = rc.readScaled(0); // CH1 = steering (or yaw-trim in AUTO)
  throttle = rc.readScaled(1); // CH2 = throttle
  ch3_aux  = rc.readScaled(2); // CH3 = aux motor (-100..100)
  mode     = rc.readScaled(4); // CH5 = MAN/AUTO
  kill     = rc.readScaled(5); // CH6 = arm/disarm

  // 3) Arm/disarm (exact thresholds)
  if (!armed && kill >= 80) {
    armed = true;
    yaw_seeded = false;
    vc.resetTiming();
    Serial.println(F("✅ ESCs ARMED"));
  } else if (armed && kill <= 20) {
    armed = false;
    Serial.println(F("🛑 ESCs DISARMED"));
  }

  // 4) Mode selection (exact threshold)
  bool want_manual = (mode <= 20);    // low → Manual, high → Auto
  if (manual != want_manual) {
    manual = want_manual;
    Serial.println(manual ? F("Mode: MANUAL") : F("Mode: AUTO"));
  }

  // If disarmed, hold neutrals and stop aux
  if (!armed) {
    holdNeutral();
    driveAuxMotor(0);
    Serial.print(F("DISARM L=")); Serial.print(L_us);
    Serial.print(F(" R="));       Serial.println(R_us);
    delay(20);
    return;
  }

  // Seed yaw target on first fresh yaw after arming
  if (!yaw_seeded && piFresh) {
    yaw_set_deg = wrapDeg180(link.last().yaw);
    yaw_seeded  = true;
    Serial.print(F("Yaw seeded at ")); Serial.print(yaw_set_deg, 2); Serial.println(F(" deg"));
  }

  // 5) ESC control
  if (manual) {
    driveAuxMotor(ch3_aux);
    int leftPct  = constrain((int)throttle - (int)steer, -100, 100);
    int rightPct = constrain((int)throttle + (int)steer, -100, 100);
    L_us = pctToPwmWithNeutral(leftPct,  ESC_NEU_L_US);
    R_us = pctToPwmWithNeutral(rightPct, ESC_NEU_R_US);
    escL.writeMicroseconds(L_us);
    escR.writeMicroseconds(R_us);
  } else {
    driveAuxMotor(ch3_aux);
    float dtTrim = stepDt(yawTrim_prev_ms, 0.02f);
    yaw_set_deg += -yawTrimRateFromStick(steer) * dtTrim;  // CH0 as setpoint rate
    yaw_set_deg = wrapDeg180(yaw_set_deg);

    if (piFresh) {
      float v_meas   = link.last().v;
      float yaw_meas = link.last().yaw;
      vc.updateYawAngle(throttle, yaw_set_deg, yaw_meas, v_meas);
      L_us = vc.leftPWM();
      R_us = vc.rightPWM();
      escL.writeMicroseconds(L_us);
      escR.writeMicroseconds(R_us);
    } else {
      int leftPct  = constrain((int)throttle - (int)steer, -100, 100);
      int rightPct = constrain((int)throttle + (int)steer, -100, 100);
      L_us = pctToPwmWithNeutral(leftPct,  ESC_NEU_L_US);
      R_us = pctToPwmWithNeutral(rightPct, ESC_NEU_R_US);
      escL.writeMicroseconds(L_us);
      escR.writeMicroseconds(R_us);
    }
  }

  // 6) AUX motor (H-bridge) — ONLY when armed
  driveAuxMotor(ch3_aux);

  // 7) Debug: always print current ESC PWMs + AUX cmd
  Serial.print(F("man="));   Serial.print(manual);
  Serial.print(F(" arm="));  Serial.print(armed);
  Serial.print(F(" thr="));  Serial.print(throttle);
  Serial.print(F(" steer="));Serial.print(steer);
  Serial.print(F(" aux="));  Serial.print(ch3_aux);
  Serial.print(F(" yawSet=")); Serial.print(yaw_set_deg, 2);
  Serial.print(F(" yaw="));  Serial.print(link.last().yaw, 2);
  Serial.print(F(" v="));    Serial.print(link.last().v, 3);
  Serial.print(F(" L="));    Serial.print(L_us);
  Serial.print(F(" R="));    Serial.println(R_us);

  delay(5); // ~200 Hz
}
