#include <Servo.h>
#include <RCChannels.h>

RCChannels rc;
Servo esc1, esc2;  

const int ESC_MIN_US       = 1000;
const int ESC_MAX_US       = 2000;
int ESC1_NEUTRAL_US        = 1500;
int ESC2_NEUTRAL_US        = 1500;
const int ESC_NEUTRAL_DB_US = 10;

bool armed = false;  // start disarmed for safety
float Ksteering = 0.8;

float yaw_deg = 0.0f; 

static inline int pctToPwmWithNeutral(int pct, int neutral_us) {
  // pct: -100..100 mapped around a custom neutral with asymmetrical span
  int span = (pct >= 0) ? (ESC_MAX_US - neutral_us) : (neutral_us - ESC_MIN_US);
  long us  = (long)neutral_us + ((long)pct * span) / 100;

  // Apply neutral deadband: snap to neutral if within ±ESC_NEUTRAL_DB_US
  if (abs((int)us - neutral_us) <= ESC_NEUTRAL_DB_US) us = neutral_us;

  return (int)constrain(us, ESC_MIN_US, ESC_MAX_US);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("\n=== Differential ESC Drive (RCChannels) ==="));
  Serial.println(F("CH0 = steering, CH2 = throttle, CH6 = kill switch"));

  // Start iBus (FlySky iBUS to Serial1 RX)
  rc.begin(Serial1);

  // Attach ESCs
  esc1.attach(2);  // Left motor
  esc2.attach(3);  // Right motor

  // Safety startup
  esc1.writeMicroseconds(ESC1_NEUTRAL_US);
  esc2.writeMicroseconds(ESC2_NEUTRAL_US);
  Serial.println(F("🛑 ESCs disarmed. Waiting for CH6 high to arm..."));
  delay(2000);
}

void loop() {
  // Read key channels
  int8_t steer    = rc.readScaled(0); // CH0 = steering
  int8_t throttle = rc.readScaled(1); // CH2 = throttle
  int8_t kill     = rc.readScaled(5); // CH6 = kill switch (CH index starts at 0)

// --- Arm/disarm logic ---
  if (kill >= 80 && !armed) {         // switch near 100%
    armed = true;
    Serial.println(F("✅ ESCs ARMED"));
  }
  else if (kill <= 20 && armed) {     // switch near 0%
    armed = false;
    Serial.println(F("🛑 ESCs DISARMED"));
  }

  // --- If disarmed, hold 1000 µs ---
  if (!armed) {
    esc1.writeMicroseconds(ESC1_NEUTRAL_US);
    esc2.writeMicroseconds(ESC2_NEUTRAL_US);
    delay(20);
    
    return;
  }

  float lin_speed = constrain(throttle, -5, 5);
  float ang_speed = constrain(steer, -20, 20);

  //send lin_speed ang_speed to pi

  //get returned pwm from pi
  int leftPWM;
  int rightPWM;

  // Apply PWM to ESCs
  esc1.writeMicroseconds(leftPWM);
  esc2.writeMicroseconds(rightPWM);

  // === Debug output ===
  Serial.print(F("Kill=")); Serial.print(kill);
  Serial.print(F("  Armed=")); Serial.print(armed);
  Serial.print(F("  Thr=")); Serial.print(throttle);
  Serial.print(F("  Str=")); Serial.print(steer);
  Serial.print(F("  L=")); Serial.print(leftPWM);
  Serial.print(F("  R=")); Serial.println(rightPWM);

  delay(20);  // ~50 Hz loop
}
