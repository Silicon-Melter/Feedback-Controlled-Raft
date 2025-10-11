#include <RCChannels.h>
#include <Servo.h>

RCChannels rc;

const uint8_t CHANNELS = 6;

// ESC pins (adjust to your board)
const uint8_t ESC_LEFT_PIN  = 9;
const uint8_t ESC_RIGHT_PIN = 10;

Servo escL, escR;

int pwmFromPercent(int8_t p) {
  // -100..100 -> 1000..2000 µs
  long out = map((long)p, -100, 100, 1000, 2000);
  return (int)constrain(out, 1000, 2000);
}

void armESCs() {
  // Simple arming: low for 2s, then center for 1s (adjust to your ESC manual)
  escL.writeMicroseconds(1000);
  escR.writeMicroseconds(1000);
  delay(2000);

  escL.writeMicroseconds(1500);
  escR.writeMicroseconds(1500);
  delay(1000);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println(F("\n=== RCChannels: Differential ESC (CH0 steering, CH2 throttle) ==="));

  rc.begin(Serial1);

  if (!rc.loadCalibration()) {
    Serial.println(F("No valid calibration in EEPROM. Using defaults 1000/1500/2000."));
    rc.resetCalibrationToDefaults();
  }

  escL.attach(ESC_LEFT_PIN);
  escR.attach(ESC_RIGHT_PIN);
  armESCs();
}

void loop() {
  // Scaled inputs
  int8_t throttle = rc.readScaled(2); // CH2
  int8_t steer    = rc.readScaled(0); // CH0

  // Mix: add/subtract steering from throttle
  // Optional: reduce steering authority (e.g., 70%) to taste
  int steerScaled = (int)steer * 70 / 100;

  int leftPct  = constrain((int)throttle + steerScaled, -100, 100);
  int rightPct = constrain((int)throttle - steerScaled, -100, 100);

  int leftPWM  = pwmFromPercent(leftPct);
  int rightPWM = pwmFromPercent(rightPct);

  escL.writeMicroseconds(leftPWM);
  escR.writeMicroseconds(rightPWM);

  // Debug
  Serial.print(F("thr=")); Serial.print((int)throttle);
  Serial.print(F("  str=")); Serial.print((int)steer);
  Serial.print(F("  L=")); Serial.print(leftPWM);
  Serial.print(F("  R=")); Serial.println(rightPWM);

  delay(20); // ~50 Hz
}
