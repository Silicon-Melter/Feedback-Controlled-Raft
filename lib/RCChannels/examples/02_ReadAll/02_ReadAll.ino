#include <RCChannels.h>
RCChannels rc;

const uint8_t CHANNELS = 6;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("\n=== RCChannels: Read All ==="));

  rc.begin(Serial1);

  if (!rc.loadCalibration()) {
    Serial.println(F("No valid calibration in EEPROM. Using defaults 1000/1500/2000."));
    rc.resetCalibrationToDefaults();
  }
}

void loop() {
  Serial.print(F("RAW: "));
  for (uint8_t ch = 0; ch < CHANNELS; ++ch) {
    uint16_t r = rc.readRaw(ch);
    Serial.print(r); Serial.print('\t');
  }

  Serial.print(F(" | SCALED: "));
  for (uint8_t ch = 0; ch < CHANNELS; ++ch) {
    int8_t s = rc.readScaled(ch);
    Serial.print((int)s); Serial.print('\t');
  }
  Serial.println();

  delay(50);
}
