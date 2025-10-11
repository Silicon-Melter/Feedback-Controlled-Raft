#include <IBusBM.h>

IBusBM ibus;

// --- Adjust how many channels you want to calibrate ---
const int NUM_CHANNELS = 6;

// Calibration data
uint16_t chMin[NUM_CHANNELS];
uint16_t chMid[NUM_CHANNELS];
uint16_t chMax[NUM_CHANNELS];

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("\n=== RC Channel Calibration ==="));
  Serial.println(F("Connect your FlySky receiver (iBUS) to Serial1 RX"));
  Serial.println(F("1️⃣ Center all sticks, switches, trims, then press ENTER..."));
  ibus.begin(Serial1);

  // Wait for user
  while (!Serial.available());
  while (Serial.available()) Serial.read();

  // Capture MID
  for (int i = 0; i < NUM_CHANNELS; i++) {
    uint16_t v = ibus.readChannel(i);
    chMid[i] = v == 0 ? 1500 : v;
  }
  Serial.println(F("✅ Midpoints captured.\n"));

  // Capture MIN/MAX
  Serial.println(F("2️⃣ Move all sticks and switches through their full range for 10 seconds..."));
  for (int i = 0; i < NUM_CHANNELS; i++) {
    chMin[i] = 2000;
    chMax[i] = 1000;
  }

  unsigned long start = millis();
  while (millis() - start < 10000) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      uint16_t v = ibus.readChannel(i);
      if (v == 0) continue;
      if (v < chMin[i]) chMin[i] = v;
      if (v > chMax[i]) chMax[i] = v;
    }
    delay(5);
  }

  Serial.println(F("✅ Min/Max captured."));
  Serial.println(F("\n--- Calibration Results ---"));

  for (int i = 0; i < NUM_CHANNELS; i++) {
    Serial.print("CH"); Serial.print(i); Serial.print(" = { ");
    Serial.print("min: "); Serial.print(chMin[i]);
    Serial.print(", mid: "); Serial.print(chMid[i]);
    Serial.print(", max: "); Serial.print(chMax[i]);
    Serial.println(" };");
  }

  Serial.println(F("\n📋 Copy these values into your RCChannels library calibration table."));
}

void loop() {
  // Idle
}
