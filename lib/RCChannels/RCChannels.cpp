#include "RCChannels.h"

RCChannels::RCChannels() {
  // Default base values for all channels
  for (uint8_t i = 0; i < RCCH_NUM_CHANNELS; ++i) {
    cal_[i] = {1000, 1500, 2000};
  }

  // ==== USER CALIBRATION (from your measured values) ====
  cal_[0] = { 1000, 1499, 2000 };
  cal_[1] = { 1009, 1503, 2000 };
  cal_[2] = { 1000, 1492, 2000 };
  cal_[3] = { 1000, 1500, 2000 };
  cal_[4] = { 1000, 1000, 2000 };
  cal_[5] = { 1000, 1000, 2000 };
  // =======================================================

  deadband_ = 2;  // default ±2%
}

bool RCChannels::begin(HardwareSerial& serial, int8_t timerid, int8_t rxPin, int8_t txPin) {
  // IBusBM requires HardwareSerial reference, not Stream
  ibus_.begin(serial, timerid, rxPin, txPin);
  return true;
}

uint16_t RCChannels::readRaw(uint8_t ch) {
  if (ch >= RCCH_NUM_CHANNELS) return 0;
  return ibus_.readChannel(ch);  // returns 0 if missing
}

int8_t RCChannels::mapWithCal(uint16_t raw, const Cal& c) const {
  if (raw == 0) return 0;

  uint16_t minv = min(c.minv, c.maxv);
  uint16_t maxv = max(c.minv, c.maxv);
  uint16_t midv = constrain(c.midv, minv, maxv);

  int value;
  if (raw >= midv) {
    long num = (long)raw - (long)midv;
    long den = (long)maxv - (long)midv;
    if (den <= 0) return 0;
    value = (int)((num * 100L) / den);
  } else {
    long num = (long)midv - (long)raw;
    long den = (long)midv - (long)minv;
    if (den <= 0) return 0;
    value = -(int)((num * 100L) / den);
  }

  if (abs(value) < deadband_) value = 0;
  return (int8_t)constrain(value, -100, 100);
}

int8_t RCChannels::readScaled(uint8_t ch) {
  if (ch >= RCCH_NUM_CHANNELS) return 0;
  return mapWithCal(readRaw(ch), cal_[ch]);
}

void RCChannels::setCalibration(uint8_t ch, uint16_t minv, uint16_t midv, uint16_t maxv) {
  if (ch >= RCCH_NUM_CHANNELS) return;
  cal_[ch] = {minv, midv, maxv};
}

RCChannels::Cal RCChannels::getCalibration(uint8_t ch) const {
  if (ch >= RCCH_NUM_CHANNELS) return Cal{1000, 1500, 2000};
  return cal_[ch];
}

void RCChannels::setDeadband(uint8_t dbAbsPercent) {
  deadband_ = constrain(dbAbsPercent, 0, 20);
}

uint8_t RCChannels::getDeadband() const {
  return deadband_;
}

void RCChannels::setCalibrationTable(const Cal* table, uint8_t n) {
  if (!table) return;
  uint8_t lim = min(n, (uint8_t)RCCH_NUM_CHANNELS);
  for (uint8_t i = 0; i < lim; ++i) cal_[i] = table[i];
}
