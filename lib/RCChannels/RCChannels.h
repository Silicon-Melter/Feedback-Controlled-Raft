#pragma once
#include <Arduino.h>
#include <IBusBM.h>

#ifndef RCCH_NUM_CHANNELS
#define RCCH_NUM_CHANNELS 10
#endif

class RCChannels {
public:
  struct Cal {
    uint16_t minv;  // ~1000
    uint16_t midv;  // ~1500
    uint16_t maxv;  // ~2000
  };

  RCChannels();

  // NOTE: IBusBM requires HardwareSerial& (not Stream)
  bool begin(HardwareSerial& serial, int8_t timerid=0, int8_t rxPin=-1, int8_t txPin=-1);

  // Raw microseconds (1000..2000); 0 if channel missing
  uint16_t readRaw(uint8_t ch);

  // Scaled using calibration: -100..100 (deadband applied)
  int8_t   readScaled(uint8_t ch);

  // Calibration + deadband controls
  void     setCalibration(uint8_t ch, uint16_t minv, uint16_t midv, uint16_t maxv);
  Cal      getCalibration(uint8_t ch) const;
  void     setDeadband(uint8_t dbAbsPercent);   // 0..20 typical
  uint8_t  getDeadband() const;

  // Optionally set many channels at once
  void     setCalibrationTable(const Cal* table, uint8_t n);

private:
  int8_t   mapWithCal(uint16_t raw, const Cal& c) const;

  IBusBM   ibus_;
  Cal      cal_[RCCH_NUM_CHANNELS];
  uint8_t  deadband_ = 2;  // ±2% default
};
