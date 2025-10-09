/*
  UNO R3 – 4-ch RC PWM + CH5 as digital from PWM
  - CH1..CH4: D3, D5, D6, D9  (pin-change interrupts)
  - CH5: D10 (PWM capture → boolean 0/1)
  - Deadband + EMA for CH1..CH4
  - Fixed scaling: 1000/1500/2000 → 1000..2000
  - 'p' toggles print output
*/

#include <Arduino.h>

// ===== USER SETTINGS =====
const float    DEAD_BAND  = 0.05f;
const float    EMA_ALPHA  = 0.25f;
const uint16_t VALID_MIN  = 800;
const uint16_t VALID_MAX  = 2200;
const uint16_t FAILSAFE   = 1500;

const uint16_t CH5_THRESH_US = 1500;
const uint16_t CH5_HYST_US   = 100;   // hysteresis band
// ==========================

// CH1..CH4
const uint8_t CH_PINS[4] = {3, 5, 6, 9};
// CH5 on D10 (PB2)
const uint8_t CH5_PIN = 10;

// Masks for ISR edge detection
const uint8_t MASK_PD3 = _BV(PD3);
const uint8_t MASK_PD5 = _BV(PD5);
const uint8_t MASK_PD6 = _BV(PD6);
const uint8_t MASK_PB1 = _BV(PB1); // D9
const uint8_t MASK_PB2 = _BV(PB2); // D10

volatile uint16_t ch_us[4]   = {0,0,0,0};
volatile uint32_t ch_rise[4] = {0,0,0,0};

volatile uint16_t ch5_us   = 0;
volatile uint32_t ch5_rise = 0;

static volatile uint8_t lastPIND = 0;
static volatile uint8_t lastPINB = 0;

float    ch_norm[4]   = {0,0,0,0};
uint16_t ch_scaled[4] = {1500,1500,1500,1500};
uint8_t  ch5_state    = 0;

bool     printEnabled = true;
uint32_t lastPrintMs  = 0;

// ---------- Helpers ----------
float applyDeadband(float v, float dz) {
  if (fabs(v) < dz) return 0.0f;
  if (v > 0) return (v - dz) / (1.0f - dz);
  return (v + dz) / (1.0f - dz);
}
float ema(float prev, float now, float a) {
  return a*now + (1.0f - a)*prev;
}
float mapFixedToNorm(uint16_t us) {
  if (us < VALID_MIN || us > VALID_MAX) us = FAILSAFE;
  const uint16_t mn = 1000, md = 1500, mx = 2000;
  float n;
  if (us >= md) n = (us - md) / float(mx - md);
  else          n = -((md - us) / float(md - mn));
  return applyDeadband(n, DEAD_BAND);
}
uint16_t normToScaled(float n) {
  long v = lround(500.0f * n) + 1500;
  if (v < 1000) v = 1000;
  if (v > 2000) v = 2000;
  return (uint16_t)v;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  for (int i=0;i<4;i++) pinMode(CH_PINS[i], INPUT);
  pinMode(CH5_PIN, INPUT);

  cli();
  // PORTD: D3,D5,D6
  PCICR  |= _BV(PCIE2);
  PCMSK2 |= _BV(PCINT19) | _BV(PCINT21) | _BV(PCINT22);
  lastPIND = PIND;
  // PORTB: D9(D1), D10(D2)
  PCICR  |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT1) | _BV(PCINT2);
  lastPINB = PINB;
  sei();

  Serial.println(F("Commands: 'p' toggle print (scaled CH1-CH4 + CH5 boolean)"));
}

// ---------- ISRs ----------
ISR(PCINT2_vect) { // Port D: D3,D5,D6
  uint8_t nowD = PIND;
  uint8_t changed = nowD ^ lastPIND;
  lastPIND = nowD;
  uint32_t t = micros();
  if (changed & MASK_PD3) { if (nowD & MASK_PD3) ch_rise[0]=t; else {uint32_t w=t-ch_rise[0]; if(w<3000) ch_us[0]=w;} }
  if (changed & MASK_PD5) { if (nowD & MASK_PD5) ch_rise[1]=t; else {uint32_t w=t-ch_rise[1]; if(w<3000) ch_us[1]=w;} }
  if (changed & MASK_PD6) { if (nowD & MASK_PD6) ch_rise[2]=t; else {uint32_t w=t-ch_rise[2]; if(w<3000) ch_us[2]=w;} }
}
ISR(PCINT0_vect) { // Port B: D9,D10
  uint8_t nowB = PINB;
  uint8_t changed = nowB ^ lastPINB;
  lastPINB = nowB;
  uint32_t t = micros();

  if (changed & MASK_PB1) { // D9 CH4
    if (nowB & MASK_PB1) ch_rise[3]=t;
    else {uint32_t w=t-ch_rise[3]; if(w<3000) ch_us[3]=w;}
  }
  if (changed & MASK_PB2) { // D10 CH5
    if (nowB & MASK_PB2) ch5_rise=t;
    else {uint32_t w=t-ch5_rise; if(w<3000) ch5_us=w;}
  }
}

// ---------- Loop ----------
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='p' || c=='P') {
      printEnabled = !printEnabled;
      Serial.print(F("Print output ")); Serial.println(printEnabled?F("ENABLED"):F("DISABLED"));
    }
  }

  // CH1–CH4 filtering/scaling
  for (int i=0;i<4;i++){
    uint16_t us = ch_us[i] ? ch_us[i] : FAILSAFE;
    float n = mapFixedToNorm(us);
    ch_norm[i]   = ema(ch_norm[i], n, EMA_ALPHA);
    ch_scaled[i] = normToScaled(ch_norm[i]);
  }

  // CH5 threshold -> 0/1
  uint16_t u5 = ch5_us ? ch5_us : CH5_THRESH_US;
  if (u5 >= CH5_THRESH_US + CH5_HYST_US) ch5_state = 1;
  else if (u5 <= CH5_THRESH_US - CH5_HYST_US) ch5_state = 0;

  // print
  uint32_t now = millis();
  if (printEnabled && (now - lastPrintMs >= 50)) {
    lastPrintMs = now;
    Serial.print(F("1:")); Serial.print(ch_scaled[0]);
    Serial.print('\t'); Serial.print(F("2:")); Serial.print(ch_scaled[1]);
    Serial.print('\t'); Serial.print(F("3:")); Serial.print(ch_scaled[2]);
    Serial.print('\t'); Serial.print(F("4:")); Serial.print(ch_scaled[3]);
    Serial.print('\t'); Serial.print(F("5:")); Serial.println(ch5_state);
  }
}
