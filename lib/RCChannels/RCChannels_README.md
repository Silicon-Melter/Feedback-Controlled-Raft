
# RCChannels — FlySky iBus Reader with Per‑Channel Calibration (-100..100)

`RCChannels` is a tiny Arduino/C++ library built on top of **IBusBM** that:
- Reads FlySky **iBus** receiver channels via a hardware UART (e.g. `Serial1`)
- Applies **per‑channel min/mid/max** calibration
- Returns **scaled values** (`-100..100`) with optional deadband
- Plays nicely with **ESC / servo control** (map to `1000..2000 µs`)

It’s ideal for RC robots, boats (differential drive), and custom vehicles.

---

## ✨ Features
- ✅ Hardware‑serial iBus using `IBusBM`
- ✅ Per‑channel calibration (hard‑coded once in `RCChannels.cpp`)
- ✅ Scaled output: `readScaled(ch) → -100..100` (deadband by default `±2%`)
- ✅ Raw output: `readRaw(ch) → 1000..2000 µs` (0 if channel missing)
- ✅ Lightweight, EEPROM‑free (simple, reproducible builds)

---

## 🧱 Requirements
- **IBusBM** library (install from Library Manager or drop in `libraries/IBusBM`)
- A board with **HardwareSerial** for receiver input (e.g., Arduino Mega, STM32/Nucleo)
  > UNO/Nano have no `Serial1`; use a board with spare UART so USB serial is free.

---

## 📦 Installation
1. Create a folder: `~/Documents/Arduino/libraries/RCChannels/`
2. Add files:
   - `library.properties`
   - `src/RCChannels.h`
   - `src/RCChannels.cpp`
3. Make sure **IBusBM** exists in `libraries/IBusBM/` (or install via Library Manager).
4. Restart the Arduino IDE.

> This repo intentionally avoids EEPROM so you can keep the build deterministic.

---

## 🔌 Wiring (iBus)
| From (Receiver) | To (MCU)        |
|-----------------|------------------|
| iBUS signal     | `Serial1.RX`     |
| +5 V (BEC)      | `5V` (if needed) |
| GND             | `GND` (common)   |

**All grounds must be common** between ESCs, receiver, and MCU.

---

## 🧠 API Overview

```cpp
#include <RCChannels.h>

RCChannels rc;

// Start iBus on Serial1
rc.begin(Serial1);

// Get raw microseconds (1000..2000, or 0 if missing)
uint16_t us = rc.readRaw(2);

// Get scaled value (-100..100)
int8_t val = rc.readScaled(2);

// Optional deadband (0..20 % of full scale)
rc.setDeadband(3);
uint8_t db = rc.getDeadband();

// Inspect/update a single channel’s calibration
RCChannels::Cal c = rc.getCalibration(0);     // {minv, midv, maxv}
rc.setCalibration(0, 1000, 1498, 2000);       // update CH0

// Replace many at once
RCChannels::Cal table[] = {
  {1000,1499,2000}, {1009,1503,2000}, {1000,1492,2000},
  {1000,1500,2000}, {1000,1000,2000}, {1000,1000,2000}
};
rc.setCalibrationTable(table, 6);
```

---

## ⚙️ Calibration (One‑Time, Hard‑Coded)

1) **Run a short calibration sketch** to print `min/mid/max` per channel.  
Copy the printed results.

```cpp
// CalibrateChannels.ino  (quick print-only)
#include <IBusBM.h>
IBusBM ibus;
const int N=6; uint16_t mn[N], md[N], mx[N];

void setup(){
  Serial.begin(115200); while(!Serial);
  Serial.println(F("Center all sticks, press ENTER."));
  ibus.begin(Serial1);
  while(!Serial.available()); while(Serial.available()) Serial.read();

  for(int i=0;i<N;i++){ uint16_t v=ibus.readChannel(i); md[i]= v? v:1500; }
  for(int i=0;i<N;i++){ mn[i]=2000; mx[i]=1000; }

  Serial.println(F("Move all sticks for 8s..."));
  unsigned long t=millis();
  while(millis()-t<8000){
    for(int i=0;i<N;i++){
      uint16_t v=ibus.readChannel(i); if(!v) continue;
      if(v<mn[i]) mn[i]=v; if(v>mx[i]) mx[i]=v;
    }
    delay(5);
  }
  Serial.println(F("
--- Paste into RCChannels.cpp ---"));
  for(int i=0;i<N;i++){
    Serial.print("cal_["),Serial.print(i),Serial.print("] = { ");
    Serial.print(mn[i]); Serial.print(", ");
    Serial.print(md[i]); Serial.print(", ");
    Serial.print(mx[i]); Serial.println(" };");
  }
}
void loop(){}
```

2) **Edit `src/RCChannels.cpp`** and paste into the constructor:
```cpp
RCChannels::RCChannels() {
  for(uint8_t i=0;i<RCCH_NUM_CHANNELS;++i) cal_[i] = {1000,1500,2000};
  // === YOUR MEASURED VALUES ===
  cal_[0] = { 1000, 1499, 2000 };
  cal_[1] = { 1009, 1503, 2000 };
  cal_[2] = { 1000, 1492, 2000 };
  cal_[3] = { 1000, 1500, 2000 };
  cal_[4] = { 1000, 1000, 2000 };
  cal_[5] = { 1000, 1000, 2000 };
  // ============================
  setDeadband(2);
}
```

Recompile once. All sketches will now use your calibration automatically.

---

## 🚀 Example: Differential ESC Drive (CH0 steer, CH2 throttle, CH6 kill)

```cpp
#include <Servo.h>
#include <RCChannels.h>

RCChannels rc;
Servo escL, escR;
bool armed=false;

void setup(){
  Serial.begin(115200); while(!Serial);
  rc.begin(Serial1);

  escL.attach(2); escR.attach(3);
  escL.writeMicroseconds(1000);
  escR.writeMicroseconds(1000);
  Serial.println(F("Disarmed. Set CH6 to 100 to arm."));
}

void loop(){
  int8_t steer = rc.readScaled(0);   // -100..100
  int8_t thr   = rc.readScaled(2);   // use 0..100 or -100..100 depending on your radio
  int8_t kill  = rc.readScaled(5);   // CH6: 0..100 (0=disarm, 100=arm)

  if(kill>=80 && !armed){ armed=true;  Serial.println(F("ARMED")); }
  if(kill<=20 &&  armed){ armed=false; Serial.println(F("DISARMED")); }

  if(!armed){ escL.writeMicroseconds(1000); escR.writeMicroseconds(1000); delay(20); return; }

  int steerMix = steer*70/100;
  int lPct = constrain((int)thr + steerMix, 0, 100);
  int rPct = constrain((int)thr - steerMix, 0, 100);

  int lUS = map(lPct,0,100,1000,2000);
  int rUS = map(rPct,0,100,1000,2000);

  escL.writeMicroseconds(lUS);
  escR.writeMicroseconds(rUS);

  delay(20);
}
```

---

## 🧪 Example: Manual Motor Test via Serial

```cpp
#include <Servo.h>
Servo esc1, esc2;

void setup(){
  Serial.begin(115200); while(!Serial);
  esc1.attach(2); esc2.attach(3);
  esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000);
  Serial.println(F("Type: M <1|2> <1000..2000>, or STOP"));
}
void loop(){
  if(!Serial.available()) return;
  String s = Serial.readStringUntil('
'); s.trim();
  if(s.equalsIgnoreCase("STOP")){
    esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000);
    Serial.println(F("Both 1000us")); return;
  }
  int m,v; if(sscanf(s.c_str(),"M %d %d",&m,&v)==2 && (m==1||m==2) && v>=1000 && v<=2000){
    (m==1?esc1:esc2).writeMicroseconds(v);
    Serial.print(F("Motor ")); Serial.print(m); Serial.print(F(" = ")); Serial.print(v); Serial.println(F("us"));
  } else {
    Serial.println(F("Usage: M <1|2> <1000..2000>"));
  }
}
```

---

## 🛠 Troubleshooting

| Issue | Cause | Fix |
|------|------|-----|
| No readings | Wrong UART | Use `Serial1` RX pin; correct wiring |
| Values jumpy | Trims/noise | Increase `setDeadband(3..5)` |
| Motor spins at neutral | ESC neutral mismatch | Add small per‑ESC neutral trims in application sketch |
| UNO won’t compile | No Serial1 | Use a board with HardwareSerial for iBus |

---

## 📄 `library.properties` (example)
```
name=RCChannels
version=1.0.0
author=YourName
maintainer=YourName
sentence=FlySky iBus reader with per-channel calibration and -100..100 scaling.
paragraph=Wraps IBusBM, stores hard-coded calibration, provides normalized channel outputs and examples including differential ESC driving.
category=Signal Input/Output
architectures=*
depends=IBusBM
```

---

## 🧬 Internals (Quick Notes)
- `begin(HardwareSerial&, int8_t timerid=0, int8_t rxPin=-1, int8_t txPin=-1)` matches **IBusBM** signature.
- Scaling is piecewise about `midv`, so asymmetrical throws still map cleanly to `-100..100`.
- Deadband is applied in **scaled units** for a consistent user feel.

---

## 📚 License
MIT. Do cool stuff, stay safe, test with props **off** first 🙂
