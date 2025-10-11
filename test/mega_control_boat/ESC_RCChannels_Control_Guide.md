# 🛩️ Controlling Brushless ESCs with RC Transmitter using `RCChannels`

This project shows how to control **two brushless motors (ESCs)** using a **FlySky iBus receiver** and a custom **`RCChannels` library** for accurate channel scaling and calibration.

You can steer and throttle both motors from your RC transmitter — ideal for **boats, differential drive robots, or dual-motor drones**.

---

## 📦 Features
✅ Reads FlySky receiver data through **iBus protocol**  
✅ Converts raw PWM to **–100 → +100 range** (based on calibration)  
✅ Drives **two ESCs** using differential mixing (tank drive style)  
✅ Includes **kill switch (CH6)** for safety  
✅ Supports per-ESC **neutral trim & deadband** tuning  
✅ All calibration handled once inside `RCChannels.cpp`

---

## 🧩 System Overview

| Component | Description |
|------------|--------------|
| **Microcontroller** | Arduino / STM32 / Nucleo board with Serial1 |
| **Receiver** | FlySky FS-iA6B or similar (iBUS output) |
| **Motors** | 2 × Brushless motors with ESCs |
| **Library** | Custom `RCChannels` for channel normalization |
| **Control** | CH0 = Steering, CH2 = Throttle, CH6 = Kill switch |

---

## ⚙️ Wiring

| Connection | From | To |
|-------------|------|----|
| FlySky iBUS | Receiver **iBUS** | Board **Serial1 RX** |
| ESC 1 (Left) | ESC signal wire | **D2** |
| ESC 2 (Right) | ESC signal wire | **D3** |
| Power | ESC BEC 5 V | Board 5 V pin |
| Ground | ESC GND + Receiver GND | Common GND |

> ⚠️ All grounds must be connected together — receiver, ESCs, and MCU.

---

## 🧠 Library: `RCChannels`

The `RCChannels` library reads and scales iBus signals from your receiver.  
Each channel’s **min, mid, and max values** are stored directly inside `RCChannels.cpp`.

Example calibration section:
```cpp
// In RCChannels::RCChannels()
cal_[0] = { 1000, 1499, 2000 }; // CH0
cal_[1] = { 1009, 1503, 2000 }; // CH1
cal_[2] = { 1000, 1492, 2000 }; // CH2
cal_[3] = { 1000, 1500, 2000 }; // CH3
cal_[4] = { 1000, 1000, 2000 }; // CH4
cal_[5] = { 1000, 1000, 2000 }; // CH5 (CH6 on transmitter)
```

After this setup, you can call:
```cpp
int8_t throttle = rc.readScaled(2);  // CH2
int8_t steering = rc.readScaled(0);  // CH0
```
Each returns –100 → +100 (or 0 → 100 if your channel doesn’t reverse).

---

## 🚀 Main Control Code

### `DifferentialESC_RCChannels.ino`
```cpp
#include <Servo.h>
#include <RCChannels.h>

RCChannels rc;
Servo esc1, esc2;
bool armed = false;  // start disarmed

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("\n=== Differential ESC Drive (RCChannels) ==="));
  Serial.println(F("CH0=Steer, CH2=Throttle, CH6=Kill Switch (0=Disarm, 100=Arm)"));

  rc.begin(Serial1);     // Start iBus
  esc1.attach(2);        // ESC1 signal
  esc2.attach(3);        // ESC2 signal

  // Safety startup
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  Serial.println(F("🛑 ESCs disarmed. Flip CH6 to 100 to arm."));
  delay(2000);
}

void loop() {
  int8_t steer    = rc.readScaled(0); // CH0
  int8_t throttle = rc.readScaled(2); // CH2
  int8_t kill     = rc.readScaled(5); // CH6

  // Kill switch logic (0 = disarm, 100 = arm)
  if (kill >= 80 && !armed) {
    armed = true;
    Serial.println(F("✅ ESCs ARMED"));
  } else if (kill <= 20 && armed) {
    armed = false;
    Serial.println(F("🛑 ESCs DISARMED"));
  }

  if (!armed) {
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    delay(20);
    return;
  }

  // Differential drive
  int steerMix = steer * 70 / 100;
  int leftPct  = constrain(throttle + steerMix, 0, 100);
  int rightPct = constrain(throttle - steerMix, 0, 100);

  int leftPWM  = map(leftPct,  0, 100, 1000, 2000);
  int rightPWM = map(rightPct, 0, 100, 1000, 2000);

  esc1.writeMicroseconds(leftPWM);
  esc2.writeMicroseconds(rightPWM);

  Serial.print(F("Kill=")); Serial.print(kill);
  Serial.print(F("  Thr=")); Serial.print(throttle);
  Serial.print(F("  Str=")); Serial.print(steer);
  Serial.print(F("  L=")); Serial.print(leftPWM);
  Serial.print(F("  R=")); Serial.println(rightPWM);

  delay(20);
}
```

---

## 🔒 Safety Logic

| State | Kill Switch (CH6) | Behavior |
|--------|-------------------|-----------|
| **Disarmed** | 0–20 | ESCs hold 1000 µs (motors off) |
| **Armed** | 80–100 | ESCs respond to throttle & steering |
| **Default** | Startup = disarmed | Prevents spin on power-up |

---

## 🔧 Calibration Tips

1. Use your calibration sketch to measure **min/mid/max** for each channel.
2. Paste those into `RCChannels.cpp` under the constructor:
   ```cpp
   cal_[0] = { min, mid, max };
   ```
3. Rebuild the library once — it will always load the correct scaling automatically.

---

## ⚡ ESC Neutral Trim

If motors don’t stop exactly at neutral (e.g., one spins at 1500 µs),  
tweak these constants in your main code:
```cpp
int ESC1_NEUTRAL_US = 1502;
int ESC2_NEUTRAL_US = 1498;
const int ESC_NEUTRAL_DB_US = 10;
```
This ensures both ESCs stay idle at neutral throttle.

---

## 🧪 Test Mode via Serial
Use this minimal sketch to manually control each ESC for testing:
```
M 1 1500
M 2 1600
STOP
```
See [SerialMotorControl.ino](SerialMotorControl.ino) for details.

---

## 🛠️ Troubleshooting

| Symptom | Likely Cause | Fix |
|----------|---------------|-----|
| One motor spins at neutral | ESC neutral drift | Adjust `ESCx_NEUTRAL_US` |
| Both spin at startup | Not disarmed | Ensure `armed = false` on boot |
| Motors don’t respond | Wrong Serial port | Use correct `Serial1` (hardware RX) |
| Random twitching | Missing common ground | Tie ESC, MCU, and receiver GND together |

---

## 🧾 Summary

| Channel | Function | Range | Description |
|----------|-----------|--------|-------------|
| **CH0** | Steering | –100 → +100 | Differential direction |
| **CH2** | Throttle | 0 → 100 | Forward/backward power |
| **CH6** | Kill Switch | 0 = disarm, 100 = arm | Safety lock |

---

## 🧰 Recommended Equipment
- FlySky FS-i6 transmitter + FS-iA6B receiver  
- 2 × 30 A+ brushless ESCs  
- 2 × Brushless motors (e.g. 2212 or 4114)  
- 5 V BEC for logic power  
- Arduino Mega / STM32 / Nucleo board with Serial1  

---
