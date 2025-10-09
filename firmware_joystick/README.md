# 🛩️ Arduino RC PWM Reader (UNO R3)

This project reads **up to 5 RC channels** from a FlySky (or similar) receiver using **hardware pin-change interrupts** on an Arduino UNO R3.  
It outputs stable, filtered channel data over Serial, with **deadband filtering**, **EMA smoothing**, and a **binary switch conversion** for Channel 5.

---

## ✨ Features

- ✅ 4 PWM input channels (CH1–CH4) with **filtered 1000–2000 output**
- ✅ 1 additional PWM input (CH5) read on pin **D10** and converted to **digital 0/1**
- ✅ **Deadband** and **exponential moving average** filtering for smooth control
- ✅ **Non-blocking** ISR-based pulse capture (no `pulseIn()` delays)
- ✅ **Serial output** at 115200 baud
- ✅ **Toggle print output** via `'p'` command

---

## ⚙️ Hardware Setup

### 🧩 Components

- Arduino **UNO R3**
- RC Receiver (e.g., **FlySky FS-iA6B**)
- Standard 3-pin servo cables (signal, +5 V, GND)

### ⚡ Wiring Table

| RC Channel    | Function                  | Arduino Pin | Type                                     | Notes                                   |
| -------------- | ------------------------- | ------------ | ---------------------------------------- | --------------------------------------- |
| CH1           | Right stick – Horizontal  | D3           | PWM input                                | Captured via pin-change interrupt       |
| CH2           | Right stick – Vertical    | D5           | PWM input                                | —                                       |
| CH3           | Left stick – Vertical     | D6           | PWM input                                | —                                       |
| CH4           | Left stick – Horizontal   | D9           | PWM input                                | —                                       |
| CH5           | 3-position switch / knob  | D10          | PWM input → digital (0/1)                | Converts ~1000 µs → `0`, ~2000 µs → `1` |
| +V (Receiver) | Power                     | 5 V          | —                                        |                                         |
| GND           | Ground                    | GND          | Must be common with receiver and Arduino |                                         |

📎 **Important:**
- The receiver must be powered from the Arduino’s **5 V and GND**.  
- Each signal wire connects to the Arduino pin listed above.

---

## 🧠 How It Works

1. **PWM Capture**  
   Each RC channel outputs a pulse between **1000–2000 µs** every 20 ms.  
   The Arduino measures pulse width using **interrupts**, avoiding delays.

2. **Filtering & Scaling**  
   Channels 1–4 are filtered using:
   - **Deadband:** Ignores small stick jitter around center.  
   - **EMA (Exponential Moving Average):** Smooths rapid value changes.  
   Then scaled to a **1000–2000** range centered at **1500**.

3. **CH5 Binary Conversion**  
   CH5 is read as PWM, then converted:
   - Pulse ≤ 1400 µs → `0`  
   - Pulse ≥ 1600 µs → `1`  
   - Between 1400–1600 µs keeps previous state (hysteresis)

4. **Serial Output**  
   Prints live values every **50 ms**:
   ```
   1:1500  2:1496  3:988  4:1496  5:1
   ```

---

## 🪶 Example Output

```
1:1500	2:1498	3:1001	4:1496	5:0
1:1502	2:1503	3:1988	4:1494	5:1
```

---

## 🧮 Channel Scaling

| Stick Position   | Output Range |
| ---------------- | ------------ |
| Full Left / Down | ~1000        |
| Center           | ~1500        |
| Full Right / Up  | ~2000        |

---

## ⚖️ Tuning Parameters

| Parameter       | Description                      | Recommended |
| ---------------- | -------------------------------- | ----------- |
| `DEAD_BAND`     | Ignore small stick noise         | `0.03–0.08` |
| `EMA_ALPHA`     | Smoothing factor (0 = slow, 1 = raw) | `0.25` |
| `CH5_THRESH_US` | Threshold midpoint (µs)          | `1500` |
| `CH5_HYST_US`   | Hysteresis band                  | `100` |

---

## 🧩 Output Format for External Software

Easy to parse in Python, ROS, etc.:

```python
import serial, re
ser = serial.Serial('COM3', 115200)
pattern = re.compile(r"(\d):(\d+)")
while True:
    line = ser.readline().decode().strip()
    matches = dict(pattern.findall(line))
    print(matches)
```

---

## 🔧 Troubleshooting

| Issue                               | Cause                   | Fix                                  |
| ----------------------------------- | ----------------------- | ------------------------------------ |
| No data / all 1500                  | No PWM signal           | Verify receiver bound & powered      |
| Jittery readings                    | Noise / long wires      | Add 100 nF cap between 5 V–GND       |
| CH5 flickers 0↔1                    | Switch near threshold   | Increase `CH5_HYST_US`               |
| Upload error (`stk500 not in sync`) | Wrong port or board     | Re-select under Tools → Port         |

---

## 📜 License

Licensed under the **MIT License** — feel free to modify and share.

---

## ❤️ Credits

Created by [Your Name](https://github.com/<your-username>)  
For FlySky & Arduino RC signal interfacing  
Tested on **Arduino UNO R3 + FS-iA6B receiver**
