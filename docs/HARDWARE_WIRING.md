# Hardware Wiring Guide

## Overview

Diagram koneksi lengkap untuk Robot Otonom dengan arsitektur Master-Slave.

```
┌─────────────────────────────────────────────────────────────┐
│                    SYSTEM ARCHITECTURE                       │
│                                                              │
│  ┌──────────────┐         USB          ┌──────────────┐    │
│  │ Raspberry Pi │◄─────────────────────►│    ESP32     │    │
│  │      5       │      Serial           │     S3       │    │
│  │   (Master)   │    Communication      │   (Slave)    │    │
│  └──────────────┘                       └──────────────┘    │
│         │                                       │            │
│         │ CSI                             PWM + Signal       │
│         ▼                                       │            │
│  ┌──────────────┐                       ┌──────┴──────┐    │
│  │   Pi Camera  │                       │   Motors &  │    │
│  │   Module     │                       │   Servos    │    │
│  └──────────────┘                       └─────────────┘    │
│         │ USB                                                │
│         ▼                                                    │
│  ┌──────────────┐                                           │
│  │   YDLIDAR    │                                           │
│  │   YDS2MD-1   │                                           │
│  └──────────────┘                                           │
└─────────────────────────────────────────────────────────────┘
```

---

## 1. Raspberry Pi 5 Connections

### 1.1 Power

```
Power Input: USB-C 5V/3A
- Use official Raspberry Pi power supply
- Minimum 3A untuk stable operation dengan peripherals
```

### 1.2 Raspberry Pi Camera Module

```
Connection: CSI Port (Camera Serial Interface)
- Kabel flat ribbon cable
- Blue side menghadap ke camera module
- Connector pada Raspi: flip clip up, insert cable, flip down

Camera Mounting:
- Position: Front-center robot
- Angle: 25° downward from horizontal
- Height: 15-20cm from ground
```

### 1.3 USB Connections

```
USB Port 1: ESP32 (Serial Communication)
- Cable: USB Type-A to Micro-USB/USB-C
- Device: /dev/ttyUSB0 atau /dev/ttyACM0

USB Port 2: YDLIDAR YDS2MD-1
- Cable: USB Type-A (dari LiDAR)
- Device: /dev/ttyUSB1 atau /dev/ttyACM1

USB Port 3-4: Available (keyboard, mouse untuk setup)
```

### 1.4 GPIO (Optional - not used in current design)

```
Raspberry Pi GPIO pins tidak digunakan langsung.
Semua control via USB Serial ke ESP32.
```

---

## 2. ESP32-S3 Connections

### 2.1 Power

```
Power Source: USB 5V dari Raspberry Pi ATAU VIN pin dari 5V regulator

Option 1: USB Powered (Recommended untuk testing)
- Power via USB cable dari Raspi
- Simplest setup

Option 2: External 5V (Recommended untuk production)
- VIN pin ← 5V dari buck converter (12V battery → 5V)
- GND pin ← Common ground
- Advantage: Isolated dari Raspi, lebih stable untuk motor noise
```

### 2.2 Motor Driver BTS7960 Connections

**Driver 1 - Left Motors (2 wheels parallel)**

```
ESP32 Pin → BTS7960
GPIO 17   → RPWM (Right PWM - Forward)
GPIO 18   → LPWM (Left PWM - Backward)

BTS7960 → Power
R_EN      → 5V
L_EN      → 5V
VCC       → 12V (from battery)
GND       → Common Ground

BTS7960 → Motors
M+        → Left Motor Positive
M-        → Left Motor Negative
```

**Driver 2 - Right Motors (2 wheels parallel)**

```
ESP32 Pin → BTS7960
GPIO 6    → RPWM (Right PWM - Forward)
GPIO 7    → LPWM (Left PWM - Backward)

BTS7960 → Power
R_EN      → 5V
L_EN      → 5V
VCC       → 12V (from battery)
GND       → Common Ground

BTS7960 → Motors
M+        → Right Motor Positive
M-        → Right Motor Negative
```

### 2.3 Servo Connections

**Servo 1 - Lift (MG90)**

```
ESP32 Pin → Servo
GPIO 4    → Signal (Orange/Yellow wire)
5V        → VCC (Red wire)
GND       → GND (Brown/Black wire)
```

**Servo 2 - Gripper (MG90)**

```
ESP32 Pin → Servo
GPIO 5    → Signal (Orange/Yellow wire)
5V        → VCC (Red wire)
GND       → GND (Brown/Black wire)
```

**Notes:**
- MG90 servos dapat powered dari ESP32 5V jika hanya 2 servos
- Untuk lebih banyak servos, gunakan external 5V supply dengan common ground

---

## 3. Power Distribution

### 3.1 Power Architecture

```
                    ┌─────────────────┐
                    │  12V LiPo/Li-ion│
                    │    Battery      │
                    │   (3S, 11.1V)   │
                    └────────┬────────┘
                             │
                ┌────────────┴───────────┐
                │                        │
                ▼                        ▼
        ┌───────────────┐      ┌──────────────┐
        │ Buck Converter│      │  BTS7960 x2  │
        │   12V → 5V    │      │     VCC      │
        │   5A Rating   │      └──────────────┘
        └───────┬───────┘               │
                │                       │
                ▼                       ▼
        ┌──────────────┐        ┌─────────────┐
        │ Raspberry Pi │        │   Motors    │
        │   USB-C 5V   │        │  4x DC 12V  │
        └──────────────┘        └─────────────┘
                │
                ├──────► ESP32 (via USB)
                ├──────► YDLIDAR (via USB)
                └──────► Camera (via CSI)
```

### 3.2 Power Requirements

```
Component          Voltage    Current    Notes
─────────────────────────────────────────────────────────────
Raspberry Pi 5     5V         3A         Peak with peripherals
ESP32-S3           5V         0.5A       Via USB or VIN
YDLIDAR YDS2MD-1   5V         0.5A       Via USB
Camera Module      3.3V       0.3A       Powered by Raspi
Servos (2x MG90)   5V         1A         Peak (both moving)
Motors (4x DC)     12V        2-5A       Under load

Total 5V needed:   ~5A
Total 12V needed:  2-5A (motors)
```

### 3.3 Recommended Power Setup

**Option 1: Dual Power (Recommended)**

```
1. 12V Battery → Buck Converter → 5V/5A for Raspi + ESP32
2. 12V Battery → BTS7960 → Motors (isolated motor power)

Advantages:
- Motor noise tidak affect Raspi
- Stable 5V untuk computing
- Safety: Raspi bisa shutdown motors via serial command
```

**Option 2: Single Battery + Powerbank**

```
1. Powerbank 5V/3A → Raspberry Pi (USB-C)
2. 12V Battery → BTS7960 → Motors
3. ESP32 powered dari Raspi USB

Advantages:
- Simpler wiring
- Raspi power very stable (powerbank regulation)
```

---

## 4. Grounding

### 4.1 Common Ground (CRITICAL!)

```
ALL grounds must be connected together:

Raspberry Pi GND
      │
      ├──── ESP32 GND
      │
      ├──── BTS7960 Driver 1 GND
      │
      ├──── BTS7960 Driver 2 GND
      │
      ├──── 12V Battery GND
      │
      └──── Buck Converter GND

⚠️  WARNING: Tanpa common ground, serial communication akan gagal
           dan motor control tidak stable!
```

### 4.2 Ground Loop Prevention

```
- Use star grounding (all grounds ke single point)
- Keep motor power wires away dari signal wires
- Twist motor power cables untuk reduce EMI
- Add 100nF ceramic capacitor across motor terminals (optional)
```

---

## 5. Complete Pin Reference

### ESP32-S3 Pinout

```
┌─────────────────────────────────┐
│          ESP32-S3               │
│                                 │
│  GPIO 17 ──► RPWM Left Motor   │
│  GPIO 18 ──► LPWM Left Motor   │
│  GPIO 6  ──► RPWM Right Motor  │
│  GPIO 7  ──► LPWM Right Motor  │
│  GPIO 4  ──► Servo Lift Signal │
│  GPIO 5  ──► Servo Grip Signal │
│                                 │
│  5V      ──► Servos VCC         │
│  GND     ──► Common Ground      │
│  VIN     ──► 5V Input (optional)│
│                                 │
│  USB     ◄─► Raspberry Pi       │
└─────────────────────────────────┘
```

### BTS7960 Pinout (per driver)

```
┌─────────────────────────────────┐
│         BTS7960 Driver          │
│                                 │
│  RPWM ◄── ESP32 (17 or 6)      │
│  LPWM ◄── ESP32 (18 or 7)      │
│  R_EN ◄── 5V                    │
│  L_EN ◄── 5V                    │
│  VCC  ◄── 12V Battery           │
│  GND  ◄── Common Ground         │
│  M+   ──► Motor Positive        │
│  M-   ──► Motor Negative        │
└─────────────────────────────────┘
```

---

## 6. Wiring Best Practices

### 6.1 Cable Management

```
1. Group cables by function:
   - Power cables (12V, 5V)
   - Signal cables (PWM, Servo)
   - Communication cables (USB, Serial)

2. Separation:
   - Keep motor power cables separated dari signal cables
   - Minimum 5cm separation untuk reduce noise

3. Strain Relief:
   - Secure cables dengan zip ties atau clips
   - Avoid sharp bends pada camera ribbon cable
   - Leave slack untuk robot movement
```

### 6.2 Connections Checklist

```
☐ Raspberry Pi Camera ribbon cable correct orientation
☐ USB cables firmly connected (tidak loose)
☐ All grounds connected together
☐ BTS7960 R_EN and L_EN connected to 5V
☐ Servo signal wires connected ke correct GPIO pins
☐ Battery voltage checked (11.1V nominal untuk 3S LiPo)
☐ Buck converter output verified (5V ±0.2V)
☐ No exposed wires atau short circuit risks
☐ All connectors secured (heat shrink atau electrical tape)
```

---

## 7. Testing Procedure

### 7.1 Power-On Sequence

```
1. Connect battery (main power OFF)
2. Verify voltage dengan multimeter:
   - Battery: ~11-12V
   - Buck converter output: 5.0V
3. Power ON main switch
4. Check Raspberry Pi boots (green LED activity)
5. Check ESP32 powered (LED on board)
6. Verify USB devices detected di Raspi:
   ls /dev/ttyUSB* /dev/ttyACM*
```

### 7.2 Component Testing

```
# Test 1: Camera
libcamera-hello

# Test 2: Serial ports
ls -l /dev/ttyUSB* /dev/ttyACM*

# Test 3: ESP32 communication
python3 src/serial_comm.py

# Test 4: LiDAR
python3 src/lidar_nav.py --simulator

# Test 5: Full system
python3 tools/test_modules.py
```

---

## 8. Safety Considerations

### 8.1 Electrical Safety

```
⚠️  Always disconnect battery saat wiring
⚠️  Check polarity sebelum connect power
⚠️  Use fuses pada battery output (recommended: 10A fuse)
⚠️  Insulate all exposed connections
⚠️  Keep water away dari electronics
```

### 8.2 Mechanical Safety

```
⚠️  Ensure robot stable (center of gravity low)
⚠️  Secure all components (no loose parts)
⚠️  Test di safe area (no stairs, obstacles)
⚠️  Always have emergency stop ready (Ctrl+C atau kill switch)
```

---

## 9. Troubleshooting

### Motor tidak bergerak

```
Check:
1. BTS7960 VCC connected ke 12V battery
2. R_EN and L_EN connected ke 5V
3. GND common ground connected
4. PWM signals dari ESP32 working (test dengan LED)
5. Motor wires tidak short circuit
```

### Servo tidak bergerak

```
Check:
1. Servo VCC connected ke 5V
2. Signal wire connected ke correct GPIO (4, 5)
3. GND connected
4. Test servo dengan manual command di Serial Monitor
```

### Communication error

```
Check:
1. USB cable firmly connected
2. Correct serial port di config.py
3. Baud rate match (115200)
4. User in dialout group: groups | grep dialout
5. Common ground connected
```

---

## 10. Wiring Diagram (ASCII Art)

```
                    RASPBERRY PI 5
                    ┌─────────────┐
                    │  ┌───────┐  │
    Camera ────CSI──┤  │ CPU   │  │
                    │  └───────┘  │
                    │             │
    LiDAR ─────USB──┤  USB Ports  │
                    │      │      │
    ESP32 ─────USB──┤      │      │
                    └──────┼──────┘
                           │ 5V/3A
                           │
                    ┌──────▼──────┐
                    │  12V → 5V   │
                    │Buck Converter│
                    └──────▲──────┘
                           │
                    ┌──────┴──────┐
                    │ 12V Battery │
                    │   (3S LiPo) │
                    └──┬───────┬──┘
                       │       │
            ┌──────────┘       └──────────┐
            │                              │
     ┌──────▼──────┐              ┌───────▼──────┐
     │  BTS7960-1  │              │  BTS7960-2   │
     │   (Left)    │              │   (Right)    │
     └──┬───────┬──┘              └──┬───────┬───┘
        │       │                    │       │
     ┌──▼──┐ ┌─▼───┐             ┌──▼──┐ ┌─▼───┐
     │Motor│ │Motor│             │Motor│ │Motor│
     │ LF  │ │ LB  │             │ RF  │ │ RB  │
     └─────┘ └─────┘             └─────┘ └─────┘

            ESP32-S3
            ┌────────────┐
    PWM ────┤ GPIO 17,18 │──── To BTS7960-1
            │ GPIO 6,7   │──── To BTS7960-2
    Servo ──┤ GPIO 4,5   │──── To Servos
            │            │
            │  USB Port  │◄─── From Raspi
            └────────────┘
                  │
             ┌────┴────┐
          ┌──▼──┐   ┌─▼───┐
          │Servo│   │Servo│
          │Lift │   │Grip │
          └─────┘   └─────┘
```

---

## Dokumentasi Lengkap

- Installation: `INSTALLATION.md`
- Color Calibration: `COLOR_CALIBRATION_GUIDE.md`
- State Machine: `STATE_MACHINE_FLOW.md`
- Troubleshooting: `TROUBLESHOOTING.md`
