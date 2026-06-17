# Kub3.8i - Arduino 3 Firmware (v2.0)

This repository contains the highly-optimized, bare-metal firmware for **Arduino 3** of the Kub3.8i system. This MCU is responsible for controlling the **Autolevel** (Z-Axis leveling) and **Drawer** (Mask and Wafer conveyors) subsystems, as well as high-frequency force sensor sampling.

The firmware is specifically tailored for the **Arduino Due (SAM3X8E)**, heavily utilizing hardware interrupts, Direct Memory Access (DMA), and hardware Timer/Counters (TC) to achieve zero-blocking, deterministic real-time control.

## 🏗 Architecture Highlights

The v2.0 refactor shifts the paradigm from a traditional polling-based Arduino sketch to a true real-time embedded architecture:

* **Hardware-Driven Stepping (`DueStepper`)**: Step pulses are entirely offloaded to the SAM3X8E's hardware Timer/Counters (TC0-TC8). The CPU simply configures the target and goes to sleep; the silicon handles the exact microsecond pulsing.
* **Bare-Metal Quadrature Decoding**: Encoders bypass the slow Arduino `digitalRead()` API. Pin states are read directly from the Parallel Input/Output (PIO) Controller Data Status Registers (`REG_PIOx_PDSR`) in a single CPU cycle (~11.9ns), ensuring zero missed steps even at maximum RPM.
* **DMA UART Transmission (`SerialTXHandler`)**: Serial packets are pushed to an `AtomicQueue`. The SAM3X8E's Peripheral DMA Controller (PDC) automatically pulls from this queue and feeds the UART hardware lanes, completely freeing the CPU from blocking `Serial.write()` waits.
* **Non-Blocking RX State Machine**: Incoming serial commands are parsed asynchronously byte-by-byte. The main loop never halts waiting for a packet to finish arriving.
* **Fault-Tolerant Limit Switches**: All 16 limit switches and safety zones are attached to hardware interrupts (EXTI). A 100ms software fallback loop runs continuously to guarantee motor halts even if EMI glitches cause a missed hardware interrupt edge.
* **Fixed-Point EMA DSP**: Force sensors are sampled at exactly 500Hz via a hardware timer and processed through an Exponential Moving Average (EMA) filter using fixed-point bitwise math to avoid Floating-Point Unit (FPU) bottlenecks.

## 🗜 Subsystems & Hardware Mapping

### Motors & Encoders
| ID | System | Function | Hardware Timer |
|---|---|---|---|
| `0` | Autolevel | Left Z Axis (`MALG`) | TC3 |
| `1` | Autolevel | Right Z Axis (`MALD`) | TC4 |
| `2` | Autolevel | Back Z Axis (`MALA`) | TC5 |
| `3` | Drawer | Mask Conveyor (`MCM`) | TC1 |
| `4` | Drawer | Wafer Conveyor (`MCW`) | TC2 |

### Limit Switches & Security Zones
The unified limit system maps 16 physical switches to motors. 
* **0-5**: Autolevel Z Extreme Limits
* **6-9**: Mask Drawer Limits
* **10-12**: Wafer Drawer Limits
* **13-15**: Elevator Security Zones (Z1, Z2, Wafer ON) - *Information only, no hard stops.*

## 🔌 Serial Protocol

The firmware communicates via UART (115200 baud). The first byte dictates the packet length. The second byte is the command opcode.

### Action Commands
* `1` **Stop Motor**: `[Len][1][MotorIdx]`
* `2` **Start Motor**: `[Len][2][Dir][Res][Freq_H][Freq_L][Steps...]`
* `T` **Move to Target**: `[Len][T][MotorIdx][TargetPos]#` (Closed-loop encoder targeting)
* `R` **Reset Encoder**: `[Len][R][MotorIdx][PosBytes...]`
* `F` **Toggle Force Sensor**: `[Len][F][SensorIdx][0/1]`

### Query Commands (`?`)
* `?` **Get Version**: Returns firmware version (e.g., `?: Arduino3 8p v2.0`)
* `?S` **Get All Limits**: Returns the state of all standard motor limit switches.
* `?Z` **Get All Zones**: Returns the state of the 3 elevator security zones.
* `?C` **Get All Encoders**: Returns current 32-bit counts for all 5 encoders.
* `?F` **Get Force Status**: Returns the enabled/disabled state of the force sensors.

*Note: Motors map to ASCII indices `'1'` to `'5'` in the protocol for legacy compatibility.*

## 📂 Directory Structure

```text
├── include/
│   ├── definitions.h       # Global macros, NVIC priorities, versioning
│   ├── encoder.h           # Quadrature tracking and getters
│   ├── forceSensors.h      # 500Hz ADC sampling and filtering
│   ├── motors.h            # Unified Stepper and Limit Switch orchestration
│   └── pins.h              # SAM3X8E physical pin mappings and PIO macros
├── lib/
│   ├── AtomicQueue/        # Lock-free interrupt-safe ring buffer
│   ├── DueStepper/         # Hardware Timer/Counter stepper engine
│   ├── DueTimer/           # Bare-metal SAM3X TC wrapper
│   ├── EMAFilter/          # Fixed-point Exponential Moving Average DSP
│   └── SerialTXHandler/    # DMA-powered UART transmission
└── src/
    ├── encoder.cpp         
    ├── forceSensors.cpp    
    ├── main.cpp            # Async RX parser and primary scheduler
    └── motors.cpp          # Limit mapping, ISR routing, and motor lifecycle
```

## 🛠 Building and Uploading

This project is designed to be compiled using **PlatformIO** (or the Arduino IDE).
Target Environment: `dueUSB` (Arduino Due Native USB / Programming Port).

### Prerequisites
* Ensure the **Arduino SAM Boards (32-bits ARM Cortex-M3)** core is installed.

### Build (PlatformIO)
```bash
pio run -e due
pio run -t upload -e due
```

## ⚠️ Embedded Engineering Notes
* **Atomicity**: Do not bypass `Com::send()`. Using `Serial.write()` directly in the main loop will defeat the DMA architecture and cause stepper timing jitter.
* **Variable Shadowing**: Use `-Wshadow` during compilation. The main loop variables must not clash with ISR-level context variables.
* **Floating Pins**: The micro-stepping resolution logic actively sets certain pins to `INPUT` (High-Z) to achieve 1/4 and 1/32 stepping on the external drivers. Do not pull these pins HIGH/LOW externally.
