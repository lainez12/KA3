# Kub3.8i - Arduino 1 Firmware (v2.0)

This repository contains the highly-optimized, bare-metal firmware for **Arduino 1** of the Kub3.8i system. This MCU is responsible for controlling the **Cameras** (Left/Right X & Y Axes), the **Alignment Stages** (X, Y, Theta Axes), and communication with the **Focal/LED drivers** via SPI.

The firmware is specifically tailored for the **Arduino Due (SAM3X8E)**, heavily utilizing hardware interrupts, Direct Memory Access (DMA), and hardware Timer/Counters (TC) to achieve zero-blocking, deterministic real-time control.

## 🏗 Architecture Highlights

The v2.0 refactor shifts the paradigm from a traditional polling-based Arduino sketch to a true real-time embedded architecture:

* **Hardware-Driven Stepping (`DueStepper`)**: Step pulses are entirely offloaded to the SAM3X8E's hardware Timer/Counters (TC0-TC8). The CPU simply configures the target and goes to sleep; the silicon handles the exact microsecond pulsing.
* **Bare-Metal Quadrature Decoding**: Encoders bypass the slow Arduino `digitalRead()` API. Pin states are read directly from the Parallel Input/Output (PIO) Controller Data Status Registers (`REG_PIOx_PDSR`) in a single CPU cycle (~11.9ns), ensuring zero missed steps even at maximum RPM.
* **DMA UART Transmission (`SerialTXHandler`)**: Serial packets are pushed to an `AtomicQueue`. The SAM3X8E's Peripheral DMA Controller (PDC) automatically pulls from this queue and feeds the UART hardware lanes, completely freeing the CPU from blocking `Serial.write()` waits.
* **Non-Blocking RX State Machine**: Incoming serial commands are parsed asynchronously byte-by-byte. The main loop never halts waiting for a packet to finish arriving.
* **Fault-Tolerant Limit Switches**: All 10 limit switches (even the analog ones like A8, A9, A10) are attached natively to hardware interrupts (EXTI). A 100ms software fallback loop runs continuously to guarantee motor halts even if EMI glitches cause a missed hardware interrupt edge.

## 🗜 Subsystems & Hardware Mapping

### Motors & Encoders
| ID | ASCII | System | Function |
|---|---|---|---|
| `0` | `'1'` | Camera Left | X Axis |
| `1` | `'2'` | Camera Left | Y Axis |
| `2` | `'3'` | Camera Right | X Axis |
| `3` | `'4'` | Camera Right | Y Axis |
| `4` | `'5'` | Alignment Stage | X Axis |
| `5` | `'6'` | Alignment Stage | Y Axis |
| `6` | `'8'` | Alignment Stage | Theta Axis |

*(Note: ASCII index `'7'` is purposely omitted in the host protocol).*

### Focal / LED Control (SPI)
Communication with the Focal/LED driver boards is handled via hardware SPI at ~4MHz (Divider 21, Mode 3, MSB First).
* `LEFT_SLAVE`: Pin 4
* `RIGHT_SLAVE`: Pin 10

## 🔌 Serial Protocol

The firmware communicates via UART (115200 baud). The first byte dictates the packet length. The second byte is the command opcode.

### Action Commands
* `1` **Stop Motor**: `[Len][1][MotorAscii]`
* `2` **Start Motor**: `[Len][2][Dir][Res][Freq_H][Freq_L][Steps...]`
* `3` **Enable/Disable Motor (nEn)**: `[Len][3][MotorAscii][0/1]`
* `4` **Send SPI (Focal/LED)**: `[Len][4][L/R][F/L][Shutdown][Value...]`
* `5` **Disable SPI (Focal/LED)**: `[Len][5][L/R]`
* `7` **Unlock Alignment**: `[Len][7]`
* `8` **Lock Alignment**: `[Len][8]`
* `T` **Move to Target**: `[Len][T][MotorAscii][TargetPos]#`
* `R` **Reset Encoder**: `[Len][R][MotorAscii][PosBytes...]`

### Query Commands (`?`)
* `?` **Get Version**: Returns firmware version (e.g., `?: Arduino1 8i v2.0`)
* `?S` **Get All Limits**: Returns the state of all motor limit switches.
* `?C` **Get All Encoders**: Returns current 32-bit counts for all 7 encoders.

## 📂 Directory Structure

```text
├── include/
│   ├── definitions.h       # Global macros, NVIC priorities, versioning
│   ├── encoder.h           # Quadrature tracking and getters
│   ├── focalLed.h          # SPI controls for focus and illumination
│   ├── motors.h            # Unified Stepper and Limit Switch orchestration
│   ├── pins.h              # SAM3X8E physical pin mappings and PIO macros
│   └── utils.h             # Mapping functions (ASCII <-> Internal indices)
├── lib/
│   ├── AtomicQueue/        # Lock-free interrupt-safe ring buffer
│   ├── DueStepper/         # Hardware Timer/Counter stepper engine
│   ├── DueTimer/           # Bare-metal SAM3X TC wrapper
│   └── SerialTXHandler/    # DMA-powered UART transmission
└── src/
    ├── encoder.cpp         
    ├── focalLed.cpp        
    ├── main.cpp            # Async RX parser and primary scheduler
    ├── motors.cpp          # Limit mapping, ISR routing, and motor lifecycle
    └── utils.cpp           
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
* **Floating Pins**: The micro-stepping resolution logic actively sets certain pins to `INPUT` (High-Z) to achieve 1/4 and 1/32 stepping on the external drivers. Do not pull these pins HIGH/LOW externally.
* **Analog Interrupts**: Because the SAM3X8E treats analog pins as digital pins under the hood, pins `A8`, `A9`, and `A10` successfully trigger hardware interrupts on state change, completely avoiding `analogRead()` polling bottlenecks.
