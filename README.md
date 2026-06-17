# KUB3Arduinos (*Kub3.8i Multi-Node Firmware Monorepo*)

Welcome to the central firmware repository for the **Kub3.8i** mask alignment and photolithography system. This monorepo consolidates the distributed real-time control software across three distinct micro-controller nodes, all built on the high-performance **Arduino Due (SAM3X8E, 84MHz ARM Cortex-M3)** platform [DueTimer.hpp].

By standardizing on a highly optimized, non-blocking, and interrupt-driven core, this firmware architecture ensures sub-microsecond determinism, zero missed encoder steps, and precise coordinate stage positioning.

---

## 🏗 System Architecture & Distributed Topography

The Kub3.8i machine splits its mechanical and logical responsibilities across three dedicated physical microcontrollers:

```
                  ┌─────────────────────────────────────────┐
                  │              Host Control               │
                  │             (PC Software)               │
                  └──────┬────────────┬────────────┬────────┘
                         │ UART       │ UART       │ UART
                         ▼            ▼            ▼
                  ┌────────────┐┌────────────┐┌────────────┐
                  │ Arduino 1  ││ Arduino 2  ││ Arduino 3  │
                  │ Opt/Align  ││ Proc/Env   ││ Mech/Load  │
                  └────────────┘└────────────┘└────────────┘
```

### 1. Arduino 1: Optical Alignment & Stage Tracking
Responsible for sub-micron stage coordinate adjustments and optical feedback systems.
*   **Coordinate Stage (X, Y, Theta)**: High-speed closed-loop stepper positioning with real-time encoder feedback [motors.cpp, pins.h].
*   **Optics Manipulation**: Micro-stepping control of left and right cameras [motors.cpp, pins.h].
*   **SPI Focus & LED Engines**: Low-latency, bare-metal SPI configuration communicating with focal plane driver boards [focalLed.cpp].

### 2. Arduino 2: Environmental, Vacuum, & Insolation Control
Responsible for process safety, substrate retention, exposure timing, and climate control.
*   **Exposure System (Insolation)**: Precision exposure gating, shutter timings, and source control [insolation.cpp, stops.cpp].
*   **Retention Subsystems (Vacuum)**: Direct control of solenoids and analog feedback monitoring for substrate flat retention [vacuum.cpp].
*   **Thermal Control**: Precision temperature tracking and environmental regulation [temperature.cpp].

### 3. Arduino 3: Substrate Handling & Mechanical Autoleveling
Responsible for stage leveling, mechanical substrate load/unload cycles, and load cell DSP [motors.cpp, forceSensors.cpp].
*   **Auto-Leveling System (Z Left, Z Right, Z Back)**: Closed-loop multi-axis planar leveling [motors.cpp].
*   **Drawer Mechanization (Mask & Wafer Conveyors)**: Cleanroom drawer entry and exit sequences [motors.cpp].
*   **Load Cell Monitoring (Force Sensors)**: Deterministic 500Hz analog sampling with adaptive digital filtering [forceSensors.cpp].

---

## 🛠 Monorepo Shared Core Libraries

To enforce strict timing guarantees, code reuse, and identical communication semantics, all nodes draw from a set of customized, bare-metal wrapper libraries located in their respective `/lib` directories:

### 🚀 `HWTimer` (DueTimer & DueStepper)
A zero-CPU-overhead motor-stepping and timer abstraction.
*   **`DueTimer`**: Directly configures the SAM3X8E's 9 hardware Timer/Counter (TC) peripheral channels [DueTimer.hpp]. It manages the PMC (Power Management Controller), clears registers, and sets NVIC priority levels [DueTimer.cpp].
*   **`DueStepper`**: Generates step pulses entirely in silicon by hooking into hardware timer compare-match interrupts [DueStepper.cpp]. Supports both open-loop step target counting and closed-loop encoder boundary checks [DueStepper.cpp].

### ⚡ `AtomicQueue`
A thread-safe, lock-free, single-producer single-consumer ring buffer template [AtomicQueue.hpp]. 
*   Utilizes assembly-level instruction masking (`__disable_irq()`, `__set_PRIMASK()`) to ensure complete data integrity between asynchronous execution contexts (Main Loop $\leftrightarrow$ hardware ISRs) [AtomicQueue.hpp].
*   Uses a fast power-of-2 bitwise mask `& (SIZE - 1)` for 1-cycle head/tail pointer wrapping [AtomicQueue.hpp].

### 📡 `SerialTXHandler` (Com)
A non-blocking, Direct Memory Access (DMA)-driven UART transmission engine [SerialTXHandler.cpp, SerialTXHandler.h].
*   Instead of waiting for the serial buffer to clear (which blocks stepper timers in standard Arduino environments), payloads are immediately pushed to `AtomicQueue` [SerialTXHandler.cpp, SerialTXHandler.h].
*   Automatically configures and feeds the SAM3X8E's **Peripheral DMA Controller (PDC)** registers (`UART_TCR`, `UART_TNCR`), entirely offloading memory-to-peripheral transfers from the CPU core [SerialTXHandler.cpp].

### 📈 `EMAFilter`
A high-performance Exponential Moving Average (EMA) filter tailored for real-time DSP [EMAFilter.cpp].
*   Bypasses Floating-Point Unit (FPU) overhead entirely by leveraging fixed-point math and bit-shifting (`SCALE_FACTOR_SHIFT 12`) [EMAFilter.cpp, EMAFilter.hpp].
*   Implements adaptive alpha smoothing: scales the filter coefficient dynamically based on the input signal's rate of change (Alpha is automatically boosted during transient changes to prevent lag, then dropped during steady states to maximize noise rejection) [EMAFilter.cpp].

---

## ⚡ Real-Time Optimization Blueprint

The repository employs several low-level mechanisms to guarantee hard real-time behavior:

### 1. Bare-Metal Register Reading (EXTI)
Standard Arduino `digitalRead()` is too slow and non-deterministic for high-frequency encoder pulses. The quadrature decoding engines in this repo read raw port pins via direct register memory access in a single CPU cycle (~12 nanoseconds):
```cpp
// 1-cycle assembly read bypassing Arduino mapping overhead
#define MALG_codA_VAL ((REG_PIOD_PDSR >> 3) & 0x1) 
#define MALG_codB_VAL ((REG_PIOD_PDSR >> 1) & 0x1)
```

### 2. Double-Gated Limit Switch (Limit) Protection
To protect high-precision translation stages from mechanical collisions, limits utilize a highly reliable dual-layer scheme:
1.  **Hardware Level (Interrupt-driven)**: Physical limit switches are wired to EXTI pins. When triggered, the ISR immediately forces a timer halt, bypassing any software loops:
    $$\text{Limit Active} \land \text{Motor Running} \land \text{Movement Direction matches Collision Path} \implies \text{TC Stop}$$
2.  **Safety Net (Software fallback)**: A periodic software loop runs asynchronously at a throttled interval (50ms - 100ms) to read DC states, protecting against any missed edge triggers or electrical noise transients [motors.cpp].

---

## 📂 Project Organization

```text
.
├── LICENSE
├── README.md               # Root Monorepo README (You are here)
│
├── arduino1/               # OPTICAL ALIGNMENT & MOTION STAGE NODE
│   ├── include/            # Focal SPI drivers, Encoder registries, Stage Motors
│   ├── lib/                # AtomicQueue, HWTimer, SerialTXHandler
│   ├── src/                # Encoders, Stage Coordination, Focal Spi, Main
│   └── platformio.ini      # Environment config for Arduino 1
│
├── arduino2/               # ENVIRONMENT, EXPOSURE, & RETENTION NODE
│   ├── include/            # Insolation timings, Vacuum telemetry, Temp sensors
│   ├── lib/                # AtomicQueue, HWTimer, EMAFilter
│   └── src/                # Exposure drivers, Temperature PID, Vacuum/Solenoid state
│
└── arduino3/               # MECHANICAL LEVELING & DECK LOADER NODE
    ├── include/            # Force sensor routing, leveling motors, limits map
    ├── lib/                # AtomicQueue, HWTimer, EMAFilter, SerialTXHandler
    └── src/                # Leveling steppers, Drawer loaders, Load-cell DSP
```

---

## 🛠 Compiling & Deployment

Each node is managed as an independent PlatformIO project within this monorepo. This allows you to compile, run tests, and upload code targeting the specific node.

### Core System Requirements
*   **IDE/Framework**: [PlatformIO](https://platformio.org/) (highly recommended) or Arduino IDE.
*   **Toolchain**: ARM GCC (Targeting Cortex-M3 / SAM3X8E).
*   **Skins**: Ensure the correct native/programming USB ports are selected for each node in their respective `platformio.ini` profiles.

### Compiling and Uploading via CLI

To compile and upload a specific node, navigate to its directory and run PlatformIO CLI commands:

```bash
# Example: Deploying the Optical Stage Node (Arduino 1)
cd arduino1
pio run --target upload

# Example: Deploying the Mechanical Loader Node (Arduino 3)
cd ../arduino3
pio run --target upload
```

---

## 🧑‍💻 Developer Guidelines
1.  **Do Not Block**: Never use blocking delays (`delay()`, `delayMicroseconds()`) or blocking UART calls. All time-sensitive events must use non-blocking timers (`millis()`, `micros()`) or `DueTimer` instances [DueTimer.hpp].
2.  **Atomicity**: Always protect shared variables modified inside ISRs and read in the main loop by wrapping them in `__disable_irq()` / `__set_PRIMASK()` blocks using the `AtomicQueue` standard [AtomicQueue.hpp].
3.  **Direct Port manipulation**: When adding new encoder lines or fast status flags, use the custom `REG_PIOx_PDSR` macro pattern to prevent execution delays [pins.h].
***
