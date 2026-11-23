# Two-Wheeled Self-Balancing Robot (STM32F103 Cascaded PID)

Firmware for a two-wheeled self-balancing robot written in C (STM32 HAL) on the **STM32F103C8T6 (Blue Pill)** microcontroller. The control system fuses accelerometer and gyroscope data from an MPU6050 using a **1D Kalman Filter** and stabilizes the chassis through a **Cascaded PID** architecture consisting of a high-speed inner angle loop (200 Hz) and an outer wheel speed loop (33.3 Hz).

## 1. Key Features

* **1D Kalman Filter Sensor Fusion:** Combines MPU6050 accelerometer and gyroscope measurements over `I2C2`  with hardware DLPF ($\approx 94\text{ Hz}$) and an automatic I2C bus recovery routine (`MPU6050_RecoverBus`) to handle bus lockups.

* **Cascaded PID Control Architecture:**

  * **Inner Angle Loop (**$5\text{ ms}$ **/** $200\text{ Hz}$**):** PD controller that feeds the unbiased angular rate directly from the Kalman filter into the derivative term (`PID_compute_WithRate`) to prevent derivative kick and minimize phase lag.

  * **Outer Speed Loop (**$30\text{ ms}$ **/** $33.3\text{ Hz}$**):** PI controller operating on low-pass filtered quadrature encoder feedback to prevent motor speed runaway, damp translational drift, and automatically compensate for minor mechanical center-of-gravity offsets.

* **Slow-Decay PWM & Deadband Compensation:** Drives an L298N H-bridge at $1\text{ kHz}$ (`TIM1`) in Slow Decay mode with static friction compensation (`MOTOR_DEADBAND`).

* **Safety & Autonomous State Machine:**

  * Automatic $200$-sample gyroscope bias calibration on startup.

  * **Auto-Arming:** Automatically engages balancing once the robot is held upright within $\pm 15^\circ$ of `ANGLE_OFFSET` for $0.5\text{ s}$.

  * **Fall Protection:** Immediately cuts motor PWM and resets PID integrators if the tilt angle exceeds $\pm 45^\circ$.

  * **Independent Watchdog (`IWDG`):** Monitors main-loop execution with a $3.0\text{ s}$ timeout.

## 2. Hardware & Pinout

### Bill of Materials (BOM)

* **Microcontroller:** STM32F103C8T6 (`72 MHz` system clock via `8 MHz` external HSE crystal)

* **IMU Sensor:** MPU6050 (I2C address `0x68`)

* **Motors:** 2x GA25-370 DC Gear Motors ($130\text{ RPM}$) with integrated quadrature encoders

* **Motor Driver:** L298N Dual H-Bridge Module

* **Power Supply:** 3S LiPo Battery ($11.1\text{ V} - 12.6\text{ V}$) + XL4015 DC-DC Buck Converter

### STM32F103C8T6 Pin Mapping

| Peripheral | STM32 Pin | Function | Notes | 
| ----- | ----- | ----- | ----- | 
| **I2C2** | `PB10` / `PB11` | `SCL` / `SDA` (MPU6050) | Fast Mode $400\text{ kHz}$ | 
| **TIM1 CH1** | `PA8` | Motor 1 PWM (`IN1`) | $1\text{ kHz}$ PWM, `ARR = 2879` | 
| **TIM1 CH2** | `PA9` | Motor 2 PWM (`IN3`) | $1\text{ kHz}$ PWM, `ARR = 2879` | 
| **GPIO Out** | `PB15` / `PA9` | Motor 1 & 2 Direction | Configured for Slow Decay mode | 
| **TIM3 CH1/2** | `PA6` / `PA7` | Motor 1 Encoder | Encoder Mode `TI12` ($4\times$ quadrature) | 
| **TIM4 CH1/2** | `PB6` / `PB7` | Motor 2 Encoder | Encoder Mode `TI12` ($4\times$ quadrature) | 
| **USART2** | `PA2` / `PA3` | `TX` / `RX` (`DMA1_Channel6`) | `115200` baud (reserved for remote control) | 
| **GPIO Out** | `PC13` | Fault Status LED | Blinks continuously if `Error_Handler()` is triggered | 

<p align="center">
  <img width="543" height="467" alt="Pin Mapping" src="https://github.com/user-attachments/assets/9d8ed436-9190-43c5-acb6-06d78272e5d1" />
</p>

## 3. Project Structure

```
├── Core/
│   ├── Inc/
│   │   ├── main.h          # GPIO definitions and system function prototypes
│   │   ├── MPU6050.h       # IMU data structures and 1D Kalman filter definitions
│   │   ├── Motor.h         # Motor driver abstraction (Slow/Fast Decay & deadband)
│   │   └── PID.h           # PID controller struct and anti-windup definitions
│   └── Src/
│       ├── main.c          # Main control loop, state machine, and cascaded PID setup
│       ├── MPU6050.c       # I2C communication, calibration, and Kalman filter logic
│       ├── Motor.c         # PWM generation, direction control, and deadband mapping
│       └── PID.c           # Standard PID and direct-rate PID computation routines

```

## 4. Control Architecture & Tuned Parameters

`TIM2` generates a deterministic $5\text{ ms}$ ($200\text{ Hz}$) timebase tick to schedule both control loops:

```
[Target Speed = 0] ---> (PID_Speed - 30ms) ---> [target_angle]
                                                      |
                                                      v
[MPU6050 Pitch/Rate] -------------------------> (PID_Angle - 5ms) ---> [PWM Output] ---> [Motors]

```

### Current Tuned Parameters (`main.c`)

* **Mechanical Balance Offset:** `ANGLE_OFFSET = 6.0f` ($6.0^\circ$)

* **Motor Deadband Compensation:** `MOTOR_DEADBAND = 500` (out of `MAX_PWM = 2879`)

* **Inner Angle Loop (`PID_Angle`):** `Kp = 340.0f`, `Ki = 0.0f`, `Kd = 12.0f`

* **Outer Speed Loop (`PID_Speed`):** `Kp = 0.09f`, `Ki = 0.01f`, `Kd = 0.0f`, output clamped to $\pm 20^\circ$, low-pass filter `SPEED_LFB_ALPHA = 0.6f`

## 5. Build, Flash & Calibration Guide

### Building and Flashing

1. Open the project in **STM32CubeIDE** or **VS Code** (with STM32 / Cortex-Debug extensions).

2. Connect an **ST-Link V2** programmer to the Blue Pill SWD header (`SWDIO`, `SWCLK`, `GND`, `3.3V`).

3. Build and flash the firmware. **Note:** Always disconnect the ST-Link cable from the MCU before powering the robot on via battery to prevent parasitic voltage through the SWD pins from stalling cold-boot initialization.

### Hardware Calibration & Tuning Workflow

1. **Calibrate `ANGLE_OFFSET`:** Whenever battery placement or IMU mounting angle changes, find the static equilibrium `MyMPU.Pitch` angle and update `#define ANGLE_OFFSET` in `main.c`.

2. **Verify Motor & Encoder Polarity:**

   * Tilting the chassis forward must drive both wheels forward (`MOTOR1_REVERSED`, `MOTOR2_REVERSED`).

   * Rolling the robot forward manually must produce `delta_en1` and `delta_en2` values with the same sign as `MyMPU.Pitch - ANGLE_OFFSET` when leaning forward.

3. **PID Tuning Guidelines:**

   * Increase `Angle_Kp` if the robot responds too sluggishly to initial tilt; increase `Angle_Kd` to damp rapid oscillations around the balance point.

   * Increase `Speed_Kp` to brake harder against external pushes; adjust `Speed_Ki` gently to pull the robot back toward its starting position without inducing slow oscillations.

## 6. Future Improvements

* Implement a serial command parser over `USART2` (using a Bluetooth HC-05/HC-06 or ESP32 module) for wireless forward, backward, and turning control.

* Add differential wheel velocity mixing (`turn_offset`) to allow smooth yaw rotation while balancing.

## 7. Acknowledgments

* The 1D Kalman filter implementation in `MPU6050.c` / `MPU6050.h` is adapted from the open-source [TKJElectronics KalmanFilter](https://github.com/TKJElectronics/KalmanFilter) library.
  ## Video demo
  

https://github.com/user-attachments/assets/a1f2747a-396a-4394-bc82-e416a61dd5ad







