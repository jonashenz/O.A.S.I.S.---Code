<p align="center">
  <img width="431" height="89" alt="image" src="https://github.com/user-attachments/assets/29da29b6-6176-4c5c-a955-e3538515780f" />
</p>

# O.A.S.I.S. - Obstacle Avoidance Sensory Interface System

O.A.S.I.S. is a wearable haptic feedback system integrated into a belt architecture designed to assist visually impaired individuals in spatial navigation. The system dynamically monitors obstacles at waist-to-chest height using infrared Time-of-Flight (ToF) telemetry and translates physical distance data into deterministic tactile frequency patterns.

This project was developed as a technical thesis (Bereichsübergreifende Projekte - BüP) at the Allgemeine Gewerbeschule Basel.

---

## Technical Features

* **Non-Linear Tactile Scaling:** Implementation of an exponential frequency modulation curve (4 Hz to 12 Hz) within the critical near-field zone to optimize human cognitive perception of hazard proximity.
* **Asynchronous Execution Architecture:** A fully non-blocking, multi-tasking state machine implemented via programmatic timer loops (millis) to ensure real-time execution of sensor polling and PWM output generation without execution lag.
* **Bus Speed Stabilization:** Low-frequency I2C configuration at 20 kHz to guarantee signal integrity and prevent data frames from corrupting across extended wearable wire paths.
* **Digital Signal Conditioning:** Implementation of a software-based Infinite Impulse Response (IIR) low-pass filter to smooth raw telemetry noise and suppress measurement outliers from ambient light interference.

---

## Hardware Architecture

The core architecture is built upon a custom 2-layer printed circuit board (PCB) with a target thickness of 1.6 mm, driven by a portable 3.7V Lithium-Polymer (LiPo) power source.

| Component | Part Number / Specification | Functional Role |
| :--- | :--- | :--- |
| **Microcontroller** | DFRobot FireBeetle 2 ESP32-S3 (DFR0975) | Central processing, I/O routing, and power path control |
| **ToF Sensor Array** | STMicroelectronics VL53L0X | 940 nm infrared distance acquisition via I2C bus |
| **Haptic Actuators** | 3V DC Eccentric Rotating Mass (ERM) | Mechanical vibration pulsing alert generation |
| **Gate Driver Switch** | NDS355AN N-Channel MOSFET | Logic-level low-side gate driver regulation |

### Power Rail Isolation Principle
To prevent critical microcontroller brownout loops caused by sudden inductive current spikes, the ERM vibration motors are wired directly to the unbuffered battery rail (VBAT, 3.7V nominal) via the low-side MOSFET gate switches. This configuration isolates the motor inrush currents entirely from the stabilized 3.3V logic rail powering the ESP32-S3 and the ToF sensor hardware.

---

## Hardware Integration Constraints

### The I2C Peripheral Pin Mapping Conflict
During hardware validation, it was determined that the physical pin designations labeled as SDA (IO1) and SCL (IO2) on the DFRobot FireBeetle 2 board layout cannot be utilized for external I2C peripheral buses. These lines are routed internally to the integrated DC-DC converter regulator circuitry and the GDI display interface. Connecting external sensor buses to these points causes immediate and permanent bus freeze conditions.

To bypass this design constraint, the functional physical I2C bus routing must be explicitly assigned to the alternate pin interface in software configuration:
* **SDA:** Pin 10 (A4)
* **SCL:** Pin 11 (A5)

---

## Control Profile Mapping

The firmware divides physical distance data vectors into deterministic haptic perception bands:

| Operation Zone | Target Distance (d) | Tactile Pulsing Characteristics | Perceptual Intensity |
| :--- | :--- | :--- | :--- |
| **Off** | > 100 cm | Dormant (0% PWM Duty Cycle) | Null |
| **Far (FERN)** | 50 cm - 100 cm | Fixed low-frequency pulsing at 1.0 Hz | Low Noticeability Baseline |
| **Medium (MITTEL)** | 25 cm - 50 cm | Constant paced pulsing at 3.0 Hz | Intermediate Noticeability |
| **Near (NAH)** | 10 cm - 25 cm | Exponential frequency modulation (4.0 Hz - 12.0 Hz) | Dynamic Urgency Ramp |
| **Critical (KRITISCH)**| < 10 cm | Continuous uninterrupted state (100% PWM Duty Cycle) | High Alert Hazard State |

---

## Software Configuration and Environment

The firmware environment is structured and compiled via PlatformIO within the Visual Studio Code integrated development environment.

### Project Structure Directory
```text
├── include/          # Project header directories
├── lib/              # Local peripheral libraries
├── src/
│   └── main.cpp      # Core application source code
└── platformio.ini    # Environment configuration and dependency paths
