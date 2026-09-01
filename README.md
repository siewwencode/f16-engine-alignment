# Inertial Distance Estimation via Tri-Axial Acceleration (WT61C-TTL & ESP32)

An analysis, mathematical breakdown, and practical implementation guide on using a 6-Axis IMU (WT61C-TTL) to calculate linear displacement through double integration.

---

## 1. Theoretical Principle

Linear distance ($x$) is derived from acceleration ($a$) by integrating twice with respect to time ($t$):

$$v(t) = \int a(t) \, dt$$

$$x(t) = \int v(t) \, dt = \iint a(t) \, dt^2$$

Where:
* $a(t)$ = Linear acceleration ($\text{m/s}^2$)
* $v(t)$ = Velocity ($\text{m/s}$)
* $x(t)$ = Position / Distance ($\text{m}$)

---

## 2. Real-World Limitations with MEMS Accelerometers

While mathematically valid, using consumer-grade MEMS accelerometers (like the WT61C-TTL) for dead-reckoning distance tracking introduces severe practical limitations:

### Quadratic Error Growth ($\frac{1}{2}\epsilon t^2$)
Any residual bias or noise $\epsilon$ in the acceleration reading compounds quadratically over time:

$$\text{Position Error} \approx \frac{1}{2} \epsilon t^2$$

* A minor offset of just **$0.01 \text{ m/s}^2$** results in:
  * **$0.5 \text{ m}$** error after 10 seconds
  * **$18.0 \text{ m}$** error after 60 seconds (even while stationary)

### Gravity Vector Leakage
Accelerometers measure proper acceleration, including Earth's constant gravitational pull ($1g \approx 9.81 \text{ m/s}^2$). 
* A tilt error of merely **$0.1^\circ$** creates a false horizontal acceleration:

$$a_{\text{false}} = 9.81 \times \sin(0.1^\circ) \approx 0.0171 \text{ m/s}^2$$

* Left uncorrected, this registers as false forward/backward motion.

### Sensor Bias & Thermal Drift
MEMS internal mechanical structures drift with ambient temperature changes and electrical noise, causing continuous integration windup without external absolute references (e.g., GPS, optical flow, or wheel encoders).

---

## 3. Mitigation Strategies Implemented in Software

To make short-duration linear movement estimation possible, the firmware implements three software mitigations:

1. **Attitude-Based Gravity Cancellation:** Uses real-time pitch ($\theta$) and roll ($\phi$) measurements to project and subtract Earth's gravity vector from the raw acceleration data.
2. **Zero-Velocity Update (ZUPT):** Detects when the sensor is stationary (acceleration falls below a configured deadband) and forcefully clamps the integrated velocity back to $0 \text{ m/s}$.
3. **Velocity Damping:** Applies a slight exponential decay factor ($0.95$) per update cycle to prevent runaway velocity accumulation.

---

## 4. Hardware Wiring (ESP32-WROVER-IE to WT61C-TTL)

> **Note for ESP32-WROVER Modules:** Avoid GPIO 16 and 17 as they are allocated internally to the integrated PSRAM. Use GPIO 32 and 33 instead.

| WT61C-TTL Pin | ESP32-WROVER-IE Pin | Function |
| :--- | :--- | :--- |
| **VCC** | **5V / 3.3V** | Power Supply |
| **GND** | **GND** | Ground |
| **TX** | **GPIO 32** | Hardware Serial RX2 |
| **RX** | **GPIO 33** | Hardware Serial TX2 |

---

## 5. Summary & Sensor Selection Recommendations

| Application | Recommended Sensor | Reason |
| :--- | :--- | :--- |
| **Direct Linear Distance / Wall Ranging** | Time-of-Flight LiDAR / Ultrasonic (e.g., TF-Mini, VL53L1X) | Direct position measurement without integration drift. |
| **Wheeled Robotics Odometer** | Optical / Hall Rotary Encoders | Direct revolution measurement immune to inertial noise. |
| **Aircraft Jacking / Inclinometer / Tilt** | **WT61C-TTL (IMU)** | High static and dynamic angular accuracy ($0.05^\circ$). |
| **Free-Space 3D Position Tracking** | Visual-Inertial Odometry (VIO) / SLAM | Fuses camera features with IMU to cancel drift. |