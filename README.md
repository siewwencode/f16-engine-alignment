# Inertial X-Axis Distance Estimation (WT61C-TTL & ESP32)

An analysis, code breakdown, and practical implementation guide for estimating movement along the sensor X axis with a WT61C-TTL IMU and ESP32.

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

## 3. Code Walkthrough: `f16EngineAlignment.ino` from Line 88

### 3.1 Sensor and kinematics variables

The variables beginning around line 88 hold the latest sensor readings and the values needed for X-axis dead reckoning:

* `ax_raw`, `ay_raw`, and `az_raw` store acceleration packets in units of `g`. The packet parser still reads all three values, but only `ax_raw` is used for distance.
* `pitch_deg` is used to estimate the gravity component along the sensor X axis.
* `linear_ax` is the gravity-corrected X acceleration in `m/s^2`.
* `linear_ax_with_gravity` is the baseline-corrected X acceleration with the gravity component retained.
* `velocity_x`, `position_x`, and `distance_x` are the velocity, signed displacement, and total travel for the gravity-removed value.
* `velocity_x_with_gravity`, `position_x_with_gravity`, and `distance_x_with_gravity` are the corresponding values for the gravity-retained value.
* `hasAcceleration`, `hasAngles`, and `isCalibrated` indicate whether usable packets have arrived and whether the origin has been set.

The angular-rate and Y/Z acceleration values are retained for packet compatibility and diagnostics, but they do not affect X distance.

### 3.2 Constants and thresholds

The firmware uses `GRAVITY = 9.80665 m/s^2` to convert the sensor's acceleration from `g` to SI units. Each X-axis stream uses `ACCEL_THRESHOLD = 0.100 m/s^2`: values at or below the threshold reset that stream's velocity to zero and do not add displacement; values above it are integrated.

`ACCEL_THRESHOLD` is `0.100 m/s^2`. Each stream is integrated only when its absolute acceleration is above this value. `DAMPING_FACTOR` remains defined, but is not used; velocity damping is therefore disabled.

### 3.3 WT61C packet parsing

`readWT61CPackets()` waits for complete 11-byte WT61C packets beginning with `0x55`. It checks the packet checksum before processing it:

* `0x51`: acceleration; values are converted from signed raw counts to `g`.
* `0x52`: angular velocity; values are converted to degrees per second.
* `0x53`: roll, pitch, and yaw; values are converted to degrees.

Only the pitch value from the angle packet is needed for the current X-axis gravity correction.

### 3.4 Origin calibration

`resetOrigin()` defines the current resting point as zero. It calculates the current X gravity component from pitch, converts the raw X acceleration to `m/s^2`, and stores the result in `ax_baseline_mps2`. It then clears velocity, position, and total distance.

The origin is set automatically after the first valid acceleration and angle packets arrive. It can also be reset with the web button or by sending `0`, `r`, or `R` through the serial monitor.

### 3.5 X-axis kinematics in `loop()`

Each loop iteration performs these steps:

1. Read web requests, serial reset commands, and the latest IMU packets.
2. Calculate `dt`, the elapsed time since the previous iteration.
3. Convert raw X acceleration to `m/s^2`.
4. Estimate X gravity from pitch:

  $$a_{gravity,x} = -g \sin(\theta)$$

5. Calculate two baseline-corrected X accelerations:

   **With gravity retained:**

   $$a_{x,with} = a_{raw,x} - a_{baseline,x}$$

   **With gravity removed:**

   $$a_{x,without} = (a_{raw,x} - a_{gravity,x}) - a_{baseline,x}$$

6. Apply the `0.100 m/s^2` threshold independently to both accelerations. At or below the threshold, that stream's velocity is reset to zero and its displacement and distance are not updated.
7. For each stream above the threshold, integrate acceleration into velocity and velocity into displacement:

  $$v_x \mathrel{+}= a_x dt$$

  $$x \mathrel{+}= v_x dt$$

8. Add the absolute incremental displacement to the matching distance accumulator:

  $$distance_x \mathrel{+}= |v_x dt|$$

This estimates travel along the sensor's X axis. It does not use Y/Z acceleration or rotate X acceleration into a world frame using yaw. The loop uses measured `dt`; the browser polls the displayed values every 100 ms.

### 3.6 Web status and telemetry

`handleStatus()` returns movement state, signed displacement, total distance, and acceleration for both gravity modes as JSON. The web page polls this endpoint every 100 ms and displays six values:

* X acceleration with gravity
* Displacement with gravity
* Total distance with gravity
* X acceleration without gravity
* Displacement without gravity
* Total distance without gravity

The gravity-retained path is expected to accumulate values when gravity projects onto X, including the false acceleration caused by tilt. The serial monitor prints both streams as well.

## 4. Mitigation Strategies Implemented in Software

To make short-duration X-axis movement estimation possible, the firmware currently uses these mitigations:

1. **Pitch-Based X Gravity Cancellation:** Uses pitch ($\theta$) to estimate and subtract Earth's gravity component along X.
2. **Per-Stream thresholding:** Each acceleration stream is integrated only when its absolute X acceleration is above `0.100 m/s^2`; otherwise its own velocity is reset to $0 \text{ m/s}$.

Movement status is `MOVING` when either stream is above the threshold and `STATIONARY` only when both streams are at or below it. Distance estimation is still subject to bias, noise, tilt-estimation error, and integration drift.

## 5. Hardware Wiring (ESP32-WROVER-IE to WT61C-TTL)

> **Note for ESP32-WROVER Modules:** Avoid GPIO 16 and 17 as they are allocated internally to the integrated PSRAM. Use GPIO 32 and 33 instead.

| WT61C-TTL Pin | ESP32-WROVER-IE Pin | Function |
| :--- | :--- | :--- |
| **VCC** | **5V / 3.3V** | Power Supply |
| **GND** | **GND** | Ground |
| **TX** | **GPIO 32** | Hardware Serial RX2 |
| **RX** | **GPIO 33** | Hardware Serial TX2 |

---

## 6. Summary & Sensor Selection Recommendations

| Application | Recommended Sensor | Reason |
| :--- | :--- | :--- |
| **Direct Linear Distance / Wall Ranging** | Time-of-Flight LiDAR / Ultrasonic (e.g., TF-Mini, VL53L1X) | Direct position measurement without integration drift. |
| **Wheeled Robotics Odometer** | Optical / Hall Rotary Encoders | Direct revolution measurement immune to inertial noise. |
| **Aircraft Jacking / Inclinometer / Tilt** | **WT61C-TTL (IMU)** | High static and dynamic angular accuracy ($0.05^\circ$). |
| **Free-Space 3D Position Tracking** | Visual-Inertial Odometry (VIO) / SLAM | Fuses camera features with IMU to cancel drift. |