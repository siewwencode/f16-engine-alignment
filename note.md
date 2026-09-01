# WT61C-TTL Displacement Tracking with ESP32

## Limitation

Accelerometer-only displacement tracking is only reliable for quick, sudden movements of approximately **1–2 seconds**. Integration errors rapidly accumulate and cause drift.

## Required Features

### Gravity Compensation

Use the WT61C pitch and roll angles to calculate and subtract the gravity vector from raw acceleration.

- Gravity constant: `9.81 m/s²`
- Purpose: isolate acceleration caused by movement.

### Zero-Velocity Update (ZUPT)

When acceleration is below a configured noise threshold, assume the sensor has stopped moving.

- Reset velocity to `0 m/s`
- Reduces accumulated displacement drift
- Prevents displacement from increasing while stationary

### Double Integration

Use Euler integration with elapsed time (`Δt`) to calculate velocity and displacement.

```text
velocity = velocity + acceleration × Δt
displacement = displacement + velocity × Δt
```

## WT61C-TTL Data Packets

| Packet ID | Description |
| --- | --- |
| `0x51` | Raw acceleration values |
| `0x53` | Orientation angles: roll, pitch, yaw |

## Intended Operation

The ESP32 Arduino sketch reads acceleration and angle packets from the WT61C-TTL sensor and calculates estimated displacement along the sensor's X-axis.

The calculation process is:

1. Read acceleration from packet `0x51`.
2. Read roll and pitch from packet `0x53`.
3. Calculate and remove gravity from acceleration.
4. Detect stationary periods using ZUPT.
5. Integrate linear X-axis acceleration into velocity.
6. Integrate velocity into X-axis displacement.

## Notes

- Keep the sensor stationary during startup.
- Calibrate the WT61C before testing.
- Tune the ZUPT threshold for the sensor noise level.
- Reset displacement before each movement test.
- For world-coordinate displacement, rotate sensor acceleration into the world frame and account for yaw.