# Project Brief: WT61C-TTL Displacement Tracking

## Objective

Use an ESP32 and WT61C-TTL to estimate short-term displacement along the sensor's X-axis.

## Main Limitation

Accelerometer-only tracking is reliable only for quick movements of about **1–2 seconds**. The movement does not need to be at constant speed; it should have clear acceleration and deceleration. Constant-speed motion produces little acceleration and is difficult to detect. Integration errors cause drift over longer periods.

## Required Method

1. Read acceleration from packet `0x51`.
2. Read roll and pitch from packet `0x53`.
3. Remove gravity using the orientation data and `9.81 m/s²`.
4. Detect when the sensor is stationary using a noise threshold.
5. Apply ZUPT by resetting velocity to `0 m/s` when stationary.
6. Integrate acceleration twice using elapsed time (`Δt`):

```text
velocity = velocity + acceleration × Δt
displacement = displacement + velocity × Δt
```

## Testing Notes

- Keep the sensor still during startup.
- Calibrate the WT61C-TTL before testing.
- Tune the stationary/noise threshold.
- Reset displacement before each test.
- If the sensor turns left or right, use its yaw angle to convert movement into fixed world directions.

## Debugging and Test Data

Add a serial debug output that records:

```text
time, raw_accel_x, raw_accel_y, raw_accel_z, roll, pitch, yaw,
linear_accel_x, velocity_x, displacement_x, stationary
```

Run these tests and save the recorded data:

1. **Stationary:** Leave the sensor still for 30 seconds. Linear acceleration, velocity, and displacement should remain close to zero.
2. **One-direction movement:** Move the sensor along its X-axis, then stop. Check that acceleration changes during the movement, velocity returns to zero after stopping, and displacement changes in the expected direction.
3. **Rotation:** Rotate the sensor left or right while moving. Check that yaw changes and that the calculated world-direction displacement remains correct.