# WT61C-TTL Sensor Guide

## What is it?
The **WT61C-TTL** is an IMU (Inertial Measurement Unit) that measures:
- **Orientation**: Roll, pitch, and yaw angles (tilt)
- **Motion**: Acceleration forces (g-forces)

---

## Quick Answer: Is it right for your project?

| Need | Can it do it? | Details |
|------|---------------|---------|
| **Angular Alignment** (level, tilt, pitch) | ✅ **YES** | Excellent for measuring orientation |
| **Positional Deviation** (distance moved) | ❌ **NO** | Cannot accurately track linear movement |

---

## ✅ What the WT61C-TTL Does Well

- Detect if something is upside down, tilted, or rotating
- Measure sudden impacts or acceleration (crash detection)
- Maintain level orientation (gimbal stabilization)

**Example for your project**: Mount it on an engine trailer to ensure it's perfectly level or matches the exact angle of aircraft/vehicle mounts.

---

## ❌ What it Cannot Do

**Positional Tracking (Distance Moved)**

The WT61C cannot accurately measure how far something has moved. Here's why:
- Accelerometers require mathematical integration **twice** to calculate distance
- Tiny sensor errors compound rapidly with each calculation
- Result: Within seconds, the system might incorrectly show movement of several inches or feet (called "drift")

**Need to track distance instead?** Use one of these alternatives:
- **Indoor movement tracking**: Optical flow sensors or rotary encoders
- **Distance to objects**: Time-of-Flight (ToF) lasers (VL53L0X) or ultrasonic sensors
- **Outdoor tracking**: GPS modules