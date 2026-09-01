# Dual-Optical Differential Odometry System

## Overview

**No magnetic compass needed.** This system eliminates the unreliable compass entirely, making it perfect for steel-laden hangars where magnetic fields are corrupted by:
- Steel beams and framework
- Massive engine blocks
- Reinforced concrete floors

Instead, it uses **two optical flow sensors** (like high-speed computer mice) that read the microscopic grain of the hangar floor to capture complete 3D positioning and parallel alignment without line-of-sight.

---

## 🔧 The Hardware: Three Sensors, Correct Placement

Your sensor stack consists of **three separate sensors** wired to your ESP32—each with a specific job and position.

### The Critical Insight
The common drone combo board (MatekSys 3901-L0X) combines PMW3901 + VL53L0X on one board. But this creates a **physical impossibility** for your application:
- PMW3901 cameras must hover 1-2 inches above the floor to track texture
- If the laser also sits 1-2 inches up, it can't measure a 2,100 mm POR height
- **Solution**: Separate the sensors—floor trackers at the bottom, height sensor at the top

| Component | Purpose | Position |
|-----------|---------|----------|
| **Two PMW3901 Cameras** | X/Y movement + rotation angle | **Bottom of cart**, 1-2" above floor, ~500 mm apart |
| **Ultrasonic or ToF Sensor** (MaxBotix or VL53L1X) | Z-axis height measurement | **Top of pole**, at the point where tool touches POR |

**How they work together:**
- **Bottom pair** reads floor grain → tracks lateral drift and calculates rotation
- **Top sensor** shoots straight down to empty floor → measures exact height of pylon/trailer

---

## 🛠️ Building Your Tool: Bottom Cameras + Top Pole

The key is understanding the **two-level architecture**:

### Bottom Level: Floor Tracking Assembly
- **Two PMW3901 optical cameras** mounted rigidly to the rolling cart
- Both pointing straight down at the concrete
- Hovering exactly 1-2 inches above the floor (keeps cameras in focus on floor texture)
- Spaced exactly 500 mm apart (this distance is critical for rotation math)

### Top Level: Height Measurement Pole
- **One ultrasonic or ToF sensor** mounted at the very top of a pole
- Mounts where the pole **physically touches the aircraft pylon** and later the trailer mounting point
- Points straight down to the empty floor beneath it
- Measures the distance from the pole's tip to the concrete floor

**Physical setup**: Imagine a rolling cart with two "eyes" near the ground watching floor texture, connected to a pole with a "sensor head" at the top that measures altitude. As you roll and position it, you're getting X/Y position from the cart, Z height from the pole, and rotation from comparing the two bottom cameras.

---

## ⚙️ How the System Calculates Everything

The three sensors read simultaneously and independently:

### Height (Z-Axis) — Top Pole Sensor
- The ultrasonic/ToF sensor at the pole's tip shoots a pulse **straight down** to the empty floor
- **At the aircraft**: Hold pole against POR, measure height (e.g., 2,100 mm)
- **At the trailer**: Position pole against mounting point, jack up until sensor again reads exactly 2,100 mm ✅
- The pole physically touches the reference point; the sensor measures down from there

### Centerline Translation (X/Y Shift) — Bottom Camera Pair
- Both optical cameras at the bottom read the floor texture as you push the cart
- Push the cart sideways → both cameras see floor texture move the opposite direction
- ESP32 averages both readings to determine lateral drift
- Tracks exactly how many millimeters left/right from centerline ✅

### Parallel Alignment (Yaw/Rotation) — Bottom Camera Differential
This is how you achieve perfect parallelism **without a compass**:

- Spin the cart in place
- **Front camera** swings right, **rear camera** swings left  
- The difference between their sideways movements reveals your exact rotation angle

$$\Delta \theta = \arctan\left(\frac{X_{\text{front}} - X_{\text{rear}}}{\text{Distance}}\right)$$

When front and rear sensors move in the **exact same direction at the exact same speed**, your rotation is **0.0°** = perfectly parallel ✅

---

## 📋 Step-by-Step: How to Use It (Mechanic's Workflow)

### Step 1: Capture the Reference Point
- Roll the cart **under the aircraft wing**
- Position the **top pole's sensor tip** so it's directly above the aircraft's **mounting pylon (POR)**
- The pole points down to the empty floor beneath the pylon

### Step 2: Zero the System
- Open the **Wi-Fi dashboard on your phone**
- Tap **"Zero"**
- ESP32 saves:
  - Exact POR height (measured from pole to floor)
  - X, Y, Angle coordinates all set to 0

### Step 3: The Blind Walk
- Roll the cart **around obstacles** to the engine trailer
- As you walk, the two bottom cameras track **every millimeter** of floor texture
- The bottom cart doesn't need the pylon in sight—it's reading the floor grain
- No line-of-sight required—the cameras see the concrete beneath you

### Step 4: Align the Trailer
- Position the **top pole's sensor tip** directly above the **trailer's mounting point**
- The bottom cart is just for tracking your path; the pole measures the trailer's height
- Look at your **phone screen** — it tells you exactly what to do:

| Reading | Action |
|---------|--------|
| Height: +140 mm | **Raise jacks 140 mm up** |
| Translation: 50 mm Left | **Push cart/trailer 50 mm LEFT** |
| Angle: +2.5° CW | **Rotate cart/trailer 2.5° clockwise** |

### Step 5: Perfect Alignment
- Keep adjusting until **all three values read 0**
- **✅ Trailer is now mathematically locked** to the exact same coordinates as the aircraft POR
- Engine is ready to load with confidence