#include <Arduino.h>

// ============================================================================
// 1. HARDWARE PIN DEFINITIONS & SERIAL CONFIGURATION
// ============================================================================
// On ESP32-WROVER boards, pins 16 and 17 are used internally for PSRAM memory.
// We use GPIO 32 and 33 to prevent crashes and ensure stable UART communication.
#define RXD2 32 // ESP32 Pin 32 connects to WT61C TX (Transmitter) pin
#define TXD2 33 // ESP32 Pin 33 connects to WT61C RX (Receiver) pin

// HardwareSerial(2) utilizes the ESP32's built-in UART2 peripheral
HardwareSerial SensorSerial(2);

// ============================================================================
// 2. SENSOR DATA & KINEMATICS VARIABLES
// ============================================================================
// Raw readings from the WT61C sensor:
// Acceleration is measured in 'g' (1g = 9.80665 m/s^2, Earth's standard gravity)
float ax_raw = 0.0, ay_raw = 0.0, az_raw = 0.0;

// Angles are measured in degrees (-180.0° to +180.0°)
float roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;

// Kinematics variables for distance estimation:
float linear_ax = 0.0;      // Real movement acceleration after removing gravity (m/s^2)
float velocity_x = 0.0;     // Current forward/backward speed (m/s)
float position_x = 0.0;     // Estimated distance traveled (meters)

// Timing trackers for delta-time (dt) integration and printing
unsigned long lastTime_us = 0; // Tracks elapsed microseconds between loop cycles
unsigned long lastPrint = 0;   // Controls serial monitor refresh rate (10 Hz)

// ============================================================================
// 3. DRIFT SUPPRESSION THRESHOLDS
// ============================================================================
// Accelerometers always produce tiny electrical/sensor noise even when stationary.
// ACCEL_DEADBAND ignores any acceleration smaller than 0.25 m/s^2 to stop baseline drift.
const float ACCEL_DEADBAND = 0.25; 

// DAMPING_FACTOR slightly reduces velocity every cycle to prevent the speed from 
// accelerating infinitely due to integration errors (windup).
const float DAMPING_FACTOR = 0.95; 

// ============================================================================
// 4. WT61C PROTOCOL PACKET PARSER
// ============================================================================
/**
 * The WT61C-TTL transmits data in 11-byte binary packets.
 * Packet structure:
 *   Byte 0: Header (Always 0x55)
 *   Byte 1: Flag (0x51 = Acceleration, 0x52 = Gyroscope, 0x53 = Angle)
 *   Bytes 2-7: 16-bit signed sensor data for X, Y, Z axes
 *   Bytes 8-9: Temperature data
 *   Byte 10: Checksum (Sum of Bytes 0 to 9, masked with 0xFF)
 */
void readWT61CPackets() {
  // Check if at least one complete 11-byte frame is waiting in the buffer
  while (SensorSerial.available() >= 11) {
    
    // Look at the first byte without removing it (peek).
    // If it is NOT the start header (0x55), discard it to align the stream.
    if (SensorSerial.peek() != 0x55) {
      SensorSerial.read(); // Discard unaligned byte
      continue;
    }

    // Read the full 11-byte packet into a buffer array
    uint8_t buf[11];
    SensorSerial.readBytes(buf, 11);

    // Verify Checksum: Sum the first 10 bytes and compare to byte 10
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) {
      sum += buf[i];
    }
    
    // If the data was corrupted during transmission, discard this packet
    if (sum != buf[10]) {
      continue;
    }

    // ------------------------------------------------------------------------
    // CASE A: Acceleration Packet (Type 0x51)
    // ------------------------------------------------------------------------
    if (buf[1] == 0x51) {
      // Reconstruct 16-bit signed integers from two 8-bit bytes (Low Byte first)
      int16_t raw_ax = (int16_t)((buf[3] << 8) | buf[2]);
      int16_t raw_ay = (int16_t)((buf[5] << 8) | buf[4]);
      int16_t raw_az = (int16_t)((buf[7] << 8) | buf[6]);

      // Convert raw integers to units of 'g' (range is +/- 16g mapped over 32768)
      // 32768 is 2¹⁵, the maximum positive range of a signed 16-bit integer
      // 16.0 is the full-scale dynamic measurement range of the accelerometer (+/-16g)
      ax_raw = (float)raw_ax / 32768.0 * 16.0;
      ay_raw = (float)raw_ay / 32768.0 * 16.0;
      az_raw = (float)raw_az / 32768.0 * 16.0;
    }
    // ------------------------------------------------------------------------
    // CASE B: Attitude Angles Packet (Type 0x53)
    // ------------------------------------------------------------------------
    else if (buf[1] == 0x53) {
      int16_t raw_r = (int16_t)((buf[3] << 8) | buf[2]);
      int16_t raw_p = (int16_t)((buf[5] << 8) | buf[4]);
      int16_t raw_y = (int16_t)((buf[7] << 8) | buf[6]);

      // Convert raw integers to degrees (-180.0° to +180.0°)
      // 32768 is 2¹⁵, the maximum positive range of a signed 16-bit integer
      // 16.0 is the full-scale dynamic measurement range of the accelerometer (+/-180°)
      roll_deg  = (float)raw_r / 32768.0 * 180.0;
      pitch_deg = (float)raw_p / 32768.0 * 180.0;
      yaw_deg   = (float)raw_y / 32768.0 * 180.0;
    }
  }
}

// ============================================================================
// 5. ODOMETER RESET FUNCTION
// ============================================================================
/**
 * Clears accumulated velocity and distance back to zero.
 */
void resetOdometer() {
  velocity_x = 0.0;
  position_x = 0.0;
  Serial.println("\n>>> [RESET] Position and Velocity cleared to 0 <<<\n");
}

// ============================================================================
// 6. SETUP (Runs once on startup)
// ============================================================================
void setup() {
  // Initialize USB communication to computer at 115200 baud
  // Serial communication line transmits 115,200 signal changes (bits) per second
  Serial.begin(115200);
  
  // Initialize UART2 for WT61C communication at 115200 baud
  SensorSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Record initial timestamp
  lastTime_us = micros();

  Serial.println("\n--- WT61C Double-Integration Distance Estimator ---");
  Serial.println("Type '0' or 'r' in the Serial Monitor to reset distance.");
  Serial.println("---------------------------------------------------");
}

// ============================================================================
// 7. MAIN LOOP (Runs continuously)
// ============================================================================
void loop() {
  // --------------------------------------------------------------------------
  // Step 1: Check for user commands from Serial Monitor
  // --------------------------------------------------------------------------
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '0' || c == 'r' || c == 'R') {
      resetOdometer();
    }
  }

  // --------------------------------------------------------------------------
  // Step 2: Read latest acceleration and angle data from WT61C
  // --------------------------------------------------------------------------
  readWT61CPackets();

  // --------------------------------------------------------------------------
  // Step 3: Compute elapsed time (dt) in seconds since last loop execution
  // --------------------------------------------------------------------------
  unsigned long now_us = micros();
  float dt = (float)(now_us - lastTime_us) / 1000000.0; // Convert microseconds to seconds
  lastTime_us = now_us;

  // --------------------------------------------------------------------------
  // Step 4: Gravity Compensation
  // --------------------------------------------------------------------------
  // Accelerometers measure both motion and Earth's 1g gravity. When tilted,
  // part of the gravity vector leaks into the X-axis: Gravity_X = 1g * sin(pitch).
  // We must calculate and subtract this gravity component to get pure motion acceleration.
  float pitch_rad = pitch_deg * (PI / 180.0); // Convert degrees to radians
  float raw_ax_mps2 = ax_raw * 9.80665;       // Convert 'g' to m/s^2
  float gravity_component_x = 9.80665 * sin(pitch_rad);
  
  linear_ax = raw_ax_mps2 - gravity_component_x; // Pure linear acceleration

  // --------------------------------------------------------------------------
  // Step 5: Deadband Filtering & Zero-Velocity Update (ZUPT)
  // --------------------------------------------------------------------------
  // If the measured acceleration is smaller than the deadband threshold, assume
  // the sensor is sitting stationary and aggressively damp velocity to zero.
  if (abs(linear_ax) < ACCEL_DEADBAND) {
    linear_ax = 0.0;
    velocity_x *= 0.8; // Decay velocity quickly when motion stops
    if (abs(velocity_x) < 0.01) {
      velocity_x = 0.0; // Force clamp to full stop
    }
  } else {
    // ------------------------------------------------------------------------
    // Step 6: First Integration (Acceleration -> Velocity)
    // ------------------------------------------------------------------------
    // Formula: v(t) = v(t-1) + a * dt
    velocity_x += linear_ax * dt;
    velocity_x *= DAMPING_FACTOR; // Apply slight damping to prevent velocity runaway
  }

  // --------------------------------------------------------------------------
  // Step 7: Second Integration (Velocity -> Position / Distance)
  // --------------------------------------------------------------------------
  // Formula: x(t) = x(t-1) + v * dt
  position_x += velocity_x * dt;

  // --------------------------------------------------------------------------
  // Step 8: Print formatted results to Serial Monitor at 10 Hz (every 100 ms)
  // --------------------------------------------------------------------------
  // pitch_deg: Current pitch angle in degrees, relative to horizontal ground
  // linear_ax: Linear acceleration along the X-axis in m/s² (removing gravity component), if stationary will read 0.00
  // velocity_x: Velocity along the X-axis in m/s,, estimated speed at this instant 
  // position_x: Position along the X-axis in meters, cumulative distance traveled since startup/reset
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.printf("Pitch: %+5.1f° | AccelX: %+5.2f m/s² | VelX: %+5.2f m/s | DistX: %+6.3f m\n",
                  pitch_deg, linear_ax, velocity_x, position_x);
  }
}