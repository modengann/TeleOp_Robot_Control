/*
 * RobotServo.h — Timer-interrupt servo driver for ESP32-S3
 * Arduino-ESP32 core 3.x  |  Freenove FNK0099
 *
 * WHY THIS EXISTS
 * ───────────────
 * ESP32Servo uses LEDC or MCPWM to generate servo pulses.  On the S3,
 * only two MCPWM units exist, so only two servos work.  And LEDC / MCPWM
 * are shared with motor controllers in this robot, so they must stay free.
 *
 * RobotServo instead uses ONE general-purpose hardware timer.  A single ISR
 * sequences up to 8 servo pulses inside each 20 ms frame.  Because the pulses
 * come from a hardware interrupt, they stay accurate even when loop() is
 * blocked by WiFi, a web server, or slow sensor reads.
 *
 * USAGE (drop-in replacement for ESP32Servo)
 * ──────────────────────────────────────────
 *   #include "RobotServo.h"
 *
 *   RobotServo shoulder;
 *   RobotServo elbow;
 *
 *   void setup() {
 *     shoulder.attach(7);      // GPIO 7, default pulse limits 500–2400 µs
 *     elbow.attach(10, 544, 2400);  // custom limits
 *   }
 *
 *   void loop() {
 *     shoulder.write(90);      // center
 *     elbow.write(45);
 *   }
 */

#pragma once
#include <Arduino.h>

// How many servos can run at the same time.
// Raising this adds one small ISR phase per extra slot (~100 µs each).
#define ROBOT_SERVO_MAX_SERVOS     8

// Pulse widths used when attach(pin) is called without explicit limits.
// 500 µs → 0 °,  2400 µs → 180 °  (works for most hobby servos).
#define ROBOT_SERVO_DEFAULT_MIN_US 500
#define ROBOT_SERVO_DEFAULT_MAX_US 2400

class RobotServo {
public:
  RobotServo();
  ~RobotServo();

  // Attach to a GPIO pin using default pulse-width limits (500–2400 µs).
  void attach(int pin);

  // Attach with custom limits.  minUs maps to 0°, maxUs maps to 180°.
  void attach(int pin, int minUs, int maxUs);

  // Command a position in degrees (clamped to 0–180).
  // Safe to call at any time, even while the ISR is running.
  void write(int angle);

  // Return the last angle passed to write().
  // Does not read hardware — returns the software-tracked angle.
  int  read();

  // Release this servo's pin and free its slot for another servo.
  void detach();

  // True while attached to a pin.
  bool attached();

private:
  int  _servoSlot;   // index in the shared slot table (0–7), or -1 if detached
  int  _angle;       // last angle written
  int  _minUs;       // pulse width corresponding to 0°
  int  _maxUs;       // pulse width corresponding to 180°

  // Static helpers manage the shared timer and slot table.
  static int  _findFreeSlot();
  static void _startTimer();
  static void _stopTimer();
};
