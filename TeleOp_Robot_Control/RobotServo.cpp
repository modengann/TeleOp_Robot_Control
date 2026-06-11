/*
 * RobotServo.cpp — Timer-interrupt servo driver for ESP32-S3
 * Arduino-ESP32 core 3.x  |  Freenove FNK0099
 *
 * HOW THE PULSE TRAIN WORKS
 * ──────────────────────────
 * A standard RC servo expects one pulse every 20 ms (50 Hz).
 * Pulse width sets the angle: ~500 µs = 0°, ~2400 µs = 180°.
 *
 * We run ONE hardware timer at 1 MHz (1 tick = 1 µs).
 * The timer fires an interrupt at the END of each "phase":
 *
 *   [servo 0 pulse] → [servo 1 pulse] → … → [servo 7 pulse] → [gap] → repeat
 *
 * Each phase has a variable length, so the ISR reschedules the alarm
 * dynamically.  The total frame is always 20 000 µs (50 Hz).
 *
 * ISR SAFETY NOTES
 * ─────────────────
 * • IRAM_ATTR tells the linker to store the ISR in on-chip RAM so it can
 *   run even if the flash instruction cache is momentarily busy.
 * • All variables the ISR touches are declared volatile.
 * • GPIO is set/cleared via direct register writes (W1TS / W1TC) — a single
 *   memory-mapped write, no read-modify-write race, always ISR-safe.
 * • timerAlarm() calls the IDF gptimer_set_alarm_action(), which is
 *   designed to be called from within a GPTimer alarm callback (ISR context).
 * • 32-bit aligned writes from write() are atomic on Xtensa LX7 (ESP32-S3),
 *   so pulse_us can be updated from loop() without a critical section.
 */

#include "RobotServo.h"
// Arduino.h (already included via RobotServo.h) pulls in:
//   esp32-hal-timer.h  →  timerBegin / timerAttachInterrupt / timerAlarm …
//   soc/gpio_reg.h     →  GPIO_OUT_W1TS_REG / GPIO_OUT1_W1TS_REG …

// ── Timing constants (values are in µs; timer runs at 1 MHz) ─────────────────

// A servo frame is 20 ms = 50 Hz.
#define FRAME_US           20000UL

// The gap phase must be at least this long so the waveform stays valid even
// when all servos are at maximum pulse width.
#define MIN_GAP_US           500UL

// Minimum duration for a slot that has no servo attached.
// Prevents the ISR from rescheduling a zero-length alarm, which would cause
// it to fire again immediately in an infinite loop.
#define INACTIVE_SLOT_US     100UL

// Timer resolution.  1 MHz → alarm values are in whole microseconds.
#define TIMER_FREQ_HZ    1000000UL

// ── Shared ISR state ──────────────────────────────────────────────────────────
//
// "volatile" tells the C++ compiler: do not cache this in a register —
// read it fresh from memory every time, because the ISR can change it.

struct ServoSlot {
  volatile int      pin;       // GPIO number, or -1 if the slot is empty
  volatile uint32_t pulse_us;  // current pulse width in microseconds
};

// The slot table.  Unattached slots have pin = -1.
static ServoSlot g_slots[ROBOT_SERVO_MAX_SERVOS];

// Current phase of the ISR state machine:
//   0 .. MAX_SERVOS-1  →  servo pulse phase for that slot index
//   MAX_SERVOS         →  gap phase (all pulses done, filling out the 20 ms frame)
static volatile int g_phase = ROBOT_SERVO_MAX_SERVOS;

// Absolute timer tick count at which the NEXT alarm should fire.
// Adding phase durations here (rather than reading the counter each time)
// ensures that any small delay between the alarm firing and the ISR running
// does not accumulate and slowly stretch the frame length.
static volatile uint64_t g_nextAlarm = 0;

// Hardware timer handle (Arduino-ESP32 3.x opaque pointer).
static hw_timer_t* g_timer = nullptr;

// Number of currently attached servos.  The timer starts on the first
// attach() call and stops when the last servo is detached().
static int g_attachCnt = 0;

// ── Fast GPIO helpers ─────────────────────────────────────────────────────────
//
// The W1TS (write-1-to-set) and W1TC (write-1-to-clear) GPIO registers let
// us set or clear a pin with ONE memory write — no read-modify-write needed.
//
// The ESP32-S3 has two sets:
//   GPIO_OUT_W1TS/TC_REG   →  pins 0–31
//   GPIO_OUT1_W1TS/TC_REG  →  pins 32–47
//
// IRAM_ATTR on inline functions ensures that if the compiler emits a
// standalone copy (e.g. when taking the address), that copy lives in IRAM.

static inline void IRAM_ATTR _pinHigh(int pin) {
  if (pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG,       1UL << pin);
  else           REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (pin - 32));
}

static inline void IRAM_ATTR _pinLow(int pin) {
  if (pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG,       1UL << pin);
  else           REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (pin - 32));
}

// ── The ISR ───────────────────────────────────────────────────────────────────
//
// This function runs every time the hardware timer alarm fires.
// It must be short, must not call blocking functions, and must reschedule
// the alarm before returning so the next phase fires on time.
//
// State machine walkthrough (example: 3 servos attached):
//
//   g_phase = 3 (gap)   ←── initial state after startTimer()
//       ISR fires → gap ended → advance to phase 0 → start servo 0 pulse → alarm in pulse_us[0]
//       ISR fires → servo 0 ended → pull pin 0 LOW → advance to phase 1 → start servo 1 pulse
//       ISR fires → servo 1 ended → pull pin 1 LOW → advance to phase 2 → start servo 2 pulse
//       ISR fires → servo 2 ended → pull pin 2 LOW → advance to phase 3 (gap) → alarm for gap
//       ISR fires → gap ended → advance to phase 0 → start servo 0 pulse → … (repeats)

void IRAM_ATTR servoISR() {

  // ── Step 1: End the phase that triggered this alarm ───────────────────────
  if (g_phase < ROBOT_SERVO_MAX_SERVOS) {
    // A servo pulse just finished — pull its signal wire LOW.
    int pin = g_slots[g_phase].pin;
    if (pin >= 0) _pinLow(pin);
  }
  // else: the gap phase just ended; no pin to touch.

  // ── Step 2: Advance to the next phase ────────────────────────────────────
  g_phase++;
  if (g_phase > ROBOT_SERVO_MAX_SERVOS) {
    // After the gap, wrap back to servo slot 0 to begin a new frame.
    g_phase = 0;
  }

  // ── Step 3: Start the new phase and decide how long it will last ──────────
  uint32_t duration;

  if (g_phase < ROBOT_SERVO_MAX_SERVOS) {
    // ── Servo pulse phase ──────────────────────────────────────────────────
    int      pin = g_slots[g_phase].pin;
    uint32_t pw  = g_slots[g_phase].pulse_us;

    if (pin >= 0) {
      _pinHigh(pin);   // raise the signal wire to start the pulse
      duration = pw;   // hold it HIGH for pw microseconds
    } else {
      // This slot has no servo attached.  Wait a short phantom time so we
      // don't schedule a zero-length alarm that would spin the ISR.
      duration = INACTIVE_SLOT_US;
    }

  } else {
    // ── Gap phase ──────────────────────────────────────────────────────────
    // Add up the time already used by all servo (and phantom) phases.
    // The gap fills whatever is left in the 20 ms frame.
    uint32_t used = 0;
    for (int i = 0; i < ROBOT_SERVO_MAX_SERVOS; i++) {
      used += (g_slots[i].pin >= 0) ? g_slots[i].pulse_us : INACTIVE_SLOT_US;
    }
    uint32_t gap = (FRAME_US > used) ? (FRAME_US - used) : MIN_GAP_US;
    if (gap < MIN_GAP_US) gap = MIN_GAP_US;
    duration = gap;
  }

  // ── Step 4: Schedule the alarm for the end of this new phase ─────────────
  // g_nextAlarm is an absolute tick count (1 tick = 1 µs at 1 MHz).
  // Accumulating instead of reading timerRead() means ISR entry latency
  // never inflates the total frame length over time.
  g_nextAlarm += duration;

  // timerAlarm(timer, alarm_count, autoreload, reload_count)
  // Sets the alarm to fire ONCE when the counter reaches g_nextAlarm.
  // Under the hood this calls gptimer_set_alarm_action(), which is
  // documented as safe to call from within a GPTimer alarm callback.
  timerAlarm(g_timer, g_nextAlarm, false, 0);
}

// ── Timer lifecycle ───────────────────────────────────────────────────────────

void RobotServo::_startTimer() {
  if (g_timer != nullptr) return;  // already running

  // Clear all slots before the ISR can touch them.
  for (int i = 0; i < ROBOT_SERVO_MAX_SERVOS; i++) {
    g_slots[i].pin      = -1;
    g_slots[i].pulse_us = 0;
  }

  // timerBegin(frequency) — Arduino-ESP32 3.x API.
  // Allocates one general-purpose hardware timer running at TIMER_FREQ_HZ.
  // Returns NULL if no hardware timer is available.
  g_timer = timerBegin(TIMER_FREQ_HZ);
  if (g_timer == nullptr) return;

  // Register our ISR.  The hardware will call servoISR() when the alarm fires.
  timerAttachInterrupt(g_timer, &servoISR);

  // g_phase is already ROBOT_SERVO_MAX_SERVOS (gap phase).
  // The first ISR call will wrap g_phase to 0 and start servo slot 0's phase.
  // We read the current counter value so g_nextAlarm is an absolute tick count.
  g_nextAlarm = timerRead(g_timer) + MIN_GAP_US;
  timerAlarm(g_timer, g_nextAlarm, false, 0);
}

void RobotServo::_stopTimer() {
  if (g_timer == nullptr) return;
  timerDetachInterrupt(g_timer);
  timerEnd(g_timer);
  g_timer = nullptr;
  g_phase = ROBOT_SERVO_MAX_SERVOS;
}

// ── Slot allocation ───────────────────────────────────────────────────────────

int RobotServo::_findFreeSlot() {
  for (int i = 0; i < ROBOT_SERVO_MAX_SERVOS; i++) {
    if (g_slots[i].pin < 0) return i;
  }
  return -1;  // all 8 slots are occupied
}

// ── Public API ────────────────────────────────────────────────────────────────

RobotServo::RobotServo()
  : _servoSlot(-1),
    _angle(90),
    _minUs(ROBOT_SERVO_DEFAULT_MIN_US),
    _maxUs(ROBOT_SERVO_DEFAULT_MAX_US)
{}

RobotServo::~RobotServo() {
  detach();
}

void RobotServo::attach(int pin) {
  attach(pin, ROBOT_SERVO_DEFAULT_MIN_US, ROBOT_SERVO_DEFAULT_MAX_US);
}

void RobotServo::attach(int pin, int minUs, int maxUs) {
  if (_servoSlot >= 0) detach();  // re-attaching: release the old slot first

  // _startTimer() initializes all g_slots[].pin to -1.  This MUST happen
  // before _findFreeSlot() because the BSS-zero-initialized array starts
  // with pin = 0 everywhere, and _findFreeSlot() uses pin < 0 as the
  // "empty slot" test — 0 is not < 0, so without this init every slot
  // would appear occupied and attach() would fail silently.
  if (g_attachCnt == 0) _startTimer();

  int slot = _findFreeSlot();
  if (slot < 0) return;  // all 8 slots are in use

  g_attachCnt++;
  _minUs     = minUs;
  _maxUs     = maxUs;
  _servoSlot = slot;

  // pinMode and digitalWrite are slow — do them before entering the critical
  // section so the ISR stays paused as briefly as possible.
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  // Update the slot atomically.  Disable interrupts so the ISR always sees
  // a consistent pair of (pin, pulse_us) — never a half-written slot.
  // Write pulse_us first; write pin last because the ISR checks pin >= 0
  // as the "is this slot active?" flag.
  portDISABLE_INTERRUPTS();
  g_slots[slot].pulse_us = (uint32_t)((minUs + maxUs) / 2);  // center position
  g_slots[slot].pin      = pin;
  portENABLE_INTERRUPTS();

  write(90);  // command 90° as a safe initial position
}

void RobotServo::detach() {
  if (_servoSlot < 0) return;

  portDISABLE_INTERRUPTS();

  // If the ISR happens to be mid-pulse for this slot right now, end the pulse
  // cleanly before marking the slot empty.
  if (g_phase == _servoSlot) {
    int pin = g_slots[_servoSlot].pin;
    if (pin >= 0) _pinLow(pin);
  }

  // Mark slot empty.  Write pin = -1 LAST so the ISR never sees
  // a valid pin with a stale pulse_us.
  g_slots[_servoSlot].pulse_us = 0;
  g_slots[_servoSlot].pin      = -1;

  portENABLE_INTERRUPTS();

  _servoSlot = -1;

  // Stop the timer when the last servo is detached.
  g_attachCnt--;
  if (g_attachCnt <= 0) {
    g_attachCnt = 0;
    _stopTimer();
  }
}

void RobotServo::write(int angle) {
  if (_servoSlot < 0) return;

  _angle = constrain(angle, 0, 180);

  // Map the angle to a pulse width using long arithmetic to avoid overflow.
  uint32_t pw = (uint32_t)map((long)_angle, 0L, 180L, (long)_minUs, (long)_maxUs);

  // On the Xtensa LX7 core (ESP32-S3), a 32-bit write to a naturally-aligned
  // address is atomic — so no critical section is needed.  The ISR may read
  // either the old or new value for the current pulse; both are valid states.
  g_slots[_servoSlot].pulse_us = pw;
}

int RobotServo::read() {
  return _angle;
}

bool RobotServo::attached() {
  return _servoSlot >= 0;
}
