#pragma once

#include "NRL.h"
#include <Preferences.h>
#include <cmath>
#include <cstring>

// ============================================================================
//  TEAM 205 (PIXEL POKERS) — SHARED MECHANISM HELPERS
// ============================================================================
//
//  This is a HELPER HEADER, not an OpMode. It contains no REGISTER_OPMODE
//  and adds nothing to the Controller's OpMode list. Every OpMode that needs
//  it just adds one #include line.
//
//
//  WHY THIS FILE EXISTS
//  ---------------------------------------------------------------------------
//  HexaServo::setPosition() does TWO things at once:
//
//    1. It sets the TARGET angle the servo should reach.
//    2. If a ramp rate is configured, it advances the servo ONE ramp step
//       toward that target, by (elapsed_ms / 1000) * rate degrees.
//
//  The ramp ONLY advances when setPosition() is called. The driver's 250 ms
//  heartbeat (HexaServo::tick) re-writes the PWM duty for the CURRENT angle
//  but never moves the mechanism.
//
//  Four consequences, ALL of which bit this robot on the field:
//
//    (a) Calling setPosition() ONCE with a new angle moves the servo by ONE
//        ramp step and then it stops there. It does NOT travel to the target.
//        A payload gate toggled on a button edge therefore moved ~2 degrees
//        and stopped — the gate "never opens".
//
//    (b) A mechanism only travels if setPosition() is called REPEATEDLY.
//        The ramp IS the motion.
//
//    (c) getPosition() reports the driver's RAMP position, not the real servo
//        angle. The driver has no idea that the jaws are jammed on a piece,
//        so it happily reports that a straining claw has "reached" 168.
//
//    (d) The PWM duty is refreshed every tick when setPosition() is called
//        every tick. A servo that is only refreshed every 250 ms can drop
//        its pulses if the ESP32 stalls or the battery rail sags — which is
//        exactly what "the servo just stops working" looks like.
//
//  ServoAxis makes one rule impossible to get wrong:
//
//      Every mechanism has exactly ONE owner. The owner sets the TARGET.
//      apply() is called EVERY tick and pushes the target to the driver.
//
//  The target lives in the axis, not in the driver, so an action can move a
//  mechanism while loop() keeps re-asserting the target every tick without
//  the two fighting each other.
// ============================================================================


// ============================================================================
//  ServoAxis — one mechanism on one servo
// ============================================================================
struct ServoAxis {
    HexaServo* servo = nullptr;

    float target = 0.0f;      // the travel pose we are heading for
    float lo     = 0.0f;      // travel limits
    float hi     = 180.0f;

    // Optional PRESS override. A servo only applies force while it is still
    // trying to REACH its commanded angle. Commanding exactly the closed
    // angle means the servo arrives, stops pushing, and holds nothing — the
    // piece is then held by friction alone and falls out the moment the arm
    // moves. So while gripping, the servo is commanded PAST the closed angle:
    // the piece blocks the jaws short of it, the servo never arrives, and it
    // keeps straining and pressing.
    float pressAngle = -1.0f;  // < 0 = press disabled
    bool  pressing   = false;

    // Attach this axis to a driver. `start` is the mechanism's home pose.
    void bind(HexaServo& s, float start, float travelLo, float travelHi) {
        servo    = &s;
        lo       = travelLo;
        hi       = travelHi;
        target   = constrain(start, travelLo, travelHi);
        pressing = false;
    }

    // Set the travel target. This does NOT move the servo — apply() does.
    // While `pressing` the press angle wins, so call stopPressing() first if
    // the mechanism needs to travel.
    void command(float angle) {
        target = constrain(angle, lo, hi);
    }

    // Freeze where the mechanism is RIGHT NOW. This is what "release the
    // trigger and it stops" actually requires: the ramp chases a target, so
    // the target has to become the current position.
    //
    // A straining servo cannot be frozen — it is held by the piece, and the
    // driver does not know where the jaws really are.
    void hold() {
        if (pressing) return;
        target = actual();
    }

    // Strain past the jaws and press.
    void press(float angle) {
        pressAngle = angle;
        pressing   = true;
    }

    // Stop pressing. The mechanism goes back to holding `target`.
    void stopPressing() {
        pressing = false;
    }

    bool isPressing() const { return pressing; }

    // The angle actually sent to the servo.
    float commanded() const {
        return (pressing && pressAngle >= 0.0f) ? pressAngle : target;
    }

    // Push the commanded angle to the driver. Call this EVERY tick — this is
    // what actually moves the mechanism AND what keeps its PWM alive.
    void apply() {
        if (servo == nullptr) return;
        servo->setPosition(commanded());
    }

    // Where the driver thinks the servo is (the ramp position). When the servo
    // is straining against a piece this runs AHEAD of the real jaw angle.
    float actual() const { return servo ? servo->getPosition() : commanded(); }

    // True once the driver has been ramped to the commanded angle.
    bool arrived(float tol = 1.0f) const {
        return fabsf(actual() - commanded()) <= tol;
    }

    // True once the driver has been ramped to the TRAVEL target (ignoring any
    // press override).
    bool travelArrived(float tol = 1.0f) const {
        return fabsf(actual() - target) <= tol;
    }

    // True while the mechanism is still travelling toward its commanded
    // angle. A straining (pressing) servo is NOT travelling — it is jammed
    // against a piece — so it never blocks another mechanism.
    bool travelling(float tol = 0.75f) const {
        return fabsf(actual() - commanded()) > tol;
    }
};


// ============================================================================
//  MotionArbiter — never ramp two servos at the same time
// ============================================================================
//  "The servos share the battery rail. There is NO BEC and no separate servo
//  regulator." (HexaServo.h)
//
//  Two servos ramping at once double the current the rail must supply. On a
//  loaded rail that sag is what makes one servo stop responding.
//
//  The arbiter serialises mechanism travel: while one mechanism is moving,
//  another mechanism's request is DEFERRED (the caller simply keeps its old
//  target and asks again next tick). Nothing is dropped — the deferred move
//  starts as soon as the rail is quiet again.
//
//  Pressing (holding a strain against a piece) is NOT travel: a straining
//  servo is not moving, so it never blocks anybody.
//
//  The owner is a function-local static, so every OpMode that includes this
//  header shares one arbiter per program — the arm and the claw arbitrate
//  against each other without any extra wiring.
inline bool motionRequest(ServoAxis& axis) {
    static ServoAxis* owner = nullptr;
    if (owner != nullptr && owner != &axis && owner->travelling()) {
        return false;              // rail is busy — defer this move
    }
    owner = &axis;
    return true;
}

// Release the rail. Call when a mechanism is idle.
inline void motionRelease(ServoAxis& axis) {
    static ServoAxis* owner = nullptr;
    if (owner == &axis) owner = nullptr;
}


// ============================================================================
//  PowerGuard — watch the rail so a brownout never surprises you
// ============================================================================
//  A brownout (the ESP32 resetting because the battery rail sagged) is the
//  classic explanation for "the servo just stops working" and for "the claw
//  opens by itself": init() runs again and puts the claw back to its open
//  angle.
//
//  PowerGuard does two things:
//
//    1. DERATE. When the pack sags, the drive output is scaled back so the
//       robot crawls instead of pulling the rail under. This is what stops
//       "drive while gripping" from resetting the board.
//
//    2. REPORT. The boot counter and reset reason go to the Controller, so a
//       brownout is visible instead of mysterious.
//
//  THRESHOLDS: measure your own pack. Run the Power Test OpMode, watch the
//  voltage with the servos moving and the motors driving, and set these two
//  numbers either side of what you saw.
struct PowerGuard {
    // Start backing off below this pack voltage (V).
    float sagVoltage = 9.80f;
    // Resume full power above this pack voltage (V).
    float recoverVoltage = 10.60f;
    // Drive output scale at and below sagVoltage.
    float minDerate = 0.25f;

    float minVoltage = 99.0f;   // lowest pack voltage seen this run
    float peakServoA = 0.0f;    // peak servo-rail current seen this run
    bool  protecting = false;

    // Call once per tick. Returns the drive scale factor to use this tick.
    float update() {
        float v = power.getBatteryVoltage();
        float a = power.getServoCurrent();

        if (v > 0.0f && v < minVoltage) minVoltage = v;
        if (a > peakServoA)       peakServoA = a;

        if (v <= 0.0f) return 1.0f;   // sensor not ready — do not interfere

        if (!protecting && v < sagVoltage) {
            protecting = true;
        } else if (protecting && v > recoverVoltage) {
            protecting = false;
        }

        if (!protecting) return 1.0f;

        // Linear ramp from minDerate at sagVoltage up to 1.0 at recoverVoltage.
        float span = recoverVoltage - sagVoltage;
        if (span <= 0.0f) return minDerate;
        float t = constrain((v - sagVoltage) / span, 0.0f, 1.0f);
        return minDerate + t * (1.0f - minDerate);
    }

    // True while the pack is sagging and the drive is being backed off.
    bool isProtecting() const { return protecting; }
};


// ============================================================================
//  Reset-reason helpers. Declared up here because BrownoutBeacon uses them.
// ============================================================================

// Number of times the MCU has booted since power-up. If this climbs while
// you are driving, the board is resetting — that is a brownout, not firmware.
inline uint32_t team205BootCount() { return nrlBootCount(); }

// Short token for the last reset: "BROWNOUT", "POWERON", "TASK_WDT", ...
inline const char* team205ResetReason() { return nrlLastResetReasonShort(); }

// True if the last reset was a brownout.
inline bool team205LastResetWasBrownout() {
    const char* r = team205ResetReason();
    return (r != nullptr) && (strcmp(r, "BROWNOUT") == 0);
}


// ============================================================================
//  BrownoutBeacon — make a reset visible without a laptop
// ============================================================================
//  It OWNS the user LED while active — skip your normal LED logic when
//  service() returns true.
struct BrownoutBeacon {
    bool     active  = false;
    uint32_t untilMs = 0;

    // Call once in init().
    void arm() {
        if (!team205LastResetWasBrownout()) return;
        active  = true;
        untilMs = millis() + 2000;
    }

    // Call every tick in loop(). Returns true while it owns the LED.
    bool service() {
        if (!active) return false;
        if ((int32_t)(millis() - untilMs) >= 0) {
            active = false;
            userLed.setOff();
            return false;
        }
        userLed.setSolid(255, 0, 0);
        return true;
    }
};

// Publish the power / brownout state to the Controller. Call from loop().
inline void team205ReportPower(NRLTelemetry& tel, const PowerGuard& g) {
    tel.addData("Boot #",  (float)team205BootCount());
    tel.addData("Reset",   team205ResetReason());
    tel.addData("Servo A", power.getServoCurrent());
    tel.addData("Min V",   g.minVoltage);
    tel.addData("Peak A",  g.peakServoA);
}


// ============================================================================
//  GripMemory — survive a brownout reset
// ============================================================================
//  If the board browns out mid-match, RAM is gone and the claw would be
//  re-initialised OPEN — the piece drops on the floor and the driver has no
//  idea why.
//
//  Storing the grip state in NVS (flash) and re-applying it on the next boot
//  means a brownout costs you a second of driving, not the piece.
//
//  Writes happen only when the grip state CHANGES (a button press), so the
//  flash wear is negligible.
struct GripMemory {
    bool engaged = false;

    // Call in init(). Restores the last known grip state.
    void begin() {
        Preferences p;
        if (!p.begin("team205", false)) return;
        engaged = p.getBool("grip", false);
        p.end();
    }

    // Call when the grip state changes.
    void set(bool on) {
        if (engaged == on) return;
        engaged = on;
        Preferences p;
        if (!p.begin("team205", false)) return;
        p.putBool("grip", on);
        p.end();
    }
};


// ============================================================================
//  ClawControl — the claw's state machine
// ============================================================================
//  The claw is the one mechanism with real state (open / closing / closed /
//  gripping), and getting that state wrong is what makes it "open by itself".
//  It lives here so every OpMode gets the same correct behaviour.
//
//  THE STATES
//  ---------------------------------------------------------------------------
//    NONE      jaws hold wherever they are. The TARGET is frozen to the
//              current position, so the servo stops where it is instead of
//              creeping on toward its last commanded pose.
//    EXPAND    the jaws open. The grip is released first — but only once the
//              rail is free, so the claw never relaxes-and-recloses while
//              another mechanism is moving.
//    CONTRACT  the jaws close. The grip engages only after the jaws have
//              REALLY arrived closed.
//    GRIPPING  the servo is commanded PAST the closed angle so it strains
//              against the piece and presses. The grip is a LATCH: it
//              survives releasing the trigger, and it is re-asserted every
//              tick so the squeeze never fades.
//
//  Why "really arrived closed" matters: the driver reports its own RAMP
//  position, and it has no idea the jaws are blocked by a piece. If the grip
//  engaged on a half-open jaw the servo would strain against nothing.
// ============================================================================

// Which way the jaws are being driven.
enum class ClawDir { NONE, EXPAND, CONTRACT };

struct ClawControl {
    ServoAxis* axis = nullptr;

    float openAngle   = 125.0f;
    float closedAngle = 155.0f;
    float gripAngle   = 168.0f;

    ClawDir dir     = ClawDir::NONE;
    bool   gripping = false;   // latched: the servo is straining on a piece
    bool   closing  = false;   // we have commanded closed and are waiting

    void bind(ServoAxis& a, float open, float closed, float grip) {
        axis        = &a;
        openAngle   = open;
        closedAngle = closed;
        gripAngle   = grip;
        dir         = ClawDir::NONE;
        gripping    = false;
        closing     = false;
    }

    bool isGripping() const { return gripping; }
    bool isClosing() const  { return closing; }

    // --- transitions ---------------------------------------------------

    // Start closing. Returns false if the rail is busy — the caller should
    // ask again next tick. Nothing is lost by waiting.
    bool tryClose() {
        if (!motionRequest(*axis)) return false;
        if (!gripping) {
            axis->command(closedAngle);
            closing = true;
        }
        return true;
    }

    // Start opening. Returns false if the rail is busy.
    bool tryOpen() {
        if (!motionRequest(*axis)) return false;
        // Let go FIRST, then open. If we opened first while still pressing,
        // the servo would relax back toward the closed angle and re-clamp.
        if (gripping) {
            gripping = false;
            axis->stopPressing();
        }
        closing = false;
        axis->command(openAngle);
        return true;
    }

    // Engage the grip now. Used by the auto-grab macro and by BTN_A.
    void engageNow() {
        gripping = true;
        closing  = false;
        axis->press(gripAngle);
    }

    // Release the grip now.
    void releaseNow() {
        gripping = false;
        closing  = false;
        axis->stopPressing();
    }

    // --- per-tick update -----------------------------------------------

    // Call every tick with the direction the driver is asking for.
    void update(ClawDir requested) {
        dir = requested;

        if (requested == ClawDir::EXPAND) {
            // Deferring is safe: the claw simply holds its current command
            // (still gripping, still at the grip angle) until the rail frees
            // up. The piece is NOT dropped.
            tryOpen();

        } else if (requested == ClawDir::CONTRACT) {
            tryClose();

            // Engage the grip only once the jaws have REALLY arrived closed.
            // The driver reports its ramp position, so this is the best
            // available signal that the jaws are shut.
            if (closing && axis->travelArrived(1.0f)) {
                engageNow();
            }

        } else {
            // No trigger engaged: the jaws stop where they are. Because the
            // servo ramps toward a TARGET, "stop" means the target must
            // become the current position — otherwise the claw keeps creeping
            // toward its last commanded pose.
            closing = false;
            motionRelease(*axis);
            if (!gripping) axis->hold();
        }
    }
};
