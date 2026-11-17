#pragma once
#include "NRL.h"
#include <Preferences.h>
#include <esp_system.h>

// ============================================================================
//  TEAM 205 (PIXEL POKER) - SHARED MECHANISM & POWER LIBRARY (team205_mech.h)
//  Supports: drivemode.cpp, normaldrive.cpp, tiltsteering.cpp, pixelpoker_auto.cpp
// ============================================================================

class ServoAxis {
public:
    void bind(HexaServo& driver, float homeDeg, float poseADeg, float poseBDeg) {
        drv        = &driver;
        homeAngle  = homeDeg;
        minAngle   = fminf(poseADeg, poseBDeg);
        maxAngle   = fmaxf(poseADeg, poseBDeg);
        targetDeg  = homeDeg;
        lastMoveMs = millis();
    }

    void command(float deg, bool allowOvertravel = false) {
        if (!allowOvertravel) {
            deg = constrain(deg, minAngle, maxAngle);
        } else {
            deg = constrain(deg, 0.0f, 180.0f);
        }

        if (fabsf(deg - targetDeg) > 0.25f) {
            targetDeg  = deg;
            lastMoveMs = millis();
        }
    }

    void press(float gripDeg) {
        command(gripDeg, true);
    }

    void stopPressing() {
        targetDeg = constrain(targetDeg, minAngle, maxAngle);
    }

    void apply() {
        if (drv != nullptr) {
            drv->setPosition(targetDeg);
        }
    }

    float actual() const {
        return (drv != nullptr) ? drv->getPosition() : targetDeg;
    }

    float target() const {
        return targetDeg;
    }

    bool travelArrived(float tolDeg = 0.5f) const {
        return fabsf(actual() - targetDeg) <= tolDeg;
    }

    bool isSettled(float tolDeg = 1.0f) const {
        return travelArrived(tolDeg);
    }

    uint32_t msSinceCommand() const {
        return millis() - lastMoveMs;
    }

private:
    HexaServo* drv        = nullptr;
    float      homeAngle  = 90.0f;
    float      minAngle   = 0.0f;
    float      maxAngle   = 180.0f;
    float      targetDeg  = 90.0f;
    uint32_t   lastMoveMs = 0;
};

// Non-blocking motion arbiter with 650ms auto-release timeout
static ServoAxis* activeMotionOwner = nullptr;
static uint32_t   motionLockStartMs = 0;
static constexpr uint32_t MOTION_LOCK_TIMEOUT_MS = 650;

inline bool motionRequest(ServoAxis& axis) {
    uint32_t now = millis();

    if (activeMotionOwner == &axis) {
        return true;
    }

    if (activeMotionOwner == nullptr ||
        activeMotionOwner->travelArrived(1.0f) ||
        (now - motionLockStartMs) >= MOTION_LOCK_TIMEOUT_MS) {
        activeMotionOwner = &axis;
        motionLockStartMs = now;
        return true;
    }

    return false;
}

inline void motionRelease(ServoAxis& axis) {
    if (activeMotionOwner == &axis) {
        activeMotionOwner = nullptr;
    }
}

enum class ClawDir { NONE, EXPAND, CONTRACT };

class ClawControl {
public:
    ClawDir dir = ClawDir::NONE;

    void bind(ServoAxis& clawAxisRef, float openDeg, float closedDeg, float gripDeg) {
        axis       = &clawAxisRef;
        openAngle  = openDeg;
        closeAngle = closedDeg;
        gripAngle  = gripDeg;
        gripping   = false;
        dir        = ClawDir::NONE;
    }

    void update(ClawDir desired) {
        dir = desired;
        if (axis == nullptr) return;

        if (desired == ClawDir::EXPAND) {
            gripping = false;
            if (motionRequest(*axis)) {
                axis->command(openAngle);
            }
        } else if (desired == ClawDir::CONTRACT) {
            if (motionRequest(*axis)) {
                axis->command(closeAngle);
            }
        } else {
            if (!gripping && prevDir != ClawDir::NONE) {
                axis->command(axis->actual());
            }
            motionRelease(*axis);
        }

        prevDir = desired;
    }

    void tryClose() {
        if (axis == nullptr) return;
        gripping = false;
        axis->command(closeAngle);
    }

    void tryOpen() {
        if (axis == nullptr) return;
        gripping = false;
        axis->command(openAngle);
    }

    void engageNow() {
        if (axis == nullptr) return;
        gripping = true;
        axis->press(gripAngle);
    }

    void releaseNow() {
        gripping = false;
        if (axis != nullptr) {
            axis->stopPressing();
            motionRelease(*axis);
        }
    }

    bool isGripping() const {
        return gripping;
    }

private:
    ServoAxis* axis       = nullptr;
    ClawDir    prevDir    = ClawDir::NONE;
    float      openAngle  = 125.0f;
    float      closeAngle = 155.0f;
    float      gripAngle  = 160.5f;
    bool       gripping   = false;
};

class PowerGuard {
public:
    float update() {
        if (power.isBatteryLow()) {
            lowSagActive = true;
            scale = fmaxf(0.65f, scale - 0.02f);
        } else {
            lowSagActive = false;
            scale = fminf(1.00f, scale + 0.01f);
        }
        return scale;
    }

    bool isSagging() const { return lowSagActive; }
    float currentScale() const { return scale; }

private:
    float scale        = 1.0f;
    bool  lowSagActive = false;
};

class BrownoutBeacon {
public:
    void arm() {
        wasBrownout = (esp_reset_reason() == ESP_RST_BROWNOUT);
        if (wasBrownout) {
            Preferences prefs;
            if (prefs.begin("t205_pwr", false)) {
                bootCount = prefs.getUInt("bo_cnt", 0) + 1;
                prefs.putUInt("bo_cnt", bootCount);
                prefs.end();
            }
            beaconEndMs = millis() + 3000;
        } else {
            Preferences prefs;
            if (prefs.begin("t205_pwr", true)) {
                bootCount = prefs.getUInt("bo_cnt", 0);
                prefs.end();
            }
            beaconEndMs = 0;
        }
    }

    bool service() {
        if (!wasBrownout || millis() >= beaconEndMs) {
            return false;
        }
        bool on = ((millis() / 150) % 2) == 0;
        if (on) {
            userLed.setSolid(255, 0, 0);
        } else {
            userLed.setOff();
        }
        return true;
    }

    uint32_t brownoutCount() const { return bootCount; }

private:
    bool     wasBrownout = false;
    uint32_t beaconEndMs = 0;
    uint32_t bootCount   = 0;
};

class GripMemory {
public:
    bool engaged = false;

    void begin() {
        Preferences prefs;
        if (prefs.begin("t205_grip", true)) {
            engaged = prefs.getBool("held", false);
            prefs.end();
        }
    }

    void set(bool state) {
        if (engaged == state) return;
        engaged = state;
        Preferences prefs;
        if (prefs.begin("t205_grip", false)) {
            prefs.putBool("held", state);
            prefs.end();
        }
    }
};

template <typename TelemetryT>
inline void team205ReportPower(TelemetryT& telem, const PowerGuard& pg) {
    telem.addData("Rail", pg.isSagging() ? "LOW SAG" : "OK");
}
