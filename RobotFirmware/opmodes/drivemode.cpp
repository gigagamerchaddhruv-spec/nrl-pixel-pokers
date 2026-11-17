#include "NRL.h"
#include "team205_mech.h"
#include <Wire.h>
#include <math.h>

// ============================================================
//  DriveMode — Tank Drive, IMU Auto-Level, Arm, Claw & Payload
// ============================================================
//  SERVO PORTS ON HEXA COMMAND HUB (Standard 3-pin plug, Red in middle):
//      SERVO_1 (S1) -> ARM           (45° = up, 115° = floor)
//      SERVO_4 (S4) -> CLAW          (30° = open, 95° = closed)
//      SERVO_3 (S3) -> PAYLOAD GATE  (0° = closed, 90° = open)
//
//  Controls:
//  - TRUE TANK DRIVE:
//      * Left Stick Y   = Left Wheels
//      * Right Stick Y  = Right Wheels
//  - SPACESTATION AUTO-LEVEL (Onboard IMU):
//      * BTN_X          = Press once to START leveling, press X again to STOP
//  - PRECISION TURN SHORTCUTS:
//      * BTN_RB         = Turn 140° at 90% speed
//      * BTN_LB         = Turn 50°  at 80% speed
//  - ARM (S1):
//      * BTN_DPAD_UP    = Arm UP
//      * BTN_DPAD_DOWN  = Arm DOWN + Automatically OPENS Claw (30°)
//  - CLAW (S4):
//      * BTN_DPAD_RIGHT = Claw OPEN (30°)
//      * BTN_DPAD_LEFT  = Claw CLOSE (95°)
//  - PAYLOAD GATE (S3):
//      * BTN_A          = Press once to OPEN 90°, press A again to CLOSE 0°
// ============================================================

static HexaDCMotor leftMotor ({ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true,  .minDrive = 29 });
static HexaDCMotor rightMotor({ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = false, .minDrive = 32 });
static TankDrive   drive(leftMotor, rightMotor);

// Servo ports on the HEXA Command Hub
static constexpr uint8_t ARM_SIGNAL_PIN     = SERVO_1;  // S1
static constexpr uint8_t CLAW_SIGNAL_PIN    = SERVO_4;  // S4 (J20)
static constexpr uint8_t PAYLOAD_SIGNAL_PIN = SERVO_3;  // S3 (J17)

// ---- IMU AUTO-LEVEL TUNING (Button X Toggle) -----------------------------
// 1. If tilting the robot nose-up does NOT change "Tilt" on the screen,
//    flip USE_Y_AXIS_FOR_PITCH from false to true.
// 2. If the robot drives DOWN the ramp instead of UP toward level,
//    flip LEVEL_DRIVE_SIGN between 1.0f and -1.0f.
static constexpr bool  USE_Y_AXIS_FOR_PITCH = false;
static constexpr float LEVEL_DRIVE_SIGN     =  1.0f;
static constexpr float LEVEL_KP             =  0.022f;  // Proportional climb gain
static constexpr float LEVEL_KD             =  0.008f;  // Damping gain (stops see-saw bounce)
static constexpr float LEVEL_DEADBAND_DEG   =  1.5f;    // Stop motors within ±1.5° of flat
static constexpr float LEVEL_MIN_POWER      =  0.16f;   // Minimum power to overcome ramp friction
static constexpr float LEVEL_MAX_POWER      =  0.48f;   // Max power so robot won't overshoot ramp

// ---- PRECISION TURN TUNING (RB = 140° @ 90%, LB = 50° @ 80%) --------------
static constexpr float    TURN_140_DIR      =  1.0f;
static constexpr float    TURN_140_SPEED    =  0.90f;   // 90% speed
static constexpr uint32_t TURN_140_MS       =  380;     // Duration for 140°

static constexpr float    TURN_50_DIR       = -1.0f;
static constexpr float    TURN_50_SPEED     =  0.80f;   // 80% speed
static constexpr uint32_t TURN_50_MS        =  165;     // Duration for 50°

static constexpr uint32_t TURN_BRAKE_MS     =  45;      // Active reverse brake pulse

// ---- ARM ANGLES (S1) -----------------------------------------------------
static constexpr float ARM_UP_ANGLE         = 45.0f;    // Raised carry height
static constexpr float ARM_DOWN_ANGLE       = 115.0f;   // Lowered floor height
static constexpr float ARM_STEP_PER_TICK    = 1.2f;     // Smooth 120 deg/s travel

// ---- CLAW ANGLES (S4 — Calibrated: 30° open, 95° fully closed) -----------
static constexpr float CLAW_OPEN_ANGLE      = 30.0f;    // Fully open
static constexpr float CLAW_CLOSED_ANGLE    = 95.0f;    // Fully closed
static constexpr float CLAW_STEP_PER_TICK   = 2.2f;     // Crisp ~0.3s open/close

// ---- PAYLOAD GATE (S3): 90° OPEN / CLOSE TOGGLE ON BUTTON A ---------------
static constexpr float PAYLOAD_CLOSED_ANGLE = 0.0f;     // Closed position
static constexpr float PAYLOAD_OPEN_ANGLE   = 90.0f;    // +90° Open position
static constexpr float PAYLOAD_STEP_PER_TICK = 3.5f;    // Crisp 90° toggle

// ---- CARRY-MODE DRIVE PROTECTION -----------------------------------------
static constexpr float CARRY_MAX_DRIVE      = 0.80f;
static constexpr float CARRY_ACCEL_PER_TICK = 0.05f;

static HexaServo arm    ({ .signalPin = ARM_SIGNAL_PIN,     .startAngle = ARM_UP_ANGLE });
static HexaServo claw   ({ .signalPin = CLAW_SIGNAL_PIN,    .startAngle = CLAW_OPEN_ANGLE });
static HexaServo payload({ .signalPin = PAYLOAD_SIGNAL_PIN, .startAngle = PAYLOAD_CLOSED_ANGLE });

static float armCmdDeg      = ARM_UP_ANGLE;
static float clawCmdDeg     = CLAW_OPEN_ANGLE;
static float payloadCmdDeg  = PAYLOAD_CLOSED_ANGLE;

static bool  clawGripLocked = false;
static bool  payloadOpen    = false;
static float smoothedLeft   = 0.0f;
static float smoothedRight  = 0.0f;

// Timed + Brake Turn state
static bool     turnActive       = false;
static float    turnPowerCmd     = 0.0f;
static uint32_t turnSpinEndMs    = 0;
static uint32_t turnBrakeEndMs   = 0;
static float    turnTargetDegLog = 0.0f;

// Onboard LSM6DS IMU Auto-Level state
static uint8_t  imuAddr          = 0;
static bool     autoLevelActive  = false;
static float    levelZeroDeg     = 0.0f;
static float    filteredTiltDeg  = 0.0f;
static float    prevLevelErrDeg  = 0.0f;
static uint32_t prevLevelMs      = 0;

static PowerGuard     powerGuard;
static BrownoutBeacon brownoutBeacon;
static GripMemory     gripMemory;

static uint16_t rawButtons = 0;

// ------------------------------------------------------------
//  Direct Non-Invasive LSM6DS IMU Helpers (Reads pitch safely)
// ------------------------------------------------------------
static bool probeImuAddress(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(0x0F); // WHO_AM_I register
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)addr, (uint8_t)1) != 1) return false;
    uint8_t who = Wire.read();
    return (who == 0x69 || who == 0x6A || who == 0x6C);
}

static bool readRawImuTilt(float& outDeg) {
    if (imuAddr == 0) return false;

    Wire.beginTransmission(imuAddr);
    Wire.write(0x28); // OUTX_L_A (6 accelerometer bytes)
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)imuAddr, (uint8_t)6) != 6) return false;

    uint8_t xl = Wire.read();
    uint8_t xh = Wire.read();
    uint8_t yl = Wire.read();
    uint8_t yh = Wire.read();
    uint8_t zl = Wire.read();
    uint8_t zh = Wire.read();

    int16_t ax = (int16_t)((xh << 8) | xl);
    int16_t ay = (int16_t)((yh << 8) | yl);
    int16_t az = (int16_t)((zh << 8) | zl);

    float pitchAxis = USE_Y_AXIS_FOR_PITCH ? (float)ay : (float)ax;
    float zAxis     = (float)az;
    if (fabsf(pitchAxis) < 1.0f && fabsf(zAxis) < 1.0f) return false;

    outDeg = atan2f(pitchAxis, zAxis) * 57.29578f;
    return true;
}

static void initImuForLeveling() {
    imuAddr = 0;
    if (probeImuAddress(0x6A)) {
        imuAddr = 0x6A;
    } else if (probeImuAddress(0x6B)) {
        imuAddr = 0x6B;
    }

    if (imuAddr == 0) return;

    // Ensure accelerometer is enabled (104 Hz, ±2g) if it was powered down
    Wire.beginTransmission(imuAddr);
    Wire.write(0x10); // CTRL1_XL
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)imuAddr, (uint8_t)1) == 1) {
        uint8_t ctrl1 = Wire.read();
        if (ctrl1 == 0x00) {
            Wire.beginTransmission(imuAddr);
            Wire.write(0x10);
            Wire.write(0x40);
            Wire.endTransmission(true);
        }
    }

    // Calibrate resting flat angle as 0.0° baseline
    float sum = 0.0f;
    int validSamples = 0;
    for (int i = 0; i < 12; i++) {
        float deg = 0.0f;
        if (readRawImuTilt(deg)) {
            sum += deg;
            validSamples++;
        }
        delay(5);
    }

    if (validSamples > 0) {
        levelZeroDeg    = sum / (float)validSamples;
        filteredTiltDeg = levelZeroDeg;
    }
}

static void refreshRawButtons() {
    float lx, ly, rx, ry;
    uint16_t btns = 0;
    NRLGamepad::read(lx, ly, rx, ry, btns);
    rawButtons = btns;
}

static bool isBtnPressed(uint16_t btn) {
    return gamepad1.pressed(btn) || ((rawButtons & btn) != 0);
}

static float stepAngle(float current, float target, float maxStep) {
    if (target > current + maxStep) return current + maxStep;
    if (target < current - maxStep) return current - maxStep;
    return target;
}

static float slewDrive(float current, float target, float maxStep) {
    if (target > current + maxStep) return current + maxStep;
    if (target < current - maxStep) return current - maxStep;
    return target;
}

static void startPrecisionTurn(float targetDeg, float speed, uint32_t durationMs, float directionSign) {
    uint32_t now     = millis();
    turnTargetDegLog = targetDeg * directionSign;
    turnPowerCmd     = speed * directionSign;
    turnSpinEndMs    = now + durationMs;
    turnBrakeEndMs   = turnSpinEndMs + TURN_BRAKE_MS;
    turnActive       = true;
    autoLevelActive  = false;
}

static void updatePrecisionTurn(float driveScale) {
    uint32_t now = millis();
    if (now < turnSpinEndMs) {
        drive.drive(0.0f, turnPowerCmd * driveScale);
    } else if (now < turnBrakeEndMs) {
        drive.drive(0.0f, -turnPowerCmd * 0.55f);
    } else {
        drive.stop();
        turnActive = false;
    }
}

class DriveMode : public NRLOpMode {
public:
    void init() override {
        gripMemory.begin();

        arm.begin();
        claw.begin();
        payload.begin();
        leftMotor.begin();
        rightMotor.begin();

        initImuForLeveling();

        armCmdDeg       = ARM_UP_ANGLE;
        payloadOpen     = false;
        payloadCmdDeg   = PAYLOAD_CLOSED_ANGLE;
        turnActive      = false;
        autoLevelActive = false;
        smoothedLeft    = 0.0f;
        smoothedRight   = 0.0f;

        if (gripMemory.engaged) {
            clawGripLocked = true;
            clawCmdDeg     = CLAW_CLOSED_ANGLE;
        } else {
            clawGripLocked = false;
            clawCmdDeg     = CLAW_OPEN_ANGLE;
        }

        arm.setPosition(armCmdDeg);
        claw.setPosition(clawCmdDeg);
        payload.setPosition(payloadCmdDeg);

        brownoutBeacon.arm();
        userLed.setSolid(0, 255, 0);
    }

    void loop() override {
        refreshRawButtons();
        const float driveScale = powerGuard.update();

        // Update filtered IMU tilt angle every tick
        float rawDeg = 0.0f;
        if (readRawImuTilt(rawDeg)) {
            filteredTiltDeg = (0.75f * filteredTiltDeg) + (0.25f * rawDeg);
        }
        const float relativeTiltDeg = filteredTiltDeg - levelZeroDeg;

        // --------------------------------------------------------
        //  0. BUTTON X TOGGLE — Start / Stop Spacestation Auto-Level
        // --------------------------------------------------------
        if (gamepad1.justPressed(BTN_X)) {
            if (imuAddr != 0) {
                autoLevelActive = !autoLevelActive;
                turnActive      = false;
                prevLevelErrDeg = relativeTiltDeg;
                prevLevelMs     = millis();
                if (!autoLevelActive) {
                    drive.stop();
                }
            } else {
                autoLevelActive = false;
            }
        }

        // Instant joystick override cancels auto-turn or auto-level if driver pushes sticks
        const bool sticksMoved = (fabsf(gamepad1.leftY()) > 0.30f || fabsf(gamepad1.rightY()) > 0.30f);
        if ((turnActive || autoLevelActive) && sticksMoved) {
            drive.stop();
            turnActive      = false;
            autoLevelActive = false;
        }

        // --------------------------------------------------------
        //  1. TURN SHORTCUTS (RB = 140° @ 90%, LB = 50° @ 80%)
        // --------------------------------------------------------
        if (!turnActive && !autoLevelActive) {
            if (gamepad1.justPressed(BTN_RB)) {
                startPrecisionTurn(140.0f, TURN_140_SPEED, TURN_140_MS, TURN_140_DIR);
            } else if (gamepad1.justPressed(BTN_LB)) {
                startPrecisionTurn(50.0f,  TURN_50_SPEED,  TURN_50_MS,  TURN_50_DIR);
            }
        }

        // --------------------------------------------------------
        //  2. DRIVE — Auto-Level, Precision Turn, or Dual-Stick Tank
        // --------------------------------------------------------
        const float activeMaxDrive = clawGripLocked ? CARRY_MAX_DRIVE : 1.0f;

        if (autoLevelActive) {
            smoothedLeft  = 0.0f;
            smoothedRight = 0.0f;

            uint32_t now = millis();
            float dt = (now - prevLevelMs) * 0.001f;
            if (dt < 0.005f) dt = 0.01f;
            prevLevelMs = now;

            float error = relativeTiltDeg;
            float dErr  = (error - prevLevelErrDeg) / dt;
            prevLevelErrDeg = error;

            if (fabsf(error) <= LEVEL_DEADBAND_DEG) {
                drive.stop();
                telemetry.addData("LEVEL", "BALANCED!");
            } else {
                float cmd = (error * LEVEL_KP + dErr * LEVEL_KD) * LEVEL_DRIVE_SIGN;
                if (cmd > 0.0f) {
                    cmd = constrain(cmd, LEVEL_MIN_POWER, LEVEL_MAX_POWER);
                } else {
                    cmd = constrain(cmd, -LEVEL_MAX_POWER, -LEVEL_MIN_POWER);
                }
                drive.drive(cmd * driveScale, 0.0f);
                telemetry.addData("LEVEL", "BALANCING...");
            }
        } else if (turnActive) {
            smoothedLeft  = 0.0f;
            smoothedRight = 0.0f;
            updatePrecisionTurn(driveScale);
            telemetry.addData("DRIVE", "AUTO TURN");
            telemetry.addData("Turn Deg", turnTargetDegLog);
        } else {
            float leftStick  = gamepad1.leftY();
            float rightStick = gamepad1.rightY();

            const float DEADZONE = 0.05f;
            if (fabsf(leftStick)  < DEADZONE) leftStick  = 0.0f;
            if (fabsf(rightStick) < DEADZONE) rightStick = 0.0f;

            float targetL = leftStick  * activeMaxDrive * driveScale;
            float targetR = rightStick * activeMaxDrive * driveScale;

            if (clawGripLocked) {
                smoothedLeft  = slewDrive(smoothedLeft,  targetL, CARRY_ACCEL_PER_TICK);
                smoothedRight = slewDrive(smoothedRight, targetR, CARRY_ACCEL_PER_TICK);
            } else {
                smoothedLeft  = targetL;
                smoothedRight = targetR;
            }

            float forward = (smoothedLeft + smoothedRight) * 0.5f;
            float turn    = (smoothedLeft - smoothedRight) * 0.5f;

            drive.drive(forward, turn);
            telemetry.addData("LEVEL", imuAddr != 0 ? "OFF (Press X)" : "NO IMU");
        }

        // --------------------------------------------------------
        //  3. ARM (S1) — D-Down = Arm Down + Auto-Open Claw (30°)
        //                D-Up   = Arm Up
        // --------------------------------------------------------
        const bool dpadUp   = isBtnPressed(BTN_DPAD_UP);
        const bool dpadDown = isBtnPressed(BTN_DPAD_DOWN);

        if (dpadUp && !dpadDown) {
            armCmdDeg = stepAngle(armCmdDeg, ARM_UP_ANGLE, ARM_STEP_PER_TICK);
        } else if (dpadDown && !dpadUp) {
            armCmdDeg = stepAngle(armCmdDeg, ARM_DOWN_ANGLE, ARM_STEP_PER_TICK);

            // Automatically open the claw to 30° when arm goes down
            clawGripLocked = false;
            gripMemory.set(false);
            clawCmdDeg = stepAngle(clawCmdDeg, CLAW_OPEN_ANGLE, CLAW_STEP_PER_TICK);
        }

        // --------------------------------------------------------
        //  4. CLAW (S4) — D-Right = Open (30°), D-Left = Close (95°)
        // --------------------------------------------------------
        const bool dpadRight = isBtnPressed(BTN_DPAD_RIGHT);
        const bool dpadLeft  = isBtnPressed(BTN_DPAD_LEFT);

        if (dpadRight && !dpadLeft) {
            clawGripLocked = false;
            gripMemory.set(false);
            clawCmdDeg = stepAngle(clawCmdDeg, CLAW_OPEN_ANGLE, CLAW_STEP_PER_TICK);
        } else if (dpadLeft && !dpadRight) {
            clawGripLocked = true;
            gripMemory.set(true);
            clawCmdDeg = stepAngle(clawCmdDeg, CLAW_CLOSED_ANGLE, CLAW_STEP_PER_TICK);
        }

        // --------------------------------------------------------
        //  5. PAYLOAD SERVO (S3) — Press Button A to Open / Close (90°)
        // --------------------------------------------------------
        if (gamepad1.justPressed(BTN_A)) {
            payloadOpen = !payloadOpen;
        }
        float payloadTarget = payloadOpen ? PAYLOAD_OPEN_ANGLE : PAYLOAD_CLOSED_ANGLE;
        payloadCmdDeg = stepAngle(payloadCmdDeg, payloadTarget, PAYLOAD_STEP_PER_TICK);

        // --------------------------------------------------------
        //  6. SEND CLEAN POSITION COMMANDS TO ALL 3 SERVOS
        // --------------------------------------------------------
        arm.setPosition(armCmdDeg);
        claw.setPosition(clawCmdDeg);
        payload.setPosition(payloadCmdDeg);

        // --------------------------------------------------------
        //  7. STATUS LED & LIVE TELEMETRY
        // --------------------------------------------------------
        if (!brownoutBeacon.service()) {
            if (power.isBatteryLow()) {
                userLed.setBlink(255, 0, 0, 300);
            } else if (autoLevelActive) {
                userLed.setSolid(0, 180, 255); // Cyan: Auto-Level Active
            } else if (clawGripLocked) {
                userLed.setSolid(180, 0, 255); // Purple: Element Gripped
            } else {
                userLed.setSolid(0, 255, 0);   // Green: Ready
            }
        }

        telemetry.addData("Tilt Deg",  relativeTiltDeg);
        telemetry.addData("Arm (S1)",  armCmdDeg);
        telemetry.addData("Claw (S4)", clawCmdDeg);
        telemetry.addData("Pay (S3)",  payloadCmdDeg);
        telemetry.addData("Payload",   payloadOpen ? "OPEN (90)" : "CLOSED (0)");
        team205ReportPower(telemetry, powerGuard);
    }

    void stop() override {
        cancelActions();
        drive.stop();
        gripMemory.set(false);
        turnActive      = false;
        autoLevelActive = false;

        arm.detach();
        claw.detach();
        payload.detach();
        userLed.setOff();
    }
};

REGISTER_OPMODE(DriveMode, "Drive Mode", TELEOP);
