#include "NRL.h"
#include "team205_mech.h"
#include <memory>
#include <cmath>

// ============================================================================
//  TEAM 205 (PIXEL POKER) — NRL 2026 AUTONOMOUS SKILL CHALLENGE
//
//  This OpMode runs as an AUTO routine: queue the mission once in start(),
//  then leave loop() for telemetry only.
//
//  ---------------------------------------------------------------------------
//  LINE FOLLOWING — READ THIS FIRST
//  ---------------------------------------------------------------------------
//  The line-following stack (HexaLineArray / LineActions / LineTuning) ships
//  in NRL SDK 1.5.0. This project vendors SDK 1.4.1, where that driver does
//  not exist, so the mission below compiles in two modes:
//
//    NRL_HAS_LINE_API = 1  -> the real ARC-8 line follower is used, exactly
//                              as the mission planner expects.
//    NRL_HAS_LINE_API = 0  -> a BLIND fallback drives the route on the gyro
//                              and timed distances instead. The stage labels
//                              still work and the arm/claw/payload mechanics
//                              are identical, but the ROUTE DISTANCES ARE
//                              GUESSES — measure them on the mat and fill in
//                              the BLIND_* constants below.
//
//  The mode is auto-detected: drop the SDK 1.5.0 libraries into
//  RobotFirmware/lib/ and this file picks the line follower up with no edits.
//  To force a mode, add -DNRL_HAS_LINE_API=1 to build_flags in platformio.ini.
//
//  The controller does NOT need reflashing for any of this — all of it is
//  robot-side.
// ============================================================================

#if defined(__has_include)
#  if __has_include("HexaLineArray.h")
#    define NRL_HAS_LINE_API 1
#  endif
#endif
#ifndef NRL_HAS_LINE_API
#  define NRL_HAS_LINE_API 0
#endif

#if NRL_HAS_LINE_API
#  include "HexaLineArray.h"
#endif

// ============================================================================
//  SECTION 1: HARDWARE & TUNING CONFIGURATION (edit only this section)
// ============================================================================

// --- 1A. DRIVE MOTOR SETTINGS ----------------------------------------------
// Measure in Controller -> CALIBRATE -> MOTOR TEST.
constexpr bool  LEFT_MOTOR_FLIPPED   = true;
constexpr bool  RIGHT_MOTOR_FLIPPED  = false;

// Lowest PWM (0-255) where the wheel starts turning. Below this the motor
// just buzzes, which a closed loop reads as "no response" and answers by
// winding the command up further.
constexpr int   LEFT_MIN_DRIVE_PWM   = 29;
constexpr int   RIGHT_MIN_DRIVE_PWM  = 32;

// --- 1B. ARC-8 LINE SENSOR ORIENTATION ------------------------------------
// Set true ONLY if ch0 is on the robot's RIGHT.
constexpr bool  SENSOR_BAR_REVERSED  = false;

// --- 1C. SERVO PORTS & CALIBRATED ANGLES -----------------------------------
// Measured in Controller -> CALIBRATE -> SERVO TEST.
//
// ARM:    45 deg = raised (carry)   115 deg = lowered (floor)
// CLAW:  125 deg = open             155 deg = closed
// The arm goes UP by DECREASING its angle; the claw CLOSES by INCREASING it.
constexpr int   ARM_SERVO_PIN        = SERVO_1;
constexpr int   GRIPPER_SERVO_PIN    = SERVO_2;
constexpr int   PAYLOAD_SERVO_PIN    = SERVO_3;

constexpr float ARM_FLOOR_ANGLE      = 115.0f;
constexpr float ARM_CARRY_ANGLE      = 45.0f;
constexpr float GRIP_OPEN_ANGLE      = 125.0f;
constexpr float GRIP_CLOSED_ANGLE    = 155.0f;

// The angle the servo is COMMANDED to while gripping. It is PAST the closed
// angle on purpose: a servo only applies force while it is still trying to
// REACH its commanded angle. Commanding exactly GRIP_CLOSED_ANGLE means the
// servo arrives, stops pushing, and holds nothing — the piece is held by
// friction alone and falls out the moment the arm moves.
//
// Command past the closed angle and the piece blocks the jaws short of it, so
// the servo keeps straining and presses.
//
// PRESSURE TUNING: a LARGER value = a tighter squeeze = more force. Raise it
// for a stronger grip; lower it if the servo buzzes hard, bogs down, or the
// arm lags behind when it moves.
//
// ⚠ This is the biggest current draw on the board. See the brownout note
// below before raising it.
constexpr float GRIP_PRESS_ANGLE     = 168.0f;

// Payload gate — a third mechanism on its own servo.
constexpr float PAYLOAD_CLOSED_ANGLE = 0.0f;
constexpr float PAYLOAD_OPEN_ANGLE   = 90.0f;

// Travel speeds, in DEGREES PER SECOND.
//
// The SERVO's ramp rate is the single source of truth for motion. Do not
// also step the target angle by hand each tick — two rate systems fight and
// the servo never converges.
//
// These are also the current limit for each mechanism. The servos share the
// battery rail with NO BEC, so a faster rate makes the rail dip harder. If
// the board browns out (see the "Boot #" counter) when two servos move at
// once, slow these down.
constexpr float ARM_RAMP_DEG_PER_S   = 90.0f;
constexpr float GRIP_RAMP_DEG_PER_S  = 120.0f;
constexpr float PAYLOAD_RAMP_DEG_PER_S = 180.0f;

// Extra dwell time (ms) after a servo reaches its target angle.
constexpr uint32_t SERVO_SETTLE_MS   = 250;

// --- 1D. MISSION OPTIONS ----------------------------------------------------

// Set true if the robot starts the match with the first element already in
// the claw. The claw then presses on it from the very first tick.
constexpr bool  PRELOAD_FIRST_ELEMENT = true;

// Set true if your event awards bonus points for parking back in the box.
constexpr bool  RETURN_TO_START_BOX  = false;

// Set true to open the payload gate when placing an element at a T-bar.
constexpr bool  PAYLOAD_OPEN_ON_PLACE = false;

// --- 1E. DISTANCE / SPEED TUNING --------------------------------------------
// Milliseconds to travel one inch at full drive speed. This is the single
// most important number for the BLIND fallback — recalibrate by driving a
// known time at driveSpeed and measuring the distance travelled.
static constexpr float MS_PER_INCH = 18.0f;

// BLIND-ONLY route distances, in INCHES. These are placeholders until they
// are measured on the competition mat.
//
//   START -> J1 -> T1 is the first leg. From J1 the robot turns left onto
//   the J1 -> J2 branch, then right onto the stem to T2.
constexpr float BLIND_START_EXIT_INCHES = 12.0f;  // out of the start box
constexpr float BLIND_J1_INCHES         = 24.0f;  // start box -> J1
constexpr float BLIND_J1_T1_INCHES      = 18.0f;  // J1 -> T1
constexpr float BLIND_J1_J2_INCHES      = 40.0f;  // J1 -> J2
constexpr float BLIND_J2_T2_INCHES      = 20.0f;  // J2 -> T2

// --- 1F. PID LINE FOLLOWING & TURN TUNING -----------------------------------
// Only used when NRL_HAS_LINE_API == 1.
#if NRL_HAS_LINE_API
static LineTuning buildLineTuning() {
    LineTuning t;  // starts with official NRL SDK defaults

    // Line Tracking PID (P + D; I is always 0 for line following)
    t.pid.kP     = 0.50f;  // raise if drifting wide on bends; lower if zig-zagging
    t.pid.kI     = 0.00f;
    t.pid.kD     = 0.04f;  // raise if wobbling after turns; lower if twitching
    t.pid.dAlpha = 0.00f;  // try 0.5 if sensor readings twitch

    // Speed profile (0.0 to 1.0)
    t.followSpeed   = 0.35f;
    t.junctionSpeed = 0.25f;
    t.speedDrop     = 0.70f;
    t.minSpeedFrac  = 0.25f;
    t.rampRate      = 0.25f;

    // Junction & corner detection
    t.eventThreshold = 500;
    t.confirmTicks   = 2;
    t.brakeMs        = 80;
    t.brakeSpeed     = 0.30f;

    // Turn & pivot geometry (sensor bar is 86mm ahead of the chassis pivot)
    t.pivotMs       = 60;
    t.pivotSpeed    = 0.25f;
    t.turnSpeed     = 0.45f;
    t.turnSlowSpeed = 0.30f;
    t.turnBrakeMs   = 80;
    t.turnTimeoutMs = 3000;

    // IMU gyro & grid squaring
    t.useGyro       = true;
    t.gyroEarlyDeg  = 30;
    t.useGrid       = true;
    t.squareTolDeg  = 6;
    t.squareDoneDeg = 2;

    // Start / finish box
    t.startSpeed     = 0.35f;
    t.startTimeoutMs = 4000;
    t.parkMs         = 300;
    t.parkSpeed      = 0.25f;

    return t;
}
#endif

// ============================================================================
//  SECTION 2: HARDWARE & STATIC STATE (must be outside the class)
// ============================================================================

static HexaDCMotor leftDriveMotor {{
    .dirPin   = MOTOR_L_DIR,
    .pwmPin   = MOTOR_L_PWM,
    .flipped  = LEFT_MOTOR_FLIPPED,
    .minDrive = LEFT_MIN_DRIVE_PWM
}};

static HexaDCMotor rightDriveMotor {{
    .dirPin   = MOTOR_R_DIR,
    .pwmPin   = MOTOR_R_PWM,
    .flipped  = RIGHT_MOTOR_FLIPPED,
    .minDrive = RIGHT_MIN_DRIVE_PWM
}};

static TankDrive drive(leftDriveMotor, rightDriveMotor);

// Distance tuning for the BLIND fallback. DriveActions owns straight-line
// distance on this hardware (there are no wheel encoders).
static DriveTuning blindTuning() {
    DriveTuning t;
    t.msPerInch = MS_PER_INCH;
    return t;
}

static DriveActions driveActions(drive, blindTuning());

#if NRL_HAS_LINE_API
static HexaLineArrayConfig createLineConfig() {
    HexaLineArrayConfig cfg = hexaLineConfig();
    cfg.reversed = SENSOR_BAR_REVERSED;
    return cfg;
}

static HexaLineArray line{ createLineConfig() };
static LineActions   lineActions(drive, line, buildLineTuning());
#endif

// Servos. The arm starts raised in carry position; the claw starts closed
// holding the pre-loaded first element.
static HexaServo armServo    {{ .signalPin = ARM_SERVO_PIN,     .startAngle = ARM_CARRY_ANGLE }};
static HexaServo gripperServo{{ .signalPin = GRIPPER_SERVO_PIN, .startAngle = GRIP_CLOSED_ANGLE }};
static HexaServo payloadServo{{ .signalPin = PAYLOAD_SERVO_PIN, .startAngle = PAYLOAD_CLOSED_ANGLE }};

// The mechanism wrappers. See team205_mech.h for what these fix: the target
// lives here, and apply() pushes it to the driver every tick.
static ServoAxis armAxis;
static ServoAxis gripperAxis;
static ServoAxis payloadAxis;

// Route memory & live telemetry
static int  junctionJ1Mark = -1;
static const char* activeStageLabel = "INIT_READY";

// Grip state — whether the claw is straining past its closed angle.
static bool gripEngaged = false;

// Power / brownout state.
static PowerGuard      powerGuard;
static BrownoutBeacon brownoutBeacon;
static GripMemory      gripMemory;

// ============================================================================
//  SECTION 3: SERVO & MECHANISM HELPERS
// ============================================================================

// Update the Controller telemetry label at the start of each step.
static ActionPtr logStage(const char* label) {
    return instant([label] { activeStageLabel = label; });
}

// Engage the grip: the servo is commanded PAST the closed angle so it
// strains against the piece and presses.
//
// The press is re-asserted every tick by loop() (see applyAll()), so the
// squeeze never fades out the way a one-shot command does.
static void engageGrip() {
    gripperAxis.press(GRIP_PRESS_ANGLE);
    gripEngaged = true;
    gripMemory.set(true);
}

// Release the grip: the servo lets go and returns to holding the jaw angle.
static void releaseGrip() {
    gripperAxis.stopPressing();
    gripEngaged = false;
    gripMemory.set(false);
}

// Push every mechanism's target to its driver. Call this EVERY tick.
//
// This does two jobs that both matter on the field:
//   1. It advances the ramp. The servo only travels while setPosition() is
//      being called — the driver's own 250 ms heartbeat never moves it.
//   2. It refreshes the PWM duty every tick. A servo refreshed only every
//      250 ms can drop its pulses when the rail sags, which is what "the
//      servo just stopped working" looks like.
static void applyAll() {
    armAxis.apply();
    gripperAxis.apply();
    payloadAxis.apply();
}

// Move a mechanism to `targetAngle` and wait `dwellMs` after it arrives.
//
// The action only sets the TARGET. loop() does the driving, so the ramp
// advances continuously and the PWM stays refreshed while the move runs.
static ActionPtr moveServoSmooth(ServoAxis& axis, float targetAngle, uint32_t dwellMs) {
    auto arrivedTimestamp = std::make_shared<uint32_t>(0);
    return sequential({
        instant([&axis, targetAngle, arrivedTimestamp] {
            axis.command(targetAngle);
            *arrivedTimestamp = 0;
        }),
        wait_until([&axis, dwellMs, arrivedTimestamp] {
            if (*arrivedTimestamp == 0) {
                // travelArrived() compares against the TRAVEL target, so it
                // still reports "arrived" while the claw is pressing past it.
                if (axis.travelArrived(0.5f)) {
                    *arrivedTimestamp = millis() | 1;   // non-zero arrival stamp
                }
                return false;
            }
            return (millis() - *arrivedTimestamp) >= dwellMs;
        })
    });
}

// PICK UP an element at a T-bar:
//   open claw -> lower arm -> close claw -> PRESS -> raise arm
static ActionPtr executePickSequence() {
    return sequential({
        logStage("PICK: OPEN CLAW"),
        moveServoSmooth(gripperAxis, GRIP_OPEN_ANGLE,  200),

        logStage("PICK: ARM DOWN"),
        moveServoSmooth(armAxis,     ARM_FLOOR_ANGLE,  SERVO_SETTLE_MS),

        logStage("PICK: GRAB ELEMENT"),
        moveServoSmooth(gripperAxis, GRIP_CLOSED_ANGLE, SERVO_SETTLE_MS),

        // Strain the servo past the jaws so it actually presses on the
        // element. Without this the claw holds nothing and the element falls
        // out the moment the arm moves.
        logStage("PICK: PRESS"),
        instant([]{ engageGrip(); }),
        sleep_ms(150),

        logStage("PICK: ARM UP"),
        moveServoSmooth(armAxis,     ARM_CARRY_ANGLE,  200)
    });
}

// PLACE an element at a T-bar:
//   lower arm -> open claw -> (optional payload gate) -> raise arm
static ActionPtr executePlaceSequence() {
    return sequential({
        logStage("PLACE: ARM DOWN"),
        moveServoSmooth(armAxis,     ARM_FLOOR_ANGLE,  SERVO_SETTLE_MS),

        logStage("PLACE: RELEASE"),
        instant([]{ releaseGrip(); }),
        moveServoSmooth(gripperAxis, GRIP_OPEN_ANGLE,  SERVO_SETTLE_MS),

        // Open the payload gate at the T-bar if the mission wants that.
        instant([]{
            if (PAYLOAD_OPEN_ON_PLACE) {
                payloadAxis.command(PAYLOAD_OPEN_ANGLE);
            }
        }),

        logStage("PLACE: ARM UP"),
        moveServoSmooth(armAxis,     ARM_CARRY_ANGLE,  200)
    });
}

// ============================================================================
//  SECTION 4: THE AUTONOMOUS MISSION
// ============================================================================

#if NRL_HAS_LINE_API

// ---------------------------------------------------------------------------
//  FULL LINE-FOLLOWING MISSION
//
//  Every turn (left/right) is written from the ROBOT'S perspective at that
//  moment. If any step hits a fault, the remaining steps are skipped by the
//  SDK's own fault handling.
//
//  NOTE: this branch uses the SDK 1.5.0 builder API, which is why it only
//  compiles when the 1.5.0 line-following libraries are present.
// ---------------------------------------------------------------------------
static ActionPtr buildAutonomousMission() {
    auto plan = lineActions.auto_()
        // Phase 1: Exit start box & deliver the pre-loaded element to T1
        .then(logStage("01. LEAVE START"))   .leaveStart()
        .then(logStage("02. DRIVE TO J1"))   .followToJunction()
        .markHere(&junctionJ1Mark)
        .then(logStage("03. DRIVE TO T1"))   .followToEnd()
        .then(executePlaceSequence())

        // Phase 2: Navigate from T1 -> J1 -> J2 -> T2
        .then(logStage("04. U-TURN AT T1"))  .uTurn()
        .then(logStage("05. RETURN TO J1"))  .retraceTo(&junctionJ1Mark)
        .then(logStage("06. TURN LEFT J1"))  .left()
        .then(logStage("07. DRIVE TO J2"))   .followToJunction()
        .then(logStage("08. TURN RIGHT J2")) .right()
        .then(logStage("09. DRIVE TO T2"))   .followToEnd()

        // Phase 3: Pick up the 2nd element at T2
        .then(executePickSequence())

        // Phase 4: Return via route memory & deliver the 2nd element to T1
        .then(logStage("10. U-TURN AT T2"))  .uTurn()
        .then(logStage("11. RETRACE TO J1")) .retraceTo(&junctionJ1Mark)
        .then(logStage("12. TURN RIGHT J1")) .right()
        .then(logStage("13. DRIVE TO T1"))   .followToEnd()
        .then(executePlaceSequence());

    // Optional Phase 5: Return to the start / finish grey box
    if (RETURN_TO_START_BOX) {
        plan.then(logStage("14. U-TURN T1"))    .uTurn()
            .then(logStage("15. RETRACE HOME")).retraceHome()
            .then(logStage("16. FIND GREY"))    .followToGrey()
            .then(logStage("17. PARK IN BOX"))  .park();
    }

    return plan.then(logStage("18. MISSION COMPLETE")).build();
}

#else

// ---------------------------------------------------------------------------
//  BLIND FALLBACK MISSION (SDK 1.4.1 — no line sensor driver)
//
//  The same sequence of STAGES as the real mission, but each leg is driven
//  blind on the gyro and a measured distance instead of by following a line.
//
//  ⚠ THE DISTANCES ARE PLACEHOLDERS. Measure them on the mat and put the
//  real numbers into the BLIND_* constants in Section 1 before running this
//  in competition. A wrong distance here means the robot drives into things.
//
//  The arm/claw/payload behaviour is identical to the line-following version.
//
//  This branch uses sequential() rather than the DriveActions builder,
//  because the 1.4.1 builder has no .then() — it only exposes the drive
//  primitives. Stage labels match the line-following mission exactly, so the
//  telemetry reads the same either way.
// ---------------------------------------------------------------------------
static ActionPtr buildAutonomousMission() {
    return sequential({
        // Phase 1: Exit start box & deliver the pre-loaded element to T1
        logStage("01. LEAVE START"),
        driveActions.waitForHeadingReady(),
        driveActions.driveForMs((unsigned long)(BLIND_START_EXIT_INCHES * MS_PER_INCH)),

        logStage("02. DRIVE TO J1"),
        driveActions.driveInches(BLIND_J1_INCHES),

        logStage("03. DRIVE TO T1"),
        driveActions.driveInches(BLIND_J1_T1_INCHES),
        executePlaceSequence(),

        // Phase 2: Navigate from T1 back to J1, then along the J1 -> J2 branch
        logStage("04. U-TURN AT T1"),
        driveActions.turn(180),

        logStage("05. RETURN TO J1"),
        driveActions.driveInches(BLIND_J1_T1_INCHES),

        logStage("06. TURN LEFT J1"),
        driveActions.turn(-90),

        logStage("07. DRIVE TO J2"),
        driveActions.driveInches(BLIND_J1_J2_INCHES),

        logStage("08. TURN RIGHT J2"),
        driveActions.turn(90),

        logStage("09. DRIVE TO T2"),
        driveActions.driveInches(BLIND_J2_T2_INCHES),

        // Phase 3: Pick up the 2nd element at T2
        executePickSequence(),

        // Phase 4: Return and deliver the 2nd element to T1
        logStage("10. U-TURN AT T2"),
        driveActions.turn(180),

        logStage("11. RETRACE TO J1"),
        driveActions.driveInches(BLIND_J2_T2_INCHES + BLIND_J1_J2_INCHES),

        logStage("12. TURN RIGHT J1"),
        driveActions.turn(90),

        logStage("13. DRIVE TO T1"),
        driveActions.driveInches(BLIND_J1_T1_INCHES),
        executePlaceSequence()
    });
}

#endif  // NRL_HAS_LINE_API

// ============================================================================
//  SECTION 5: OPMODE LIFECYCLE
// ============================================================================

class PixelPokerAuto : public NRLOpMode {
public:
    // Runs once on INIT. Initialise hardware without blocking.
    void init() override {
        leftDriveMotor.begin();
        rightDriveMotor.begin();

#if NRL_HAS_LINE_API
        line.begin();
#endif

        // setRampRate() must be called before the first setPosition() for the
        // ramp to apply from the very start.
        armServo.setRampRate(ARM_RAMP_DEG_PER_S);
        gripperServo.setRampRate(GRIP_RAMP_DEG_PER_S);
        payloadServo.setRampRate(PAYLOAD_RAMP_DEG_PER_S);

        armServo.begin();
        gripperServo.begin();
        payloadServo.begin();

        armAxis.bind(armServo,     ARM_CARRY_ANGLE,     ARM_CARRY_ANGLE, ARM_FLOOR_ANGLE);
        gripperAxis.bind(gripperServo, GRIP_CLOSED_ANGLE, GRIP_OPEN_ANGLE, GRIP_CLOSED_ANGLE);
        payloadAxis.bind(payloadServo, PAYLOAD_CLOSED_ANGLE, PAYLOAD_CLOSED_ANGLE, PAYLOAD_OPEN_ANGLE);

        junctionJ1Mark = -1;
        activeStageLabel = "READY (PRESS START)";

        // If the board came up from a brownout, restore the grip so the claw
        // comes back CLOSED on the piece instead of dropping it.
        gripMemory.begin();
        if (gripMemory.engaged) {
            gripperAxis.command(GRIP_CLOSED_ANGLE);
            gripperAxis.press(GRIP_PRESS_ANGLE);
            gripEngaged = true;
        } else {
            gripperAxis.command(GRIP_CLOSED_ANGLE);
            if (PRELOAD_FIRST_ELEMENT) {
                // The first element is already in the jaws — press on it so
                // it cannot fall out when the robot starts moving.
                gripperAxis.press(GRIP_PRESS_ANGLE);
                gripEngaged = true;
                gripMemory.set(true);
            }
        }

        armAxis.command(ARM_CARRY_ANGLE);
        payloadAxis.command(PAYLOAD_CLOSED_ANGLE);
        applyAll();

        // Flash the LED red if this boot was a brownout reset.
        brownoutBeacon.arm();
    }

    // Runs once when the AUTO match starts. Queue the mission here — it runs
    // exactly once, instead of restarting 100 times a second.
    void start() override {
#if NRL_HAS_LINE_API
        lineActions.clearFault();
        lineActions.resetCounts();
        lineActions.clearRoute();
#endif

        activeStageLabel = "STARTING MISSION";
        runAction(buildAutonomousMission());
    }

    // Runs at 100 Hz. The mission owns the mechanisms' TARGETS; loop() owns
    // the actual servo commands.
    void loop() override {
        // Watch the battery rail. The scale factor backs the drive off when
        // the pack sags, which is what keeps "drive while gripping" from
        // resetting the board.
        powerGuard.update();

        // Push every target to its driver. This is what makes the mechanisms
        // move (the ramp only advances inside setPosition) and what keeps
        // their PWM alive every tick.
        applyAll();

        telemetry.addData("STAGE", activeStageLabel);
        telemetry.addData("Arm",     armAxis.actual());
        telemetry.addData("Grip",    gripperAxis.actual());
        telemetry.addData("Gripping", gripEngaged ? "PRESSING" : "loose");
        telemetry.addData("Payload", payloadAxis.actual());
        telemetry.addData("Heading", NRLComms::getHeading());
        team205ReportPower(telemetry, powerGuard);

#if NRL_HAS_LINE_API
        const LineJunction& j = lineActions.lastJunction();
        char junctionShape[4] = {
            j.left     ? 'L' : '-',
            j.right    ? 'R' : '-',
            j.straight ? 'S' : '-',
            '\0'
        };

        telemetry.addData("FAULT", lineActions.faultName());
        telemetry.addData("JX",    (float)lineActions.junctionCount());
        telemetry.addData("CNR",   (float)lineActions.cornerCount());
        telemetry.addData("TURN",  lineActions.lastTurnDegrees());
        telemetry.addData("LAST",  junctionShape);
        telemetry.addData("ERR",   (float)line.error());
#else
        telemetry.addData("MODE",   "BLIND (no line sensor)");
#endif
    }

    // Cancel every queued action and stop the motors on STOP.
    void stop() override {
        cancelActions();
        drive.stop();

        // A clean stop means the match is over — clear the persisted grip so
        // the next run does not come up clamping onto nothing. (A BROWNOUT
        // never reaches here, which is exactly why the memory is useful.)
        gripMemory.set(false);

        armServo.detach();
        gripperServo.detach();
        payloadServo.detach();
        userLed.setOff();
        noTone(BUZZER_PIN);
    }
};

REGISTER_OPMODE(PixelPokerAuto, "205 PixelPoker Auto", AUTO);
