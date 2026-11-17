#include "NRL.h"
#include "team205_mech.h"

// ============================================================
//  NormalDrive — Tank/Arcade Drive with Arm & Claw
// ============================================================
//  HARDWARE (2 servos, 2 separate mechanisms):
//      SERVO_1 -> ARM   (one axis: up / down)
//      SERVO_2 -> CLAW  (one axis: expand / contract)
//
// Controls:
//  - DRIVE MODE TOGGLE (BTN_X): TRUE DUAL-STICK TANK <-> SPLIT-ARCADE
//  - FULL STRAIGHT DRIVE:
//      * Hold BTN_RB: 100% full speed sprint with Gyro Heading Lock!
//  - 180° & 90° QUICK SPINS:
//      * BTN_LB          = Instant 180° gyro turnaround
//      * BTN_DPAD_LEFT   = Instant 90° left pivot
//      * BTN_DPAD_RIGHT  = Instant 90° right pivot
//  - ARM (hold to move, release to stop):
//      * BTN_DPAD_UP    = arm UP
//      * BTN_DPAD_DOWN  = arm DOWN
//  - CLAW (toggle on, travels to its limit; toggle off to stop):
//      * BTN_LT         = claw EXPAND
//      * BTN_RT         = claw CONTRACT
//  - CLAW SNAPS & MACRO:
//      * BTN_A          = snap fully closed (and engage the grip)
//      * BTN_Y          = one-touch auto-grab (open -> clamp + chime) / release
//
//  ------------------------------------------------------------------
//  SERVO ANGLE DIRECTION — MEASURED, DO NOT "FIX"
//  ------------------------------------------------------------------
//  Both servos are mounted so that a HIGHER angle rotates them in the
//  direction that CLOSES / LOWERS the mechanism:
//
//      ARM:    45 deg = raised (carry)   115 deg = lowered (floor)
//      CLAW:  125 deg = open             155 deg = closed
//
//  These angles come from the team's own calibration on the real robot
//  (Controller -> SERVO TEST). They are the source of truth for this robot.
// ============================================================

// Drive motors on the Hexa Command Hub
static HexaDCMotor leftMotor({ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true });
static HexaDCMotor rightMotor({ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = false });
static TankDrive   drive(leftMotor, rightMotor);
static DriveActions driveActions(drive);

// Which physical servo is the arm and which is the claw.
static constexpr uint8_t ARM_SIGNAL_PIN  = SERVO_1;  // GPIO 39
static constexpr uint8_t CLAW_SIGNAL_PIN = SERVO_2;  // GPIO 40

// ---- ARM: calibrated angles (measured on this robot) ---------------------
static constexpr float ARM_CARRY_ANGLE = 45.0f;   // raised / carry position
static constexpr float ARM_FLOOR_ANGLE = 115.0f;  // lowered to the floor
static constexpr float ARM_HOME_ANGLE  = ARM_CARRY_ANGLE;

// ---- CLAW: calibrated angles (measured on this robot) --------------------
static constexpr float CLAW_OPEN_ANGLE   = 125.0f;  // wide open
static constexpr float CLAW_CLOSED_ANGLE = 155.0f;  // jaws shut

// ------------------------------------------------------------------
//  GRIP PRESSURE
//
//  A servo applies force only while it is still trying to REACH its commanded
//  angle. Commanding exactly CLAW_CLOSED_ANGLE means the servo arrives,
//  stops pushing, and holds nothing — the piece is held by friction alone
//  and falls out as soon as the robot moves.
//
//  So once the jaws are shut on a piece, the servo is commanded PAST
//  CLAW_CLOSED_ANGLE to CLAW_GRIP_ANGLE. The piece blocks the jaws short of
//  that, so the servo never arrives, keeps straining, and presses.
//
//  PRESSURE TUNING: a LARGER CLAW_GRIP_ANGLE = a tighter squeeze = more
//  force. Raise it for a stronger grip; lower it if the servo starts buzzing
//  hard, bogging down, or the arm lags behind when it moves.
// ------------------------------------------------------------------
static constexpr float CLAW_GRIP_ANGLE = 168.0f;

// ---- Travel speeds, DEGREES PER SECOND ----------------------------------
// The servo ramps toward the target on its own, advancing one step per
// setPosition() call. The ramp rate is the single source of truth for
// motion — do NOT also step the target by hand, because two rate systems
// fight and the servo never converges.
static constexpr float ARM_RATE_DEG_PER_SEC  = 90.0f;   // arm travel speed
static constexpr float CLAW_RATE_DEG_PER_SEC = 120.0f;  // claw travel speed

// The two mechanisms.
static HexaServo arm ({ .signalPin = ARM_SIGNAL_PIN,  .startAngle = ARM_HOME_ANGLE });
static HexaServo claw({ .signalPin = CLAW_SIGNAL_PIN, .startAngle = CLAW_OPEN_ANGLE });

// The mechanism wrappers. See team205_mech.h for what these fix.
static ServoAxis armAxis;
static ServoAxis clawAxis;

// The claw's state machine (open / closing / gripping).
static ClawControl clawCtl;

// Which way the arm is being driven right now.
enum class ArmDir { NONE, UP, DOWN };

// What an in-flight macro owns. Macro::NONE means the manual controls have the
// parts back. A macro that has run to completion stops owning anything.
enum class Macro { NONE, TURN, GRAB };

static Macro  activeMacro        = Macro::NONE;
static ArmDir armDir             = ArmDir::NONE;
static bool   objectSecured      = false;
static bool   useTankDrive       = true;
static bool   fullStraightActive = false;
static float  lockedHeading      = 0.0f;

// Power / brownout state.
static PowerGuard      powerGuard;
static BrownoutBeacon brownoutBeacon;
static GripMemory      gripMemory;

// ------------------------------------------------------------
//  Raw button state — see the note in Drive Mode. gamepad1.pressed()
//  hides a button already held when the OpMode started; the raw bits do
//  not, so a control held from the start works immediately.
// ------------------------------------------------------------
static uint16_t rawButtons = 0;

static void readRawButtons() {
    float lx, ly, rx, ry;
    uint16_t btns = 0;
    NRLGamepad::read(lx, ly, rx, ry, btns);
    rawButtons = btns;
}

static bool rawPressed(uint16_t btn) { return (rawButtons & btn) != 0; }

// ------------------------------------------------------------
//  Auto-Grab Non-Blocking Action Routine
// ------------------------------------------------------------
static ActionPtr autoGrabRoutine() {
    return sequential({
        // 1. Clamp firmly on game piece immediately
        instant([]{
            userLed.setSolid(255, 120, 0); // Orange: Clamping
            clawCtl.tryClose();
        }),
        sleep_ms(300),

        // 2. Strain the servo past the jaws so it presses on the piece
        instant([]{
            clawCtl.engageNow();
            objectSecured = true;
            gripMemory.set(true);
        }),
        sleep_ms(80),

        // 3. Dual audio chime & set secured indicator
        instant([]{
            tone(BUZZER_PIN, 1800, 60);
        }),
        sleep_ms(60),
        instant([]{
            tone(BUZZER_PIN, 2400, 100);
            userLed.setSolid(180, 0, 255); // Purple: Object Secured
        })
    });
}

class NormalDrive : public NRLOpMode {
public:
    void init() override {
        arm.setRampRate(ARM_RATE_DEG_PER_SEC);
        claw.setRampRate(CLAW_RATE_DEG_PER_SEC);
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();

        armAxis.bind(arm, ARM_HOME_ANGLE, ARM_CARRY_ANGLE, ARM_FLOOR_ANGLE);
        clawAxis.bind(claw, CLAW_OPEN_ANGLE, CLAW_OPEN_ANGLE, CLAW_CLOSED_ANGLE);
        clawCtl.bind(clawAxis, CLAW_OPEN_ANGLE, CLAW_CLOSED_ANGLE, CLAW_GRIP_ANGLE);

        objectSecured      = false;
        activeMacro        = Macro::NONE;
        armDir             = ArmDir::NONE;
        fullStraightActive = false;

        // If the board came up from a brownout, restore the grip so the claw
        // comes back CLOSED on the piece instead of dropping it.
        gripMemory.begin();
        if (gripMemory.engaged) {
            clawCtl.engageNow();
            objectSecured = true;
        } else {
            clawAxis.command(CLAW_OPEN_ANGLE);
        }

        armAxis.command(ARM_HOME_ANGLE);
        armAxis.apply();
        clawAxis.apply();

        // Flash the LED red if this boot was a brownout reset.
        brownoutBeacon.arm();

        userLed.setSolid(0, 255, 0); // Green ready
    }

    void loop() override {
        readRawButtons();

        // Watch the battery rail. The scale factor backs the drive off when
        // the pack sags, which is what keeps "drive while gripping" from
        // resetting the board.
        const float driveScale = powerGuard.update();

        if (activeMacro != Macro::NONE && !isActionRunning()) {
            activeMacro = Macro::NONE;
        }

        // --------------------------------------------------------
        //  1. GYRO SPIN MACROS (LB = 180°, DPAD_LEFT/RIGHT = ±90°)
        // --------------------------------------------------------
        if (activeMacro != Macro::TURN) {
            if (gamepad1.justPressed(BTN_LB)) {
                runAction(driveActions.turn(180));
                activeMacro        = Macro::TURN;
                fullStraightActive = false;
            } else if (gamepad1.justPressed(BTN_DPAD_LEFT)) {
                runAction(driveActions.turn(-90));
                activeMacro        = Macro::TURN;
                fullStraightActive = false;
            } else if (gamepad1.justPressed(BTN_DPAD_RIGHT)) {
                runAction(driveActions.turn(90));
                activeMacro        = Macro::TURN;
                fullStraightActive = false;
            }
        }

        // --------------------------------------------------------
        //  2. DRIVE — tank, arcade, or the 100% heading-locked sprint.
        //     While a turn macro is running it owns the motors.
        // --------------------------------------------------------
        if (activeMacro == Macro::TURN) {
            // driveActions.turn() is driving — do not fight it for the motors.
        } else if (rawPressed(BTN_RB)) {
            if (!fullStraightActive) {
                lockedHeading = NRLComms::getHeading();
                fullStraightActive = true;
            }

            float headingError   = 0.0f;
            float turnCorrection = 0.0f;
            if (NRLComms::isHeadingReady()) {
                headingError   = lockedHeading - NRLComms::getHeading();
                turnCorrection = constrain(headingError * 0.04f, -0.3f, 0.3f);
            }

            drive.drive(1.0f * driveScale, turnCorrection);
            userLed.setSolid(255, 255, 255); // White LED: FULL SPRINT

            telemetry.addData("DRIVE", "FULL STRAIGHT 100%");
            telemetry.addData("Heading Err", headingError);
        } else {
            fullStraightActive = false;

            // --------------------------------------------------------
            //  3. DRIVE MODE — TRUE TANK vs SPLIT-ARCADE
            // --------------------------------------------------------
            if (gamepad1.justPressed(BTN_X)) {
                useTankDrive = !useTankDrive;
                tone(BUZZER_PIN, useTankDrive ? 1600 : 2000, 80);
            }

            const float DEADZONE = 0.05f;

            if (useTankDrive) {
                // TRUE TANK DRIVE: Left Stick Y = Left, Right Stick Y = Right
                float leftStick  = gamepad1.leftY();
                float rightStick = gamepad1.rightY();

                if (abs(leftStick)  < DEADZONE) leftStick  = 0.0f;
                if (abs(rightStick) < DEADZONE) rightStick = 0.0f;

                float forward = (leftStick + rightStick) * 0.5f;
                float turn    = (leftStick - rightStick) * 0.5f;
                drive.drive(forward * driveScale, turn);

                telemetry.addData("MODE", "TRUE TANK (2-Stick)");
            } else {
                // SPLIT-ARCADE DRIVE: Left Stick Y = Fwd, Right Stick X = Turn
                float forward = gamepad1.leftY();
                float turn    = gamepad1.rightX();

                if (abs(forward) < DEADZONE) forward = 0.0f;
                if (abs(turn)    < DEADZONE) turn    = 0.0f;
                drive.drive(forward * driveScale, turn);

                telemetry.addData("MODE", "ARCADE (Split-Stick)");
            }
        }

        // --------------------------------------------------------
        //  4a. ARM — D-PAD UP / DOWN
        // --------------------------------------------------------
        if (rawPressed(BTN_DPAD_UP)) {
            armDir = ArmDir::UP;
        } else if (rawPressed(BTN_DPAD_DOWN)) {
            armDir = ArmDir::DOWN;
        } else {
            armDir = ArmDir::NONE;
        }

        if (armDir == ArmDir::UP) {
            // UP = the LOWER angle on this robot (45 deg = carry).
            if (motionRequest(armAxis)) armAxis.command(ARM_CARRY_ANGLE);
        } else if (armDir == ArmDir::DOWN) {
            // DOWN = the HIGHER angle on this robot (115 deg = floor).
            if (motionRequest(armAxis)) armAxis.command(ARM_FLOOR_ANGLE);
        } else {
            motionRelease(armAxis);
        }

        // --------------------------------------------------------
        //  4b. CLAW — LT / RT TRIGGERS
        // --------------------------------------------------------
        static ClawDir triggerDir = ClawDir::NONE;
        static uint16_t prevRaw = 0;
        if ((rawButtons & BTN_LT) && !(prevRaw & BTN_LT)) {
            triggerDir = ClawDir::EXPAND;
        } else if ((rawButtons & BTN_RT) && !(prevRaw & BTN_RT)) {
            triggerDir = ClawDir::CONTRACT;
        }
        prevRaw = rawButtons;

        const bool ltEngaged = rawPressed(BTN_LT);
        const bool rtEngaged = rawPressed(BTN_RT);

        ClawDir desired = ClawDir::NONE;
        if (triggerDir == ClawDir::EXPAND && ltEngaged) {
            desired = ClawDir::EXPAND;
        } else if (triggerDir == ClawDir::CONTRACT && rtEngaged) {
            desired = ClawDir::CONTRACT;
        } else if (ltEngaged) {
            desired = ClawDir::EXPAND;
        } else if (rtEngaged) {
            desired = ClawDir::CONTRACT;
        }

        if (activeMacro != Macro::GRAB) {
            clawCtl.update(desired);

            // Tap BTN_Y to Toggle Grab / Release
            if (gamepad1.justPressed(BTN_Y)) {
                if (objectSecured) {
                    clawCtl.releaseNow();
                    clawCtl.tryOpen();
                    objectSecured = false;
                    gripMemory.set(false);
                    userLed.setSolid(0, 255, 0);
                    tone(BUZZER_PIN, 1200, 70);
                } else {
                    runAction(autoGrabRoutine());
                    activeMacro = Macro::GRAB;
                }
            }

            // BTN_A snaps the claw shut and engages the grip
            if (gamepad1.justPressed(BTN_A)) {
                clawCtl.engageNow();
                objectSecured = true;
                gripMemory.set(true);
            }
        }

        // --------------------------------------------------------
        //  5. PUSH THE TARGETS TO THE SERVOS
        //
        //  This MUST run every tick, and it MUST be the only thing that
        //  talks to the servos. It advances the ramp (the servo only
        //  travels while setPosition() is called) and refreshes the PWM
        //  duty every tick, so the servo keeps receiving pulses.
        // --------------------------------------------------------
        armAxis.apply();
        clawAxis.apply();

        // --------------------------------------------------------
        //  6. STATUS LED MANAGEMENT
        // --------------------------------------------------------
        if (brownoutBeacon.service()) {
            // The beacon owns the LED — the last boot was a brownout reset.
        } else if (power.isBatteryLow()) {
            userLed.setBlink(255, 0, 0, 300);

            // Beep at most twice a second. A tone() on every 10 ms tick is a
            // continuous buzz that drowns out the grab chime.
            static uint32_t lastBeepMs = 0;
            uint32_t nowMs = millis();
            if (nowMs - lastBeepMs >= 500) {
                lastBeepMs = nowMs;
                tone(BUZZER_PIN, 1000, 50);
            }
        } else if (activeMacro == Macro::GRAB) {
            // The grab macro owns the LED.
        } else if (fullStraightActive) {
            userLed.setSolid(255, 255, 255);
        } else if (objectSecured) {
            userLed.setSolid(180, 0, 255);
        } else {
            userLed.setSolid(0, 255, 0);
        }

        // Telemetry stream
        telemetry.addData("Arm Angle",  armAxis.actual());
        telemetry.addData("Claw Angle", clawAxis.actual());
        telemetry.addData("Grip",       clawCtl.isGripping() ? "PRESSING" : "loose");
        telemetry.addData("Object",     objectSecured ? "SECURED" : "NONE");
        telemetry.addData("Drive Sc",   driveScale);
        telemetry.addData("Heading",    NRLComms::getHeading());
        telemetry.addData("RAW",        rawButtons);
        team205ReportPower(telemetry, powerGuard);
    }

    void stop() override {
        cancelActions();
        drive.stop();

        // A clean stop means the match is over — clear the persisted grip so
        // the next run does not come up clamping onto nothing.
        gripMemory.set(false);

        arm.detach();
        claw.detach();
        userLed.setOff();
        noTone(BUZZER_PIN);
    }
};

REGISTER_OPMODE(NormalDrive, "Normal Drive", TELEOP);
