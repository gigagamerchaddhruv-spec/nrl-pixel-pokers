#include "NRL.h"
#include "team205_mech.h"

// ============================================================
//  DriveMode — Dual-Stick Tank Drive, Arm, Claw & Payload Gate
// ============================================================
//  HARDWARE (3 servos, 3 separate mechanisms):
//      SERVO_1 -> ARM           (one axis: up / down)
//      SERVO_2 -> CLAW          (one axis: expand / contract)
//      SERVO_3 -> PAYLOAD GATE  (one axis: open / closed)
//
//  Controls:
//  - TRUE TANK DRIVE:
//      * Left stick Y  = Left Motor directly
//      * Right stick Y = Right Motor directly
//  - ARM (hold to move, release to stop):
//      * BTN_DPAD_UP    = arm UP
//      * BTN_DPAD_DOWN  = arm DOWN
//  - CLAW (toggle on, travels to its limit; toggle off to stop):
//      * BTN_LT         = claw EXPAND
//      * BTN_RT         = claw CONTRACT
//  - CLAW SNAPS & MACRO:
//      * BTN_A          = snap fully closed (and engage the grip)
//      * BTN_Y          = one-touch auto-grab (open -> clamp + chime) / release
//  - PAYLOAD GATE (press once to toggle):
//      * BTN_X          = toggle the payload gate OPEN / CLOSED
//  - FULL STRAIGHT DRIVE:
//      * Hold BTN_RB: 100% full speed sprint with Gyro Heading Lock!
//  - GYRO SPIN MACROS:
//      * BTN_LB          = Instant 180° gyro turnaround
//      * BTN_DPAD_LEFT   = Instant 90° left pivot
//      * BTN_DPAD_RIGHT  = Instant 90° right pivot
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
//  So the arm goes UP by DECREASING its angle, and the claw CLOSES by
//  INCREASING its angle. The direction flags below are written to match that,
//  and each one carries a comment saying which way the angle moves.
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
static constexpr uint8_t PAYLOAD_SIGNAL_PIN = SERVO_3;  // GPIO 41

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
//  angle. If the claw is commanded to exactly CLAW_CLOSED_ANGLE, the servo
//  arrives, stops pushing, and holds nothing — the piece is then held by
//  friction alone and falls out the moment the robot moves.
//
//  So once the jaws are shut on a piece, the servo is commanded PAST
//  CLAW_CLOSED_ANGLE to CLAW_GRIP_ANGLE. The piece blocks the jaws short of
//  that, so the servo never arrives, keeps straining, and presses.
//
//  PRESSURE TUNING: a LARGER CLAW_GRIP_ANGLE = a tighter squeeze = more
//  force. Raise it for a stronger grip; lower it if the servo starts buzzing
//  hard, bogging down, or the arm lags behind when it moves.
//
//  ⚠ This is also the biggest current draw on the board. See the brownout
//  note below before raising it.
// ------------------------------------------------------------------
static constexpr float CLAW_GRIP_ANGLE = 168.0f;

// ---- PAYLOAD GATE: angles and button ------------------------------------
// The payload gate is a third mechanism on its own servo. CLOSED is its
// starting pose; OPEN swings it 90 degrees clear.
//
// These two angles are the ones from the team's payload-toggle design. Unlike
// the arm and the claw, they were NOT measured on this robot — measure them
// in Controller -> SERVO TEST and correct them here if the gate does not
// travel the way you expect.
static constexpr float PAYLOAD_CLOSED_ANGLE = 0.0f;
static constexpr float PAYLOAD_OPEN_ANGLE   = 90.0f;

// Which button toggles the payload gate. Change this one line to move it.
static constexpr uint16_t PAYLOAD_BUTTON = BTN_X;

// ------------------------------------------------------------------
//  TRAVEL SPEEDS — DEGREES PER SECOND
//
//  The servo ramps toward the target on its own, advancing one step per
//  setPosition() call. The ramp rate is the single source of truth for
//  motion — do NOT also step the target angle by hand, because two rate
//  systems fight and the servo never converges.
//
//  These are also the current limit for the mechanism: a faster rate makes
//  the battery rail dip harder (see the brownout note below).
// ------------------------------------------------------------------
static constexpr float ARM_RATE_DEG_PER_SEC  = 90.0f;   // arm travel speed
static constexpr float CLAW_RATE_DEG_PER_SEC = 120.0f;  // claw travel speed
static constexpr float PAYLOAD_RATE_DEG_PER_SEC = 180.0f; // gate travel speed

// ------------------------------------------------------------------
//  BROWNOUT — why two servos moving at once can stop them working
//
//  "The servos share the battery rail. There is NO BEC and no separate servo
//  regulator." (HexaServo.h) Moving the arm AND the claw at the same time
//  doubles the current the rail must supply. If that pulls the rail down
//  far enough, the ESP32 browns out and resets.
//
//  A reset is not a small inconvenience: init() runs again, and the claw is
//  re-initialised OPEN. That is the "servo automatically opens" symptom —
//  and the piece you were carrying is on the floor.
//
//  Four things here stop that from happening:
//
//    1. MOTION ARBITRATION (see team205_mech.h). Only one mechanism TRAVELS
//       at a time. A deferred move is not lost — it starts as soon as the
//       rail is quiet again. This is the big one.
//
//    2. POWER DERATE (PowerGuard). When the pack sags, the drive output is
//       scaled back so the robot crawls instead of pulling the rail under.
//       This is what fixes "drive while gripping kills the claw".
//
//    3. GRIP MEMORY (GripMemory). If a reset does happen, the claw comes back
//       CLOSED on the piece instead of open.
//
//    4. Modest travel rates. Raising them makes the rail dip harder.
//
//  To confirm a brownout on the field: watch "Boot #" on the Controller. If
//  it climbs while you drive, the board is resetting.
// ------------------------------------------------------------------

// The three mechanisms.
static HexaServo arm    ({ .signalPin = ARM_SIGNAL_PIN,     .startAngle = ARM_HOME_ANGLE });
static HexaServo claw   ({ .signalPin = CLAW_SIGNAL_PIN,    .startAngle = CLAW_OPEN_ANGLE });
static HexaServo payload({ .signalPin = PAYLOAD_SIGNAL_PIN, .startAngle = PAYLOAD_CLOSED_ANGLE });

// The mechanism wrappers. See team205_mech.h for what these fix.
static ServoAxis  armAxis;
static ServoAxis  clawAxis;
static ServoAxis  payloadAxis;

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
static bool   fullStraightActive = false;
static float  lockedHeading      = 0.0f;
static bool   payloadOpen        = false;    // payload gate state

// Power / brownout state.
static PowerGuard      powerGuard;
static BrownoutBeacon brownoutBeacon;
static GripMemory      gripMemory;

// ------------------------------------------------------------
//  Raw button state
//
//  gamepad1.pressed() hides a button that was already held when the OpMode
//  started, until it is released once (Gamepad::ignoreHeldButtons). That is
//  right for menus, but it means a trigger the driver is ALREADY holding when
//  the OpMode starts does nothing — which reads as "the button doesn't work".
//
//  NRLGamepad::read() gives the raw packet contents with no hiding, so a
//  held control works immediately. The arm and claw read through it for that
//  reason; the discrete macros still use gamepad1.justPressed() because they
//  need a clean edge.
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

class DriveMode : public NRLOpMode {
public:
    void init() override {
        // Servos first, then drive motors
        arm.setRampRate(ARM_RATE_DEG_PER_SEC);
        claw.setRampRate(CLAW_RATE_DEG_PER_SEC);
        payload.setRampRate(PAYLOAD_RATE_DEG_PER_SEC);
        arm.begin();
        claw.begin();
        payload.begin();
        leftMotor.begin();
        rightMotor.begin();

        // Bind the mechanism wrappers to the drivers. The wrapper holds the
        // TARGET; apply() pushes it to the driver every tick.
        armAxis.bind(arm, ARM_HOME_ANGLE, ARM_CARRY_ANGLE, ARM_FLOOR_ANGLE);
        clawAxis.bind(claw, CLAW_OPEN_ANGLE, CLAW_OPEN_ANGLE, CLAW_CLOSED_ANGLE);
        payloadAxis.bind(payload, PAYLOAD_CLOSED_ANGLE, PAYLOAD_CLOSED_ANGLE, PAYLOAD_OPEN_ANGLE);
        clawCtl.bind(clawAxis, CLAW_OPEN_ANGLE, CLAW_CLOSED_ANGLE, CLAW_GRIP_ANGLE);

        objectSecured      = false;
        activeMacro        = Macro::NONE;
        armDir             = ArmDir::NONE;
        fullStraightActive = false;
        payloadOpen        = false;

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
        payloadAxis.command(PAYLOAD_CLOSED_ANGLE);

        armAxis.apply();
        clawAxis.apply();
        payloadAxis.apply();

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

        // A macro that has run to completion gives its part back to the
        // manual controls. Checked before anything reads activeMacro.
        if (activeMacro != Macro::NONE && !isActionRunning()) {
            activeMacro = Macro::NONE;
        }

        // --------------------------------------------------------
        //  1. GYRO SPIN MACROS (LB = 180°, DPAD_LEFT/RIGHT = ±90°)
        // --------------------------------------------------------
        // Guarded so a second press cannot stack two turns on the queue.
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
        //  2. DRIVE — tank, or the 100% heading-locked sprint.
        //     While a turn macro is running it owns the motors.
        // --------------------------------------------------------
        if (activeMacro == Macro::TURN) {
            // driveActions.turn() is driving — do not fight it for the motors.
        } else if (rawPressed(BTN_RB)) {
            if (!fullStraightActive) {
                lockedHeading = NRLComms::getHeading();
                fullStraightActive = true;
            }

            // Gyro heading lock. The heading is meaningless until the gyro has
            // finished its bias calibration (~0.5 s), so drive straight until
            // then rather than steering on a garbage reference.
            float headingError   = 0.0f;
            float turnCorrection = 0.0f;
            if (NRLComms::isHeadingReady()) {
                headingError   = lockedHeading - NRLComms::getHeading();
                turnCorrection = constrain(headingError * 0.04f, -0.3f, 0.3f);
            }

            // Command 100% full forward speed
            drive.drive(1.0f * driveScale, turnCorrection);

            telemetry.addData("DRIVE", "FULL STRAIGHT 100%");
            telemetry.addData("Heading Err", headingError);
        } else {
            fullStraightActive = false;

            // --------------------------------------------------------
            //  3. TRUE DUAL-STICK TANK DRIVE
            // --------------------------------------------------------
            // Left Stick Y controls Left Wheels, Right Stick Y controls Right Wheels
            float leftStick  = gamepad1.leftY();
            float rightStick = gamepad1.rightY();

            const float DEADZONE = 0.05f;
            if (abs(leftStick)  < DEADZONE) leftStick  = 0.0f;
            if (abs(rightStick) < DEADZONE) rightStick = 0.0f;

            // Kinematic conversion:
            // Left  = forward + turn = leftStick
            // Right = forward - turn = rightStick
            float forward = (leftStick + rightStick) * 0.5f;
            float turn    = (leftStick - rightStick) * 0.5f;

            drive.drive(forward * driveScale, turn);

            telemetry.addData("DRIVE", "DUAL TANK");
            telemetry.addData("Left Stick",  leftStick);
            telemetry.addData("Right Stick", rightStick);
        }

        // --------------------------------------------------------
        //  4a. ARM — D-PAD UP / DOWN
        //
        //  The D-pad is a set of momentary buttons, read as a LEVEL: hold UP
        //  and the arm rises, release and it stops where it is. It travels at
        //  ARM_RATE_DEG_PER_SEC and stops on its own at the calibrated poses.
        //
        //  Read from the RAW button state, so a D-pad button already held
        //  when the OpMode starts works immediately instead of doing nothing
        //  until it is released and pressed again.
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
            // Nothing asked for the arm. Let go of the rail.
            motionRelease(armAxis);
        }

        // --------------------------------------------------------
        //  4b. CLAW — LT / RT TRIGGERS
        //
        //  LT and RT are TOGGLE switches on this controller, read as a LEVEL:
        //  flip one on and the claw travels that way, flip it off and the
        //  claw stops where it is.
        //
        //  Which direction wins when BOTH are engaged: the one flipped most
        //  recently. `triggerDir` remembers that; the level checks below
        //  decide whether the remembered direction is still live.
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
            desired = ClawDir::EXPAND;          // LT still on, still expanding
        } else if (triggerDir == ClawDir::CONTRACT && rtEngaged) {
            desired = ClawDir::CONTRACT;        // RT still on, still contracting
        } else if (ltEngaged) {
            desired = ClawDir::EXPAND;          // LT on again after RT released
        } else if (rtEngaged) {
            desired = ClawDir::CONTRACT;        // RT on again after LT released
        }

        // While the auto-grab macro is running it owns the claw.
        if (activeMacro != Macro::GRAB) {
            clawCtl.update(desired);
        }

        // --------------------------------------------------------
        //  4c. CLAW SNAPS & AUTO-GRAB MACRO
        // --------------------------------------------------------
        if (activeMacro != Macro::GRAB) {
            // Tap BTN_Y to Toggle Grab / Release
            if (gamepad1.justPressed(BTN_Y)) {
                if (objectSecured) {
                    // If currently holding, release immediately
                    clawCtl.releaseNow();
                    clawCtl.tryOpen();
                    objectSecured = false;
                    gripMemory.set(false);
                    userLed.setSolid(0, 255, 0);
                    tone(BUZZER_PIN, 1200, 70);
                } else {
                    // Execute Auto Grab
                    runAction(autoGrabRoutine());
                    activeMacro = Macro::GRAB;
                }
            }

            // Manual snap — shut the claw and press on the piece.
            //
            // There is deliberately no "snap open" on BTN_X any more: that
            // button is the payload gate now. To let go of a piece, press
            // BTN_Y (release) or flip LT (expand).
            if (gamepad1.justPressed(BTN_A)) {
                clawCtl.engageNow();
                objectSecured = true;
                gripMemory.set(true);
            }
        }

        // --------------------------------------------------------
        //  4d. PAYLOAD GATE — press once to toggle open / closed
        //
        //  Just a toggle between two angles. The servo ramps to the target
        //  on its own, so the target is set once and apply() does the rest.
        // --------------------------------------------------------
        if (gamepad1.justPressed(PAYLOAD_BUTTON)) {
            payloadOpen = !payloadOpen;
            payloadAxis.command(payloadOpen ? PAYLOAD_OPEN_ANGLE : PAYLOAD_CLOSED_ANGLE);
            tone(BUZZER_PIN, payloadOpen ? 2000 : 1400, 60);
        }

        // --------------------------------------------------------
        //  5. PUSH THE TARGETS TO THE SERVOS
        //
        //  This MUST run every tick, and it MUST be the only thing that
        //  talks to the servos. Two jobs:
        //
        //    1. It advances the ramp — the servo only travels while
        //       setPosition() is being called.
        //    2. It refreshes the PWM duty every tick, so the servo keeps
        //       receiving pulses. A servo that is only refreshed every
        //       250 ms (the driver's idle heartbeat) can drop its pulses
        //       when the rail sags — which is what "the servo just stops
        //       working" looks like on the field.
        // --------------------------------------------------------
        armAxis.apply();
        clawAxis.apply();
        payloadAxis.apply();

        // --------------------------------------------------------
        //  6. STATUS LED MANAGEMENT
        // --------------------------------------------------------
        if (brownoutBeacon.service()) {
            // The beacon owns the LED — it is telling the driver the last
            // boot was a brownout reset.
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
            // The grab macro owns the LED (orange = clamping, purple = secured).
        } else if (fullStraightActive) {
            userLed.setSolid(255, 255, 255); // White LED: FULL SPRINT
        } else if (objectSecured) {
            userLed.setSolid(180, 0, 255);   // Purple: Object Secured
        } else {
            userLed.setSolid(0, 255, 0);     // Green: Ready
        }

        // --------------------------------------------------------
        //  7. TELEMETRY
        // --------------------------------------------------------
        // RAW shows the button bits actually arriving from the controller, so
        // a control that does nothing can be traced (an empty RAW while a
        // button is held means the controller is not sending that bit).
        telemetry.addData("Arm Angle",  armAxis.actual());
        telemetry.addData("Claw Angle", clawAxis.actual());
        telemetry.addData("Arm",        armDir  == ArmDir::UP   ? "UP" :
                                     armDir  == ArmDir::DOWN ? "DOWN" : "HOLD");
        telemetry.addData("Claw",       clawCtl.dir == ClawDir::EXPAND   ? "EXPAND" :
                                     clawCtl.dir == ClawDir::CONTRACT ? "CONTRACT" : "HOLD");
        telemetry.addData("Grip",       clawCtl.isGripping() ? "PRESSING" : "loose");
        telemetry.addData("Payload",     payloadOpen ? "GATE OPEN" : "GATE CLOSED");
        telemetry.addData("Pay Angle",  payloadAxis.actual());
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
        // the next run does not come up clamping onto nothing. (A BROWNOUT
        // never reaches here, which is exactly why the memory is useful.)
        gripMemory.set(false);

        arm.detach();
        claw.detach();
        payload.detach();
        userLed.setOff();
        noTone(BUZZER_PIN);
    }
};

REGISTER_OPMODE(DriveMode, "Drive Mode", TELEOP);
