#include "NRL.h"
#include "team205_mech.h"
#include <cmath>

// ============================================================
//  TiltSteering — IMU Board Tilt Control Mode
// ============================================================
// How it works:
//  - Tilting the robot forward / backward commands Forward / Reverse drive!
//  - Tilting the robot left / right commands Turning left / right!
//  - Smooth 3° deadzone around neutral so it stays completely still on flat ground.
//
//  ARM (hold to move, release to stop):
//    * BTN_DPAD_UP    = arm UP
//    * BTN_DPAD_DOWN  = arm DOWN
//  CLAW (toggle on, travels to its limit; toggle off to stop):
//    * BTN_LT         = claw EXPAND
//    * BTN_RT         = claw CONTRACT
//  CLAW SNAPS:
//    * BTN_A          = snap fully closed (and engage the grip)
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

// Hardware configuration
static HexaDCMotor leftMotor({ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM, .flipped = true });
static HexaDCMotor rightMotor({ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = false });
static TankDrive   drive(leftMotor, rightMotor);

// On-board IMU (LSM6DSOX on dedicated Wire bus 0, 0x6B)
static HexaIMU imu{ hexaImuConfig() };

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

// Responsive tilt tuning
constexpr float TILT_DEADZONE_DEG = 3.0f;   // Ignore tilts under ±3°
constexpr float TILT_MAX_DEG      = 22.0f;  // Full 100% motor speed reached at 22° tilt

static HexaServo arm ({ .signalPin = ARM_SIGNAL_PIN,  .startAngle = ARM_HOME_ANGLE });
static HexaServo claw({ .signalPin = CLAW_SIGNAL_PIN, .startAngle = CLAW_OPEN_ANGLE });

// The mechanism wrappers. See team205_mech.h for what these fix.
static ServoAxis armAxis;
static ServoAxis clawAxis;

// The claw's state machine (open / closing / gripping).
static ClawControl clawCtl;

// Which way the arm is being driven right now.
enum class ArmDir { NONE, UP, DOWN };

static ArmDir armDir       = ArmDir::NONE;
static bool   objectSecured = false;

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

class TiltSteering : public NRLOpMode {
public:
    void init() override {
        // Initialize sensors and actuators
        imu.begin();
        arm.setRampRate(ARM_RATE_DEG_PER_SEC);
        claw.setRampRate(CLAW_RATE_DEG_PER_SEC);
        arm.begin();
        claw.begin();
        leftMotor.begin();
        rightMotor.begin();

        armAxis.bind(arm, ARM_HOME_ANGLE, ARM_CARRY_ANGLE, ARM_FLOOR_ANGLE);
        clawAxis.bind(claw, CLAW_OPEN_ANGLE, CLAW_OPEN_ANGLE, CLAW_CLOSED_ANGLE);
        clawCtl.bind(clawAxis, CLAW_OPEN_ANGLE, CLAW_CLOSED_ANGLE, CLAW_GRIP_ANGLE);

        armDir        = ArmDir::NONE;
        objectSecured = false;

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

        userLed.setSolid(0, 200, 255); // Cyan: Tilt steering active
    }

    void loop() override {
        readRawButtons();

        // Watch the battery rail. The scale factor backs the drive off when
        // the pack sags, which is what keeps "drive while gripping" from
        // resetting the board.
        const float driveScale = powerGuard.update();

        // --------------------------------------------------------
        //  1. ACQUIRE ACCELEROMETER TILT DATA
        // --------------------------------------------------------
        imu.read();

        float ax = imu.getAccelX();
        float ay = imu.getAccelY();
        float az = imu.getAccelZ();

        // Calculate pitch (forward/back tilt) and roll (left/right tilt)
        float pitchDeg = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0f / PI);
        float rollDeg  = atan2(ay, az) * (180.0f / PI);

        // --------------------------------------------------------
        //  2. PROPORTIONAL TILT DRIVE MAPPING
        // --------------------------------------------------------
        float forward = 0.0f;
        float turn    = 0.0f;

        // Pitch tilt -> Forward / Reverse
        if (abs(pitchDeg) > TILT_DEADZONE_DEG) {
            float sign = (pitchDeg > 0.0f) ? 1.0f : -1.0f;
            forward = sign * (abs(pitchDeg) - TILT_DEADZONE_DEG) / (TILT_MAX_DEG - TILT_DEADZONE_DEG);
            forward = constrain(forward, -1.0f, 1.0f);
        }

        // Roll tilt -> Left / Right Turn
        if (abs(rollDeg) > TILT_DEADZONE_DEG) {
            float sign = (rollDeg > 0.0f) ? 1.0f : -1.0f;
            turn = sign * (abs(rollDeg) - TILT_DEADZONE_DEG) / (TILT_MAX_DEG - TILT_DEADZONE_DEG);
            turn = constrain(turn, -1.0f, 1.0f);
        }

        // Command motors
        drive.drive(forward * driveScale, turn);

        // Visual feedback based on movement
        if (!brownoutBeacon.service()) {
            if (abs(forward) > 0.1f || abs(turn) > 0.1f) {
                userLed.setSolid(0, 255, 0); // Green when actively driving
            } else {
                userLed.setSolid(0, 150, 255); // Cyan when resting flat
            }
        }

        // --------------------------------------------------------
        //  3. ARM — D-PAD UP / DOWN
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
        //  4. CLAW — LT / RT TRIGGERS
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

        clawCtl.update(desired);

        // BTN_A snaps the claw shut and engages the grip
        if (gamepad1.justPressed(BTN_A)) {
            clawCtl.engageNow();
            objectSecured = true;
            gripMemory.set(true);
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
        //  6. TELEMETRY STREAM
        // --------------------------------------------------------
        telemetry.addData("Pitch Deg",  pitchDeg);
        telemetry.addData("Roll Deg",   rollDeg);
        telemetry.addData("Forward",    forward);
        telemetry.addData("Turn",       turn);
        telemetry.addData("Arm Angle",  armAxis.actual());
        telemetry.addData("Claw Angle", clawAxis.actual());
        telemetry.addData("Grip",       clawCtl.isGripping() ? "PRESSING" : "loose");
        telemetry.addData("Object",     objectSecured ? "SECURED" : "NONE");
        telemetry.addData("Drive Sc",   driveScale);
        telemetry.addData("RAW",        rawButtons);
        team205ReportPower(telemetry, powerGuard);
    }

    void stop() override {
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

REGISTER_OPMODE(TiltSteering, "Tilt Steering", TELEOP);
