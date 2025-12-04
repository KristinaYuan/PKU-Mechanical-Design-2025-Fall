/*
 * engine.ino - Glass-cleaner elevator controller
 *
 * This program controls 3 servos to lift a window-cleaner from track to wall.
 *
 * Updated 12/04/2025:
 *  - Built the foundamental frame.
 *  - Implemented essential functions to operate the forward process.
 *  - Specific parameters are to be further measured.
 *  - Some functions are redundant and the logic can be smoothed later on.
 */
#include <Servo.h>

/* Define servo objects */
Servo armLift;  // servo at bottom the the arms
Servo endLeft;  // servo in the left arm
Servo endRight; // servo in the right arm

/* Define pins */
#define ARM_LIFT_PIN 9     // PWB pin for main lifting servo
#define END_LEFT_PIN 10    // PWB pin for left end servo
#define END_RIGHT_PIN 11   // PWB pin for right end servo
#define BUTTON_START_PIN 2 // start button
#define BUTTON_STOP_PIN 3  // stop button
#define LIMIT_DOWN_PIN 4   // lower limit switch
#define LIMIT_UP_PIN 5     // upper limit switch
#define LED_PIN 13         // status indicator LED
#define BUZZER_PIN 12      // buzzer for audible alerts
#define POT_PIN A0         // potentiometer for manual adjustment

/* Define mechanical parameters */
#define BIG_ARM_LENGTH 25.0   // big arm length (centimeter)
#define SMALL_ARM_LENGTH 17.0 // small arm length

/* Define gear ratio */
#define ARM_GEAR_RATIO 2.0 // servo:bigarm = 2:1 (decelerate)
#define END_GEAR_RATIO 1.0 // end drives directly 1:1

/* Define mechanical angles */
/* Big arm's relative angle to horizontal level */
#define MECH_BIG_ARM_MIN 0   // big arm horizontally backwards
#define MECH_BIG_ARM_MAX 135 // big arm leaning towards wall
#define MECH_BIG_ARM_HOME 0  // big arm initial position

/* Small arm's relative angle to big arm */
#define MECH_SMALL_ARM_MIN -180 // small arm folding backwrds
#define MECH_SMALL_ARM_MAX 135  // small arm vertically upright
#define MECH_SMALL_ARM_HOME -180

/* Mechanical angles in the three steps */
#define MECH_BIG_ARM_LIFT 0       // lift phase: big arm stay put
#define MECH_BIG_ARM_VERTICAL 90  // vertical phase: big arm turn vertical
#define MECH_BIG_ARM_WALL 135     // wall phase: big arm leans towards wall
#define MECH_SMALL_ARM_LIFT 45    // lift phase: small arm lift up 45°
#define MECH_SMALL_ARM_VERTICAL 0 // vertical phase: small arm turn vertical
#define MECH_SMALL_ARM_WALL 0     // wall phase: small arm approaches wall

/* Safety margins */
#define MECH_SAFETY_MARGIN 10
#define MECH_BIG_ARM_SAFE_MIN (MECH_BIG_ARM_MIN + MECH_SAFETY_MARGIN) // 10°
#define MECH_BIG_ARM_SAFE_MAX (MECH_BIG_ARM_MAX - MECH_SAFETY_MARGIN) // 80°
#define MECH_SMALL_ARM_SAFE_MIN (MECH_SMALL_ARM_MIN + MECH_SAFETY_MARGIN)
#define MECH_SMALL_ARM_SAFE_MAX (MECH_SMALL_ARM_MAX - MECH_SAFETY_MARGIN)

/* Calculate servo angle */
/* Mechanical angle to servo angle */
inline int mechBigArmToServo(float mechAngle)
{
    return mechAngle * ARM_GEAR_RATIO;
}

inline int mechSmallArmToServo(float mechAngle)
{
    return mechAngle * END_GEAR_RATIO;
}

/* Servo angle to mechanical angle */
inline float servoBigArmToMech(int servoAngle)
{
    return servoAngle / ARM_GEAR_RATIO;
}

inline float servoSmallArmToMech(int servoAngle)
{
    return servoAngle / END_GEAR_RATIO;
}

/* Define servo angle restrictions */
/* Main lift servo - 180° */
#define ARM_LIFT_HOME mechBigArmToServo(MECH_BIG_ARM_HOME)
#define ARM_LIFT_MIN mechBigArmToServo(MECH_BIG_ARM_MIN)
#define ARM_LIFT_MAX mechBigArmToServo(MECH_BIG_ARM_MAX)
#define ARM_LIFT_SAFE_MIN mechBigArmToServo(MECH_BIG_ARM_SAFE_MIN) // 20°
#define ARM_LIFT_SAFE_MAX mechBigArmToServo(MECH_BIG_ARM_SAFE_MAX) // 160°

/* End servos - 270° */
#define END_HOME mechSmallArmToServo(MECH_SMALL_ARM_HOME)
#define END_MIN mechSmallArmToServo(MECH_SMALL_ARM_MIN)
#define END_MAX mechSmallArmToServo(MECH_SMALL_ARM_MAX)
#define END_SAFE_MIN mechSmallArmToServo(MECH_SMALL_ARM_SAFE_MIN)
#define END_SAFE_MAX mechSmallArmToServo(MECH_SMALL_ARM_SAFE_MAX)
#define END_CONTACT mechSmallArmToServo(0)

/* Define time parameters */
#define ARM_STEP_DELAY 15
#define END_STEP_DELAY 10
#define SYNC_TOLERANCE 3

/* Organize robot information */
class RobotPose
{
private:
    int armAngle; // angle of big arm servo
    int endAngle; // angle of small arm servo
    int liftSpeed;
    int endSpeed;

public:
    /* Constructor 1: using servo angles */
    RobotPose(int AA, int EA, int LS = ARM_STEP_DELAY, int ES = END_STEP_DELAY)
        : armAngle(AA), endAngle(EA), liftSpeed(LS), endSpeed(ES) {}
    /* Constructor 2: using mechanical angles */
    RobotPose(float MBA, float MSA, int LS = ARM_STEP_DELAY, int ES = END_STEP_DELAY)
        : armAngle(mechBigArmToServo(MBA)), endAngle(mechSmallArmToServo(MSA)),
          liftSpeed(LS), endSpeed(ES) {}

    /* Getters */
    int getArmAngle()
    {
        return constrain(this->armAngle, ARM_LIFT_SAFE_MIN, ARM_LIFT_SAFE_MAX);
    }
    float getBigArmMechAngle()
    {
        return servoBigArmToMech(this->getArmAngle());
    }
    int getEndAngle()
    {
        return constrain(this->endAngle, END_SAFE_MIN, END_SAFE_MAX);
    }
    float getSmallArmMechAngle()
    {
        return servoSmallArmToMech(this->getEndAngle());
    }
    int getLiftSpeed()
    {
        return this->liftSpeed;
    }
    int getEndSpeed()
    {
        return this->endSpeed;
    }

    /* Check */
    bool isValid()
    {
        bool armValid = (armAngle >= ARM_LIFT_SAFE_MIN) &&
                        (armAngle <= ARM_LIFT_SAFE_MAX);
        bool endValid = (endAngle >= END_SAFE_MIN) &&
                        (endAngle <= END_SAFE_MAX);
        return armValid && endValid;
    }

    /* Calculate end position */
    void calculateEndPosition(float &x, float &y)
    {
        float theta = getBigArmMechAngle() * PI / 180.0; // big arm absolute angle
        float phi = getSmallArmMechAngle() * PI / 180.0; // small arm relative angle
        float theta2 = theta + phi;                      // small arm absolute angle
        x = BIG_ARM_LENGTH * cos(theta) + SMALL_ARM_LENGTH * cos(theta2);
        y = BIG_ARM_LENGTH * sin(theta) + SMALL_ARM_LENGTH * sin(theta2);
    }

    /* Print information */
    void print(const char *name = "Pose")
    {
        Serial.print(name);
        Serial.print(": Lift servo = ");
        Serial.print(getArmAngle());
        Serial.print("° (Mech ");
        Serial.print(getBigArmMechAngle());
        Serial.print("°), End servo = ");
        Serial.print(getEndAngle());
        Serial.print("° (Mech ");
        Serial.print(getSmallArmMechAngle());
        Serial.print("°)");
        float x, y;
        calculateEndPosition(x, y);
        Serial.print(", End position = (");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.println(")");
    }
};

/* Important positions */
const RobotPose poseInit(MECH_BIG_ARM_HOME, MECH_SMALL_ARM_HOME);
const RobotPose poseLift(MECH_BIG_ARM_HOME, MECH_SMALL_ARM_LIFT);
const RobotPose poseVertical(90, MECH_SMALL_ARM_VERTICAL);
const RobotPose poseWall(MECH_BIG_ARM_SAFE_MAX, MECH_SMALL_ARM_WALL);

/* Define system states */
enum SystemState
{
    STATE_IDLE,           // state ready, awaiting command
    STATE_LIFTING,        // lift phase
    STATE_VERTICAL,       // vertical phase
    STATE_WALL_CONTACT,   // wall phase
    STATE_WORKING,        // robot is working on the wall
    STATE_RETURNING,      // returning to ground stage
    STATE_EMERGENCY_STOP, // emergency stop activated
    STATE_CALIBRATING     // system calibration in progress
};

/* Define operation modes */
enum OperationMode
{
    MODE_AUTO,       // automatic sequence execution
    MODE_MANUAL,     // manual step-by-step control
    MODE_CALIBRATION // calibration mode for setup
};

/* Define error codes */
enum ErrorCode
{
    ERROR_NONE,         // no error
    ERROR_LIMIT_SWITCH, // limit switch triggered unexpectedly
    ERROR_SYNC_FAIL,    // end servo synchronization failure
    ERROR_OVER_CURRENT  // motor stall or over-current detected
};

/* Define global variables */
int armCurrentAngle = ARM_LIFT_HOME;   // current angle of main lifting servo
int endCurrentAngle = END_HOME;        // current angle of end servos
int armTargetAngle = ARM_LIFT_HOME;    // target angle for main lifting servo
int endTargetAngle = END_CONTACT;      // target angle for end servos
SystemState currentState = STATE_IDLE; // current system state
OperationMode currentMode = MODE_AUTO; // current operation mode
bool isSystemReady = false;            // system initialization completion flag
bool isEmergency = false;              // emergency stop active flag
bool isEndServoSynced = true;          // end servo synchronization status
unsigned long stateStartTime = 0;      // timestamp when current state began
unsigned long lastOperationTime = 0;   // timestamp of last successful operation
unsigned long lastDebounceTime = 0;    // timestamp for button debouncing
int errorCount = 0;                    // total number of errors encountered
int syncErrorTotal = 0;                // cumulative synchronization error

/* Define time constants */
const unsigned long DEBOUNCE_DELAY = 50;    // button debounce delay
const unsigned long LIFT_DURATION = 3000;   // expected duration for lifting sequence
const unsigned long ADJUST_DURATION = 2000; // expected duration for adjustment sequence
const unsigned long BUZZER_SHORT = 100;     // short beep duration
const unsigned long BUZZER_LONG = 500;      // long beep duration

/* Declare functions */
/* Initialization functions */
void setup();
void initializePins();
void initializeServos();
void systemCalibration();
/* Main loop function */
void loop();
/* Core motion control functions */
void moveArmLift(int targetAngle, int stepDelay = ARM_STEP_DELAY, bool checkLimit = true);
void moveEndServos(int targetAngle, int stepDelay = END_STEP_DELAY);
void syncEndServos();
void moveToPose(RobotPose pose);
/* Operation sequences */
void executeFullSequence();
void executeReverseSequence();
void performHoming();
/* Safety functions */
void emergencyStop();
bool checkStallCondition();
bool shouldStopMovement();
void checkLimits();
/* Input handlings */
void checkButtons();
void handleSerialCommand();
/* State machine functions */
void runStateMachine();
void updateSystemState();
/* Utility functions */
void beep(int times, int duration = BUZZER_SHORT);
void updateStatusLED();
void printSystemStatus();
void logError(int errorCode);
/* Three motion phases */
void executeLifting();
void executeVertical();
void executeWallContact();

/* Function implementation */

/*
 * setup - Main setup function
 *
 * Set pins, servo positions and other flags to initial stage.
 * Print original parameters and signals when ready.
 */
void setup()
{
    Serial.begin(9600);
    Serial.println("=== Glass Cleaner Elevator ===");

    /* Initialize pins and servos */
    initializePins();
    initializeServos();

    /* Show current configuration */
    Serial.print("Gear ratio: Servo:Big Arm = ");
    Serial.print(ARM_GEAR_RATIO);
    Serial.println(":1");
    Serial.print("Lift servo safety region: ");
    Serial.print(ARM_LIFT_SAFE_MIN);
    Serial.print("-");
    Serial.print(ARM_LIFT_SAFE_MAX);
    Serial.print("° (Mech ");
    Serial.print(MECH_BIG_ARM_SAFE_MIN);
    Serial.print("-");
    Serial.print(MECH_BIG_ARM_SAFE_MAX);
    Serial.println("°)");

    /* Show important positions */
    poseInit.print("Initial Position");
    poseLift.print("Lift Position");
    poseVertical.print("Vertival Position");
    poseWall.print("Wall Position");

    /* Set initial variables */
    currentState = STATE_IDLE;
    currentMode = MODE_AUTO;
    isEmergency = false;
    armCurrentAngle = ARM_LIFT_HOME;
    endCurrentAngle = END_HOME;
    syncErrorTotal = 0;
    isEndServoSynced = true;
    errorCount = 0;

    /* Perform homing at startup */
    performHoming();

    /* Startup beep */
    beep(2, BUZZER_SHORT);

    Serial.println("Ready!");
}

/*
 * initializePins - initialize pins
 *
 * Connect arduino pins to appropriate signals.
 */
void initializePins()
{
    pinMode(BUTTON_START_PIN, INPUT_PULLUP);
    pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
    pinMode(LIMIT_UP_PIN, INPUT_PULLUP);
    pinMode(LIMIT_DOWN_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
}

/*
 * initializeServos - initialize servos
 *
 * Attach servo signals to corresponding pins.
 * Setting servo angles to home position.
 */
void initializeServos()
{
    armLift.attach(ARM_LIFT_PIN);
    endLeft.attach(END_LEFT_PIN);
    endRight.attach(END_RIGHT_PIN);

    /* Initial position */
    armLift.write(ARM_LIFT_HOME);
    endLeft.write(END_HOME);
    endRight.write(END_HOME);

    delay(1000);
}

void performHoming()
{
    armLift.write(ARM_LIFT_HOME);
    endLeft.write(END_HOME);
    endRight.write(END_HOME);

    delay(1500);

    armCurrentAngle = armLift.read();
    endCurrentAngle = endLeft.read();

    Serial.println("Homing Complete.");
}

void beep(int times, int duration)
{
    for (int i = 0; i < times; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(duration);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < times - 1)
        {
            delay(duration);
        }
    }
}

/*
 * loop - Main loop function
 *
 * Check current state and run state machine accordingly.
 * The loop function is continuously called during the process.
 */
void loop()
{
    /* Check inputs */
    checkButtons();
    handleSerialCommand();

    /* Run state machine */
    runStateMachine();

    /* Monitor limit switches while idle */
    if (currentState == STATE_IDLE && !isEmergency)
    {
        if (digitalRead(LIMIT_DOWN_PIN) == LOW || digitalRead(LIMIT_UP_PIN) == LOW)
        {
            Serial.println(F("Limit switch active while idle; check mechanical position."));
            delay(200);
        }
    }
}

void checkButtons()
{
    /* start button (active low) with simple debounce */
    static unsigned long lastStart = 0;
    if (digitalRead(BUTTON_START_PIN) == LOW && (millis() - lastStart) > 300)
    {
        lastStart = millis();
        if (currentState == STATE_IDLE && isSystemReady && !isEmergency)
        {
            Serial.println(F("Start button pressed -> start sequence"));
            executeFullSequence();
        }
        else
        {
            Serial.println(F("Start pressed but system not idle or not ready."));
        }
    }

    /* stop button handled in shouldStopMovement */
    if (digitalRead(BUTTON_STOP_PIN) == LOW)
    {
        emergencyStop();
    }
}

void handleSerialCommand()
{
    if (!Serial.available())
    {
        return;
    }
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    Serial.print(F("CMD: "));
    Serial.println(cmd);
    if (cmd == "START")
    {
        executeFullSequence();
    }
    else if (cmd == "STOP")
    {
        emergencyStop();
    }
    else if (cmd == "HOME")
    {
        performHoming();
    }
    else if (cmd == "LIFT")
    {
        executeLifting();
    }
    else if (cmd == "VERTICAL")
    {
        executeVertical();
    }
    else if (cmd == "WALL")
    {
        executeWallContact();
    }
    else if (cmd == "REVERSE")
    {
        executeReverseSequence();
    }
    else if (cmd == "STATUS")
    {
        printSystemStatus();
    }
    else
    {
        Serial.println(F("Unknown command."));
    }
}

void executeFullSequence()
{
    if (!isSystemReady)
    {
        Serial.println("System not ready");
        return;
    }

    Serial.println("Start operating full on-wall process...");

    executeLifting();
    executeVertical();
    executeWallContact();

    Serial.println("On-wall process finished!");
    beep(3);

    currentState = STATE_IDLE;
}

void executeLifting()
{
    Serial.println("Phase 1: raise small arm.");
    currentState = STATE_LIFTING;

    /* End servos operate, lift servo motionless */
    moveEndServos(poseLift.getEndAngle(), poseLift.getEndSpeed());

    Serial.println("Lift phase finished.");
    delay(500);
}

void moveEndServos(int targetAngle, int stepDelay)
{
    targetAngle = constrain(targetAngle, END_SAFE_MIN, END_SAFE_MAX);

    Serial.print("Move end servos: ");
    Serial.print(endCurrentAngle);
    Serial.print(" -> ");
    Serial.print(targetAngle);
    Serial.print(" (Mech ");
    Serial.print(servoSmallArmToMech(endCurrentAngle));
    Serial.print(" -> ");
    Serial.print(servoSmallArmToMech(targetAngle));
    Serial.println("°)");

    int step = (targetAngle > endCurrentAngle) ? 1 : -1;

    while (endCurrentAngle != targetAngle)
    {
        endCurrentAngle += step;

        /* Write servos */
        endLeft.write(endCurrentAngle);
        endRight.write(endCurrentAngle);

        delay(stepDelay);

        if (shouldStopMovement())
            return;

        /* Check synchronization */
        int currentL = endLeft.read();
        int currentR = endRight.read();
        if (abs(currentL - currentR) > SYNC_TOLERANCE)
        {
            Serial.print("Sync error: ");
            Serial.println(abs(currentL - currentR));
            syncEndServos(); // auto-correct
        }
    }

    endTargetAngle = targetAngle;
}

void syncEndServos()
{
    int currentL = endLeft.read();
    endRight.write(currentL);
    endCurrentAngle = currentL;
    Serial.println("End servos synchronized.");
}

bool shouldStopMovement()
{
    return isEmergency || digitalRead(BUTTON_STOP_PIN) == LOW;
}

void executeVertical()
{
    Serial.println("Phase 2: arms move upright.");
    currentState = STATE_VERTICAL;

    /* Move big arm and small arm at the same time */
    moveArmLift(poseVertical.getArmAngle(), poseVertical.getLiftSpeed() * 2);
    moveEndServos(poseVertical.getEndAngle(), poseVertical.getEndSpeed() * 2);

    Serial.println("Vertical phase finished.");
    delay(500);
}

void moveArmLift(int targetAngle, int stepDelay, bool checkLimit)
{
    targetAngle = constrain(targetAngle, ARM_LIFT_SAFE_MIN, ARM_LIFT_SAFE_MAX);

    Serial.print("Move lift servo: ");
    Serial.print(armCurrentAngle);
    Serial.print(" -> ");
    Serial.print(targetAngle);
    Serial.print(" (Mech ");
    Serial.print(servoBigArmToMech(armCurrentAngle));
    Serial.print(" -> ");
    Serial.print(servoBigArmToMech(targetAngle));
    Serial.println("°)");

    int step = (targetAngle > armCurrentAngle) ? 1 : -1;

    while (armCurrentAngle != targetAngle)
    {
        armCurrentAngle += step;
        armLift.write(armCurrentAngle);
        delay(stepDelay);

        if (shouldStopMovement())
            return;

        /* Check upper and lower bounds */
        if (checkLimit)
        {
            if (step > 0 && digitalRead(LIMIT_UP_PIN) == LOW)
            {
                Serial.println("Reached upper limit");
                break;
            }
            if (step < 0 && digitalRead(LIMIT_DOWN_PIN) == LOW)
            {
                Serial.println("Reached lower limit");
                break;
            }
        }
    }

    armTargetAngle = targetAngle;
}

void executeWallContact()
{
    Serial.println("Phase 3: move arms towards wall.");
    currentState = STATE_WALL_CONTACT;

    /* Big arm closes wall while small arm stays vertical */
    moveArmLift(poseWall.getArmAngle(), poseWall.getLiftSpeed());
    moveEndServos(poseWall.getEndAngle(), poseWall.getEndSpeed());

    Serial.println("Wall phase finished.");
    Serial.println("Robot is now pressed to the wall.");
    currentState = STATE_WORKING;
}

void emergencyStop()
{
    Serial.println("Emergency stop!");
    isEmergency = true;

    for (int i = 0; i < 5; i++)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        delay(200);
    }

    currentState = STATE_EMERGENCY_STOP;
}

void moveToPose(RobotPose pose)
{
    if (!pose.isValid())
    {
        Serial.println("Error: position parameters invalid!");
        return;
    }

    pose.print("Move towards");

    moveArmLift(pose.getArmAngle(), pose.getLiftSpeed());
    delay(200);
    moveEndServos(pose.getEndAngle(), pose.getEndSpeed());
}

void printSystemStatus()
{
    Serial.println("System Status:");

    /* Servo status */
    Serial.print("Lift servo: ");
    Serial.print(armCurrentAngle);
    Serial.print("° (Mech ");
    Serial.print(servoBigArmToMech(armCurrentAngle));
    Serial.println("°)");
    Serial.print("End servos: ");
    Serial.print(endCurrentAngle);
    Serial.print("° (Mech ");
    Serial.print(servoSmallArmToMech(endCurrentAngle));
    Serial.println("°)");

    /* System status */
    switch (currentState)
    {
    case STATE_IDLE:
        Serial.println("awaiting");
        break;
    case STATE_LIFTING:
        Serial.println("lift phase");
        break;
    case STATE_VERTICAL:
        Serial.println("vertical phase");
        break;
    case STATE_WALL_CONTACT:
        Serial.println("wall phase");
        break;
    case STATE_WORKING:
        Serial.println("working");
        break;
    default:
        Serial.println("other status");
    }
    Serial.println("=====================");
    Serial.println("");
}

void runStateMachine()
{
    updateStatusLED();
}

void updateStatusLED()
{
    static unsigned long last = 0;
    static bool on = false;
    if (millis() - last > 500)
    {
        on = !on;
        digitalWrite(LED_PIN, on ? HIGH : LOW);
        last = millis();
    }
}

/* Functions not implemented yet */

bool checkStallCondition()
{
    return false;
}

void systemCalibration()
{
    return;
}

void checkLimits()
{
    return;
}

void updateSystemState()
{
    return;
}

void logError(int errorCode)
{
    return;
}

void executeReverseSequence()
{
    return;
}