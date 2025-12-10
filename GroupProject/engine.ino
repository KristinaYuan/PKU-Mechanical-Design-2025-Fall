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
 *
 * Updated 12/05/2025:
 *  - Deleted needless codes related to button, limit, buzzer and LED.
 *
 * Updated 12/10/2025:
 *  - Rewrote the signal handling and control implementing logic.
 *  - Reorganized information of positions and poses not using classes.
 *  - Adapted to the new kinematic process.
 *  - Added controls for car operations.
 */
#include <Servo.h>

/* Define servo objects */
Servo armLift;  // servo at bottom the the arms
Servo endLeft;  // servo in the left arm
Servo endRight; // servo in the right arm

/* Define servo pins */
#define ARM_LIFT_PIN 9   // PWB pin for main lifting servo
#define END_LEFT_PIN 10  // PWB pin for left end servo
#define END_RIGHT_PIN 11 // PWB pin for right end servo

/* Define mechanical parameters */
#define BIG_ARM_LENGTH 25.0   // big arm length (centimeter)
#define SMALL_ARM_LENGTH 17.0 // small arm length

/* Define gear ratio */
#define ARM_GEAR_RATIO 2.0 // servo:bigarm = 2:1 (decelerate)
#define END_GEAR_RATIO 1.0 // end drives directly 1:1

/* Define mechanical angles */
/* Big arm's relative angle to horizontal level */
#define MECH_LIFT_INIT 0
#define MECH_LIFT_STEP1 45
#define MECH_LIFT_STEP2 37

/* Small arm's relative angle to big arm (inside) */
#define MECH_END_INIT 0
#define MECH_END_STEP1 127
#define MECH_END_STEP2 127

/* Calculate servo angles */
inline int mechLiftToServo(int mechAngle)
{
    return 90 + mechAngle * ARM_GEAR_RATIO;
}

inline int mechEndToServo(int mechAngle)
{
    return 180 - mechAngle * END_GEAR_RATIO;
}

/* Define servo angles */
#define LIFT_INIT mechLiftToServo(MECH_LIFT_INIT)
#define LIFT_STEP1 mechLiftToServo(MECH_LIFT_STEP1)
#define LIFT_STEP2 mechLiftToServo(MECH_LIFT_STEP2)
#define END_INIT mechLiftToServo(MECH_END_INIT)
#define END_STEP1 mechLiftToServo(MECH_END_STEP1)
#define END_STEP2 mechLiftToServo(MECH_END_STEP2)

/* Define time parameters */
#define ARM_STEP_DELAY 15
#define END_STEP_DELAY 10
#define DELAY_TIME 1500

/* Define car pins */
#define ENA_PIN 6
#define IN1_PIN 5
#define IN2_PIN 4
#define ENB_PIN 3
#define IN3_PIN 2
#define IN4_PIN 12

/* Define car speed */
#define CAR_SPEED 200

/* Define system states */
enum ProgramState
{
    S_INIT,
    S_CAR_STOPPED,
    S_CAR_FORWARD,
    S_CAR_BACKWARD,
    S_ARM_LIFT_STEP1,
    S_ARM_LIFT_STEP2,
    S_ARM_LOWER_STEP2,
    S_ARM_LOWER_STEP1,
    S_IDLE
};

ProgramState currentState = S_INIT;

/* Declare functions */
void setup();
void loop();
void printHelp();
void setServoPins();
void setCarPins();
void checkExternalCommand();
void initializePosition();
void positionStep1();
void positionStep2();
void liftMove(int target);
void endMove(int target);
void stopMotion();
void stopCar();
void runCarForward(int speed);
void runCarBackward(int speed);

/* Function implementation */

void setup()
{
    Serial.begin(9600);
    printHelp();

    setServoPins();
    setCarPins();

    initializePosition();
    stopCar();
}

void loop()
{
    checkExternalCommand();

    switch (currentState)
    {
    case S_INIT:
    case S_CAR_STOPPED:
        stopCar();
        delay(100);
        break;
    case S_CAR_FORWARD:
        runCarForward(CAR_SPEED);
        delay(50);
        break;
    case S_CAR_BACKWARD:
        runCarBackward(CAR_SPEED);
        delay(50);
        break;
    case S_ARM_LIFT_STEP1:
        positionStep1();
        currentState = S_ARM_LIFT_STEP2;
        break;
    case S_ARM_LIFT_STEP2:
        positionStep2();
        currentState = S_CAR_FORWARD;
        break;
    case S_ARM_LOWER_STEP2:
        positionStep2();
        currentState = S_ARM_LOWER_STEP1;
        break;
    case S_ARM_LOWER_STEP1:
        positionStep1();
        currentState = S_CAR_BACKWARD;
        break;
    case S_IDLE:
        stopCar();
        delay(5000);
        break;
    }
}

void printHelp()
{
}

void setServoPins()
{
    armLift.attach(ARM_LIFT_PIN);
    endLeft.attach(END_LEFT_PIN);
    endRight.attach(END_RIGHT_PIN);
}

void setCarPins()
{
    pinMode(ENA_PIN, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(ENB_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
}

void checkExternalCommand()
{
    if (!Serial.available())
    {
        return;
    }

    char cmd = Serial.read();
    switch (cmd)
    {
    case 'S':
    case 's':
        Serial.println("命令: 停止 (Stop)");
        if (currentState == S_CAR_FORWARD || currentState == S_CAR_BACKWARD)
        {
            stopCar();
            currentState = S_CAR_STOPPED;
        }
        else if (currentState >= S_ARM_LIFT_STEP1 && currentState <= S_ARM_LOWER_STEP1)
        {
            stopMotion();
            currentState = S_IDLE;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;

    case 'R':
    case 'r':
        Serial.println("命令: 重启 (Restart)");
        stopCar();
        initializePosition();
        currentState = S_INIT;
        break;

    case 'F':
    case 'f':
        if (currentState == S_INIT || currentState == S_CAR_STOPPED)
        {
            Serial.println("命令: 前进 (Forward)");
            runCarForward(CAR_SPEED);
            currentState = S_CAR_FORWARD;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;

    case 'B':
    case 'b':
        if (currentState == S_INIT || currentState == S_CAR_STOPPED)
        {
            Serial.println("命令: 后退 (Backward)");
            runCarBackward(CAR_SPEED);
            currentState = S_CAR_BACKWARD;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;

    case 'L':
    case 'l':
        if (currentState <= S_CAR_FORWARD)
        {
            Serial.println("命令: 展开 (Launch)");
            stopCar();
            currentState = S_ARM_LIFT_STEP1;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;

    case 'W':
    case 'w':
        if (currentState == S_CAR_BACKWARD || currentState == S_CAR_STOPPED)
        {
            Serial.println("命令: 收回 (Withdraw)");
            currentState = S_ARM_LOWER_STEP2;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;

    default:
        Serial.println("未知命令，请重新输入 (S/R/F/B/L/W)。");
        break;
    }
}

void initializePosition()
{
    liftMove(LIFT_INIT);
    endMove(END_INIT);
    delay(DELAY_TIME);
}

void positionStep1()
{
    liftMove(LIFT_STEP1);
    endMove(END_STEP1);
    delay(DELAY_TIME);
}

void positionStep2()
{
    liftMove(LIFT_STEP2);
    delay(DELAY_TIME);
}

void liftMove(int target)
{
    int current = armLift.read();
    int steps = abs(target - current);
    if (steps == 0)
    {
        return;
    }
    for (int i = 0; i <= steps; i++)
    {
        int interp = map(i, 0, steps, current, target);
        armLift.write(interp);
        delay(ARM_STEP_DELAY);
    }
}

void endMove(int target)
{
    int current = endLeft.read();
    int steps = abs(target - current);
    if (steps == 0)
    {
        return;
    }
    for (int i = 0; i <= steps; i++)
    {
        int interp = map(i, 0, steps, current, target);
        endLeft.write(interp);
        endRight.write(interp);
        delay(END_STEP_DELAY);
    }
}

void stopMotion()
{
}

void stopCar()
{
    analogWrite(ENA_PIN, 0);
    analogWrite(ENB_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
}

void runCarForward(int speed)
{
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENA_PIN, speed);
    analogWrite(ENB_PIN, speed);
}

void runCarBackward(int speed)
{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENA_PIN, speed);
    analogWrite(ENB_PIN, speed);
}