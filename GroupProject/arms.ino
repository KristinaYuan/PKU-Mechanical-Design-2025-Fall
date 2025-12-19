/*
 * arms.ino - Mechanical arms controller
 *
 * An individual file used only to test big and small arms functions.
 *
 * Updated 12/19/2025:
 * - Split from engine.ino to separate the arms control logic.
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
#define BIG_ARM_LENGTH 15.0  // big arm length (centimeter)
#define SMALL_ARM_LENGTH 8.0 // small arm length

/* Define gear ratio */
#define ARM_GEAR_RATIO 2.0 // servo:bigarm = 2:1 (decelerate)
#define END_GEAR_RATIO 1.0 // end drives directly 1:1

/* Define mechanical angles */
/* Big arm's relative angle to horizontal level */
#define MECH_LIFT_INIT 0
#define MECH_LIFT_STEP1 45
#define MECH_LIFT_STEP2 20

/* Small arm's relative angle to big arm (inside) */
#define MECH_END_INIT 0
#define MECH_END_STEP1 110
#define MECH_END_STEP2 110

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

/* Define system states */
enum ProgramState
{
    S_INIT,            // arms initializing
    S_ARM_LIFT_STEP1,  // arm lifting to middle position
    S_ARM_LIFT_STEP2,  // arm lifting to stretched position
    S_ARM_LOWER_STEP2, // arm lowering to middle position
    S_ARM_LOWER_STEP1, // arm lowering to retracted position
    S_IDLE1,           // idle state at retracted position
    S_IDLE2            // idle state at stretched position
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

/* Function implementation */

void setup()
{
    Serial.begin(9600);
    printHelp();

    setServoPins();
}

void loop()
{
    checkExternalCommand();

    switch (currentState)
    {
    case S_INIT:
        initializePosition();
        currentState = S_IDLE1;
        break;
    case S_ARM_LIFT_STEP1:
        positionStep1();
        currentState = S_ARM_LIFT_STEP2;
        break;
    case S_ARM_LIFT_STEP2:
        positionStep2();
        currentState = S_IDLE2;
        break;
    case S_ARM_LOWER_STEP2:
        positionStep2();
        currentState = S_ARM_LOWER_STEP1;
        break;
    case S_ARM_LOWER_STEP1:
        positionStep1();
        currentState = S_IDLE1;
        break;
    case S_IDLE1:
    case S_IDLE2:
        break;
    }
}

void printHelp()
{
    Serial.println("机械臂测试程序已启动。");
    Serial.println("------------------------------------------");
    Serial.println("输入 'L' -> 序列展开 (0->45->37)");
    Serial.println("输入 'W' -> 序列收回 (37->45->0)");
    Serial.println("输入 'R' -> 重启 (返回 0度初始位置)");
    Serial.println("------------------------------------------");
}

void setServoPins()
{
    armLift.attach(ARM_LIFT_PIN);
    endLeft.attach(END_LEFT_PIN);
    endRight.attach(END_RIGHT_PIN);
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
    case 'R':
    case 'r':
        Serial.println("命令: 重启 (Restart)");
        currentState = S_INIT;
        break;
    case 'L':
    case 'l':
        Serial.println("命令: 展开机械臂 (Launch Arms)");
        if (currentState == S_IDLE1)
        {
            currentState = S_ARM_LIFT_STEP1;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;
    case 'W':
    case 'w':
        Serial.println("命令: 收回机械臂 (Withdraw Arms)");
        if (currentState == S_IDLE2)
        {
            currentState = S_ARM_LOWER_STEP2;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;
    default:
        Serial.println("未知命令，请重新输入 (R/L/W)。");
        break;
    }
}

void initializePosition()
{
    int currentL = armLift.read();
    if (currentL == LIFT_INIT)
    {
        return;
    }
    if (currentL == LIFT_STEP1)
    {
        endMove(END_INIT);
        delay(500);
        liftMove(LIFT_INIT);
        delay(DELAY_TIME);
        return;
    }
    if (currentL == LIFT_STEP2)
    {
        liftMove(LIFT_STEP1);
        delay(500);
        endMove(END_INIT);
        delay(500);
        liftMove(LIFT_INIT);
        delay(DELAY_TIME);
        return;
    }
    endMove(END_INIT);
    delay(500);
    liftMove(LIFT_INIT);
    delay(DELAY_TIME);
}

void positionStep1()
{
    if (armLift.read() == LIFT_INIT)
    {
        liftMove(LIFT_STEP1);
        delay(500);
        endMove(END_STEP1);
    }
    else
    {
        endMove(END_INIT);
        delay(500);
        liftMove(LIFT_INIT);
    }
    delay(DELAY_TIME);
}

void positionStep2()
{
    if (armLift.read() == LIFT_STEP1)
    {
        liftMove(LIFT_STEP2);
    }
    else
    {
        liftMove(LIFT_STEP1);
    }
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