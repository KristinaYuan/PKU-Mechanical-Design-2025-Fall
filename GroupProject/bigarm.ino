/*
 * bigarm.ino - Big-arm mechanical arm controller
 *
 * An individual file used only to test big-arm functions.
 *
 * Updated 12/11/2025:
 * - Split from engine.ino to separate the big-arm control logic.
 *
 * Updated 12/12/2025:
 *  - Added a state machine to avoid infinite loops of arm movement.
 *  - Adjusted the command handling logic accordingly.
 *  - Used more intelligent position functions to fix several bugs.
 */
#include <Servo.h>

/* Define servo object */
Servo armLift; // servo at bottom the the arms

/* Define servo pins */
#define ARM_LIFT_PIN 9 // PWB pin for main lifting servo

/* Define mechanical parameters */
#define BIG_ARM_LENGTH 25.0 // big arm length (centimeter)

/* Define gear ratio */
#define ARM_GEAR_RATIO 2.0 // servo:bigarm = 2:1 (decelerate)

/* Define mechanical angles */
/* Big arm's relative angle to horizontal level */
#define MECH_LIFT_INIT 0
#define MECH_LIFT_STEP1 45
#define MECH_LIFT_STEP2 37

enum ProgramState
{
    S_INIT, // initializing state
    S_ARM_LIFT_STEP1,
    S_ARM_LIFT_STEP2,
    S_ARM_LOWER_STEP2,
    S_ARM_LOWER_STEP1,
    S_IDLE1, // idle state at retracted position
    S_IDLE2  // idle state at stretched position
};

ProgramState currentState = S_INIT;

/* Calculate servo angles */
inline int mechLiftToServo(int mechAngle)
{
    return 90 + mechAngle * ARM_GEAR_RATIO;
}

/* Define servo angles */
#define LIFT_INIT mechLiftToServo(MECH_LIFT_INIT)
#define LIFT_STEP1 mechLiftToServo(MECH_LIFT_STEP1)
#define LIFT_STEP2 mechLiftToServo(MECH_LIFT_STEP2)

/* Define time parameters */
#define ARM_STEP_DELAY 15
#define DELAY_TIME 1500

/* Declare functions */
void setup();
void loop();
void printHelp();
void setServoPins();
void checkExternalCommand();
void initializePosition();
void positionStep1();
void positionStep2();
void liftMove(int target);

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
    Serial.println("大臂机械臂测试程序已启动。");
    Serial.println("------------------------------------------");
    Serial.println("输入 'L' -> 序列展开 (0->45->37)");
    Serial.println("输入 'W' -> 序列收回 (37->45->0)");
    Serial.println("输入 'R' -> 重启 (返回 0度初始位置)");
    Serial.println("------------------------------------------");
}

void setServoPins()
{
    armLift.attach(ARM_LIFT_PIN);
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
        Serial.println("命令: 展开大臂 (Launch Big Arm)");
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
        Serial.println("命令: 收回大臂 (Withdraw Big Arm)");
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
    int current = armLift.read();
    if (current == LIFT_INIT)
    {
        return;
    }
    if (current == LIFT_STEP1)
    {
        liftMove(LIFT_INIT);
        delay(DELAY_TIME);
        return;
    }
    if (current == LIFT_STEP2)
    {
        liftMove(LIFT_STEP1);
        delay(500);
        liftMove(LIFT_INIT);
        delay(DELAY_TIME);
        return;
    }
    liftMove(LIFT_INIT);
    delay(DELAY_TIME);
}

void positionStep1()
{
    if (armLift.read() == LIFT_INIT)
    {
        liftMove(LIFT_STEP1);
    }
    else
    {
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