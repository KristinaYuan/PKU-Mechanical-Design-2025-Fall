/*
 * engine.ino - Glass-cleaner elevator controller
 *
 * This program operates 3 servos controlling mechanical arms and 4 engines
 * controlling L298N on a car to carry a window-cleaner from ground to wall.
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
 *  - Implemented the backward process.
 *  - Added controls for car operations.
 *
 * Updated 12/12/2025:
 *  - Used only three pins to control the car movement.
 *  - Used more intelligent position functions to fix several bugs.
 *
 * Updated 12/19/2025:
 *  - Altered the state machine logic to avoid unwanted signal interference.
 *  - Adjusted to the new angles and arm lengths.
 *  - Implemented the printHelp function.
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

/* Define car pins */
#define ENA_PIN 6
#define IN1_PIN 5
#define IN2_PIN 4

/* Define car speed */
#define CAR_SPEED 200

/* Define system states */
enum ProgramState
{
    S_INIT,            // arms initializing
    S_CAR_STOPPED,     // car stopped with arms retracted or stretched
    S_CAR_FORWARD,     // car moving forward
    S_CAR_BACKWARD,    // car moving backward
    S_ARM_LIFT_STEP1,  // arm lifting to middle position
    S_ARM_LIFT_STEP2,  // arm lifting to stretched position
    S_ARM_LOWER_STEP2, // arm lowering to middle position
    S_ARM_LOWER_STEP1, // arm lowering to retracted position
    S_IDLE             // idle state
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
}

void loop()
{
    checkExternalCommand();

    switch (currentState)
    {
    case S_INIT:
        initializePosition();
        currentState = S_CAR_STOPPED;
        break;
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
    Serial.println("机械臂电梯控制程序已启动。");
    Serial.println("------------------------------------------");
    Serial.println("输入 'L' -> 机械臂展开 (0->45->20)");
    Serial.println("输入 'W' -> 机械臂收回 (20->45->0)");
    Serial.println("输入 'R' -> 重启 (返回 0度初始位置)");
    Serial.println("输入 'S' -> 停止 (停止当前动作)");
    Serial.println("输入 'F' -> 前进 (小车前进)");
    Serial.println("输入 'B' -> 后退 (小车后退)");
    Serial.println("------------------------------------------");
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
            currentState = S_CAR_STOPPED;
        }
        else if (currentState >= S_ARM_LIFT_STEP1 && currentState <= S_ARM_LOWER_STEP1)
        {
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
        currentState = S_INIT;
        break;

    case 'F':
    case 'f':
        if (currentState == S_CAR_STOPPED)
        {
            Serial.println("命令: 前进 (Forward)");
            currentState = S_CAR_FORWARD;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;

    case 'B':
    case 'b':
        if (currentState == S_CAR_STOPPED)
        {
            Serial.println("命令: 后退 (Backward)");
            currentState = S_CAR_BACKWARD;
        }
        else
        {
            Serial.println("此状态下命令不可使用，请重新输入。");
        }
        break;

    case 'L':
    case 'l':
        if (currentState == S_CAR_FORWARD || currentState == S_CAR_STOPPED)
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
            stopCar();
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

void stopCar()
{
    analogWrite(ENA_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
}

void runCarForward(int speed)
{
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    analogWrite(ENA_PIN, speed);
}

void runCarBackward(int speed)
{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    analogWrite(ENA_PIN, speed);
}