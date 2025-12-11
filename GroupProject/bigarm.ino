/*
 * bigarm.ino - Big-arm mechanical arm controller
 *
 * An individual file used only to test big-arm functions.
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

    initializePosition();
}

void loop()
{
    checkExternalCommand();
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
        initializePosition();
        break;
    case 'L':
    case 'l':
        Serial.println("命令: 展开大臂 (Launch Big Arm)");
        positionStep1();
        positionStep2();
        break;
    case 'W':
    case 'w':
        Serial.println("命令: 收回大臂 (Withdraw Big Arm)");
        positionStep2();
        positionStep1();
        break;
    default:
        Serial.println("未知命令，请重新输入 (R/L/W)。");
        break;
    }
}

void initializePosition()
{
    liftMove(LIFT_INIT);
    delay(DELAY_TIME);
}

void positionStep1()
{
    liftMove(LIFT_STEP1);
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