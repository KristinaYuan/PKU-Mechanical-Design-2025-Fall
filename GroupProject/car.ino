/*
 * car.ino - Car-moving controller
 *
 * An individual file used only to test L298N functions.
 */
/* Define pins */
#define ENA_PIN 6
#define IN1_PIN 5
#define IN2_PIN 4
#define ENB_PIN 3
#define IN3_PIN 2
#define IN4_PIN 12

/* Define car speed */
#define CAR_SPEED 200

void setCarPins();
void stopCar();
void printHelp();
void runCarForward(int speed);
void runCarBackward(int speed);

void setup()
{
    setCarPins();

    stopCar();

    Serial.begin(9600);

    printHelp();
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

void stopCar()
{
    analogWrite(ENA_PIN, 0);
    analogWrite(ENB_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
}

void printHelp()
{
    Serial.println("L298N 小车全功能测试程序已启动。");
    Serial.println("------------------------------------------");
    Serial.println("输入 'F' -> 前进 (Forward)");
    Serial.println("输入 'B' -> 后退 (Backward)");
    Serial.println("输入 'S' -> 停止 (Stop)");
    Serial.println("------------------------------------------");
}

void loop()
{
    if (Serial.available())
    {
        char cmd = Serial.read();
        switch (cmd)
        {
        case 'F':
        case 'f':
            Serial.println("命令: 前进 (Forward)");
            runCarForward(CAR_SPEED);
            break;

        case 'B':
        case 'b':
            Serial.println("命令: 后退 (Backward)");
            runCarBackward(CAR_SPEED);
            break;

        case 'S':
        case 's':
            Serial.println("命令: 停止 (Stop)");
            stopCar();
            break;

        default:
            Serial.println("未知命令，请重新输入 (F/B/S)。");
            break;
        }
    }
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