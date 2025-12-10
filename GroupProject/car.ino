/*
 * car.ino - Car-moving controller
 *
 * An individual file used only to test L298N functions.
 */
/* Define pins */
#define ENL_PIN 6
#define INLF_PIN 5
#define INLB_PIN 4
#define ENR_PIN 3
#define INRF_PIN 2
#define INRB_PIN 12

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
    pinMode(ENL_PIN, OUTPUT);
    pinMode(INLF_PIN, OUTPUT);
    pinMode(INLB_PIN, OUTPUT);
    pinMode(ENR_PIN, OUTPUT);
    pinMode(INRF_PIN, OUTPUT);
    pinMode(INRB_PIN, OUTPUT);
}

void stopCar()
{
    analogWrite(ENL_PIN, 0);
    analogWrite(ENR_PIN, 0);
    digitalWrite(INLF_PIN, LOW);
    digitalWrite(INLB_PIN, LOW);
    digitalWrite(INRF_PIN, LOW);
    digitalWrite(INRB_PIN, LOW);
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
    digitalWrite(INLF_PIN, HIGH);
    digitalWrite(INLB_PIN, LOW);
    digitalWrite(INRF_PIN, HIGH);
    digitalWrite(INRB_PIN, LOW);
    analogWrite(ENL_PIN, speed);
    analogWrite(ENR_PIN, speed);
}

void runCarBackward(int speed)
{
    digitalWrite(INLF_PIN, LOW);
    digitalWrite(INLB_PIN, HIGH);
    digitalWrite(INRF_PIN, LOW);
    digitalWrite(INRB_PIN, HIGH);
    analogWrite(ENL_PIN, speed);
    analogWrite(ENR_PIN, speed);
}