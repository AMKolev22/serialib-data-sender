#include <Servo.h>

const int motor1Pin1 = 3;
const int motor1Pin2 = 4;
const int motor2Pin1 = 5;
const int motor2Pin2 = 6;
const int enaPin = 10;
const int enbPin = 11;
const int servoPin = 9;

Servo myservo;
int targetSpeed = 0;
int currentSpeed = 0;
const int speedStep = 10;
const int minSpeed = 100;
const int stopThreshold = 50;

unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 20;

void controlMotor(int pin1, int pin2, int enablePin, bool forward, int speed) {
    digitalWrite(pin1, forward ? HIGH : LOW);
    digitalWrite(pin2, forward ? LOW : HIGH);
    analogWrite(enablePin, speed);
}

void updateMotorSpeed() {
    if (abs(currentSpeed - targetSpeed) <= speedStep) {
        currentSpeed = targetSpeed;
    } else if (currentSpeed < targetSpeed) {
        currentSpeed += speedStep;
    } else {
        currentSpeed -= speedStep;
    }
    
    bool forward = currentSpeed >= 0;
    int absSpeed = abs(currentSpeed);
    
    controlMotor(motor1Pin1, motor1Pin2, enaPin, forward, absSpeed);
    controlMotor(motor2Pin1, motor2Pin2, enbPin, forward, absSpeed);
}

void processLookaheadPoint(int x, int y) {
    int servoAngle = constrain(map(x, 0, 640, 45, 135), 45, 135);
    myservo.write(servoAngle);

    if (y < stopThreshold) {
        targetSpeed = 0;
    } else {
        targetSpeed = map(constrain(y, 0, 480), 0, 480, minSpeed, 255);
    }
    
    Serial.print("Processed: x=");
    Serial.print(x);
    Serial.print(", y=");
    Serial.println(y);
}

void processCommand(String command) {
    command.trim();
    
    if (command == "stop") {
        targetSpeed = 0;
        Serial.println("Stopping");
        return;
    }
    
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
        String xStr = command.substring(0, commaIndex);
        String yStr = command.substring(commaIndex + 1);
        
        int x = xStr.toInt();
        int y = yStr.toInt();
        
        if (x >= 0 && x <= 640 && y >= 0 && y <= 480) {
            processLookaheadPoint(x, y);
        } else {
            Serial.println("Invalid coordinates");
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(enaPin, OUTPUT);
    pinMode(enbPin, OUTPUT);
    
    myservo.attach(servoPin);
    myservo.write(90);
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
        updateMotorSpeed();
    }
    
} 