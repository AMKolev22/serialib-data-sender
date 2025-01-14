#include <Servo.h>

const int motor1Pin1 = 3;
const int motor1Pin2 = 4;
const int motor2Pin1 = 5;
const int motor2Pin2 = 6;
const int enaPin = 10;
const int enbPin = 11;
const int servoPin = 9;

Servo myservo;

bool shouldMove = false;

unsigned long lastReadTime = 0;
const unsigned long timeout = 50;

unsigned long lastServoMoveTime = 0;
unsigned long servoMoveInterval = 1000;
int currentServoAngle = 90;

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enaPin, OUTPUT);
  pinMode(enbPin, OUTPUT);

  myservo.attach(servoPin);
  myservo.write(currentServoAngle);

  Serial.begin(115200);

  analogWrite(enaPin, 0);
  analogWrite(enbPin, 0);
}

void controlMotor1(bool forward, int speed) {
  if (forward) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  }
  analogWrite(enaPin, speed);
}

void controlMotor2(bool forward, int speed) {
  if (forward) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
  analogWrite(enbPin, speed);
}

void loop() {
 controlMotor1(true, 100);
  controlMotor2(true, 100);
}


