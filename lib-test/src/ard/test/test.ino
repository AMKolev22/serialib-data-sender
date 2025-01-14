#include <Servo.h>

// Motor pin definitions
const int motor1Pin1 = 3;
const int motor1Pin2 = 4;
const int motor2Pin1 = 5;
const int motor2Pin2 = 6;
const int enaPin = 10;
const int enbPin = 11;

// Servo pin
const int servoPin = 9;

// Servo and movement variables
Servo myservo;
bool shouldMove = false;
unsigned long lastReadTime = 0;
const unsigned long timeout = 50;
unsigned long lastServoMoveTime = 0;
unsigned long servoMoveInterval = 1000;
int currentServoAngle = 90;

// Function to control motor direction and speed
void controlMotor(int pin1, int pin2, int enablePin, bool forward, int speed) {
  digitalWrite(pin1, forward ? HIGH : LOW); // Control motor direction
  digitalWrite(pin2, forward ? LOW : HIGH); // Control motor direction
  analogWrite(enablePin, forward ? speed : 0); // Enable motor with speed or stop it
}

// Function to stop both motors
void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enaPin, 0); // Disable motor driver 1
  analogWrite(enbPin, 0); // Disable motor driver 2
}

// Function to move the servo back and forth
void moveServo() {
  unsigned long currentTime = millis();
  if (currentTime - lastServoMoveTime >= servoMoveInterval) {
    currentServoAngle = (currentServoAngle == 90) ? 45 : 90; // Alternate angle
    myservo.write(currentServoAngle);
    lastServoMoveTime = currentTime;
  }
}

void setup() {
  // Initialize motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enaPin, OUTPUT);
  pinMode(enbPin, OUTPUT);

  // Initialize servo
  myservo.attach(servoPin);
  myservo.write(currentServoAngle);  // Set servo to initial position (default 90°)

  // Start Serial communication
  Serial.begin(115200);

  // Stop motors initially (disabled by default)
  stopMotors();
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Handle commands
    if (command == "move") {
      Serial.println("Motors moving forward.");
      controlMotor(motor1Pin1, motor1Pin2, enaPin, true, 150);
      controlMotor(motor2Pin1, motor2Pin2, enbPin, true, 150);
      shouldMove = true;
    } else if (command == "stop") {
      Serial.println("Motors stopped.");
      stopMotors();
      shouldMove = false;
    } else {
      Serial.println("Unknown command.");
    }
  }

  // Move the servo at intervals
  
}
