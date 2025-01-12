bool isMoving = false; 

void setup() {
  Serial.begin(115200); 
}

void loop() {
  static char buffer[5];
  if (Serial.available()) {
    size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[len] = '\0';

    String command = String(buffer);
    if (command == "move" && !isMoving) {
      isMoving = true;
      Serial.write("moving\n");
    } 
    else if (command == "stop" && isMoving) {
      isMoving = false;
      Serial.write("stopping\n");
    }
  }
  delay(30);
}




