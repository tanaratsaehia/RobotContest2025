// motors.ino - Differential Drive Motor Control

// ==================== MOTOR INITIALIZATION ====================
void initializeMotors() {
  // Configure all motor pins as outputs
  pinMode(MOTOR_FRONT_A1, OUTPUT);
  pinMode(MOTOR_FRONT_A2, OUTPUT);
  pinMode(MOTOR_FRONT_B1, OUTPUT);
  pinMode(MOTOR_FRONT_B2, OUTPUT);
  pinMode(MOTOR_BACK_A1, OUTPUT);
  pinMode(MOTOR_BACK_A2, OUTPUT);
  pinMode(MOTOR_BACK_B1, OUTPUT);
  pinMode(MOTOR_BACK_B2, OUTPUT);
  
  // Initialize all motors to stop
  stopMotors();
  
  DEBUG_PRINTLN("✅ Motors initialized");
}

// ==================== BASIC MOTOR CONTROL ====================
void stopMotors() {
  // Turn off all motors
  digitalWrite(MOTOR_FRONT_A1, LOW);
  digitalWrite(MOTOR_FRONT_A2, LOW);
  digitalWrite(MOTOR_FRONT_B1, LOW);
  digitalWrite(MOTOR_FRONT_B2, LOW);
  digitalWrite(MOTOR_BACK_A1, LOW);
  digitalWrite(MOTOR_BACK_A2, LOW);
  digitalWrite(MOTOR_BACK_B1, LOW);
  digitalWrite(MOTOR_BACK_B2, LOW);
  
  moveState = MOVE_STOP;
}

void moveForward(int speed) {
  speed = CLAMP(speed, 0, 100);
  int pwmValue = map(speed, 0, 100, 0, 255);
  
  // Right motors forward
  analogWrite(MOTOR_FRONT_A1, pwmValue);
  digitalWrite(MOTOR_FRONT_A2, LOW);
  analogWrite(MOTOR_BACK_A1, pwmValue);
  digitalWrite(MOTOR_BACK_A2, LOW);
  
  // Left motors forward
  digitalWrite(MOTOR_FRONT_B1, LOW);
  analogWrite(MOTOR_FRONT_B2, pwmValue);
  digitalWrite(MOTOR_BACK_B1, LOW);
  analogWrite(MOTOR_BACK_B2, pwmValue);
  
  moveState = MOVE_FORWARD;
  updateRobotPosition(speed, 0);
}

void moveBackward(int speed) {
  speed = CLAMP(speed, 0, 100);
  int pwmValue = map(speed, 0, 100, 0, 255);
  
  // Right motors backward
  digitalWrite(MOTOR_FRONT_A1, LOW);
  analogWrite(MOTOR_FRONT_A2, pwmValue);
  digitalWrite(MOTOR_BACK_A1, LOW);
  analogWrite(MOTOR_BACK_A2, pwmValue);
  
  // Left motors backward
  analogWrite(MOTOR_FRONT_B1, pwmValue);
  digitalWrite(MOTOR_FRONT_B2, LOW);
  analogWrite(MOTOR_BACK_B1, pwmValue);
  digitalWrite(MOTOR_BACK_B2, LOW);
  
  moveState = MOVE_BACKWARD;
  updateRobotPosition(-speed, 0);
}

void turnLeft(int speed)