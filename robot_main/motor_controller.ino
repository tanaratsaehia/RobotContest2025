// motor_controller.ino - ระบบควบคุมมอเตอร์แบบ Differential Drive

#define MOTOR_FRONT_A1 19 // Front right motor +
#define MOTOR_FRONT_A2 18 // Front right motor -
#define MOTOR_FRONT_B1 33 // Front left motor +
#define MOTOR_FRONT_B2 23 // Front left motor -
#define MOTOR_BACK_A1 4   // Back right motor +
#define MOTOR_BACK_A2 13  // Back right motor -
#define MOTOR_BACK_B1 16  // Back left motor +
#define MOTOR_BACK_B2 17  // Back left motor -

#define SERVO_PIN1 26
#define SERVO_PIN2 25

// Motor calibration values (adjust these for your robot)
#define LEFT_MOTOR_OFFSET 1.05   // Left motor compensation
#define RIGHT_MOTOR_OFFSET 1.00  // Right motor baseline
#define MIN_PWM_VALUE 60         // Minimum PWM for movement
#define MAX_PWM_VALUE 255        // Maximum PWM value

// PID constants for straight line movement
#define KP_STRAIGHT 2.0
#define KI_STRAIGHT 0.1
#define KD_STRAIGHT 0.5

// Global variables for PID control
float previousError = 0;
float integral = 0;

void motor_begin() {
  // Attach servos
  servo_left.attach(SERVO_PIN1);
  servo_right.attach(SERVO_PIN2);
  
  // Set all motor pins as output
  pinMode(MOTOR_FRONT_A1, OUTPUT);
  pinMode(MOTOR_FRONT_A2, OUTPUT);
  pinMode(MOTOR_FRONT_B1, OUTPUT);
  pinMode(MOTOR_FRONT_B2, OUTPUT);
  pinMode(MOTOR_BACK_A1, OUTPUT);
  pinMode(MOTOR_BACK_A2, OUTPUT);
  pinMode(MOTOR_BACK_B1, OUTPUT);
  pinMode(MOTOR_BACK_B2, OUTPUT);
  
  // Initialize all motors to stop
  stop_motor();
  
  Serial.println("✅ Motors initialized");
}

// Convert speed percentage to PWM value with calibration
int speedToPWM(int speedPercent, bool isLeftMotor) {
  float offset = isLeftMotor ? LEFT_MOTOR_OFFSET : RIGHT_MOTOR_OFFSET;
  int pwmValue = (speedPercent / 100.0) * MAX_PWM_VALUE * offset;
  
  // Apply minimum threshold
  if (speedPercent > 0 && pwmValue < MIN_PWM_VALUE) {
    pwmValue = MIN_PWM_VALUE;
  }
  
  return constrain(pwmValue, 0, MAX_PWM_VALUE);
}

// Differential drive control for precise movement
void move_motors_differential(int leftSpeed, int rightSpeed) {
  int leftPWM = speedToPWM(leftSpeed, true);
  int rightPWM = speedToPWM(rightSpeed, false);
  
  // Left motors
  if (leftSpeed > 0) {
    analogWrite(MOTOR_FRONT_B2, leftPWM);
    analogWrite(MOTOR_FRONT_B1, 0);
    analogWrite(MOTOR_BACK_B2, leftPWM);
    analogWrite(MOTOR_BACK_B1, 0);
  } else if (leftSpeed < 0) {
    analogWrite(MOTOR_FRONT_B1, leftPWM);
    analogWrite(MOTOR_FRONT_B2, 0);
    analogWrite(MOTOR_BACK_B1, leftPWM);
    analogWrite(MOTOR_BACK_B2, 0);
  } else {
    analogWrite(MOTOR_FRONT_B1, 0);
    analogWrite(MOTOR_FRONT_B2, 0);
    analogWrite(MOTOR_BACK_B1, 0);
    analogWrite(MOTOR_BACK_B2, 0);
  }
  
  // Right motors
  if (rightSpeed > 0) {
    analogWrite(MOTOR_FRONT_A1, rightPWM);
    analogWrite(MOTOR_FRONT_A2, 0);
    analogWrite(MOTOR_BACK_A1, rightPWM);
    analogWrite(MOTOR_BACK_A2, 0);
  } else if (rightSpeed < 0) {
    analogWrite(MOTOR_FRONT_A2, rightPWM);
    analogWrite(MOTOR_FRONT_A1, 0);
    analogWrite(MOTOR_BACK_A2, rightPWM);
    analogWrite(MOTOR_BACK_A1, 0);
  } else {
    analogWrite(MOTOR_FRONT_A1, 0);
    analogWrite(MOTOR_FRONT_A2, 0);
    analogWrite(MOTOR_BACK_A1, 0);
    analogWrite(MOTOR_BACK_A2, 0);
  }
}

// Forward movement with optional gyro correction
void move_forward(int speedPercent) {
  int pwmValue = speedToPWM(speedPercent, false);
  int leftPWM = speedToPWM(speedPercent, true);
  
  // Apply forward motion to all motors
  analogWrite(MOTOR_FRONT_A1, pwmValue);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, leftPWM);
  analogWrite(MOTOR_BACK_A1, pwmValue);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, leftPWM);
}

// Backward movement
void move_backward(int speedPercent) {
  int pwmValue = speedToPWM(speedPercent, false);
  int leftPWM = speedToPWM(speedPercent, true);
  
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, pwmValue);
  analogWrite(MOTOR_FRONT_B1, leftPWM);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, pwmValue);
  analogWrite(MOTOR_BACK_B1, leftPWM);
  analogWrite(MOTOR_BACK_B2, 0);
}

// Turn left - pivot turn
void turn_left(int speedPercent) {
  int pwmValue = speedToPWM(speedPercent, false);
  
  // Right motors forward, left motors stop/slow
  analogWrite(MOTOR_FRONT_A1, pwmValue);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, pwmValue);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
}

// Turn right - pivot turn
void turn_right(int speedPercent) {
  int pwmValue = speedToPWM(speedPercent, true);
  
  // Left motors forward, right motors stop/slow
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, pwmValue);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, pwmValue);
}

// Rotate in place (tank turn)
void rotate_left(int speedPercent) {
  int pwmValue = speedToPWM(speedPercent, false);
  int leftPWM = speedToPWM(speedPercent, true);
  
  // Left motors backward, right motors forward
  analogWrite(MOTOR_FRONT_A1, pwmValue);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, leftPWM);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, pwmValue);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, leftPWM);
  analogWrite(MOTOR_BACK_B2, 0);
}

void rotate_right(int speedPercent) {
  int pwmValue = speedToPWM(speedPercent, false);
  int leftPWM = speedToPWM(speedPercent, true);
  
  // Left motors forward, right motors backward
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, pwmValue);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, leftPWM);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, pwmValue);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, leftPWM);
}

// Complete stop
void stop_motor() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
  
  // Reset PID values
  integral = 0;
  previousError = 0;
}

// Smooth acceleration/deceleration
void move_forward_smooth(int targetSpeed, int duration) {
  int steps = 20;
  int stepDelay = duration / steps;
  
  for (int i = 0; i <= steps; i++) {
    int currentSpeed = (targetSpeed * i) / steps;
    move_forward(currentSpeed);
    delay(stepDelay);
  }
}

// PID control for straight line movement
void move_forward_with_pid(int baseSpeed, float headingError) {
  // PID calculation
  integral += headingError;
  integral = constrain(integral, -100, 100); // Prevent integral windup
  
  float derivative = headingError - previousError;
  float correction = (KP_STRAIGHT * headingError) + 
                    (KI_STRAIGHT * integral) + 
                    (KD_STRAIGHT * derivative);
  
  previousError = headingError;
  
  // Apply correction to motor speeds
  int leftSpeed = baseSpeed - (int)correction;
  int rightSpeed = baseSpeed + (int)correction;
  
  // Ensure speeds are within valid range
  leftSpeed = constrain(leftSpeed, 0, 100);
  rightSpeed = constrain(rightSpeed, 0, 100);
  
  move_motors_differential(leftSpeed, rightSpeed);
}

// Process movement commands from ESP-NOW
void move_motor_with_command(const char* command) {
  Serial.print("Executing command: ");
  Serial.println(command);
  
  if (strcmp(command, "forward") == 0) {
    move_forward(70);
    isMovingStraight = true;
    targetHeading = robotHeading; // Set current heading as target
  } else if (strcmp(command, "backward") == 0) {
    move_backward(70);
    isMovingStraight = false;
  } else if (strcmp(command, "turn_left") == 0) {
    turn_left(70);
    isMovingStraight = false;
  } else if (strcmp(command, "turn_right") == 0) {
    turn_right(70);
    isMovingStraight = false;
  } else if (strcmp(command, "rotate_left") == 0) {
    rotate_left(60);
    isMovingStraight = false;
  } else if (strcmp(command, "rotate_right") == 0) {
    rotate_right(60);
    isMovingStraight = false;
  } else if (strcmp(command, "stop") == 0) {
    stop_motor();
    isMovingStraight = false;
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
    stop_motor();
  }
}

// Test motor functionality
void test_motors() {
  Serial.println("Testing motors...");
  
  Serial.println("Forward");
  move_forward(70);
  delay(1000);
  
  Serial.println("Backward");
  move_backward(70);
  delay(1000);
  
  Serial.println("Left");
  turn_left(70);
  delay(1000);
  
  Serial.println("Right");
  turn_right(70);
  delay(1000);
  
  Serial.println("Stop");
  stop_motor();
  
  Serial.println("Motor test complete");
}