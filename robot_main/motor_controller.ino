
#define MOTOR_FRONT_A1 19 // right
#define MOTOR_FRONT_A2 18 // right
#define MOTOR_FRONT_B1 33 // left
#define MOTOR_FRONT_B2 23 // left
#define MOTOR_BACK_A1 4  // right
#define MOTOR_BACK_A2 13 // right
#define MOTOR_BACK_B1 16 // left
#define MOTOR_BACK_B2 17 // left

#define SERVO_PIN1 26
#define SERVO_PIN2 25

const int MOTOR_SPEED = 90; // 0-100 %
const float TURN_FORWARD_RATIO = 0.45;

void motor_begin(){
  servo_left.attach(SERVO_PIN1);
  servo_right.attach(SERVO_PIN2);

  pinMode(MOTOR_FRONT_A1, OUTPUT);
  pinMode(MOTOR_FRONT_A2, OUTPUT);
  pinMode(MOTOR_FRONT_B1, OUTPUT);
  pinMode(MOTOR_FRONT_B2, OUTPUT);
  pinMode(MOTOR_BACK_A1, OUTPUT);
  pinMode(MOTOR_BACK_A2, OUTPUT);
  pinMode(MOTOR_BACK_B1, OUTPUT);
  pinMode(MOTOR_BACK_B2, OUTPUT);

  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
}


void move_motor(int action){
  if (action == 0){
    analogWrite(MOTOR_FRONT_A1, 0);
    analogWrite(MOTOR_FRONT_A2, 0);
    analogWrite(MOTOR_FRONT_B1, 0);
    analogWrite(MOTOR_FRONT_B2, 0);
    analogWrite(MOTOR_BACK_A1, 0);
    analogWrite(MOTOR_BACK_A2, 0);
    analogWrite(MOTOR_BACK_B1, 0);
    analogWrite(MOTOR_BACK_B2, 0);
  }else if (action == 1){
    analogWrite(MOTOR_FRONT_A1, 200);
    analogWrite(MOTOR_FRONT_A2, 0);
    analogWrite(MOTOR_FRONT_B1, 0);
    analogWrite(MOTOR_FRONT_B2, 200);
    analogWrite(MOTOR_BACK_A1, 200);
    analogWrite(MOTOR_BACK_A2, 0);
    analogWrite(MOTOR_BACK_B1, 0);
    analogWrite(MOTOR_BACK_B2, 200);
  }else if (action == 2){
    analogWrite(MOTOR_FRONT_A1, 0);
    analogWrite(MOTOR_FRONT_A2, 200);
    analogWrite(MOTOR_FRONT_B1, 200);
    analogWrite(MOTOR_FRONT_B2, 0);
    analogWrite(MOTOR_BACK_A1, 0);
    analogWrite(MOTOR_BACK_A2, 200);
    analogWrite(MOTOR_BACK_B1, 200);
    analogWrite(MOTOR_BACK_B2, 0);
  }else{
    Serial.println("Motor wrong action: " + String(action));
  }
}

void move_motor_with_command(const char* command){
  if (strcmp(command, "forward") == 0) {
    Serial.println("MOVE MOTOR FORWARD");
    // move_forward(MOTOR_SPEED);
    calibrate_gyro_bias();      // <— add this
    last_imu_us = micros();
    straight_start(MOTOR_SPEED);
  } else if (strcmp(command, "backward") == 0) {
    Serial.println("MOVE MOTOR BACKWARD");
    move_backward(MOTOR_SPEED);
  } else if (strcmp(command, "turn_left") == 0) {
    Serial.println("MOVE MOTOR TURN LEFT");
    // turn_left(MOTOR_SPEED);
    turn_left_backward(MOTOR_SPEED);
    // turn_left_forward(MOTOR_SPEED);
  } else if (strcmp(command, "turn_right") == 0) {
    Serial.println("MOVE MOTOR TURN LEFT");
    // turn_right(MOTOR_SPEED);
    turn_right_backward(MOTOR_SPEED);
    // turn_right_forward(MOTOR_SP[EED);
  } else {
    Serial.println("STOP MOTOR");
    stop_motor();
    straight_stop();
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

void move_forward(int speed_percent){
  int duty = ((float)speed_percent/100)*255;
  analogWrite(MOTOR_FRONT_A1, duty);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, duty);
  analogWrite(MOTOR_BACK_A1, duty);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, duty);
}

void move_backward(int speed_percent){
  int duty = ((float)speed_percent/100)*255;
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, duty);
  analogWrite(MOTOR_FRONT_B1, duty);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, duty);
  analogWrite(MOTOR_BACK_B1, duty);
  analogWrite(MOTOR_BACK_B2, 0);
}

void stop_motor(){
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
}

void turn_left_stop(int speed_percent){
  int duty = ((float)speed_percent/100)*255;
  analogWrite(MOTOR_FRONT_A1, duty);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, duty);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
}

void turn_left_forward(int speed_percent){
  int duty = ((float)speed_percent/100)*255;
  analogWrite(MOTOR_FRONT_A1, duty);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, int(duty*TURN_FORWARD_RATIO));
  analogWrite(MOTOR_BACK_A1, duty);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, int(duty*TURN_FORWARD_RATIO));
}

void turn_left_backward(int speed_percent){
  int duty = ((float)speed_percent/100)*255;
  analogWrite(MOTOR_FRONT_A1, duty);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, duty);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, duty);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, duty);
  analogWrite(MOTOR_BACK_B2, 0);
}

void turn_right_stop(int speed_percent){
  int duty = ((float)speed_percent/100)*255;
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, duty);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, duty);
}

void turn_right_forward(int speed_percent){
  int duty = ((float)speed_percent/100)*255;
  analogWrite(MOTOR_FRONT_A1, int(duty*TURN_FORWARD_RATIO));
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, duty);
  analogWrite(MOTOR_BACK_A1, int(duty*TURN_FORWARD_RATIO));
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, duty);
}

void turn_right_backward(int speed_percent){
  int duty = ((float)speed_percent/100)*255;
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, duty);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, duty);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, duty);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, duty);
}




// imu implements

// Forward duty per side (your mapping from move_forward)
void set_forward_side_pwm(uint8_t left_pwm, uint8_t right_pwm) {
  // Right side = A1 pins forward
  analogWrite(MOTOR_FRONT_A1, left_pwm);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_BACK_A1,  left_pwm);
  analogWrite(MOTOR_BACK_A2,  0);

  // Left side = B2 pins forward
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, right_pwm);
  analogWrite(MOTOR_BACK_B1,  0);
  analogWrite(MOTOR_BACK_B2,  right_pwm);
}
