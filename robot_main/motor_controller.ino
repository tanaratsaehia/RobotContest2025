
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

#define MOTOR_SPEED 90

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

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

  digitalWrite(MOTOR_FRONT_A1, LOW);
  digitalWrite(MOTOR_FRONT_A2, LOW);
  digitalWrite(MOTOR_FRONT_B1, LOW);
  digitalWrite(MOTOR_FRONT_B2, LOW);
  digitalWrite(MOTOR_BACK_A1, LOW);
  digitalWrite(MOTOR_BACK_A2, LOW);
  digitalWrite(MOTOR_BACK_B1, LOW);
  digitalWrite(MOTOR_BACK_B2, LOW);
  
}


void move_motor(int action){
  if (action == 0){
    digitalWrite(MOTOR_FRONT_A1, LOW);
    digitalWrite(MOTOR_FRONT_A2, LOW);
    digitalWrite(MOTOR_FRONT_B1, LOW);
    digitalWrite(MOTOR_FRONT_B2, LOW);
    digitalWrite(MOTOR_BACK_A1, LOW);
    digitalWrite(MOTOR_BACK_A2, LOW);
    digitalWrite(MOTOR_BACK_B1, LOW);
    digitalWrite(MOTOR_BACK_B2, LOW);
  }else if (action == 1){
    digitalWrite(MOTOR_FRONT_A1, HIGH);
    digitalWrite(MOTOR_FRONT_A2, LOW);
    digitalWrite(MOTOR_FRONT_B1, LOW);
    digitalWrite(MOTOR_FRONT_B2, HIGH);
    digitalWrite(MOTOR_BACK_A1, HIGH);
    digitalWrite(MOTOR_BACK_A2, LOW);
    digitalWrite(MOTOR_BACK_B1, LOW);
    digitalWrite(MOTOR_BACK_B2, HIGH);
  }else if (action == 2){
    digitalWrite(MOTOR_FRONT_A1, LOW);
    digitalWrite(MOTOR_FRONT_A2, HIGH);
    digitalWrite(MOTOR_FRONT_B1, HIGH);
    digitalWrite(MOTOR_FRONT_B2, LOW);
    digitalWrite(MOTOR_BACK_A1, LOW);
    digitalWrite(MOTOR_BACK_A2, HIGH);
    digitalWrite(MOTOR_BACK_B1, HIGH);
    digitalWrite(MOTOR_BACK_B2, LOW);
  }else{
    Serial.println("Motor wrong action: " + String(action));
  }
}

void move_motor_with_command(const char* command){
  if (strcmp(command, "forward") == 0) {
    Serial.println("MOVE MOTOR FORWARD");
    move_forward(MOTOR_SPEED);
  } else if (strcmp(command, "backward") == 0) {
    Serial.println("MOVE MOTOR BACKWARD");
    move_backward(MOTOR_SPEED);
  } else if (strcmp(command, "turn_left") == 0) {
    Serial.println("MOVE MOTOR TURN LEFT");
    turn_left(MOTOR_SPEED);
  } else if (strcmp(command, "turn_right") == 0) {
    Serial.println("MOVE MOTOR TURN LEFT");
    turn_right(MOTOR_SPEED);
  } else {
    Serial.println("STOP MOTOR");
    stop_motor();
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}

void move_forward(int speed_percent){
  int percent2analog = int((speed_percent/100)*255);
  // Serial.println("Motor speed: " + String(percent2analog));
  // analogWrite(MOTOR_FRONT_A1, percent2analog);
  // digitalWrite(MOTOR_FRONT_A2, LOW);
  // digitalWrite(MOTOR_FRONT_B1, LOW);
  // analogWrite(MOTOR_FRONT_B2, percent2analog);
  // analogWrite(MOTOR_BACK_A1, percent2analog);
  // digitalWrite(MOTOR_BACK_A2, LOW);
  // digitalWrite(MOTOR_BACK_B1, LOW);
  // analogWrite(MOTOR_BACK_B2, percent2analog);
  digitalWrite(MOTOR_FRONT_A1, HIGH);
  digitalWrite(MOTOR_FRONT_A2, LOW);
  digitalWrite(MOTOR_FRONT_B1, LOW);
  digitalWrite(MOTOR_FRONT_B2, HIGH);
  digitalWrite(MOTOR_BACK_A1, HIGH);
  digitalWrite(MOTOR_BACK_A2, LOW);
  digitalWrite(MOTOR_BACK_B1, LOW);
  digitalWrite(MOTOR_BACK_B2, HIGH);
}

void move_backward(int speed_percent){
  int percent2analog = int((speed_percent/100)*255);
  // Serial.println("Motor speed: " + String(percent2analog));
  // digitalWrite(MOTOR_FRONT_A1, LOW);
  // analogWrite(MOTOR_FRONT_A2, percent2analog);
  // analogWrite(MOTOR_FRONT_B1, percent2analog);
  // digitalWrite(MOTOR_FRONT_B2, LOW);
  // digitalWrite(MOTOR_BACK_A1, LOW);
  // analogWrite(MOTOR_BACK_A2, percent2analog);
  // analogWrite(MOTOR_BACK_B1, percent2analog);
  // digitalWrite(MOTOR_BACK_B2, LOW);
  digitalWrite(MOTOR_FRONT_A1, LOW);
  digitalWrite(MOTOR_FRONT_A2, HIGH);
  digitalWrite(MOTOR_FRONT_B1, HIGH);
  digitalWrite(MOTOR_FRONT_B2, LOW);
  digitalWrite(MOTOR_BACK_A1, LOW);
  digitalWrite(MOTOR_BACK_A2, HIGH);
  digitalWrite(MOTOR_BACK_B1, HIGH);
  digitalWrite(MOTOR_BACK_B2, LOW);
}

void stop_motor(){
  digitalWrite(MOTOR_FRONT_A1, LOW);
  digitalWrite(MOTOR_FRONT_A2, LOW);
  digitalWrite(MOTOR_FRONT_B1, LOW);
  digitalWrite(MOTOR_FRONT_B2, LOW);
  digitalWrite(MOTOR_BACK_A1, LOW);
  digitalWrite(MOTOR_BACK_A2, LOW);
  digitalWrite(MOTOR_BACK_B1, LOW);
  digitalWrite(MOTOR_BACK_B2, LOW);
}

void turn_left(int speed_percent){
  int percent2analog = int((speed_percent/100)*255);
}

void turn_right(int speed_percent){
  int percent2analog = int((speed_percent/100)*255);
}