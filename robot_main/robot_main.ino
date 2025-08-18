/*
 * Step-by-Step Robot Controller
 * กดครั้งหนึ่ง เคลื่อนที่ทีละนิด (ไม่ต้องกดค้าง)
 */

#include "BluetoothSerial.h"
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// Motor pins
#define MOTOR_FRONT_A1 19
#define MOTOR_FRONT_A2 18
#define MOTOR_FRONT_B1 33
#define MOTOR_FRONT_B2 23
#define MOTOR_BACK_A1 4
#define MOTOR_BACK_A2 13
#define MOTOR_BACK_B1 16
#define MOTOR_BACK_B2 17

// Servo pin
#define SERVO_PIN 25

// Speed settings
#define SPEED_SLOW    100
#define SPEED_NORMAL  140
#define SPEED_FAST    180

// Movement duration for each step (milliseconds)
#define STEP_DURATION 300  // เคลื่อนที่ 300ms ต่อครั้ง

// Servo settings
#define SERVO_HOME    10 // ตำแหน่งเริ่มต้น
#define SERVO_DROP    100   // ตำแหน่งทิ้งของ
#define SERVO_RETURN_TIME 2000  // กลับภายใน 2 วินาที

// Global variables
int currentSpeed = SPEED_NORMAL;
int servoPosition = SERVO_HOME;
String lastCommand = "NONE";
unsigned long stepStartTime = 0;
unsigned long servoDropTime = 0;
bool isMoving = false;
bool servoAutoReturn = false;

Servo dropServo;

// Motor control functions
void stop_motor() {
  // ใช้ analogWrite(pin, 0) แทน digitalWrite เพื่อให้หยุดจริงๆ
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
  isMoving = false;
  Serial.println("[MOTOR] ALL STOPPED with analogWrite(0)");
}

void move_forward() {
  analogWrite(MOTOR_FRONT_A1, currentSpeed);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, currentSpeed);
  analogWrite(MOTOR_BACK_A1, currentSpeed);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, currentSpeed);
  isMoving = true;
}

void move_backward() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, currentSpeed);
  analogWrite(MOTOR_FRONT_B1, currentSpeed);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, currentSpeed);
  analogWrite(MOTOR_BACK_B1, currentSpeed);
  analogWrite(MOTOR_BACK_B2, 0);
  isMoving = true;
}

void turn_left() {
  analogWrite(MOTOR_FRONT_A1, currentSpeed);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, currentSpeed);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, currentSpeed);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, currentSpeed);
  analogWrite(MOTOR_BACK_B2, 0);
  isMoving = true;
}

void turn_right() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, currentSpeed);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, currentSpeed);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, currentSpeed);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, currentSpeed);
  isMoving = true;
}

// Diagonal movements
void move_front_right() {
  analogWrite(MOTOR_FRONT_A1, currentSpeed);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, currentSpeed);
  isMoving = true;
}

void move_front_left() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, currentSpeed);
  analogWrite(MOTOR_BACK_A1, currentSpeed);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
  isMoving = true;
}

void move_back_right() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, currentSpeed);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, currentSpeed);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
  isMoving = true;
}

void move_back_left() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, currentSpeed);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, currentSpeed);
  analogWrite(MOTOR_BACK_B2, 0);
  isMoving = true;
}

void setServo(int angle) {
  angle = constrain(angle, 0, 180);
  dropServo.write(angle);
  servoPosition = angle;
  Serial.printf("[SERVO] %d°\n", angle);
  SerialBT.printf("Servo: %d°\n", angle);
}

void startMovement(String direction) {
  // หยุด motor ก่อน แล้วค่อยเริ่มใหม่
  stop_motor();
  delay(50);  // รอให้หยุดสนิท
  
  stepStartTime = millis();
  lastCommand = direction;
  
  // เริ่มการเคลื่อนไหวตามทิศทาง
  if (direction == "F") {
    move_forward();
    SerialBT.println(">>> STEP FORWARD");
  }
  else if (direction == "B") {
    move_backward();
    SerialBT.println(">>> STEP BACKWARD");
  }
  else if (direction == "L") {
    turn_left();
    SerialBT.println(">>> STEP LEFT");
  }
  else if (direction == "R") {
    turn_right();
    SerialBT.println(">>> STEP RIGHT");
  }
  else if (direction == "FR") {
    move_front_right();
    SerialBT.println(">>> STEP FRONT-RIGHT");
  }
  else if (direction == "FL") {
    move_front_left();
    SerialBT.println(">>> STEP FRONT-LEFT");
  }
  else if (direction == "BR") {
    move_back_right();
    SerialBT.println(">>> STEP BACK-RIGHT");
  }
  else if (direction == "BL") {
    move_back_left();
    SerialBT.println(">>> STEP BACK-LEFT");
  }
  
  Serial.printf("[MOTOR] Started %s for %dms\n", direction.c_str(), STEP_DURATION);
}

void checkMovementStep() {
  // ตรวจสอบว่าถึงเวลาหยุดแล้วหรือยัง
  if (isMoving && (millis() - stepStartTime >= STEP_DURATION)) {
    stop_motor();
    Serial.printf("[STEP] AUTO-STOP after %dms\n", STEP_DURATION);
    SerialBT.println("*** AUTO STOPPED ***");
  }
}

void checkServoAutoReturn() {
  // ตรวจสอบ servo auto-return
  if (servoAutoReturn && (millis() - servoDropTime >= SERVO_RETURN_TIME)) {
    setServo(SERVO_HOME);
    servoAutoReturn = false;
    SerialBT.printf("Auto-return to %d° after 2s\n", SERVO_HOME);
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  Serial.printf("[CMD] %s\n", cmd.c_str());
  
  // Movement commands - กดครั้งหนึ่ง เคลื่อนที่ทีละนิด
  if (cmd == "F" || cmd == "FORWARD" || cmd == "W") {
    startMovement("F");
  }
  else if (cmd == "B" || cmd == "BACKWARD") {
    startMovement("B");
  }
  else if (cmd == "L" || cmd == "LEFT" || cmd == "A") {
    startMovement("L");
  }
  else if (cmd == "R" || cmd == "RIGHT" || cmd == "D") {
    startMovement("R");
  }
  
  // Diagonal movement commands
  else if (cmd == "FR" || cmd == "FRONTRIGHT") {
    startMovement("FR");
  }
  else if (cmd == "FL" || cmd == "FRONTLEFT") {
    startMovement("FL");
  }
  else if (cmd == "BR" || cmd == "BACKRIGHT") {
    startMovement("BR");
  }
  else if (cmd == "BL" || cmd == "BACKLEFT") {
    startMovement("BL");
  }
  
  // Stop command - บังคับหยุดทันที
  else if (cmd == "STOP" || cmd == "X" || cmd == "S") {
    stop_motor();
    servoAutoReturn = false;  // ยกเลิก servo auto-return ด้วย
    Serial.println("[CMD] EMERGENCY STOP!");
    SerialBT.println("*** EMERGENCY STOP ***");
  }
  
  // Speed commands
  else if (cmd == "S1" || cmd == "SLOW") {
    currentSpeed = SPEED_SLOW;
    SerialBT.printf("Speed: S1-SLOW (%d)\n", SPEED_SLOW);
  }
  else if (cmd == "S2" || cmd == "NORMAL") {
    currentSpeed = SPEED_NORMAL;
    SerialBT.printf("Speed: S2-NORMAL (%d)\n", SPEED_NORMAL);
  }
  else if (cmd == "S3" || cmd == "FAST") {
    currentSpeed = SPEED_FAST;
    SerialBT.printf("Speed: S3-FAST (%d)\n", SPEED_FAST);
  }
  else if (cmd == "Z") {
    // Cycle through speeds
    if (currentSpeed == SPEED_SLOW) {
      currentSpeed = SPEED_NORMAL;
      SerialBT.printf("Speed: S2-NORMAL (%d)\n", SPEED_NORMAL);
    } else if (currentSpeed == SPEED_NORMAL) {
      currentSpeed = SPEED_FAST;
      SerialBT.printf("Speed: S3-FAST (%d)\n", SPEED_FAST);
    } else {
      currentSpeed = SPEED_SLOW;
      SerialBT.printf("Speed: S1-SLOW (%d)\n", SPEED_SLOW);
    }
  }
  
  // Servo commands
  else if (cmd == "Y") {
    // Servo หมุนไป Drop แล้วกลับเอง ใน 2 วินาที
    setServo(SERVO_DROP);
    servoDropTime = millis();
    servoAutoReturn = true;
    SerialBT.printf("Servo: %d° → %d° (auto-return in 2s)\n", SERVO_HOME, SERVO_DROP);
  }
  else if (cmd == "DROP") {
    setServo(SERVO_DROP);
    servoAutoReturn = false;  // ยกเลิก auto-return
    SerialBT.println("Manual drop (no auto-return)");
  }
  else if (cmd == "RESET" || cmd == "HOME") {
    setServo(SERVO_HOME);
    servoAutoReturn = false;  // ยกเลิก auto-return
    SerialBT.println("Reset to home");
  }
  else if (cmd.startsWith("SERVO:")) {
    int angle = cmd.substring(6).toInt();
    if (angle >= 0 && angle <= 180) {
      setServo(angle);
      servoAutoReturn = false;  // ยกเลิก auto-return
    } else {
      SerialBT.println("Invalid angle! (0-180)");
    }
  }
  
  // Time adjustment commands
  else if (cmd.startsWith("TIME:")) {
    int newTime = cmd.substring(5).toInt();
    if (newTime >= 50 && newTime <= 2000) {
      // อัปเดต STEP_DURATION ไม่ได้เพราะเป็น #define
      // แต่สามารถแสดงค่าปัจจุบันได้
      SerialBT.printf("Step duration: %dms (fixed)\n", STEP_DURATION);
      SerialBT.println("To change: modify STEP_DURATION in code");
    } else {
      SerialBT.println("Invalid time! (50-2000ms)");
    }
  }
  
  // System commands
  else if (cmd == "STATUS") {
    SerialBT.printf("Last: %s | Speed: %d | Servo: %d° | Moving: %s\n", 
                    lastCommand.c_str(), currentSpeed, servoPosition,
                    isMoving ? "YES" : "NO");
    SerialBT.printf("Step duration: %dms | Servo auto-return: %s\n", 
                    STEP_DURATION, servoAutoReturn ? "ON" : "OFF");
  }
  else if (cmd == "HELP") {
    SerialBT.println("\n=== STEP-BY-STEP CONTROLLER ===");
    SerialBT.println("Move: F/B/L/R/FR/FL/BR/BL");
    SerialBT.println("Stop: STOP/X/S (force stop)");
    SerialBT.println("Speed: S1/S2/S3/Z(cycle)");
    SerialBT.println("Servo: Y(auto-return)/DROP/HOME");
    SerialBT.println("System: STATUS/HELP");
    SerialBT.printf("Each step: %dms, Servo return: %dms\n", STEP_DURATION, SERVO_RETURN_TIME);
    SerialBT.println("=== PRESS ONCE TO MOVE ===");
  }
  
  else {
    SerialBT.printf("Unknown: %s\n", cmd.c_str());
    SerialBT.println("Type HELP for commands");
  }
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("=== Step-by-Step Robot Controller ===");
  Serial.printf("Step duration: %dms per command\n", STEP_DURATION);
  
  // Initialize motor pins
  pinMode(MOTOR_FRONT_A1, OUTPUT);
  pinMode(MOTOR_FRONT_A2, OUTPUT);
  pinMode(MOTOR_FRONT_B1, OUTPUT);
  pinMode(MOTOR_FRONT_B2, OUTPUT);
  pinMode(MOTOR_BACK_A1, OUTPUT);
  pinMode(MOTOR_BACK_A2, OUTPUT);
  pinMode(MOTOR_BACK_B1, OUTPUT);
  pinMode(MOTOR_BACK_B2, OUTPUT);
  
  stop_motor();
  Serial.println("[INIT] Motors ready");
  
  // Initialize servo at home position
  dropServo.attach(SERVO_PIN);
  dropServo.write(SERVO_HOME);
  servoPosition = SERVO_HOME;
  Serial.printf("[INIT] Servo ready at %d°\n", SERVO_HOME);
  
  // Initialize status LED
  pinMode(2, OUTPUT);
  
  // Initialize Bluetooth
  if (SerialBT.begin("Robot_Controller")) {
    Serial.println("[BT] Started - Robot_Controller");
  } else {
    Serial.println("[BT] Failed!");
    while(1);
  }
  
  Serial.println("=== READY ===");
  Serial.println("Press once to move step-by-step");
  Serial.println("Commands: F/B/L/R for basic movement");
  Serial.println("FR/FL/BR/BL for diagonal movement");
}

void loop() {
  unsigned long now = millis();
  
  // Status LED - blink fast when moving, slow when stopped
  static bool ledState = false;
  static unsigned long lastBlink = 0;
  int blinkInterval = isMoving ? 100 : 1000;
  
  if (now - lastBlink >= blinkInterval) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(2, ledState);
  }
  
  // Process Bluetooth commands
  if (SerialBT.available()) {
    String command = SerialBT.readString();
    processCommand(command);
  }
  
  // Check if movement step is completed
  checkMovementStep();
  
  // Check servo auto-return
  checkServoAutoReturn();
  
  // Status report every 5 seconds when not moving
  static unsigned long lastStatus = 0;
  if (!isMoving && (now - lastStatus >= 5000)) {
    lastStatus = now;
    Serial.printf("[STATUS] Ready | Last: %s | Speed: %d\n", 
                  lastCommand.c_str(), currentSpeed);
  }
  
  // Small delay for stability
  delay(10);
}