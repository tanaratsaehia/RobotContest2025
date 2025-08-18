/*
 * Real-time Robot Control
 * Responsive control with reduced speed
 */

#include <WiFi.h>
#include <esp_now.h>

// MAC Addresses
uint8_t controllerAddress[] = {0x7c, 0x87, 0xce, 0x2f, 0xe3, 0x20};

// Ultrasonic pins
#define TRIG_PIN   32
#define ECHO1_PIN  39  // Front
#define ECHO2_PIN  34  // Right
#define ECHO3_PIN  36  // Back
#define ECHO4_PIN  35  // Left

#define SOUND_SPEED 0.0343

// Motor pins
#define MOTOR_FRONT_A1 19
#define MOTOR_FRONT_A2 18
#define MOTOR_FRONT_B1 33
#define MOTOR_FRONT_B2 23
#define MOTOR_BACK_A1 4
#define MOTOR_BACK_A2 13
#define MOTOR_BACK_B1 16
#define MOTOR_BACK_B2 17

// Motor speed settings (reduced for better control)
#define SPEED_SLOW    80   // ความเร็วช้า
#define SPEED_NORMAL  120  // ความเร็วปกติ
#define SPEED_FAST    150  // ความเร็วเร็ว

// Global variables
volatile unsigned long echo_start[4] = {0, 0, 0, 0};
volatile unsigned long echo_end[4] = {0, 0, 0, 0};
volatile bool echo_done[4] = {false, false, false, false};
float sensor_distances[4] = {999.0, 999.0, 999.0, 999.0};

// Control variables
String currentCommand = "stop";
String lastCommand = "";
unsigned long commandStartTime = 0;
unsigned long lastCommandTime = 0;
int currentSpeed = SPEED_NORMAL;

unsigned long lastTriggerTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastSensorSendTime = 0;
bool esp_now_initialized = false;

// ESP-NOW message structure
typedef struct struct_message {
  char text[100];  // เพิ่มขนาดสำหรับข้อมูลเพิ่มเติม
} struct_message;

struct_message incoming;
struct_message outgoing;

// Motor control functions with variable speed
void stop_motor() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
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
}

// Strafe movements for mecanum wheels (if applicable)
void strafe_left() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, currentSpeed);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, currentSpeed);
  analogWrite(MOTOR_BACK_A1, currentSpeed);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, currentSpeed);
  analogWrite(MOTOR_BACK_B2, 0);
}

void strafe_right() {
  analogWrite(MOTOR_FRONT_A1, currentSpeed);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, currentSpeed);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, currentSpeed);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, currentSpeed);
}

// Execute motor command
void executeCommand(String command) {
  if (command == "forward") {
    move_forward();
  } else if (command == "backward") {
    move_backward();
  } else if (command == "turn_left" || command == "left") {
    turn_left();
  } else if (command == "turn_right" || command == "right") {
    turn_right();
  } else if (command == "strafe_left") {
    strafe_left();
  } else if (command == "strafe_right") {
    strafe_right();
  } else if (command == "stop") {
    stop_motor();
  } else {
    stop_motor();
  }
}

// Set motor speed
void setSpeed(String speedLevel) {
  if (speedLevel == "slow") {
    currentSpeed = SPEED_SLOW;
    Serial.println("[SPEED] Set to SLOW");
  } else if (speedLevel == "normal") {
    currentSpeed = SPEED_NORMAL;
    Serial.println("[SPEED] Set to NORMAL");
  } else if (speedLevel == "fast") {
    currentSpeed = SPEED_FAST;
    Serial.println("[SPEED] Set to FAST");
  }
}

// Ultrasonic interrupt handlers
void IRAM_ATTR echo1ISR() {
  if (digitalRead(ECHO1_PIN)) {
    echo_start[0] = micros();
  } else {
    echo_end[0] = micros();
    echo_done[0] = true;
  }
}

void IRAM_ATTR echo2ISR() {
  if (digitalRead(ECHO2_PIN)) {
    echo_start[1] = micros();
  } else {
    echo_end[1] = micros();
    echo_done[1] = true;
  }
}

void IRAM_ATTR echo3ISR() {
  if (digitalRead(ECHO3_PIN)) {
    echo_start[2] = micros();
  } else {
    echo_end[2] = micros();
    echo_done[2] = true;
  }
}

void IRAM_ATTR echo4ISR() {
  if (digitalRead(ECHO4_PIN)) {
    echo_start[3] = micros();
  } else {
    echo_end[3] = micros();
    echo_done[3] = true;
  }
}

// ESP-NOW callbacks
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  // ลดการแสดงผลเพื่อความเร็ว
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Send FAILED");
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  String command = String(incoming.text);
  
  lastCommandTime = millis();
  
  // Parse command and speed if included
  if (command.indexOf("speed:") != -1) {
    int speedIndex = command.indexOf("speed:");
    String speedLevel = command.substring(speedIndex + 6);
    speedLevel.trim();
    setSpeed(speedLevel);
    return;
  }
  
  // Update current command
  if (command != currentCommand) {
    Serial.print("[CMD] ");
    Serial.print(currentCommand);
    Serial.print(" -> ");
    Serial.println(command);
    
    currentCommand = command;
    commandStartTime = millis();
    
    // Execute immediately for real-time response
    executeCommand(currentCommand);
  }
}

void sendSensorData() {
  if (!esp_now_initialized) return;
  
  // Send sensor data every 100ms for real-time feedback
  snprintf(outgoing.text, sizeof(outgoing.text), 
           "SENSORS:%.1f,%.1f,%.1f,%.1f|CMD:%s|SPEED:%d", 
           sensor_distances[0], sensor_distances[1], 
           sensor_distances[2], sensor_distances[3],
           currentCommand.c_str(), currentSpeed);
  
  esp_now_send(controllerAddress, (uint8_t*)&outgoing, sizeof(outgoing));
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Reduced startup delay
  
  Serial.println("================================");
  Serial.println("  Real-time Robot Control");
  Serial.println("================================");
  
  // Initialize pins
  pinMode(MOTOR_FRONT_A1, OUTPUT);
  pinMode(MOTOR_FRONT_A2, OUTPUT);
  pinMode(MOTOR_FRONT_B1, OUTPUT);
  pinMode(MOTOR_FRONT_B2, OUTPUT);
  pinMode(MOTOR_BACK_A1, OUTPUT);
  pinMode(MOTOR_BACK_A2, OUTPUT);
  pinMode(MOTOR_BACK_B1, OUTPUT);
  pinMode(MOTOR_BACK_B2, OUTPUT);
  stop_motor();
  
  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(ECHO2_PIN, INPUT);
  pinMode(ECHO3_PIN, INPUT);
  pinMode(ECHO4_PIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ECHO1_PIN), echo1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO2_PIN), echo2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO3_PIN), echo3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO4_PIN), echo4ISR, CHANGE);
  
  // Initialize WiFi and ESP-NOW
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controllerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      esp_now_initialized = true;
      Serial.println("[ESP-NOW] Ready");
    }
  }
  
  pinMode(2, OUTPUT);
  
  Serial.println("================================");
  Serial.println("[READY] Real-time control active!");
  Serial.printf("[SPEED] Current: %d (slow:%d, normal:%d, fast:%d)\n", 
                currentSpeed, SPEED_SLOW, SPEED_NORMAL, SPEED_FAST);
  Serial.println("Commands: forward, backward, left, right, strafe_left, strafe_right, stop");
  Serial.println("Speed commands: speed:slow, speed:normal, speed:fast");
  Serial.println("================================");
}

void loop() {
  unsigned long now = millis();
  
  // Fast LED blink for activity indicator
  static bool ledState = false;
  static unsigned long lastBlink = 0;
  if (now - lastBlink >= 200) {  // Faster blink
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(2, ledState);
  }
  
  // Auto-stop if no command received for 1 second (safety feature)
  if (currentCommand != "stop" && (now - lastCommandTime > 1000)) {
    Serial.println("[SAFETY] Auto-stop - no commands");
    currentCommand = "stop";
    stop_motor();
  }
  
  // Continue executing current command
  if (currentCommand != "stop") {
    executeCommand(currentCommand);
  }
  
  // Trigger ultrasonic sensors more frequently
  if (now - lastTriggerTime >= 50) {  // 50ms = 20Hz
    lastTriggerTime = now;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
  
  // Process sensor readings
  for (int i = 0; i < 4; i++) {
    if (echo_done[i]) {
      noInterrupts();
      unsigned long duration = echo_end[i] - echo_start[i];
      echo_done[i] = false;
      interrupts();
      
      float distance = (duration * SOUND_SPEED) / 2.0;
      if (distance > 2 && distance < 400) {
        sensor_distances[i] = distance;
      } else {
        sensor_distances[i] = 999.0;
      }
    }
  }
  
  // Send sensor data every 100ms for real-time feedback
  if (now - lastSensorSendTime >= 100) {
    lastSensorSendTime = now;
    sendSensorData();
  }
  
  // Status every 5 seconds
  if (now - lastStatusTime >= 5000) {
    lastStatusTime = now;
    Serial.printf("[STATUS] CMD:%s, SPEED:%d, SENSORS:%.1f,%.1f,%.1f,%.1f\n", 
                  currentCommand.c_str(), currentSpeed,
                  sensor_distances[0], sensor_distances[1], 
                  sensor_distances[2], sensor_distances[3]);
  }
  
  // Minimal delay for maximum responsiveness
  delay(5);
}