/*
 * Real-time Robot Control
 * Responsive control with improved speed control and space bar stop
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

// Motor speed settings (adjusted for reliable operation)
#define SPEED_SLOW    100  // ความเร็วช้า
#define SPEED_NORMAL  140  // ความเร็วปกติ
#define SPEED_FAST    180  // ความเร็วเร็ว

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

// Enhanced motor stop function with forced stop capability
void stop_motor() {
  // Clear all PWM signals to motors
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
  
  // Add small delay to ensure PWM signals are applied
  delayMicroseconds(100);
  
  // Force digital LOW as backup method for emergency stops
  digitalWrite(MOTOR_FRONT_A1, LOW);
  digitalWrite(MOTOR_FRONT_A2, LOW);
  digitalWrite(MOTOR_FRONT_B1, LOW);
  digitalWrite(MOTOR_FRONT_B2, LOW);
  digitalWrite(MOTOR_BACK_A1, LOW);
  digitalWrite(MOTOR_BACK_A2, LOW);
  digitalWrite(MOTOR_BACK_B1, LOW);
  digitalWrite(MOTOR_BACK_B2, LOW);
}

// Motor control functions with variable speed
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

// Strafe movements for mecanum wheels
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

// Execute motor command with improved reliability
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
    // Default to stop for any unknown command for safety
    stop_motor();
  }
}

// Enhanced speed setting function with detailed feedback
void setSpeed(String speedLevel) {
  speedLevel.toLowerCase(); // Convert to lowercase for consistent comparison
  speedLevel.trim(); // Remove any whitespace
  
  Serial.print("[SPEED] Processing speed command: '");
  Serial.print(speedLevel);
  Serial.println("'");
  
  if (speedLevel == "slow") {
    currentSpeed = SPEED_SLOW;
    Serial.printf("[SPEED] ✓ Set to SLOW (%d)\n", SPEED_SLOW);
  } else if (speedLevel == "normal") {
    currentSpeed = SPEED_NORMAL;
    Serial.printf("[SPEED] ✓ Set to NORMAL (%d)\n", SPEED_NORMAL);
  } else if (speedLevel == "fast") {
    currentSpeed = SPEED_FAST;
    Serial.printf("[SPEED] ✓ Set to FAST (%d)\n", SPEED_FAST);
  } else {
    Serial.print("[SPEED] ✗ Unknown speed level: '");
    Serial.print(speedLevel);
    Serial.println("' - keeping current speed");
  }
  
  Serial.printf("[SPEED] Current speed is now: %d\n", currentSpeed);
}

// Ultrasonic interrupt handlers (unchanged)
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

// ESP-NOW send callback (unchanged)
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Send FAILED");
  }
}

// Enhanced ESP-NOW receive callback with proper command parsing
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  String command = String(incoming.text);
  command.trim(); // Remove any whitespace that might cause issues
  
  lastCommandTime = millis();
  
  // Debug output to track all received commands
  Serial.print("[DEBUG] Received command: '");
  Serial.print(command);
  Serial.println("'");
  
  // Handle STOP command with highest priority for safety
  if (command == "stop") {
    Serial.println("[EMERGENCY] STOP command received!");
    currentCommand = "stop";
    stop_motor(); // Execute stop immediately
    commandStartTime = millis();
    return; // Exit function immediately after stop
  }
  
  // Handle speed commands with careful parsing
  if (command.startsWith("speed:")) {
    String speedLevel = command.substring(6); // Extract everything after "speed:"
    speedLevel.trim(); // Remove any extra whitespace
    
    Serial.print("[SPEED] Extracted speed level: '");
    Serial.print(speedLevel);
    Serial.println("'");
    
    setSpeed(speedLevel);
    
    // Send confirmation back to controller
    Serial.println("[SPEED] Speed change completed");
    return; // Exit function after processing speed command
  }
  
  // Handle movement commands
  if (command == "forward" || command == "backward" || 
      command == "left" || command == "right" || 
      command == "turn_left" || command == "turn_right" ||
      command == "strafe_left" || command == "strafe_right") {
    
    // Only change command if it's different from current command
    if (command != currentCommand) {
      Serial.print("[MOVEMENT] Command change: ");
      Serial.print(currentCommand);
      Serial.print(" -> ");
      Serial.println(command);
      
      currentCommand = command;
      commandStartTime = millis();
      
      // Execute the new movement command immediately
      executeCommand(currentCommand);
    }
    return; // Exit function after processing movement command
  }
  
  // Handle any unrecognized commands
  Serial.print("[WARNING] Unrecognized command: '");
  Serial.print(command);
  Serial.println("' - ignoring");
}

// Send sensor data function (unchanged)
void sendSensorData() {
  if (!esp_now_initialized) return;
  
  // Send comprehensive sensor data and status information
  snprintf(outgoing.text, sizeof(outgoing.text), 
           "SENSORS:%.1f,%.1f,%.1f,%.1f|CMD:%s|SPEED:%d", 
           sensor_distances[0], sensor_distances[1], 
           sensor_distances[2], sensor_distances[3],
           currentCommand.c_str(), currentSpeed);
  
  esp_now_send(controllerAddress, (uint8_t*)&outgoing, sizeof(outgoing));
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Allow serial connection to stabilize
  
  Serial.println("================================");
  Serial.println("  Real-time Robot Control v2.0");
  Serial.println("  Enhanced Speed Control & Stop");
  Serial.println("================================");
  
  // Initialize all motor control pins as outputs
  pinMode(MOTOR_FRONT_A1, OUTPUT);
  pinMode(MOTOR_FRONT_A2, OUTPUT);
  pinMode(MOTOR_FRONT_B1, OUTPUT);
  pinMode(MOTOR_FRONT_B2, OUTPUT);
  pinMode(MOTOR_BACK_A1, OUTPUT);
  pinMode(MOTOR_BACK_A2, OUTPUT);
  pinMode(MOTOR_BACK_B1, OUTPUT);
  pinMode(MOTOR_BACK_B2, OUTPUT);
  
  // Ensure all motors start in stopped state
  stop_motor();
  Serial.println("[MOTORS] All motors initialized and stopped");
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(ECHO2_PIN, INPUT);
  pinMode(ECHO3_PIN, INPUT);
  pinMode(ECHO4_PIN, INPUT);
  
  // Attach interrupt handlers for ultrasonic sensors
  attachInterrupt(digitalPinToInterrupt(ECHO1_PIN), echo1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO2_PIN), echo2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO3_PIN), echo3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO4_PIN), echo4ISR, CHANGE);
  Serial.println("[SENSORS] Ultrasonic sensors initialized");
  
  // Initialize WiFi in station mode for ESP-NOW
  WiFi.mode(WIFI_STA);
  
  // Initialize ESP-NOW communication
  if (esp_now_init() == ESP_OK) {
    Serial.println("[ESP-NOW] Protocol initialized successfully");
    
    // Register callback functions
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    // Add the controller as a peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controllerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      esp_now_initialized = true;
      Serial.println("[ESP-NOW] Controller peer added successfully");
    } else {
      Serial.println("[ESP-NOW] Failed to add controller peer");
    }
  } else {
    Serial.println("[ESP-NOW] Failed to initialize protocol");
  }
  
  // Initialize status LED
  pinMode(2, OUTPUT);
  
  Serial.println("================================");
  Serial.println("[READY] Robot control system active!");
  Serial.printf("[SPEED] Current: %d (slow:%d, normal:%d, fast:%d)\n", 
                currentSpeed, SPEED_SLOW, SPEED_NORMAL, SPEED_FAST);
  Serial.println("[CONTROLS] Available commands:");
  Serial.println("  Movement: forward, backward, left, right, strafe_left, strafe_right");
  Serial.println("  Control: stop (SPACE BAR)");
  Serial.println("  Speed: speed:slow (1), speed:normal (2), speed:fast (3)");
  Serial.println("================================");
}

void loop() {
  unsigned long now = millis();
  
  // Status LED blink to indicate system activity
  static bool ledState = false;
  static unsigned long lastBlink = 0;
  if (now - lastBlink >= 200) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(2, ledState);
  }
  
  // Safety auto-stop if no commands received for extended period
  if (currentCommand != "stop" && (now - lastCommandTime > 1000)) {
    Serial.println("[SAFETY] Auto-stop activated - no commands received for 1 second");
    currentCommand = "stop";
    stop_motor();
  }
  
  // Prioritize STOP command checking in main loop for safety
  if (currentCommand == "stop") {
    stop_motor(); // Continuously ensure motors are stopped
  } else {
    // Execute current movement command if not stopping
    executeCommand(currentCommand);
  }
  
  // Trigger ultrasonic sensors at regular intervals
  if (now - lastTriggerTime >= 50) {  // 20Hz sensor update rate
    lastTriggerTime = now;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
  
  // Process ultrasonic sensor readings
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
        sensor_distances[i] = 999.0; // Invalid reading indicator
      }
    }
  }
  
  // Send sensor data and status updates to controller
  if (now - lastSensorSendTime >= 100) {  // 10Hz data transmission rate
    lastSensorSendTime = now;
    sendSensorData();
  }
  
  // Periodic status report for debugging
  if (now - lastStatusTime >= 3000) {  // Every 3 seconds instead of 5
    lastStatusTime = now;
    Serial.printf("[STATUS] CMD:%s, SPEED:%d, UPTIME:%lu, SENSORS:%.1f,%.1f,%.1f,%.1f\n", 
                  currentCommand.c_str(), currentSpeed, now,
                  sensor_distances[0], sensor_distances[1], 
                  sensor_distances[2], sensor_distances[3]);
  }
  
  // Minimal delay for maximum system responsiveness
  delay(5);
}