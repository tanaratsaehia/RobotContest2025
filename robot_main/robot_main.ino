<<<<<<< Updated upstream
<<<<<<< Updated upstream
<<<<<<< Updated upstream
// communication.ino - ESP-NOW Communication System (FIXED)
=======
/*
 * OPTIMIZED Robot Main Code
 * - เซ็นเซอร์ส่งข้อมูลเร็วขึ้น (ทุก 50ms)
 * - Map ขนาดเล็กลง (50x50)
 * - ข้อมูลส่งไวขึ้น (ทุก 100ms)
 */
>>>>>>> Stashed changes

// Global message structures
ESPNowMessage outgoingMsg;
ESPNowMessage incomingMsg;

// ==================== ESP-NOW INITIALIZATION ====================
void initializeESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  DEBUG_PRINTLN("📡 Initializing ESP-NOW...");
  DEBUG_PRINTLN("Robot MAC: " + WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    DEBUG_PRINTLN("❌ ESP-NOW initialization failed");
    return;
  }
  
  // Register callbacks - FIXED VERSION
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add control station as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, stationAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    DEBUG_PRINTLN("❌ Failed to add control station peer");
    return;
  }
  
  DEBUG_PRINT("✅ ESP-NOW initialized, Station MAC: ");
  for (int i = 0; i < 6; i++) {
    DEBUG_PRINT(String(stationAddress[i], HEX));
    if (i < 5) DEBUG_PRINT(":");
  }
  DEBUG_PRINTLN("");
}

// ==================== FIXED ESP-NOW CALLBACKS ====================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG_MODE && status != ESP_NOW_SEND_SUCCESS) {
    DEBUG_PRINTLN("📤 Send failed to: " + macToString(mac_addr));
  }
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ESPNowMessage)) {
    memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
    
    DEBUG_PRINTLN("📨 Received: " + String(incomingMsg.type) + " - " + String(incomingMsg.data));
    
    // Process the received message
    processIncomingMessage();
  }
}

// ==================== MESSAGE PROCESSING ====================
void processIncomingMessage() {
  String msgType = String(incomingMsg.type);
  String msgData = String(incomingMsg.data);
  
  if (msgType == "CMD") {
    processCommand(msgData);
  } else if (msgType == "WAYPOINT") {
    processWaypoint(msgData);
  } else if (msgType == "MISSION") {
    processMissionCommand(msgData);
  } else if (msgType == "CONFIG") {
    processConfig(msgData);
  } else {
    DEBUG_PRINTLN("❓ Unknown message type: " + msgType);
  }
  
  // Send acknowledgment
  sendAcknowledgment(msgType);
}

void processCommand(String command) {
  if (command == "START_MAPPING") {
    startMapping();
  } else if (command == "STOP_MAPPING") {
    stopMapping();
  } else if (command == "START_MISSION") {
    startMission();
  } else if (command == "STOP_MISSION") {
    stopMission();
  } else if (command == "EMERGENCY_STOP") {
    emergencyStop();
  } else if (command == "RESET") {
    resetSystem();
  } else if (command == "GET_STATUS") {
    sendStatus();
  } else if (command == "GET_MAP") {
    sendMapData();
  } else if (command == "CLEAR_WAYPOINTS") {
    clearWaypoints();
  } else if (command == "RETURN_HOME") {
    returnToStart();
  } else {
    DEBUG_PRINTLN("❓ Unknown command: " + command);
  }
}

void processWaypoint(String waypointData) {
  // Parse waypoint data: "x,y" or "index,x,y"
  int firstComma = waypointData.indexOf(',');
  int secondComma = waypointData.indexOf(',', firstComma + 1);
  
  if (secondComma > 0) {
    // Format: "index,x,y"
    int index = waypointData.substring(0, firstComma).toInt();
    float x = waypointData.substring(firstComma + 1, secondComma).toFloat();
    float y = waypointData.substring(secondComma + 1).toFloat();
    
    if (setWaypoint(index, x, y)) {
      DEBUG_PRINTLN("✅ Waypoint " + String(index) + " set to (" + String(x, 1) + ", " + String(y, 1) + ")");
    }
  } else if (firstComma > 0) {
    // Format: "x,y" - add new waypoint
    float x = waypointData.substring(0, firstComma).toFloat();
    float y = waypointData.substring(firstComma + 1).toFloat();
    
    if (addWaypoint(x, y)) {
      DEBUG_PRINTLN("✅ Waypoint added at (" + String(x, 1) + ", " + String(y, 1) + ")");
    }
  }
}

void processMissionCommand(String missionData) {
  if (missionData == "START") {
    startMission();
  } else if (missionData == "STOP") {
    stopMission();
  } else if (missionData == "PAUSE") {
    stopMotors(); // Simple pause
  } else if (missionData == "RESUME") {
    if (currentMission.active) {
      systemState = STATE_MISSION_ACTIVE;
    }
  } else if (missionData == "STATUS") {
    sendMissionStatus();
  } else {
    DEBUG_PRINTLN("❓ Unknown mission command: " + missionData);
  }
}

void processConfig(String configData) {
  // Parse configuration data
  // Format: "param=value"
  int equalIndex = configData.indexOf('=');
  if (equalIndex > 0) {
    String param = configData.substring(0, equalIndex);
    String value = configData.substring(equalIndex + 1);
    
    DEBUG_PRINTLN("⚙️ Config: " + param + " = " + value);
    // Add configuration processing here
  }
}

// ==================== MESSAGE SENDING ====================
void sendMessage(String type, String data) {
  strncpy(outgoingMsg.type, type.c_str(), sizeof(outgoingMsg.type) - 1);
  strncpy(outgoingMsg.data, data.c_str(), sizeof(outgoingMsg.data) - 1);
  outgoingMsg.checksum = calculateChecksum(&outgoingMsg);
  
  esp_err_t result = esp_now_send(stationAddress, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
  
  if (result != ESP_OK && DEBUG_MODE) {
    DEBUG_PRINTLN("❌ Send failed: " + String(result));
  }
}

void sendStatus() {
  String statusData = String(robotPos.x, 1) + "," + String(robotPos.y, 1) + "," + 
                     String(robotPos.heading, 1) + "," + String(getStateName()) + "," +
                     String(getMapCompletionPercentage(), 1);
  
  // Add sensor data
  statusData += ",";
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) statusData += ":";
    statusData += sensors.valid[i] ? String(sensors.distances[i], 1) : "0";
  }
  
  sendMessage("STATUS", statusData);
}

void sendMapData() {
  DEBUG_PRINTLN("📤 Sending map data...");
  
  // Send map data in chunks
  int chunkSize = 50; // Cells per message
  int totalCells = 0;
  
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      if (globalMap[x][y].confidence > 20) {
        String mapChunk = String(x) + "," + String(y) + "," + String(globalMap[x][y].value);
        sendMessage("MAP", mapChunk);
        
        totalCells++;
        delay(10); // Small delay to avoid overwhelming the receiver
        
        if (totalCells % chunkSize == 0) {
          delay(50); // Longer delay every chunk
        }
      }
    }
  }
  
  // Send end marker
  sendMessage("MAP", "END");
  DEBUG_PRINTLN("✅ Map data sent (" + String(totalCells) + " cells)");
}

void sendMissionStatus() {
  String missionData = String(currentMission.active ? "1" : "0") + "," +
                      String(currentMission.waypointCount) + "," +
                      String(currentMission.currentWaypoint) + "," +
                      String(getMissionProgress(), 1);
  
  sendMessage("MISSION", missionData);
}

void sendAcknowledgment(String originalType) {
  sendMessage("ACK", originalType);
}

// ==================== UTILITY FUNCTIONS ====================
String macToString(const uint8_t* mac) {
  String result = "";
  for (int i = 0; i < 6; i++) {
    if (i > 0) result += ":";
    if (mac[i] < 0x10) result += "0";
    result += String(mac[i], HEX);
  }
  return result;
}

uint8_t calculateChecksum(ESPNowMessage* msg) {
  uint8_t checksum = 0;
  uint8_t* data = (uint8_t*)msg;
  
  // Calculate checksum for all data except the checksum field itself
  for (int i = 0; i < sizeof(ESPNowMessage) - 1; i++) {
    checksum ^= data[i];
  }
  
  return checksum;
}

bool verifyChecksum(ESPNowMessage* msg) {
  uint8_t calculatedChecksum = calculateChecksum(msg);
  return calculatedChecksum == msg->checksum;
}

// ==================== HEARTBEAT AND MONITORING ====================
void sendHeartbeat() {
  static unsigned long lastHeartbeat = 0;
  unsigned long now = millis();
  
  if (now - lastHeartbeat >= 5000) { // Every 5 seconds
    lastHeartbeat = now;
    
    String heartbeatData = String(now / 1000) + "," + String(getStateName());
    sendMessage("HEARTBEAT", heartbeatData);
  }
}

void sendTelemetry() {
  static unsigned long lastTelemetry = 0;
  unsigned long now = millis();
  
  if (now - lastTelemetry >= 2000) { // Every 2 seconds
    lastTelemetry = now;
    
    // Create telemetry data
    String telemetryData = String(analogRead(A0)) + "," + // Battery level (example)
                          String(getValidSensorCount()) + "," +
                          String(moveState) + "," +
                          String(millis() / 1000); // Uptime
    
    sendMessage("TELEMETRY", telemetryData);
  }
}

// ==================== COMMUNICATION TEST ====================
void testCommunication() {
  DEBUG_PRINTLN("📡 Testing ESP-NOW communication...");
  
  sendMessage("TEST", "Robot communication test");
  
  // Wait for response
  unsigned long startTime = millis();
  bool responseReceived = false;
  
  while (millis() - startTime < 3000 && !responseReceived) {
    delay(100);
    // Check if we received an ACK
    // This would be handled in the main loop
  }
  
  DEBUG_PRINTLN("✅ Communication test complete");
=======
=======
// ====== ADVANCED ROBOT CONTROL SYSTEM - FULLY CORRECTED VERSION ======
// Fixed compilation errors, improved stability, and added comprehensive error handling

>>>>>>> Stashed changes
#include <WiFi.h>
#include <esp_now.h>
#include <BMI160Gen.h>
#include <ESP32Servo.h>
<<<<<<< Updated upstream
#include <vector>

<<<<<<< Updated upstream
// ======================== PIN DEFINITIONS ========================
=======
#include <esp_task_wdt.h>
=======
// MAC Addresses - ตรวจสอบให้ตรงกับอุปกรณ์จริง
uint8_t controllerAddress[] = {0x48, 0xe7, 0x29, 0xc9, 0x57, 0x28};
>>>>>>> Stashed changes

// ====== UTILITY FUNCTIONS ======
static inline uint8_t pct_to_pwm(int p) { 
  p = constrain(p, 0, 100); 
  return (uint8_t)(p * 255 / 100); 
}

// ====== STRUCTS AND ENUMS - MOVED TO TOP FOR PROPER DECLARATION ORDER ======
struct Position {
  float x;
  float y;
  float angle;
};

struct MapCell {
  bool occupied;
  bool visited;
  int confidence;
};

struct Waypoint {
  float x;
  float y;
  bool reached;
};

typedef struct struct_message {
  char text[50];
} struct_message;

// These enums MUST be declared before any functions that use them
enum RobotMode {
  MODE_MANUAL,
  MODE_MAPPING,
  MODE_NAVIGATION,
  MODE_EMERGENCY
};

enum NavState {
  NAV_IDLE,
  NAV_TURNING,
  NAV_MOVING,
  NAV_AVOIDING,
  NAV_REACHED
};

// This struct uses the enums above, so it must come after them
struct RobotState {
  RobotMode current_mode;
  NavState nav_state;
  bool motors_active;
  unsigned long mode_start_time;
  int error_count;
  int consecutive_obstacles;
};

// System Health Monitoring Structure
struct SystemHealth {
  unsigned long last_imu_update;
  unsigned long last_sensor_update;
  unsigned long last_communication;
  int critical_error_count;
  bool system_stable;
  unsigned long boot_time;
};

// ====== CONSTANTS ======
static const float GYRO_LSB_PER_DPS = 131.2f;
static const float KP = 1.8f;
static const float KD = 0.0f;
static const uint8_t MAX_PWM = 255;

// Pin definitions
>>>>>>> Stashed changes
#define TRIG_PIN   32
<<<<<<< Updated upstream
#define ECHO1_PIN  39
#define ECHO2_PIN  34
#define ECHO3_PIN  36
#define ECHO4_PIN  35

#define BUTTON_PIN 27
#define BUZZER_PIN 14
<<<<<<< Updated upstream

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

#define IMU_I2C_ADDR 0x69
#define MOTOR_SPEED 60
#define ANALOG_VALUE 105

// ======================== CONSTANTS ========================
=======
#define ECHO1_PIN  39  // Front
#define ECHO2_PIN  34  // Right
#define ECHO3_PIN  36  // Back
#define ECHO4_PIN  35  // Left
>>>>>>> Stashed changes
#define SOUND_SPEED 0.0343

// ======================== ESP-NOW SETUP ========================
uint8_t peerAddress[] = {0x48, 0xe7, 0x29, 0xc9, 0x57, 0x28}; // Control station MAC

typedef struct struct_message {
  char command[50];
  float param1;
  float param2;
  int mode;  // 0=manual, 1=autonomous_mapping, 2=monitoring
} struct_message;

struct_message incoming_cmd;
struct_message outgoing_status;

// ======================== ROBOT STATE VARIABLES ========================
enum RobotMode {
  MANUAL_MODE = 0,
  AUTONOMOUS_MAPPING_MODE = 1,
  PAUSED_MODE = 2
};

enum SimpleState { 
  EXPLORING, 
  RETURNING, 
  FINISHED 
};

RobotMode robot_mode = MANUAL_MODE;
SimpleState current_simple_state = EXPLORING;
int motor_speed_percent = 60;
bool mapping_active = false;
bool mapping_paused = false;

// ======================== SENSOR VARIABLES ========================
=======
#define IMU_I2C_ADDR 0x69

#define MOTOR_FRONT_A1 19
#define MOTOR_FRONT_A2 18
#define MOTOR_FRONT_B1 33
#define MOTOR_FRONT_B2 23
#define MOTOR_BACK_A1 4
#define MOTOR_BACK_A2 13
#define MOTOR_BACK_B1 16
#define MOTOR_BACK_B2 17
#define SERVO_PIN1 26
#define SERVO_PIN2 25

<<<<<<< Updated upstream
#define ECHO1_PIN  39
#define ECHO2_PIN  34
#define ECHO3_PIN  36
#define ECHO4_PIN  35

#define SOUND_SPEED 0.0343

// Memory-optimized constants
#define MAX_MAP_SIZE 30
#define GRID_SIZE 15
#define MAX_WAYPOINTS 10
#define WAYPOINT_TOLERANCE 15.0
#define OBSTACLE_THRESHOLD 25.0
#define SAFE_DISTANCE 40.0
#define EMERGENCY_DISTANCE 15.0
#define TURN_ANGLE 90.0

// Timing intervals for better performance
const int SENSOR_READ_INTERVAL = 150;
const int MAP_UPDATE_INTERVAL = 300;
const int STATUS_SEND_INTERVAL = 2000;
const int HEALTH_CHECK_INTERVAL = 5000;

// Motor speed configurations
const int MOTOR_SPEED = 70;
const int MAPPING_SPEED = 50;
const int NAVIGATION_SPEED = 60;
const int MIN_MOTOR_SPEED = 25;
const int EMERGENCY_SPEED = 20;
const float TURN_FORWARD_RATIO = 0.45;

// ====== GLOBAL VARIABLES ======
volatile float yaw_deg = 0.0f;
volatile float gyro_z_bias = 0.0f;
unsigned long last_imu_us = 0;
bool straight_active = false;
float target_yaw_deg = 0.0f;
uint8_t base_forward_pwm = 0;
float prev_err = 0.0f;

// ESP-NOW communication
uint8_t peerAddress[] = {0x7c, 0x87, 0xce, 0x2f, 0xe3, 0x20};
struct_message incoming;
struct_message outgoing;

// Sensor variables with improved management
>>>>>>> Stashed changes
volatile unsigned long echo_start[4] = {0, 0, 0, 0};
volatile unsigned long echo_end[4] = {0, 0, 0, 0};
volatile bool echo_done[4] = {false, false, false, false};
volatile bool sensor_reading_active = false;

<<<<<<< Updated upstream
float ultrasonic_distances[4] = {0, 0, 0, 0};  // front, right, back, left
unsigned long lastTriggerTime = 0;

// ======================== MAPPING VARIABLES ========================
struct SimplePosition {
  float x, y;
  float heading;
  unsigned long timestamp;
};

std::vector<SimplePosition> robot_trail;
float current_x = 0, current_y = 0, current_heading = 0;

// ======================== SERVO OBJECTS ========================
Servo servo_left;
Servo servo_right;

// ======================== INTERRUPT HANDLERS ========================
=======
// OPTIMIZED TIMING CONSTANTS
#define SENSOR_UPDATE_RATE    50    // อ่านเซ็นเซอร์ทุก 50ms
#define DATA_SEND_RATE       100    // ส่งข้อมูลทุก 100ms  
#define STATUS_PRINT_RATE   2000    // แสดงสถานะทุก 2 วินาที
#define MOTOR_TIMEOUT       1500    // หยุดมอเตอร์อัตโนมัติหลัง 1.5 วินาที

// Global variables
volatile unsigned long echo_start[4] = {0, 0, 0, 0};
volatile unsigned long echo_end[4] = {0, 0, 0, 0};
volatile bool echo_done[4] = {false, false, false, false};
float sensor_distances[4] = {999.0, 999.0, 999.0, 999.0};

// Timing variables
unsigned long lastTriggerTime = 0;
unsigned long lastDataSendTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastMotorCommand = 0;

// Robot state
float robot_x = 25.0, robot_y = 25.0, robot_heading = 0.0;  // เริ่มตรงกลาง map 50x50
bool motorActive = false;
bool esp_now_initialized = false;

// ESP-NOW message structure - COMPACT
typedef struct compact_message {
  char command[16];      // For receiving commands
  float x, y, heading;   // Position data
  float sensors[4];      // Sensor readings
} compact_message;

compact_message incoming;
compact_message outgoing;

// ================================
// MOTOR CONTROL - OPTIMIZED
// ================================

void stop_motor() {
  // Stop all motors immediately
  for (int pin = MOTOR_FRONT_A1; pin <= MOTOR_BACK_B2; pin++) {
    if (pin == MOTOR_FRONT_A1 || pin == MOTOR_FRONT_A2 || 
        pin == MOTOR_FRONT_B1 || pin == MOTOR_FRONT_B2 ||
        pin == MOTOR_BACK_A1 || pin == MOTOR_BACK_A2 ||
        pin == MOTOR_BACK_B1 || pin == MOTOR_BACK_B2) {
      analogWrite(pin, 0);
    }
  }
  motorActive = false;
  Serial.println("[MOTOR] STOP");
}

void move_forward() {
  Serial.println("[MOTOR] FORWARD");
  analogWrite(MOTOR_FRONT_A1, 180);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 180);
  analogWrite(MOTOR_BACK_A1, 180);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 180);
  motorActive = true;
  lastMotorCommand = millis();
}

void move_backward() {
  Serial.println("[MOTOR] BACKWARD");
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 180);
  analogWrite(MOTOR_FRONT_B1, 180);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 180);
  analogWrite(MOTOR_BACK_B1, 180);
  analogWrite(MOTOR_BACK_B2, 0);
  motorActive = true;
  lastMotorCommand = millis();
}

void turn_left() {
  Serial.println("[MOTOR] LEFT");
  analogWrite(MOTOR_FRONT_A1, 160);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 160);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 160);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 160);
  analogWrite(MOTOR_BACK_B2, 0);
  motorActive = true;
  lastMotorCommand = millis();
}

void turn_right() {
  Serial.println("[MOTOR] RIGHT");
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 160);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 160);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 160);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 160);
  motorActive = true;
  lastMotorCommand = millis();
}

// ================================
// ULTRASONIC INTERRUPTS - OPTIMIZED
// ================================

>>>>>>> Stashed changes
void IRAM_ATTR echo1ISR() {
  if (digitalRead(ECHO1_PIN)) {
    echo_start[0] = micros();
  } else {
    echo_end[0] = micros();
    echo_done[0] = true;
=======
bool servo_state = false;
unsigned long lastTriggerTime = 0;
unsigned long lastMotorUpdate = 0;
unsigned long lastCommandTime = 0;
unsigned long lastSensorDisplay = 0;

float sensor_distances[4] = {0, 0, 0, 0};
float prev_sensor_distances[4] = {0, 0, 0, 0}; // For filtering
bool sensor_updated[4] = {false, false, false, false};
unsigned long last_valid_reading[4] = {0, 0, 0, 0};
const char* sensor_names[4] = {"Front", "Right", "Back", "Left"};

// Hardware objects
Servo servo_left;
Servo servo_right;

// System state variables
MapCell arena_map[MAX_MAP_SIZE][MAX_MAP_SIZE];
Position robot_pos = {MAX_MAP_SIZE/2 * GRID_SIZE, MAX_MAP_SIZE/2 * GRID_SIZE, 0};
bool mapping_mode = false;
unsigned long last_map_update = 0;

Waypoint waypoints[MAX_WAYPOINTS];
int waypoint_count = 0;
int current_waypoint = 0;
bool auto_mode = false;
bool path_complete = false;

// System state and health monitoring
RobotState robot_state = {MODE_MANUAL, NAV_IDLE, false, 0, 0, 0};
SystemHealth health_monitor = {0, 0, 0, 0, true, 0};

// Performance monitoring
unsigned long loop_count = 0;
unsigned long last_performance_check = 0;
bool motors_warmed_up = false;
unsigned long motor_start_time = 0;

// PROGMEM strings to save RAM
const char PROGMEM msg_forward[] = "Moving forward";
const char PROGMEM msg_backward[] = "Moving backward";
const char PROGMEM msg_turn_left[] = "Turning left";
const char PROGMEM msg_turn_right[] = "Turning right";
const char PROGMEM msg_emergency[] = "EMERGENCY STOP ACTIVATED";

// ====== FUNCTION DECLARATIONS ======
void change_robot_mode(RobotMode new_mode);
bool is_emergency_stop_needed();
void emergency_stop();
bool is_path_clear_directional(float distance, int direction);
int find_best_direction();
void update_robot_position(int current_speed);
void autonomous_explore_optimized();
void execute_autonomous_navigation_safe();
void check_system_health();
void print_diagnostic_info();

// ====== IMPROVED INTERRUPT HANDLERS WITH DEBOUNCE PROTECTION ======
void IRAM_ATTR echo1ISR() {
  static unsigned long last_interrupt = 0;
  unsigned long now = micros();
  
  // Debounce protection
  if (now - last_interrupt < 50) return;
  last_interrupt = now;
  
  if (digitalRead(ECHO1_PIN)) {
    echo_start[0] = now;
  } else {
    if (echo_start[0] > 0 && now > echo_start[0]) { // Validity check
      echo_end[0] = now;
      echo_done[0] = true;
    }
>>>>>>> Stashed changes
  }
}

void IRAM_ATTR echo2ISR() {
<<<<<<< Updated upstream
  if (digitalRead(ECHO2_PIN)) {
    echo_start[1] = micros();
  } else {
    echo_end[1] = micros();
    echo_done[1] = true;
=======
  static unsigned long last_interrupt = 0;
  unsigned long now = micros();
  if (now - last_interrupt < 50) return;
  last_interrupt = now;
  
  if (digitalRead(ECHO2_PIN)) {
    echo_start[1] = now;
  } else {
    if (echo_start[1] > 0 && now > echo_start[1]) {
      echo_end[1] = now;
      echo_done[1] = true;
    }
>>>>>>> Stashed changes
  }
}

void IRAM_ATTR echo3ISR() {
<<<<<<< Updated upstream
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

<<<<<<< Updated upstream
// ======================== MOTOR CONTROL FUNCTIONS ========================
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

  // Initialize all motors to stop
  stop_motor();
}

void move_forward(int speed_percent){
  analogWrite(MOTOR_FRONT_A1, ANALOG_VALUE);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, ANALOG_VALUE);
  analogWrite(MOTOR_BACK_A1, ANALOG_VALUE);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, ANALOG_VALUE);
}

void move_backward(int speed_percent){
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, ANALOG_VALUE);
  analogWrite(MOTOR_FRONT_B1, ANALOG_VALUE);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, ANALOG_VALUE);
  analogWrite(MOTOR_BACK_B1, ANALOG_VALUE);
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

void turn_left(int speed_percent){
  analogWrite(MOTOR_FRONT_A1, ANALOG_VALUE);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, ANALOG_VALUE);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
}

void turn_right(int speed_percent){
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, ANALOG_VALUE);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, ANALOG_VALUE);
}

// ======================== SENSOR FUNCTIONS ========================
void ultrasonic_begin(){
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
}

void collect_sensor_data() {
  // Read ultrasonic sensors
=======
  static unsigned long last_interrupt = 0;
  unsigned long now = micros();
  if (now - last_interrupt < 50) return;
  last_interrupt = now;
  
  if (digitalRead(ECHO3_PIN)) {
    echo_start[2] = now;
  } else {
    if (echo_start[2] > 0 && now > echo_start[2]) {
      echo_end[2] = now;
      echo_done[2] = true;
    }
  }
}

void IRAM_ATTR echo4ISR() {
  static unsigned long last_interrupt = 0;
  unsigned long now = micros();
  if (now - last_interrupt < 50) return;
  last_interrupt = now;
  
  if (digitalRead(ECHO4_PIN)) {
    echo_start[3] = now;
  } else {
    if (echo_start[3] > 0 && now > echo_start[3]) {
      echo_end[3] = now;
      echo_done[3] = true;
    }
  }
}

// ====== ENHANCED SAFETY SYSTEM ======
bool is_emergency_stop_needed() {
  for (int i = 0; i < 4; i++) {
    if (sensor_updated[i] && 
        sensor_distances[i] > 2.0 && 
        sensor_distances[i] < EMERGENCY_DISTANCE) {
      Serial.print("EMERGENCY STOP: Sensor ");
      Serial.print(i + 1);
      Serial.print(" (");
      Serial.print(sensor_names[i]);
      Serial.print(") detects obstacle at ");
      Serial.print(sensor_distances[i]);
      Serial.println("cm");
      return true;
    }
  }
  return false;
}

void emergency_stop() {
  stop_motor();
  straight_stop();
  
  // Audio alert sequence
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
  
  Serial.println(F("EMERGENCY STOP ACTIVATED - All motors stopped"));
}

bool is_path_clear_directional(float distance, int direction) {
  if (direction >= 0 && direction < 4) {
    if (sensor_updated[direction] && 
        sensor_distances[direction] > 2.0 && 
        sensor_distances[direction] < 400.0) {
      return sensor_distances[direction] > distance;
    }
  }
  
  return sensor_updated[0] && sensor_distances[0] > distance;
}

int find_best_direction() {
  float max_distance = 0;
  int best_direction = -1;
  
  for (int i = 1; i < 4; i++) {
    if (sensor_updated[i] && 
        sensor_distances[i] > 2.0 && 
        sensor_distances[i] < 400.0) {
      if (sensor_distances[i] > max_distance && 
          sensor_distances[i] > SAFE_DISTANCE) {
        max_distance = sensor_distances[i];
        best_direction = i;
      }
    }
  }
  
  return best_direction;
}

// ====== SYSTEM HEALTH MONITORING ======
void check_system_health() {
  static unsigned long last_health_check = 0;
  unsigned long now = millis();
  
  if (now - last_health_check < HEALTH_CHECK_INTERVAL) return;
  last_health_check = now;
  
  bool system_ok = true;
  
  // Check IMU health
  if (now - health_monitor.last_imu_update > 3000) {
    Serial.println("WARNING: IMU not responding");
    system_ok = false;
  }
  
  // Check sensor health
  bool any_sensor_active = false;
  for (int i = 0; i < 4; i++) {
    if (sensor_updated[i]) {
      any_sensor_active = true;
      break;
    }
  }
  
  if (!any_sensor_active && now - health_monitor.last_sensor_update > 5000) {
    Serial.println("WARNING: All sensors inactive");
    system_ok = false;
  }
  
  // Check communication
  if (now - lastCommandTime > 30000) {
    Serial.println("INFO: No commands received for 30 seconds");
  }
  
  // Check memory
  if (ESP.getFreeHeap() < 10000) { // Less than 10KB free
    Serial.println("WARNING: Low memory");
    system_ok = false;
  }
  
  // Update health status
  if (!system_ok) {
    health_monitor.critical_error_count++;
    if (health_monitor.critical_error_count > 3) {
      Serial.println("CRITICAL: System unstable - Entering safe mode");
      change_robot_mode(MODE_EMERGENCY);
      health_monitor.system_stable = false;
    }
  } else {
    health_monitor.critical_error_count = max(0, health_monitor.critical_error_count - 1);
    health_monitor.system_stable = true;
  }
}

void check_wifi_stability() {
  static unsigned long last_wifi_check = 0;
  unsigned long now = millis();
  
  if (now - last_wifi_check > 10000) { // Check every 10 seconds
    last_wifi_check = now;
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected - Reinitializing");
      WiFi.mode(WIFI_STA);
      delay(100);
    }
  }
}

// ====== STATE MANAGEMENT SYSTEM ======
void change_robot_mode(RobotMode new_mode) {
  Serial.print("Mode transition: ");
  Serial.print(robot_state.current_mode);
  Serial.print(" -> ");
  Serial.println(new_mode);
  
  // Clean shutdown of current mode
  switch (robot_state.current_mode) {
    case MODE_MAPPING:
      mapping_mode = false;
      Serial.println("Mapping mode deactivated");
      break;
    case MODE_NAVIGATION:
      auto_mode = false;
      Serial.println("Navigation mode deactivated");
      break;
    case MODE_EMERGENCY:
      Serial.println("Exiting emergency mode");
      break;
  }
  
  // Safe motor shutdown
  stop_motor();
  straight_stop();
  
  // Initialize new mode
  robot_state.current_mode = new_mode;
  robot_state.mode_start_time = millis();
  robot_state.error_count = 0;
  robot_state.consecutive_obstacles = 0;
  
  switch (new_mode) {
    case MODE_MAPPING:
      mapping_mode = true;
      calibrate_gyro_bias(400);
      Serial.println("Mapping mode activated - Beginning exploration");
      break;
    case MODE_NAVIGATION:
      auto_mode = true;
      calibrate_gyro_bias(400);
      Serial.println("Navigation mode activated - Following waypoints");
      break;
    case MODE_EMERGENCY:
      emergency_stop();
      Serial.println("EMERGENCY MODE ACTIVATED");
      break;
    case MODE_MANUAL:
      Serial.println("Manual control mode activated");
      break;
  }
}

// ====== ADAPTIVE SPEED CONTROL ======
int calculate_adaptive_speed(int base_speed, float min_distance, int obstacle_count = 0) {
  if (min_distance <= 0 || min_distance > 200) {
    return base_speed;
  }
  
  int adaptive_speed;
  float obstacle_penalty = min(obstacle_count * 5, 20);
  float adjusted_base = base_speed * (1.0 - obstacle_penalty / 100.0);
  
  if (min_distance < 20) {
    adaptive_speed = MIN_MOTOR_SPEED;
  } else if (min_distance < 40) {
    float ratio = (min_distance - 20) / 20.0;
    adaptive_speed = MIN_MOTOR_SPEED + (int)((adjusted_base - MIN_MOTOR_SPEED) * ratio * 0.6);
  } else if (min_distance < 80) {
    float ratio = (min_distance - 40) / 40.0;
    adaptive_speed = (int)(adjusted_base * (0.7 + ratio * 0.3));
  } else {
    adaptive_speed = (int)adjusted_base;
  }
  
  adaptive_speed = constrain(adaptive_speed, MIN_MOTOR_SPEED, base_speed);
  return adaptive_speed;
}

float get_minimum_sensor_distance() {
  float min_dist = 999.0;
  bool has_reading = false;
  
  for (int i = 0; i < 4; i++) {
    if (sensor_updated[i] && sensor_distances[i] > 2.0 && sensor_distances[i] < 400.0) {
      has_reading = true;
      if (sensor_distances[i] < min_dist) {
        min_dist = sensor_distances[i];
      }
    }
  }
  
  return has_reading ? min_dist : -1.0;
}

// ====== IMPROVED SENSOR PROCESSING ======
void improved_process_sensor_readings() {
  unsigned long now = millis();
  
>>>>>>> Stashed changes
  for (int i = 0; i < 4; i++) {
    if (echo_done[i]) {
      noInterrupts();
      unsigned long start_time = echo_start[i];
      unsigned long end_time = echo_end[i];
      echo_done[i] = false;
      echo_start[i] = 0; // Reset start time
      interrupts();
<<<<<<< Updated upstream
      
      ultrasonic_distances[i] = (duration * SOUND_SPEED) / 2.0;
      
      // Filter out invalid readings
      if (ultrasonic_distances[i] < 2 || ultrasonic_distances[i] > 400) {
        ultrasonic_distances[i] = 400; // max range
      }
    }
  }
  
  // Update heading from gyro
  int gx, gy, gz;
  BMI160.readGyro(gx, gy, gz);
  
  static unsigned long last_gyro_time = 0;
  unsigned long now = millis();
  float dt = (now - last_gyro_time) / 1000.0;
  
  if (dt > 0 && dt < 0.1) {
    float gyro_z_dps = gz / 16.4;  // Convert to degrees/second
    current_heading += gyro_z_dps * dt;
    
    // Keep heading in 0-360 range
    if (current_heading >= 360) current_heading -= 360;
    if (current_heading < 0) current_heading += 360;
  }
  last_gyro_time = now;
}

// ======================== MAPPING FUNCTIONS ========================
void update_position_estimate(int movement_action, int duration_ms) {
  float distance_moved = (duration_ms / 1000.0) * 10.0; // 10 cm/s
  
  switch (movement_action) {
    case 1: // forward
      current_x += distance_moved * cos(current_heading * PI / 180.0);
      current_y += distance_moved * sin(current_heading * PI / 180.0);
      break;
    case 2: // backward  
      current_x -= distance_moved * cos(current_heading * PI / 180.0);
      current_y -= distance_moved * sin(current_heading * PI / 180.0);
      break;
  }
  
  // Record position
  SimplePosition pos = {current_x, current_y, current_heading, millis()};
  robot_trail.push_back(pos);
  
  // Limit trail size
  if (robot_trail.size() > 1000) {
    robot_trail.erase(robot_trail.begin(), robot_trail.begin() + 100);
  }
}

bool is_path_clear(int direction) {
  float threshold = 25.0; // 25 cm threshold
  
  switch (direction) {
    case 0: return ultrasonic_distances[0] > threshold; // front
    case 1: return ultrasonic_distances[1] > threshold; // right
    case 2: return ultrasonic_distances[2] > threshold; // back
    case 3: return ultrasonic_distances[3] > threshold; // left
    default: return false;
  }
}

int simple_exploration_decision() {
  // Simple wall-following logic
  if (is_path_clear(1)) {
    Serial.println("Decision: Turn RIGHT (wall-following)");
    return 1; // turn right
  }
  
  if (is_path_clear(0)) {
    Serial.println("Decision: Move FORWARD");
    return 0; // forward
  }
  
  if (is_path_clear(3)) {
    Serial.println("Decision: Turn LEFT");
    return 3; // turn left
  }
  
  Serial.println("Decision: TURN AROUND (dead end)");
  return 4; // turn around
}

void execute_exploration_move(int decision) {
  switch (decision) {
    case 0: // forward
      move_forward(motor_speed_percent);
      delay(800);
      stop_motor();
      update_position_estimate(1, 800);
      break;
      
    case 1: // turn right
      turn_right(motor_speed_percent);
      delay(400);
      stop_motor();
      delay(200);
      move_forward(motor_speed_percent);
      delay(600);
      stop_motor();
      update_position_estimate(1, 600);
      break;
      
    case 3: // turn left
      turn_left(motor_speed_percent);
      delay(400);
      stop_motor();
      delay(200);
      move_forward(motor_speed_percent);
      delay(600);
      stop_motor();
      update_position_estimate(1, 600);
      break;
      
    case 4: // turn around
      turn_right(motor_speed_percent);
      delay(800);
      stop_motor();
      delay(500);
      break;
  }
}

bool should_return_to_start() {
  if (robot_trail.size() < 50) return false;
  
  float distance_from_start = sqrt(current_x * current_x + current_y * current_y);
  
  if (robot_trail.size() > 100 && distance_from_start < 30.0) {
    Serial.println("Close to start position - should return!");
    return true;
  }
  
  if (robot_trail.size() > 500) {
    Serial.println("Explored enough - time to return!");
    return true;
  }
  
  return false;
}

void return_to_start_simple() {
  Serial.println("=== RETURNING TO START ===");
  
  float distance_to_start = sqrt(current_x * current_x + current_y * current_y);
  
  if (distance_to_start < 15.0) {
    Serial.println("Successfully returned to start!");
    stop_motor();
    return;
  }
  
  // Calculate direction to start
  float target_heading = atan2(-current_y, -current_x) * 180.0 / PI;
  if (target_heading < 0) target_heading += 360;
  
  float heading_error = target_heading - current_heading;
  if (heading_error > 180) heading_error -= 360;
  if (heading_error < -180) heading_error += 360;
  
  Serial.printf("Distance to start: %.2f cm, Target heading: %.1f°, Error: %.1f°\n",
                distance_to_start, target_heading, heading_error);
  
  // Adjust heading
  if (abs(heading_error) > 15) {
    if (heading_error > 0) {
      turn_left(motor_speed_percent);
      delay(200);
    } else {
      turn_right(motor_speed_percent);
      delay(200);
    }
    stop_motor();
  } else {
    // Move forward towards start
    move_forward(motor_speed_percent);
    delay(500);
    stop_motor();
    update_position_estimate(1, 500);
  }
}

void simple_autonomous_mapping() {
  collect_sensor_data();
  
  static unsigned long last_print = 0;
  if (millis() - last_print > 2000) {
    last_print = millis();
    Serial.printf("Position: X=%.2f, Y=%.2f, Heading=%.1f°, Trail=%d\n", 
                  current_x, current_y, current_heading, robot_trail.size());
  }
  
  switch (current_simple_state) {
    case EXPLORING:
      {
        int decision = simple_exploration_decision();
        execute_exploration_move(decision);
        
        if (should_return_to_start()) {
          current_simple_state = RETURNING;
          Serial.println("*** SWITCHING TO RETURN MODE ***");
        }
      }
      break;
      
    case RETURNING:
      return_to_start_simple();
      
      // Check if reached start
      if (sqrt(current_x * current_x + current_y * current_y) < 15.0) {
        current_simple_state = FINISHED;
      }
      break;
      
    case FINISHED:
      Serial.println("=== MAPPING COMPLETED ===");
      delay(5000);
      current_simple_state = EXPLORING;
      current_x = current_y = current_heading = 0;
      robot_trail.clear();
      break;
  }
}

// ======================== ESP-NOW FUNCTIONS ========================
// callback ส่งข้อมูล
void onDataSent(const wifi_tx_info_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
           recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
  Serial.print("Received data from: ");
  Serial.println(macStr);

  Serial.print("Data: ");
  for (int i = 0; i < data_len; i++) {
    Serial.print((char)data[i]);
  }
  Serial.println();
}

void esp_now_begin(){
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP32] Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP32] Failed to add peer");
    return;
  }
}

// ======================== COMMAND PROCESSING ========================
void send_status_message(const char* message, float x, float y) {
  strcpy(outgoing_status.command, message);
  outgoing_status.param1 = x;
  outgoing_status.param2 = y;
  outgoing_status.mode = robot_mode;
  
  esp_now_send(peerAddress, (uint8_t *)&outgoing_status, sizeof(outgoing_status));
}

void process_incoming_command() {
  if (strcmp(incoming_cmd.command, "forward") == 0) {
    if (robot_mode == MANUAL_MODE) {
      move_forward(motor_speed_percent);
      delay(500);
      stop_motor();
    }
  } else if (strcmp(incoming_cmd.command, "backward") == 0) {
    if (robot_mode == MANUAL_MODE) {
      move_backward(motor_speed_percent);
      delay(500);
      stop_motor();
    }
  } else if (strcmp(incoming_cmd.command, "turn_left") == 0) {
    if (robot_mode == MANUAL_MODE) {
      turn_left(motor_speed_percent);
      delay(400);
      stop_motor();
    }
  } else if (strcmp(incoming_cmd.command, "turn_right") == 0) {
    if (robot_mode == MANUAL_MODE) {
      turn_right(motor_speed_percent);
      delay(400);
      stop_motor();
    }
  } else if (strcmp(incoming_cmd.command, "stop") == 0) {
    stop_motor();
  } else if (strcmp(incoming_cmd.command, "start_mapping") == 0) {
    start_autonomous_mapping();
  } else if (strcmp(incoming_cmd.command, "return_to_start") == 0) {
    force_return_to_start();
  } else if (strcmp(incoming_cmd.command, "pause_mapping") == 0) {
    pause_mapping();
  } else if (strcmp(incoming_cmd.command, "continue_mapping") == 0) {
    continue_mapping();
  } else if (strcmp(incoming_cmd.command, "set_speed") == 0) {
    motor_speed_percent = (int)incoming_cmd.param1;
    Serial.printf("Motor speed set to %d%%\n", motor_speed_percent);
  } else if (strcmp(incoming_cmd.command, "test_sensors") == 0) {
    test_all_sensors();
  } else if (strcmp(incoming_cmd.command, "get_status") == 0) {
    send_status_update();
  }
}

void start_autonomous_mapping() {
  Serial.println("=== STARTING AUTONOMOUS MAPPING ===");
  robot_mode = AUTONOMOUS_MAPPING_MODE;
  mapping_active = true;
  mapping_paused = false;
  
  current_x = current_y = current_heading = 0;
  robot_trail.clear();
  current_simple_state = EXPLORING;
  
  send_status_message("Autonomous mapping started", current_x, current_y);
  
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
}

void pause_mapping() {
  Serial.println("=== MAPPING PAUSED ===");
  mapping_paused = true;
  robot_mode = PAUSED_MODE;
  stop_motor();
  
  send_status_message("Mapping paused", current_x, current_y);
}

void continue_mapping() {
  Serial.println("=== MAPPING RESUMED ===");
  mapping_paused = false;
  robot_mode = AUTONOMOUS_MAPPING_MODE;
  
  send_status_message("Mapping resumed", current_x, current_y);
}

void force_return_to_start() {
  Serial.println("=== FORCED RETURN TO START ===");
  current_simple_state = RETURNING;
  robot_mode = AUTONOMOUS_MAPPING_MODE;
  mapping_paused = false;
  
  send_status_message("Returning to start", current_x, current_y);
}

void test_all_sensors() {
  Serial.println("=== TESTING ALL SENSORS ===");
  
  collect_sensor_data();
  
  int gx, gy, gz, ax, ay, az;
  BMI160.readGyro(gx, gy, gz);
  BMI160.readAccelerometer(ax, ay, az);
  
  bool button_state = digitalRead(BUTTON_PIN);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
  char sensor_report[200];
  snprintf(sensor_report, sizeof(sensor_report), 
           "Sensors OK - US:%.1f/%.1f/%.1f/%.1f IMU:%d/%d/%d Btn:%d",
           ultrasonic_distances[0], ultrasonic_distances[1], 
           ultrasonic_distances[2], ultrasonic_distances[3],
           gx, gy, gz, button_state);
  
  send_status_message(sensor_report, 0, 0);
}

void send_status_update() {
  collect_sensor_data();
  
  char status_msg[100];
  snprintf(status_msg, sizeof(status_msg), 
           "Position: X=%.2f Y=%.2f H=%.1f° Trail=%d",
           current_x, current_y, current_heading, robot_trail.size());
  
  send_status_message(status_msg, current_x, current_y);
}

// ======================== MAIN SETUP AND LOOP ========================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // IMU initialization
  BMI160.begin(BMI160GenClass::I2C_MODE, IMU_I2C_ADDR);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("IMU DEVICE ID: ");
  Serial.println(dev_id, HEX);
  BMI160.setGyroRange(250);

  // Pin setup
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT);

  // Initialize components
  motor_begin();
  ultrasonic_begin();
  esp_now_begin();
  
  Serial.println("=== ROBOT READY ===");
  Serial.println("Waiting for commands from control station...");
}

void loop() {
  unsigned long now = millis();
  
  // Ultrasonic trigger
  if (now - lastTriggerTime >= 200) {
    lastTriggerTime = now;
=======

      if (end_time > start_time) { // Validity check
        unsigned long duration = end_time - start_time;
        float distance_cm = (duration * SOUND_SPEED) / 2.0;
        
        // Filter invalid readings
        if (distance_cm >= 2.0 && distance_cm <= 400.0) {
          // Apply moving average filter
          float filtered_distance = (distance_cm + prev_sensor_distances[i]) / 2.0;
          prev_sensor_distances[i] = distance_cm;
          
          sensor_distances[i] = filtered_distance;
          sensor_updated[i] = true;
          last_valid_reading[i] = now;
          health_monitor.last_sensor_update = now;
        }
      }
    }
    
    // Check for sensor timeout
    if (now - last_valid_reading[i] > 2000 && sensor_updated[i]) {
      sensor_updated[i] = false;
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" timeout - marked invalid");
    }
  }
}

// ====== CORRECTED ESP-NOW FUNCTIONS ======
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Communication error - message delivery failed");
    robot_state.error_count++;
  } else {
    health_monitor.last_communication = millis();
  }
}

void onDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.print("Received command: ");
  Serial.println(incoming.text);
  move_motor_with_command_safe(incoming.text);
  lastCommandTime = millis();
  health_monitor.last_communication = millis();
}

void esp_now_begin(){
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized successfully");
}

void send_message(const char *message) {
  snprintf(outgoing.text, sizeof(outgoing.text), "%s", message);
  esp_err_t result = esp_now_send(peerAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  if (result != ESP_OK) {
    Serial.print("Error sending message: ");
    Serial.println(result);
    robot_state.error_count++;
  }
}

// ====== HARDWARE INITIALIZATION ======
void ultrasonic_begin(){
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
  
  Serial.println("Ultrasonic sensors initialized");
}

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

  // Initialize all motors to stopped state
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
  
  motor_start_time = millis();
  Serial.println("Motors initialized");
}

// ====== IMU FUNCTIONS ======
void calibrate_gyro_bias(uint16_t samples) {
  Serial.print("Calibrating gyroscope with ");
  Serial.print(samples);
  Serial.println(" samples...");
  
  delay(500); // Allow IMU to stabilize
  long sum = 0;
  int valid_samples = 0;
  
  for (uint16_t i = 0; i < samples; i++) {
    int gx, gy, gz;
    BMI160.readGyro(gx, gy, gz);
    delay(5);
    esp_task_wdt_reset(); // Reset watchdog during calibration
  }
  
  if (valid_samples > samples/2) { // At least 50% valid readings
    gyro_z_bias = sum / (float)valid_samples;
    Serial.print("Gyro bias calculated: ");
    Serial.println(gyro_z_bias);
    health_monitor.last_imu_update = millis();
  } else {
    Serial.println("ERROR: Failed to calibrate gyro - insufficient valid readings");
    health_monitor.critical_error_count++;
  }
}

inline void imu_update_yaw() {
  int gx, gy, gz;
  BMI160.readGyro(gx, gy, gz);

  unsigned long now = micros();
  if (last_imu_us == 0) { 
    last_imu_us = now; 
    return; 
  }
  
  float dt = (now - last_imu_us) * 1e-6f;
  last_imu_us = now;

  float rate_dps = (gz - gyro_z_bias) / GYRO_LSB_PER_DPS;
  yaw_deg += rate_dps * dt;
  
  health_monitor.last_imu_update = millis();
}

void straight_start(int speed_percent) {
  base_forward_pwm = pct_to_pwm(speed_percent);
  target_yaw_deg = yaw_deg;
  prev_err = 0.0f;
  straight_active = true;
  robot_state.motors_active = true;
  
  Serial.print("Starting straight movement at ");
  Serial.print(speed_percent);
  Serial.println("% speed");
}

void straight_stop() {
  straight_active = false;
  robot_state.motors_active = false;
  stop_motor();
  Serial.println("Straight movement stopped");
}

void straight_update() {
  if (!straight_active) return;

  imu_update_yaw();

  float err = (target_yaw_deg - yaw_deg);
  float derr = (err - prev_err);
  prev_err = err;

  float corr = KP * err + KD * derr;
  float max_delta = base_forward_pwm * 0.6f;
  corr = constrain(corr, -max_delta, max_delta);

  int left_pwm = constrain((int)base_forward_pwm + (int)corr, 0, MAX_PWM);
  int right_pwm = constrain((int)base_forward_pwm - (int)corr, 0, MAX_PWM);

  safe_set_forward_side_pwm((uint8_t)left_pwm, (uint8_t)right_pwm);
}

// ====== IMPROVED MOTOR CONTROL FUNCTIONS ======
void safe_set_forward_side_pwm(uint8_t left_pwm, uint8_t right_pwm) {
  // Apply soft start for motors
  if (!motors_warmed_up) {
    unsigned long now = millis();
    if (motor_start_time == 0) motor_start_time = now;
    
    if (now - motor_start_time < 2000) { // 2 second soft start
      left_pwm = min(left_pwm, (uint8_t)100);
      right_pwm = min(right_pwm, (uint8_t)100);
    } else {
      motors_warmed_up = true;
    }
  }
  
  analogWrite(MOTOR_FRONT_A1, left_pwm);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_BACK_A1, left_pwm);
  analogWrite(MOTOR_BACK_A2, 0);

  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, right_pwm);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, right_pwm);
}

void set_forward_side_pwm(uint8_t left_pwm, uint8_t right_pwm) {
  safe_set_forward_side_pwm(left_pwm, right_pwm);
}

void move_backward(int speed_percent){
  int duty = constrain(((float)speed_percent/100)*255, 0, 255);
  
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, duty);
  analogWrite(MOTOR_FRONT_B1, duty);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, duty);
  analogWrite(MOTOR_BACK_B1, duty);
  analogWrite(MOTOR_BACK_B2, 0);
  
  robot_state.motors_active = true;
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
  
  robot_state.motors_active = false;
}

void turn_left_backward(int speed_percent){
  int duty = constrain(((float)speed_percent/100)*255, 0, 255);
  
  analogWrite(MOTOR_FRONT_A1, duty);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, duty);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, duty);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, duty);
  analogWrite(MOTOR_BACK_B2, 0);
  
  robot_state.motors_active = true;
}

void turn_right_backward(int speed_percent){
  int duty = constrain(((float)speed_percent/100)*255, 0, 255);
  
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, duty);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, duty);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, duty);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, duty);
  
  robot_state.motors_active = true;
}

// ====== MAPPING SYSTEM ======
void init_mapping() {
  Serial.println("Initializing mapping system...");
  
  for (int i = 0; i < MAX_MAP_SIZE; i++) {
    for (int j = 0; j < MAX_MAP_SIZE; j++) {
      arena_map[i][j].occupied = false;
      arena_map[i][j].visited = false;
      arena_map[i][j].confidence = 0;
    }
  }
  
  int start_x = MAX_MAP_SIZE / 2;
  int start_y = MAX_MAP_SIZE / 2;
  arena_map[start_x][start_y].visited = true;
  arena_map[start_x][start_y].confidence = 100;
  
  Serial.println("Mapping system initialized with starting position marked");
}

void start_mapping() {
  Serial.println("Initiating autonomous mapping sequence...");
  change_robot_mode(MODE_MAPPING);
}

void stop_mapping() {
  change_robot_mode(MODE_MANUAL);
  print_map();
  Serial.println("Mapping complete - returning to manual mode");
}

void update_robot_position(int current_speed) {
  static unsigned long last_pos_update = 0;
  unsigned long now = millis();
  
  if (now - last_pos_update > 200) {
    float dt = (now - last_pos_update) / 1000.0;
    float speed_factor = current_speed / 100.0;
    float movement = 15.0 * speed_factor * dt;
    
    robot_pos.x += movement * cos(robot_pos.angle * PI / 180.0);
    robot_pos.y += movement * sin(robot_pos.angle * PI / 180.0);
    last_pos_update = now;
    
    int grid_x = constrain((int)(robot_pos.x / GRID_SIZE), 0, MAX_MAP_SIZE - 1);
    int grid_y = constrain((int)(robot_pos.y / GRID_SIZE), 0, MAX_MAP_SIZE - 1);
    arena_map[grid_x][grid_y].visited = true;
    arena_map[grid_x][grid_y].confidence = min(arena_map[grid_x][grid_y].confidence + 5, 100);
  }
}

void autonomous_explore_optimized() {
  if (robot_state.current_mode != MODE_MAPPING) return;
  
  static int exploration_state = 0;
  static unsigned long state_start_time = 0;
  
  unsigned long now = millis();
  
  if (is_emergency_stop_needed()) {
    change_robot_mode(MODE_EMERGENCY);
    return;
  }
  
  if (now - last_map_update > MAP_UPDATE_INTERVAL) {
    update_map_with_sensors();
    last_map_update = now;
  }
  
  float min_distance = get_minimum_sensor_distance();
  int base_speed = MAPPING_SPEED;
  
  if (robot_state.consecutive_obstacles > 2) {
    base_speed = max(MIN_MOTOR_SPEED, base_speed - 15);
    Serial.println("Reducing speed due to obstacle-dense environment");
  }
  
  int adaptive_speed = calculate_adaptive_speed(base_speed, min_distance, robot_state.consecutive_obstacles);
  
  switch (exploration_state) {
    case 0: // Forward exploration
      if (is_path_clear_directional(SAFE_DISTANCE, 0)) {
        if (!straight_active) {
          straight_start(adaptive_speed);
          Serial.print("Exploring forward - Speed: ");
          Serial.print(adaptive_speed);
          Serial.print("%, Min distance: ");
          Serial.print(min_distance);
          Serial.println("cm");
        }
        straight_update();
        update_robot_position(adaptive_speed);
        
        robot_state.consecutive_obstacles = max(0, robot_state.consecutive_obstacles - 1);
        
      } else {
        Serial.println("Forward path blocked - Initiating intelligent turn sequence");
        straight_stop();
        exploration_state = 1;
        state_start_time = now;
        robot_state.consecutive_obstacles++;
      }
      break;
      
    case 1: // Intelligent turning
      {
        int best_direction = find_best_direction();
        
        if (best_direction != -1) {
          int turn_speed = max(MIN_MOTOR_SPEED, adaptive_speed / 2);
          
          Serial.print("Turning towards direction ");
          Serial.print(best_direction);
          Serial.print(" (");
          Serial.print(sensor_names[best_direction]);
          Serial.print(") with ");
          Serial.print(sensor_distances[best_direction]);
          Serial.println("cm clearance");
          
          if (best_direction == 1) {
            turn_right_backward(turn_speed);
            robot_pos.angle += 2.0;
          } else {
            turn_left_backward(turn_speed);
            robot_pos.angle -= 2.0;
          }
          
          if (robot_pos.angle >= 360) robot_pos.angle -= 360;
          if (robot_pos.angle < 0) robot_pos.angle += 360;
          
          if (now - state_start_time > 1000) {
            if (is_path_clear_directional(SAFE_DISTANCE, 0)) {
              stop_motor();
              exploration_state = 0;
              Serial.println("Turn completed - Path clear, resuming forward exploration");
            }
          }
        } else {
          Serial.println("All directions blocked - Emergency reverse required");
          exploration_state = 2;
          state_start_time = now;
        }
      }
      break;
      
    case 2: // Emergency reverse
      if (now - state_start_time < 1500) {
        move_backward(EMERGENCY_SPEED);
        Serial.println("Emergency reverse maneuver in progress");
        
        float reverse_movement = -0.5 * (EMERGENCY_SPEED / 50.0);
        robot_pos.x += reverse_movement * cos(robot_pos.angle * PI / 180.0);
        robot_pos.y += reverse_movement * sin(robot_pos.angle * PI / 180.0);
        
      } else {
        stop_motor();
        exploration_state = 1;
        state_start_time = now;
        robot_state.consecutive_obstacles = max(0, robot_state.consecutive_obstacles - 2);
        Serial.println("Emergency reverse complete - Attempting new direction");
      }
      break;
  }
  
  static unsigned long last_map_send = 0;
  if (now - last_map_send > STATUS_SEND_INTERVAL) {
    send_map_data();
    last_map_send = now;
  }
}

void update_map_with_sensors() {
  int grid_x = constrain((int)(robot_pos.x / GRID_SIZE), 0, MAX_MAP_SIZE - 1);
  int grid_y = constrain((int)(robot_pos.y / GRID_SIZE), 0, MAX_MAP_SIZE - 1);
  
  arena_map[grid_x][grid_y].visited = true;
  arena_map[grid_x][grid_y].confidence = min(arena_map[grid_x][grid_y].confidence + 10, 100);
}

void send_map_data() {
  float grid_x = robot_pos.x / GRID_SIZE;
  float grid_y = robot_pos.y / GRID_SIZE;
  
  char map_msg[50];
  snprintf(map_msg, sizeof(map_msg), "MAP_POS:%.1f,%.1f,%.1f", 
           grid_x, grid_y, robot_pos.angle);
  send_message(map_msg);
}

void print_map() {
  Serial.println("\n=== EXPLORATION MAP ===");
  float grid_x = robot_pos.x / GRID_SIZE;
  float grid_y = robot_pos.y / GRID_SIZE;
  int robot_grid_x = constrain((int)grid_x, 0, MAX_MAP_SIZE - 1);
  int robot_grid_y = constrain((int)grid_y, 0, MAX_MAP_SIZE - 1);
  
  Serial.print("Robot position: Grid(");
  Serial.print(robot_grid_x);
  Serial.print(",");
  Serial.print(robot_grid_y);
  Serial.print(") World(");
  Serial.print(robot_pos.x);
  Serial.print(",");
  Serial.print(robot_pos.y);
  Serial.print(") Angle:");
  Serial.println(robot_pos.angle);
  
  for (int j = MAX_MAP_SIZE - 1; j >= 0; j--) {
    for (int i = 0; i < MAX_MAP_SIZE; i++) {
      if (i == robot_grid_x && j == robot_grid_y) {
        Serial.print("R");
      } else if (arena_map[i][j].occupied && arena_map[i][j].confidence > 70) {
        Serial.print("#");
      } else if (arena_map[i][j].visited) {
        Serial.print(".");
      } else {
        Serial.print(" ");
      }
    }
    Serial.println();
  }
  Serial.println("========================\n");
}

// ====== PATH PLANNING ======
void init_path_planning() {
  waypoint_count = 0;
  current_waypoint = 0;
  auto_mode = false;
  path_complete = false;
  Serial.println("Path planning system initialized");
}

void add_waypoint(float x, float y) {
  if (waypoint_count < MAX_WAYPOINTS) {
    waypoints[waypoint_count].x = x;
    waypoints[waypoint_count].y = y;
    waypoints[waypoint_count].reached = false;
    waypoint_count++;
    
    Serial.print("Waypoint ");
    Serial.print(waypoint_count);
    Serial.print(" added: (");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.println(")");
  } else {
    Serial.println("Maximum waypoints reached!");
  }
}

void clear_waypoints() {
  waypoint_count = 0;
  current_waypoint = 0;
  path_complete = false;
  Serial.println("All waypoints cleared");
}

void start_autonomous_navigation() {
  if (waypoint_count == 0) {
    Serial.println("Cannot start navigation - no waypoints defined!");
    return;
  }
  
  Serial.println("Starting autonomous navigation sequence...");
  change_robot_mode(MODE_NAVIGATION);
  print_waypoints();
}

void stop_autonomous_navigation() {
  change_robot_mode(MODE_MANUAL);
  Serial.println("Navigation stopped - returning to manual mode");
}

float calculate_distance(float x1, float y1, float x2, float y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

float calculate_angle_to_target(float target_x, float target_y) {
  float dx = target_x - robot_pos.x;
  float dy = target_y - robot_pos.y;
  float target_angle = atan2(dy, dx) * 180.0 / PI;
  
  if (target_angle < 0) target_angle += 360;
  return target_angle;
}

float normalize_angle_difference(float angle_diff) {
  while (angle_diff > 180) angle_diff -= 360;
  while (angle_diff <= -180) angle_diff += 360;
  return angle_diff;
}

void execute_autonomous_navigation_safe() {
  if (robot_state.current_mode != MODE_NAVIGATION || path_complete) return;
  
  if (current_waypoint >= waypoint_count) {
    path_complete = true;
    change_robot_mode(MODE_MANUAL);
    Serial.println("Navigation complete - All waypoints reached!");
    return;
  }
  
  unsigned long now = millis();
  Waypoint& target = waypoints[current_waypoint];
  
  float distance_to_target = calculate_distance(robot_pos.x, robot_pos.y, target.x, target.y);
  
  if (distance_to_target < WAYPOINT_TOLERANCE) {
    target.reached = true;
    current_waypoint++;
    robot_state.nav_state = NAV_REACHED;
    
    Serial.print("Waypoint ");
    Serial.print(current_waypoint);
    Serial.print(" reached! Distance was: ");
    Serial.print(distance_to_target);
    Serial.println("cm");
    
    if (current_waypoint >= waypoint_count) {
      path_complete = true;
      stop_autonomous_navigation();
      return;
    }
    return;
  }
  
  if (is_emergency_stop_needed()) {
    change_robot_mode(MODE_EMERGENCY);
    return;
  }
  
  float min_distance = get_minimum_sensor_distance();
  int adaptive_speed = calculate_adaptive_speed(NAVIGATION_SPEED, min_distance, robot_state.consecutive_obstacles);
  
  switch (robot_state.nav_state) {
    case NAV_IDLE:
    case NAV_REACHED:
      robot_state.nav_state = NAV_TURNING;
      Serial.println("Beginning turn towards next waypoint");
      break;
      
    case NAV_TURNING:
      {
        float target_angle = calculate_angle_to_target(target.x, target.y);
        float angle_diff = normalize_angle_difference(target_angle - robot_pos.angle);
        
        if (abs(angle_diff) > 10.0) {
          int turn_speed = max(MIN_MOTOR_SPEED, adaptive_speed / 2);
          
          if (angle_diff > 0) {
            turn_right_backward(turn_speed);
            robot_pos.angle += 1.0;
          } else {
            turn_left_backward(turn_speed);
            robot_pos.angle -= 1.0;
          }
          
          if (robot_pos.angle >= 360) robot_pos.angle -= 360;
          if (robot_pos.angle < 0) robot_pos.angle += 360;
          
        } else {
          stop_motor();
          robot_state.nav_state = NAV_MOVING;
          Serial.print("Turn complete - Moving to waypoint (");
          Serial.print(target.x);
          Serial.print(", ");
          Serial.print(target.y);
          Serial.print(") Distance: ");
          Serial.print(distance_to_target);
          Serial.println("cm");
        }
      }
      break;
      
    case NAV_MOVING:
      if (is_path_clear_directional(SAFE_DISTANCE, 0)) {
        if (!straight_active) {
          straight_start(adaptive_speed);
        }
        straight_update();
        update_robot_position(adaptive_speed);
        
      } else {
        Serial.println("Obstacle detected during navigation - Initiating avoidance");
        straight_stop();
        robot_state.nav_state = NAV_AVOIDING;
        robot_state.consecutive_obstacles++;
      }
      break;
      
    case NAV_AVOIDING:
      {
        int best_direction = find_best_direction();
        
        if (best_direction != -1) {
          int avoid_speed = max(MIN_MOTOR_SPEED, 30);
          
          if (best_direction == 1) {
            turn_right_backward(avoid_speed);
            robot_pos.angle += 2.0;
          } else {
            turn_left_backward(avoid_speed);
            robot_pos.angle -= 2.0;
          }
          
          if (robot_pos.angle >= 360) robot_pos.angle -= 360;
          if (robot_pos.angle < 0) robot_pos.angle += 360;
          
          static unsigned long avoid_start = 0;
          if (avoid_start == 0) avoid_start = now;
          
          if (now - avoid_start > 2000) {
            robot_state.nav_state = NAV_TURNING;
            avoid_start = 0;
            Serial.println("Obstacle avoidance complete - Re-targeting waypoint");
          }
        } else {
          move_backward(EMERGENCY_SPEED);
          Serial.println("Emergency reverse during navigation");
        }
      }
      break;
  }
  
  static unsigned long last_nav_status = 0;
  if (now - last_nav_status > STATUS_SEND_INTERVAL) {
    send_navigation_status();
    last_nav_status = now;
  }
}

void send_navigation_status() {
  char status_msg[50];
  
  snprintf(status_msg, sizeof(status_msg), "NAV_POS:%.1f,%.1f,%.1f", 
           robot_pos.x, robot_pos.y, robot_pos.angle);
  send_message(status_msg);
  
  if (current_waypoint < waypoint_count) {
    Waypoint& target = waypoints[current_waypoint];
    float distance = calculate_distance(robot_pos.x, robot_pos.y, target.x, target.y);
    
    snprintf(status_msg, sizeof(status_msg), "NAV_WP:%d,%.1f,%.1f,%.1f", 
             current_waypoint, target.x, target.y, distance);
    send_message(status_msg);
  }
}

void print_waypoints() {
  Serial.println("\n=== NAVIGATION WAYPOINTS ===");
  for (int i = 0; i < waypoint_count; i++) {
    Serial.print("WP");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(waypoints[i].x);
    Serial.print(", ");
    Serial.print(waypoints[i].y);
    Serial.print(") ");
    Serial.println(waypoints[i].reached ? "[REACHED]" : "[PENDING]");
  }
  Serial.print("Total waypoints: ");
  Serial.println(waypoint_count);
  Serial.println("=============================\n");
}

// ====== COMMAND PROCESSING ======
void process_waypoint_command(const char* command) {
  if (strncmp(command, "wp_add:", 7) == 0) {
    float x, y;
    if (sscanf(command + 7, "%f,%f", &x, &y) == 2) {
      add_waypoint(x, y);
    } else {
      Serial.println("Invalid waypoint format - use: wp_add:x,y");
    }
  } else if (strcmp(command, "wp_clear") == 0) {
    clear_waypoints();
  } else if (strcmp(command, "wp_start") == 0) {
    start_autonomous_navigation();
  } else if (strcmp(command, "wp_stop") == 0) {
    stop_autonomous_navigation();
  } else if (strcmp(command, "wp_list") == 0) {
    print_waypoints();
  }
}

void load_competition_path_1() {
  clear_waypoints();
  add_waypoint(100, 0);
  add_waypoint(100, 100);
  add_waypoint(0, 100);
  add_waypoint(0, 0);
  Serial.println("Competition Path 1 loaded - Square pattern");
}

void load_competition_path_2() {
  clear_waypoints();
  add_waypoint(50, 0);
  add_waypoint(50, 50);
  add_waypoint(100, 50);
  add_waypoint(100, 100);
  add_waypoint(0, 100);
  add_waypoint(0, 0);
  Serial.println("Competition Path 2 loaded - Complex navigation pattern");
}

void move_motor_with_command_safe(const char* command){
  Serial.print("Processing command: ");
  Serial.println(command);
  
  if (strcmp(command, "forward") == 0) {
    change_robot_mode(MODE_MANUAL);
    calibrate_gyro_bias(400);
    last_imu_us = micros();
    straight_start(MOTOR_SPEED);
  } else if (strcmp(command, "backward") == 0) {
    change_robot_mode(MODE_MANUAL);
    move_backward(MOTOR_SPEED);
  } else if (strcmp(command, "turn_left") == 0) {
    change_robot_mode(MODE_MANUAL);
    turn_left_backward(MOTOR_SPEED);
  } else if (strcmp(command, "turn_right") == 0) {
    change_robot_mode(MODE_MANUAL);
    turn_right_backward(MOTOR_SPEED);
  } else if (strcmp(command, "start_mapping") == 0) {
    start_mapping();
  } else if (strcmp(command, "stop_mapping") == 0) {
    stop_mapping();
  } else if (strncmp(command, "wp_", 3) == 0) {
    process_waypoint_command(command);
  } else if (strcmp(command, "load_path_1") == 0) {
    load_competition_path_1();
  } else if (strcmp(command, "load_path_2") == 0) {
    load_competition_path_2();
  } else if (strcmp(command, "status") == 0) {
    print_system_status();
  } else if (strcmp(command, "map") == 0) {
    print_map();
  } else if (strcmp(command, "diagnostic") == 0) {
    print_diagnostic_info();
  } else if (strcmp(command, "") == 0 || strcmp(command, "stop") == 0) {
    change_robot_mode(MODE_MANUAL);
    stop_motor();
    straight_stop();
    Serial.println("All systems stopped - Manual mode active");
  } else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
  
  lastCommandTime = millis();
}

// ====== SYSTEM STATUS ======
void print_system_status() {
  Serial.println("\n=== ROBOT SYSTEM STATUS ===");
  
  Serial.print("Current Mode: ");
  switch (robot_state.current_mode) {
    case MODE_MANUAL: Serial.println("Manual Control"); break;
    case MODE_MAPPING: Serial.println("Autonomous Mapping"); break;
    case MODE_NAVIGATION: Serial.println("Waypoint Navigation"); break;
    case MODE_EMERGENCY: Serial.println("EMERGENCY STOP"); break;
  }
  
  Serial.print("Motors Active: ");
  Serial.println(robot_state.motors_active ? "YES" : "NO");
  
  Serial.print("System Health: ");
  Serial.println(health_monitor.system_stable ? "STABLE" : "UNSTABLE");
  
  Serial.print("Position: (");
  Serial.print(robot_pos.x);
  Serial.print(", ");
  Serial.print(robot_pos.y);
  Serial.print(") Angle: ");
  Serial.println(robot_pos.angle);
  
  Serial.print("Consecutive Obstacles: ");
  Serial.println(robot_state.consecutive_obstacles);
  
  float min_dist = get_minimum_sensor_distance();
  Serial.print("Minimum Sensor Distance: ");
  if (min_dist > 0) {
    Serial.print(min_dist);
    Serial.println("cm");
  } else {
    Serial.println("No valid readings");
  }
  
  if (auto_mode) {
    Serial.print("Navigation: Waypoint ");
    Serial.print(current_waypoint);
    Serial.print("/");
    Serial.println(waypoint_count);
  }
  
  Serial.print("Uptime: ");
  Serial.print((millis() - health_monitor.boot_time) / 1000);
  Serial.println(" seconds");
  
  Serial.println("===========================\n");
}

void print_diagnostic_info() {
  Serial.println("\n=== DIAGNOSTIC INFORMATION ===");
  
  // Memory usage
  Serial.print("Free heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  
  Serial.print("Min free heap: ");
  Serial.print(ESP.getMinFreeHeap());
  Serial.println(" bytes");
  
  Serial.print("Heap size: ");
  Serial.print(ESP.getHeapSize());
  Serial.println(" bytes");
  
  // Task stack
  Serial.print("Free stack: ");
  Serial.print(uxTaskGetStackHighWaterMark(NULL));
  Serial.println(" words");
  
  // System uptime
  Serial.print("Uptime: ");
  Serial.print((millis() - health_monitor.boot_time) / 1000);
  Serial.println(" seconds");
  
  // Error counts
  Serial.print("Critical errors: ");
  Serial.println(health_monitor.critical_error_count);
  
  Serial.print("Robot errors: ");
  Serial.println(robot_state.error_count);
  
  // Sensor status
  Serial.println("Sensor Status:");
  for (int i = 0; i < 4; i++) {
    Serial.print("  Sensor ");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(sensor_names[i]);
    Serial.print("): ");
    if (sensor_updated[i]) {
      Serial.print("OK - ");
      Serial.print(sensor_distances[i], 1);
      Serial.println("cm");
    } else {
      Serial.println("FAIL");
    }
  }
  
  // IMU status
  Serial.print("IMU Last Update: ");
  Serial.print((millis() - health_monitor.last_imu_update) / 1000);
  Serial.println(" seconds ago");
  
  // Communication status
  Serial.print("Last Command: ");
  Serial.print((millis() - lastCommandTime) / 1000);
  Serial.println(" seconds ago");
  
  // WiFi status
  Serial.print("WiFi Status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  
  Serial.println("===============================\n");
}

void display_sensors() {
  Serial.print("\rSensors: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(sensor_names[i]);
    Serial.print(":");
    if (sensor_updated[i]) {
      Serial.print(sensor_distances[i], 1);
    } else {
      Serial.print("---");
    }
    Serial.print("cm");
    if (i < 3) Serial.print(" | ");
  }
  Serial.print("          ");
}

// ====== HELPER FUNCTIONS ======
void handle_manual_mode_timeout() {
  unsigned long now = millis();
  if (robot_state.current_mode == MODE_MANUAL && 
      now - lastCommandTime > 5000 && 
      straight_active) {
    straight_stop();
    Serial.println("Manual mode timeout - Motors stopped for safety");
  }
}

void handle_button_and_servo(unsigned long now) {
  int button_status = digitalRead(BUTTON_PIN);
  
  if (!button_status && now - lastMotorUpdate >= 500) {
    lastMotorUpdate = now;
    if (servo_state) {
      servo_left.write(180);
      servo_right.write(180);
    } else {
      servo_left.write(0);
      servo_right.write(0);
    }
    servo_state = !servo_state;
  }
  
  digitalWrite(BUZZER_PIN, !button_status);
}

// ====== SETUP FUNCTION ======
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait up to 3 seconds for Serial
  
  Serial.println("=== ADVANCED ROBOT SYSTEM INITIALIZATION ===");
  health_monitor.boot_time = millis();
  
  // Initialize Watchdog Timer
 esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,  // 30 seconds in milliseconds
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
};
esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
  Serial.println("Watchdog timer initialized");
  
  // Initialize IMU
  Serial.println("Initializing IMU...");
  if (BMI160.begin(BMI160GenClass::I2C_MODE, IMU_I2C_ADDR) != 0) {
    Serial.println("ERROR: Failed to initialize IMU");
    health_monitor.critical_error_count++;
  } else {
    uint8_t dev_id = BMI160.getDeviceID();
    Serial.print("IMU Device ID: 0x");
    Serial.println(dev_id, HEX);
    BMI160.setGyroRange(250);
    Serial.println("IMU initialized successfully");
  }

  // Initialize GPIO pins
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT);
  Serial.println("GPIO pins initialized");

  // Initialize hardware subsystems
  motor_begin();
  ultrasonic_begin();
  esp_now_begin();

  // Initialize software subsystems
  init_mapping();
  init_path_planning();

  // Calibrate sensors
  Serial.println("Starting sensor calibration...");
  calibrate_gyro_bias(800);
  last_imu_us = micros();
  
  // Final system check
  if (health_monitor.critical_error_count > 0) {
    Serial.print("WARNING: System started with ");
    Serial.print(health_monitor.critical_error_count);
    Serial.println(" critical errors");
  }
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("Available Commands:");
  Serial.println("Manual: forward, backward, turn_left, turn_right, stop");
  Serial.println("Mapping: start_mapping, stop_mapping, map");
  Serial.println("Navigation: wp_start, wp_stop, wp_list, wp_clear");
  Serial.println("Presets: load_path_1, load_path_2");
  Serial.println("System: status, diagnostic");
  Serial.println("Robot ready for commands...\n");
}

// ====== IMPROVED MAIN LOOP ======
void loop() {
  unsigned long loop_start = millis();
  loop_count++;
  
  // Reset Watchdog Timer
  esp_task_wdt_reset();
  
  // Performance monitoring
  if (loop_start - last_performance_check > 10000) { // Every 10 seconds
    Serial.print("Loop performance: ");
    Serial.print(loop_count / 10);
    Serial.println(" loops/sec");
    
    // Check for performance issues
    if (loop_count < 50) { // Less than 5 loops per second
      Serial.println("WARNING: Low loop performance detected");
      health_monitor.critical_error_count++;
    }
    
    loop_count = 0;
    last_performance_check = loop_start;
  }
  
  // Critical safety check - highest priority
  if (is_emergency_stop_needed() && robot_state.current_mode != MODE_EMERGENCY) {
    Serial.println("CRITICAL: Emergency stop triggered!");
    change_robot_mode(MODE_EMERGENCY);
    return;
  }
  
  // Emergency mode recovery handling
  if (robot_state.current_mode == MODE_EMERGENCY) {
    static unsigned long emergency_start = 0;
    if (emergency_start == 0) emergency_start = loop_start;
    
    // Auto-recovery after 5 seconds if path is clear
    if (loop_start - emergency_start > 5000) {
      if (!is_emergency_stop_needed()) {
        Serial.println("Emergency cleared - Auto-recovery to manual mode");
        change_robot_mode(MODE_MANUAL);
        emergency_start = 0;
      } else {
        // Reset timer if still in emergency
        emergency_start = loop_start;
      }
    }
    return; // Don't execute other functions in emergency mode
  }
  
  // System health monitoring
  check_system_health();
  if (!health_monitor.system_stable) {
    Serial.println("System unstable - Limited functionality");
    return; // Skip normal operations if system is unstable
  }
  
  // WiFi and communication stability check
  check_wifi_stability();
  
  // Sensor reading and processing
  static unsigned long last_sensor_trigger = 0;
  if (loop_start - last_sensor_trigger >= SENSOR_READ_INTERVAL) {
    last_sensor_trigger = loop_start;
    
    // Trigger ultrasonic sensors
>>>>>>> Stashed changes
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
  
<<<<<<< Updated upstream
  // Main robot behavior
  switch (robot_mode) {
    case MANUAL_MODE:
      // Wait for commands from control station
      break;
      
    case AUTONOMOUS_MAPPING_MODE:
      if (!mapping_paused) {
        simple_autonomous_mapping();
        
        static unsigned long last_auto_status = 0;
        if (millis() - last_auto_status > 10000) {
          last_auto_status = millis();
          send_status_update();
        }
      }
      break;
      
    case PAUSED_MODE:
      stop_motor();
      break;
  }
  
  // Emergency stop
  if (!digitalRead(BUTTON_PIN)) {
    Serial.println("EMERGENCY STOP!");
    stop_motor();
    robot_mode = MANUAL_MODE;
    mapping_paused = true;
    
    send_status_message("EMERGENCY STOP - Button pressed", current_x, current_y);
    delay(1000);
  }
>>>>>>> Stashed changes
}
=======
  // Process sensor readings with improved filtering
  improved_process_sensor_readings();
  
  // Display sensor updates periodically
  static unsigned long last_display_update = 0;
  if (loop_start - last_display_update > 1000) { // Every second
    display_sensors();
    last_display_update = loop_start;
  }
  
  // Core robot control updates
  straight_update(); // Always update straight-line control
  
  // Mode-specific operations
  switch (robot_state.current_mode) {
    case MODE_MAPPING:
      autonomous_explore_optimized();
      break;
      
    case MODE_NAVIGATION:
      execute_autonomous_navigation_safe();
      break;
      
    case MODE_MANUAL:
      handle_manual_mode_timeout();
      break;
      
    case MODE_EMERGENCY:
      // Already handled above
      break;
  }
  
  // Handle button and servo operations
  handle_button_and_servo(loop_start);
  
  // Periodic status reporting
  static unsigned long last_status_report = 0;
  if (loop_start - last_status_report > 30000) { // Every 30 seconds
    Serial.println("\n--- Periodic Status Report ---");
    print_system_status();
    last_status_report = loop_start;
  }
  
  // Loop performance check
  unsigned long loop_duration = millis() - loop_start;
  if (loop_duration > 100) { // If loop takes more than 100ms
    Serial.print("WARNING: Long loop duration: ");
    Serial.print(loop_duration);
    Serial.println("ms - Performance degraded");
    health_monitor.critical_error_count++;
  }
  
  // Ensure minimum loop delay for stability
  if (loop_duration < 20) { // If loop is too fast
    delay(20 - loop_duration);
  }
}

// ====== ADDITIONAL UTILITY FUNCTIONS ======

// Function to reset system to safe state
void reset_to_safe_state() {
  Serial.println("Resetting system to safe state...");
  
  // Stop all motors
  stop_motor();
  straight_stop();
  
  // Reset state variables
  robot_state.current_mode = MODE_MANUAL;
  robot_state.motors_active = false;
  robot_state.error_count = 0;
  robot_state.consecutive_obstacles = 0;
  
  // Reset modes
  mapping_mode = false;
  auto_mode = false;
  path_complete = false;
  
  // Reset health monitor
  health_monitor.critical_error_count = 0;
  health_monitor.system_stable = true;
  
  Serial.println("System reset complete - Ready for commands");
}

// Function for emergency shutdown
void emergency_shutdown() {
  Serial.println("EMERGENCY SHUTDOWN INITIATED");
  
  // Disable all interrupts temporarily
  noInterrupts();
  
  // Stop all motors immediately
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
  
  // Re-enable interrupts
  interrupts();
  
  // Sound alarm
  for (int i = 0; i < 5; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
  
  // Set emergency state
  robot_state.current_mode = MODE_EMERGENCY;
  robot_state.motors_active = false;
  
  Serial.println("EMERGENCY SHUTDOWN COMPLETE");
}

// Memory management function
void manage_memory() {
  static unsigned long last_memory_check = 0;
  unsigned long now = millis();
  
  if (now - last_memory_check > 60000) { // Check every minute
    last_memory_check = now;
    
    size_t free_heap = ESP.getFreeHeap();
    size_t min_free_heap = ESP.getMinFreeHeap();
    
    Serial.print("Memory status - Free: ");
    Serial.print(free_heap);
    Serial.print(" bytes, Min free: ");
    Serial.print(min_free_heap);
    Serial.println(" bytes");
    
    // Warning if memory is getting low
    if (free_heap < 15000) {
      Serial.println("WARNING: Low memory condition");
      health_monitor.critical_error_count++;
      
      // Force garbage collection if available
      if (free_heap < 10000) {
        Serial.println("CRITICAL: Very low memory - Consider system reset");
      }
=======
// ================================
// ESP-NOW CALLBACKS - OPTIMIZED
// ================================

void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  // Minimal logging for performance
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Send failed");
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  String command = String(incoming.command);
  
  Serial.print("[CMD] ");
  Serial.println(command);
  
  // Process commands immediately
  if (command == "forward" || command == "w") {
    move_forward();
  } else if (command == "backward" || command == "s") {
    move_backward();
  } else if (command == "turn_left" || command == "a") {
    turn_left();
  } else if (command == "turn_right" || command == "d") {
    turn_right();
  } else if (command == "stop" || command == "x") {
    stop_motor();
  }
}

// ================================
// DATA TRANSMISSION - FAST
// ================================

void sendRobotData() {
  if (!esp_now_initialized) return;
  
  // Prepare outgoing message
  outgoing.x = robot_x;
  outgoing.y = robot_y;  
  outgoing.heading = robot_heading;
  
  for (int i = 0; i < 4; i++) {
    outgoing.sensors[i] = sensor_distances[i];
  }
  
  // Send data
  esp_err_t result = esp_now_send(controllerAddress, (uint8_t*)&outgoing, sizeof(outgoing));
  
  // Also send in text format for Python compatibility
  static char textBuffer[100];
  snprintf(textBuffer, sizeof(textBuffer), 
           "[ROBOT] POS:%.1f,%.1f,%.1f|S:%.1f,%.1f,%.1f,%.1f",
           robot_x, robot_y, robot_heading,
           sensor_distances[0], sensor_distances[1], 
           sensor_distances[2], sensor_distances[3]);
  Serial.println(textBuffer);
}

// ================================
// SENSOR PROCESSING - FAST
// ================================

void processSensorReadings() {
  for (int i = 0; i < 4; i++) {
    if (echo_done[i]) {
      noInterrupts();
      unsigned long duration = echo_end[i] - echo_start[i];
      echo_done[i] = false;
      interrupts();
      
      float distance = (duration * SOUND_SPEED) / 2.0;
      
      // Validate and filter readings
      if (distance >= 2 && distance <= 200) {
        // Simple moving average filter
        sensor_distances[i] = (sensor_distances[i] * 0.7) + (distance * 0.3);
      } else {
        sensor_distances[i] = 999.0; // Invalid reading
      }
    }
  }
}

// ================================
// SETUP - OPTIMIZED
// ================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("============================");
  Serial.println("   OPTIMIZED ROBOT v2.0");
  Serial.println("   Fast Sensor Updates");
  Serial.println("============================");
  
  // Initialize motor pins
  const int motorPins[] = {MOTOR_FRONT_A1, MOTOR_FRONT_A2, MOTOR_FRONT_B1, MOTOR_FRONT_B2,
                          MOTOR_BACK_A1, MOTOR_BACK_A2, MOTOR_BACK_B1, MOTOR_BACK_B2};
  
  for (int i = 0; i < 8; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWrite(motorPins[i], 0);
  }
  Serial.println("[MOTOR] Initialized");
  
  // Initialize ultrasonic sensors
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  const int echoPins[] = {ECHO1_PIN, ECHO2_PIN, ECHO3_PIN, ECHO4_PIN};
  void (*handlers[])() = {echo1ISR, echo2ISR, echo3ISR, echo4ISR};
  
  for (int i = 0; i < 4; i++) {
    pinMode(echoPins[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(echoPins[i]), handlers[i], CHANGE);
  }
  Serial.println("[SENSOR] Initialized with fast interrupts");
  
  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.printf("[WIFI] Robot MAC: %s\n", WiFi.macAddress().c_str());
  
  if (esp_now_init() == ESP_OK) {
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controllerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
      esp_now_initialized = true;
      Serial.println("[ESP-NOW] Initialized successfully");
>>>>>>> Stashed changes
    }
  }
}

// Comprehensive system test function
void run_system_test() {
  Serial.println("\n=== RUNNING SYSTEM TEST ===");
  
<<<<<<< Updated upstream
  // Test motors
  Serial.println("Testing motors...");
  for (int i = 25; i <= 100; i += 25) {
    Serial.print("Motor test at ");
    Serial.print(i);
    Serial.println("% power");
    
    move_backward(i);
    delay(500);
    stop_motor();
    delay(200);
    
    turn_left_backward(i);
    delay(300);
    stop_motor();
    delay(200);
    
    turn_right_backward(i);
    delay(300);
    stop_motor();
    delay(200);
  }
  
  // Test sensors
  Serial.println("Testing sensors...");
  for (int i = 0; i < 10; i++) {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    delay(100);
    improved_process_sensor_readings();
  }
  
  // Test servos
  Serial.println("Testing servos...");
  servo_left.write(0);
  servo_right.write(0);
  delay(500);
  servo_left.write(90);
  servo_right.write(90);
  delay(500);
  servo_left.write(180);
  servo_right.write(180);
  delay(500);
  servo_left.write(90);
  servo_right.write(90);
  
  // Test IMU
  Serial.println("Testing IMU...");
  for (int i = 0; i < 10; i++) {
    imu_update_yaw();
    delay(100);
  }
  
  // Test communication
  Serial.println("Testing communication...");
  send_message("TEST_MESSAGE");
  
  Serial.println("=== SYSTEM TEST COMPLETE ===\n");
  print_diagnostic_info();
}

/*
=== SUMMARY OF IMPROVEMENTS ===

1. **Watchdog Timer**: Prevents system freezes
2. **Enhanced Error Handling**: Comprehensive error detection and recovery
3. **Improved Sensor Processing**: Filtering and validation of sensor data
4. **Memory Management**: Monitoring and warnings for memory issues
5. **Performance Monitoring**: Loop timing and performance metrics
6. **System Health Monitoring**: Continuous health checks
7. **Enhanced Safety**: Multiple layers of safety checks
8. **Robust Communication**: Better ESP-NOW error handling
9. **Improved Motor Control**: Soft start and safety features
10. **Comprehensive Diagnostics**: Detailed system information
11. **Auto-recovery**: Automatic recovery from emergency states
12. **System Testing**: Built-in test functions
13. **Better State Management**: Cleaner transitions between modes
14. **Enhanced Logging**: More informative status messages
15. **Memory Optimization**: Using PROGMEM and efficient data structures

=== USAGE NOTES ===

- The system now includes comprehensive error handling and recovery
- Watchdog timer prevents system freezes
- All sensor readings are filtered and validated
- System health is continuously monitored
- Emergency states have automatic recovery mechanisms
- Performance is monitored and reported
- Memory usage is tracked and managed
- Communication errors are handled gracefully
- Motor control includes safety features like soft start
- Comprehensive diagnostic information is available

Use the "diagnostic" command to get detailed system information.
Use the "status" command for quick system overview.
The system will automatically report issues and take corrective action.

*/
>>>>>>> Stashed changes
=======
  if (!esp_now_initialized) {
    Serial.println("[ESP-NOW] Failed to initialize!");
  }
  
  // Status LED
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  
  Serial.println("============================");
  Serial.println("[READY] Robot ready!");
  Serial.printf("Map size: 50x50 (5m x 5m)\n");
  Serial.printf("Sensor rate: %dms\n", SENSOR_UPDATE_RATE);
  Serial.printf("Data rate: %dms\n", DATA_SEND_RATE);
  Serial.println("Controller: 48:E7:29:C9:57:28");
  Serial.println("============================");
}

// ================================
// MAIN LOOP - HIGH PERFORMANCE
// ================================

void loop() {
  unsigned long now = millis();
  
  // Status LED blink (low priority)
  static bool ledState = false;
  static unsigned long lastBlink = 0;
  if (now - lastBlink >= 500) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(2, ledState);
  }
  
  // HIGH PRIORITY: Trigger sensors frequently
  if (now - lastTriggerTime >= SENSOR_UPDATE_RATE) {
    lastTriggerTime = now;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
  
  // HIGH PRIORITY: Process sensor readings immediately
  processSensorReadings();
  
  // HIGH PRIORITY: Send data frequently
  if (now - lastDataSendTime >= DATA_SEND_RATE) {
    lastDataSendTime = now;
    sendRobotData();
  }
  
  // MEDIUM PRIORITY: Motor timeout safety
  if (motorActive && (now - lastMotorCommand > MOTOR_TIMEOUT)) {
    stop_motor();
    Serial.println("[SAFETY] Motor timeout");
  }
  
  // LOW PRIORITY: Status updates
  if (now - lastStatusTime >= STATUS_PRINT_RATE) {
    lastStatusTime = now;
    Serial.printf("[STATUS] Uptime: %lu sec | Sensors: %.1f %.1f %.1f %.1f\n", 
                  now/1000, sensor_distances[0], sensor_distances[1], 
                  sensor_distances[2], sensor_distances[3]);
  }
  
  // Minimal delay for stability
  delay(5);
}
>>>>>>> Stashed changes
