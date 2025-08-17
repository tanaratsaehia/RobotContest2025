<<<<<<< Updated upstream
// communication.ino - ESP-NOW Communication System (FIXED)

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
#include <WiFi.h>
#include <esp_now.h>
#include <BMI160Gen.h>
#include <ESP32Servo.h>
#include <vector>

// ======================== PIN DEFINITIONS ========================
#define TRIG_PIN   32
#define ECHO1_PIN  39
#define ECHO2_PIN  34
#define ECHO3_PIN  36
#define ECHO4_PIN  35

#define BUTTON_PIN 27
#define BUZZER_PIN 14

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
volatile unsigned long echo_start[4] = {0, 0, 0, 0};
volatile unsigned long echo_end[4]   = {0, 0, 0, 0};
volatile bool echo_done[4]           = {false, false, false, false};

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
  for (int i = 0; i < 4; i++) {
    if (echo_done[i]) {
      noInterrupts();
      unsigned long duration = echo_end[i] - echo_start[i];
      echo_done[i] = false;
      interrupts();
      
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
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
  
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