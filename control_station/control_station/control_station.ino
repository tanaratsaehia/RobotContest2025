#include <WiFi.h>
#include <esp_now.h>

<<<<<<< Updated upstream
// ==================== CONFIGURATION ====================
#define SERIAL_BAUD 115200
#define DEBUG_MODE 1
=======
// MAC of the robot car ESP32 - UPDATED ADDRESS
uint8_t peerAddress[] = {0x48, 0xE7, 0x29, 0xC9, 0xDF, 0x68};
>>>>>>> Stashed changes

<<<<<<< Updated upstream
// Robot MAC address
uint8_t robotAddress[] = {0x48, 0xe7, 0x29, 0xc9, 0xdf, 0x68};
=======
typedef struct struct_message {
  char command[50];
  float param1;
  float param2;
  int mode;  // 0=manual, 1=autonomous_mapping
} struct_message;
>>>>>>> Stashed changes

// Map configuration
#define MAP_SIZE 30
#define CELL_SIZE 5.0

<<<<<<< Updated upstream
// ==================== DATA STRUCTURES ====================
struct ESPNowMessage {
  char type[10];
  char data[200];
  uint8_t checksum;
};

struct RobotStatus {
  float x, y, heading;
  String state;
  float mapCompletion;
  float sensors[4];
  unsigned long lastUpdate;
};

struct MapData {
  int8_t grid[MAP_SIZE][MAP_SIZE];
  unsigned long lastUpdate;
  int knownCells;
};

// ==================== GLOBAL VARIABLES ====================
ESPNowMessage outgoingMsg;
ESPNowMessage incomingMsg;
RobotStatus robotStatus;
MapData globalMap;
bool robotConnected = false;
unsigned long lastHeartbeat = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial);
  
  Serial.println("🏢 Control Station Starting...");
  Serial.println("===============================");
  
  // Initialize map
  initializeMap();
  
  // Initialize ESP-NOW
  initializeESPNow();
  
  // Initialize robot status
  robotStatus.x = 0;
  robotStatus.y = 0;
  robotStatus.heading = 0;
  robotStatus.state = "UNKNOWN";
  robotStatus.mapCompletion = 0;
  robotStatus.lastUpdate = 0;
  
  Serial.println("✅ Control Station ready!");
  Serial.println("Commands:");
  Serial.println("  m - Start/Stop mapping");
  Serial.println("  s - Get status");
  Serial.println("  w - Add waypoint");
  Serial.println("  g - Start mission");
  Serial.println("  x - Emergency stop");
  Serial.println("  r - Reset robot");
  Serial.println("===============================");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Handle serial commands
  handleSerialCommands();
  
  // Check robot connection
  checkRobotConnection();
  
  // Send heartbeat
  sendHeartbeat();
  
  delay(100);
=======
enum ControlMode {
  MANUAL_CONTROL = 0,
  AUTONOMOUS_MAPPING = 1,
  MONITORING = 2
};

ControlMode current_mode = MANUAL_CONTROL;

<<<<<<< Updated upstream
// Callback when data is sent
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Serial.print("Received from: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr),
           "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);
  Serial.print(macStr);
  Serial.print(" -> ");
  Serial.write(data, len);
  Serial.println();
>>>>>>> Stashed changes
=======
// Callback when data is sent - FIXED SIGNATURE
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)  {
  Serial.printf("[ESP32] Last Send Status: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received - FIXED SIGNATURE
void onDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.println("[ESP32] Got message: " + String(incoming.text));
  
  // Process mapping and navigation data
  process_robot_data(incoming.text);
}

// Process data from robot
void process_robot_data(const char* data) {
  if (strncmp(data, "POS:", 4) == 0) {
    // Robot position data: "POS:x,y,angle"
    float x, y, angle;
    if (sscanf(data + 4, "%f,%f,%f", &x, &y, &angle) == 3) {
      Serial.printf("[MAP] Robot position: (%.1f, %.1f) angle: %.1f°\n", x, y, angle);
    }
  } else if (strncmp(data, "OBS:", 4) == 0) {
    // Obstacle data: "OBS:x,y"
    int x, y;
    if (sscanf(data + 4, "%d,%d", &x, &y) == 2) {
      Serial.printf("[MAP] Obstacle detected at grid: (%d, %d)\n", x, y);
    }
  } else if (strncmp(data, "NAV_POS:", 8) == 0) {
    // Navigation position: "NAV_POS:x,y,angle"
    float x, y, angle;
    if (sscanf(data + 8, "%f,%f,%f", &x, &y, &angle) == 3) {
      Serial.printf("[NAV] Position: (%.1f, %.1f) angle: %.1f°\n", x, y, angle);
    }
  } else if (strncmp(data, "NAV_WP:", 7) == 0) {
    // Navigation waypoint: "NAV_WP:id,x,y,distance"
    int id;
    float x, y, distance;
    if (sscanf(data + 7, "%d,%f,%f,%f", &id, &x, &y, &distance) == 4) {
      Serial.printf("[NAV] Waypoint %d: (%.1f, %.1f) distance: %.1f cm\n", id, x, y, distance);
    }
  } else if (strncmp(data, "Sensor", 6) == 0) {
    // Sensor data - keep existing format
    Serial.println("[SENSOR] " + String(data));
  }
>>>>>>> Stashed changes
}

// ==================== ESP-NOW FUNCTIONS ====================
void initializeESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.println("📡 Initializing ESP-NOW...");
  Serial.println("Station MAC: " + WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
<<<<<<< Updated upstream
    Serial.println("❌ ESP-NOW initialization failed");
=======
    Serial.println("[Control Station] Error initializing ESP-NOW");
    Serial.flush();
>>>>>>> Stashed changes
    return;
  }
  
  // Register callbacks - FIXED VERSION
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  // Add robot as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
<<<<<<< Updated upstream
    Serial.println("❌ Failed to add robot peer");
=======
    Serial.println("[Control Station] Failed to add peer");
    Serial.flush();
>>>>>>> Stashed changes
    return;
  }
  
  Serial.print("✅ ESP-NOW initialized, Robot MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(String(robotAddress[i], HEX));
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

// ==================== FIXED CALLBACK FUNCTIONS ====================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG_MODE && status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("📤 Send failed");
  }
}

<<<<<<< Updated upstream
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ESPNowMessage)) {
    memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
    
    robotConnected = true;
    lastHeartbeat = millis();
    
    processIncomingMessage();
  }
}

// ==================== MESSAGE PROCESSING ====================
void processIncomingMessage() {
  String msgType = String(incomingMsg.type);
  String msgData = String(incomingMsg.data);
  
  if (msgType == "STATUS") {
    processStatusMessage(msgData);
  } else if (msgType == "MAP") {
    processMapMessage(msgData);
  } else if (msgType == "MISSION") {
    processMissionMessage(msgData);
  } else if (msgType == "HEARTBEAT") {
    processHeartbeatMessage(msgData);
  } else if (msgType == "ACK") {
    Serial.println("📨 ACK: " + msgData);
  } else {
    Serial.println("📨 " + msgType + ": " + msgData);
  }
}

void processStatusMessage(String statusData) {
  // Parse: "x,y,heading,state,mapCompletion,sensor1:sensor2:sensor3:sensor4"
  int commaCount = 0;
  int lastIndex = 0;
  
  for (int i = 0; i <= statusData.length(); i++) {
    if (i == statusData.length() || statusData.charAt(i) == ',') {
      String value = statusData.substring(lastIndex, i);
      
      switch (commaCount) {
        case 0: robotStatus.x = value.toFloat(); break;
        case 1: robotStatus.y = value.toFloat(); break;
        case 2: robotStatus.heading = value.toFloat(); break;
        case 3: robotStatus.state = value; break;
        case 4: robotStatus.mapCompletion = value.toFloat(); break;
        case 5: parseSensorData(value); break;
      }
      
      lastIndex = i + 1;
      commaCount++;
    }
  }
  
  robotStatus.lastUpdate = millis();
  
  // Print status to serial for Python GUI
  Serial.println("[STATUS] Robot: (" + String(robotStatus.x, 1) + "," + 
                String(robotStatus.y, 1) + ") " + String(robotStatus.heading, 1) + 
                "° | State: " + robotStatus.state + " | Map: " + 
                String(robotStatus.mapCompletion, 1) + "%");
}

void parseSensorData(String sensorString) {
  int colonCount = 0;
  int lastIndex = 0;
  
  for (int i = 0; i <= sensorString.length(); i++) {
    if (i == sensorString.length() || sensorString.charAt(i) == ':') {
      if (colonCount < 4) {
        robotStatus.sensors[colonCount] = sensorString.substring(lastIndex, i).toFloat();
      }
      lastIndex = i + 1;
      colonCount++;
    }
  }
}

void processMapMessage(String mapData) {
  if (mapData == "END") {
    Serial.println("📍 Map data transfer complete");
    globalMap.lastUpdate = millis();
    return;
  }
  
  // Parse: "x,y,value"
  int firstComma = mapData.indexOf(',');
  int secondComma = mapData.indexOf(',', firstComma + 1);
  
  if (firstComma > 0 && secondComma > 0) {
    int x = mapData.substring(0, firstComma).toInt();
    int y = mapData.substring(firstComma + 1, secondComma).toInt();
    int value = mapData.substring(secondComma + 1).toInt();
    
    if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE) {
      globalMap.grid[x][y] = value;
      if (value != -1) globalMap.knownCells++;
    }
  }
}

void processMissionMessage(String missionData) {
  Serial.println("📋 Mission: " + missionData);
}

void processHeartbeatMessage(String heartbeatData) {
  // Robot is alive
  robotConnected = true;
  lastHeartbeat = millis();
}

// ==================== COMMAND HANDLING ====================
void handleSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'M':
      case 'm':
        toggleMapping();
        break;
      case 'S':
      case 's':
        requestStatus();
        break;
      case 'W':
      case 'w':
        addWaypointInteractive();
        break;
      case 'G':
      case 'g':
        startMission();
        break;
      case 'X':
      case 'x':
        emergencyStop();
        break;
      case 'R':
      case 'r':
        resetRobot();
        break;
      case 'T':
      case 't':
        printSystemStatus();
        break;
      case 'C':
      case 'c':
        clearMap();
        break;
      default:
        // Check if it's a waypoint command from Python GUI
        if (cmd >= '0' && cmd <= '9') {
          handlePythonCommand(cmd);
        }
    }
  }
}

void handlePythonCommand(char cmd) {
  // Handle commands from Python GUI
  // Format: "1x,y" where 1 is command type and x,y are coordinates
  String command = "";
  command += cmd;
  
  // Read rest of command
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') break;
    command += c;
  }
  
  processPythonCommand(command);
}

void processPythonCommand(String command) {
  if (command.startsWith("1")) {
    // Add waypoint: "1x,y"
    String coords = command.substring(1);
    int commaIndex = coords.indexOf(',');
    if (commaIndex > 0) {
      float x = coords.substring(0, commaIndex).toFloat();
      float y = coords.substring(commaIndex + 1).toFloat();
      sendWaypoint(x, y);
    }
  } else if (command.startsWith("2")) {
    // Start mission
    startMission();
  } else if (command.startsWith("3")) {
    // Stop mission
    sendCommand("STOP_MISSION");
  }
}

// ==================== ROBOT COMMANDS ====================
void sendCommand(String command) {
  sendMessage("CMD", command);
  Serial.println("📤 Sent: " + command);
}

void sendMessage(String type, String data) {
  strncpy(outgoingMsg.type, type.c_str(), sizeof(outgoingMsg.type) - 1);
  strncpy(outgoingMsg.data, data.c_str(), sizeof(outgoingMsg.data) - 1);
  outgoingMsg.checksum = calculateChecksum(&outgoingMsg);
  
  esp_now_send(robotAddress, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
}

void toggleMapping() {
  if (robotStatus.state == "MAPPING") {
    sendCommand("STOP_MAPPING");
    Serial.println("⏹️ Stopping mapping");
  } else {
    sendCommand("START_MAPPING");
    Serial.println("🗺️ Starting mapping");
  }
}

void requestStatus() {
  sendCommand("GET_STATUS");
}

void addWaypointInteractive() {
  Serial.println("Enter waypoint coordinates (x,y):");
  
  // Wait for input
  while (!Serial.available()) {
    delay(10);
  }
  
  String input = Serial.readString();
  input.trim();
  
  int commaIndex = input.indexOf(',');
  if (commaIndex > 0) {
    float x = input.substring(0, commaIndex).toFloat();
    float y = input.substring(commaIndex + 1).toFloat();
    sendWaypoint(x, y);
  } else {
    Serial.println("❌ Invalid format. Use: x,y");
  }
}

void sendWaypoint(float x, float y) {
  String waypointData = String(x, 1) + "," + String(y, 1);
  sendMessage("WAYPOINT", waypointData);
  Serial.println("📍 Waypoint added: (" + String(x, 1) + ", " + String(y, 1) + ")");
}

void startMission() {
  sendMessage("MISSION", "START");
  Serial.println("🚀 Mission started");
}

void emergencyStop() {
  sendCommand("EMERGENCY_STOP");
  Serial.println("🛑 Emergency stop sent");
}

void resetRobot() {
  sendCommand("RESET");
  Serial.println("🔄 Reset command sent");
}

// ==================== STATUS FUNCTIONS ====================
void checkRobotConnection() {
  static bool wasConnected = false;
  
  bool isConnected = (millis() - lastHeartbeat < 10000); // 10 second timeout
  
  if (isConnected != wasConnected) {
    if (isConnected) {
      Serial.println("✅ Robot connected");
    } else {
      Serial.println("❌ Robot connection lost");
    }
    wasConnected = isConnected;
  }
  
  robotConnected = isConnected;
}

void sendHeartbeat() {
  static unsigned long lastSent = 0;
  
  if (millis() - lastSent >= 5000) { // Every 5 seconds
    lastSent = millis();
    sendMessage("HEARTBEAT", String(millis() / 1000));
  }
}

void printSystemStatus() {
  Serial.println("\n=== CONTROL STATION STATUS ===");
  Serial.println("Robot Connected: " + String(robotConnected ? "YES" : "NO"));
  
  if (robotConnected) {
    Serial.println("Robot Position: (" + String(robotStatus.x, 1) + ", " + String(robotStatus.y, 1) + ")");
    Serial.println("Robot Heading: " + String(robotStatus.heading, 1) + "°");
    Serial.println("Robot State: " + robotStatus.state);
    Serial.println("Map Completion: " + String(robotStatus.mapCompletion, 1) + "%");
    
    Serial.print("Sensors: ");
    for (int i = 0; i < 4; i++) {
      Serial.print(String(robotStatus.sensors[i], 1) + " ");
    }
    Serial.println();
    
    unsigned long timeSinceUpdate = (millis() - robotStatus.lastUpdate) / 1000;
    Serial.println("Last Update: " + String(timeSinceUpdate) + " seconds ago");
  }
  
  Serial.println("Map Known Cells: " + String(globalMap.knownCells));
  Serial.println("===============================\n");
}

// ==================== MAP FUNCTIONS ====================
void initializeMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      globalMap.grid[x][y] = -1; // Unknown
    }
  }
  globalMap.knownCells = 0;
  globalMap.lastUpdate = 0;
  
  Serial.println("✅ Map initialized");
}

void clearMap() {
  initializeMap();
  sendCommand("CLEAR_MAP");
  Serial.println("🗑️ Map cleared");
}

// ==================== UTILITY FUNCTIONS ====================
uint8_t calculateChecksum(ESPNowMessage* msg) {
  uint8_t checksum = 0;
  uint8_t* data = (uint8_t*)msg;
  
  for (int i = 0; i < sizeof(ESPNowMessage) - 1; i++) {
    checksum ^= data[i];
  }
  
  return checksum;
=======
// Send command with parameters
void send_command(const char *cmd, float p1 = 0, float p2 = 0, int mode = 0) {
  strcpy(outgoing.command, cmd);
  outgoing.param1 = p1;
  outgoing.param2 = p2;
  outgoing.mode = mode;
  
  esp_now_send(peerAddress, (uint8_t *)&outgoing, sizeof(outgoing));
  Serial.printf("[Sent] %s (%.2f, %.2f, mode:%d)\n", cmd, p1, p2, mode);
}

void print_menu() {
  Serial.println("\n=== ROBOT CONTROL STATION ===");
  Serial.println("Manual Control:");
  Serial.println("  w/s/a/d - Move robot");
  Serial.println("  x       - Stop robot");
  Serial.println("");
  Serial.println("Autonomous Mapping:");
  Serial.println("  m       - Start mapping mode");
  Serial.println("  r       - Return to start");
  Serial.println("  p       - Pause mapping");
  Serial.println("  c       - Continue mapping");
  Serial.println("");
  Serial.println("Settings:");
  Serial.println("  1-9     - Set motor speed (10%-90%)");
  Serial.println("  t       - Test sensors");
  Serial.println("  h       - Show this menu");
  Serial.println("=============================\n");
}

// Print available commands
void print_help() {
  Serial.println("\n=== ROBOT CONTROL COMMANDS ===");
  Serial.println("Manual Control:");
  Serial.println("  w - Forward");
  Serial.println("  s - Backward");
  Serial.println("  a - Turn Left");
  Serial.println("  d - Turn Right");
  Serial.println("  x - Stop");
  Serial.println();
  Serial.println("Autonomous Mapping:");
  Serial.println("  m - Start mapping mode");
  Serial.println("  M - Stop mapping mode");
  Serial.println();
  Serial.println("Path Planning:");
  Serial.println("  n - Start navigation");
  Serial.println("  N - Stop navigation");
  Serial.println("  c - Clear waypoints");
  Serial.println("  l - List waypoints");
  Serial.println();
  Serial.println("Pre-defined Paths:");
  Serial.println("  1 - Load competition path 1");
  Serial.println("  2 - Load competition path 2");
  Serial.println();
  Serial.println("Other:");
  Serial.println("  h - Show this help");
  Serial.println("  + - Add sample waypoint (100,0)");
  Serial.println("================================\n");
}

void setup() {
  Serial.begin(115200);
<<<<<<< Updated upstream
  while (!Serial);
  delay(1000);
  
  Serial.println("[Control Station] Starting...");
  esp_now_begin();
  
  print_menu();
  Serial.println("Ready! Enter command:");
}

void loop() {
  // Read command from Serial
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      // ===== MANUAL CONTROL =====
      case 'w':
        send_command("forward", 0, 0, MANUAL_CONTROL);
=======
  delay(1000); // Give USB time to enumerate
  
  Serial.println("\n=== ESP32 Advanced Robot Control Station ===");
  Serial.println("Initializing ESP-NOW...");
  
  esp_now_begin();
  
  Serial.println("Control Station Ready!");
  print_help();
  
  Serial.println("Waiting for commands...");
}

void loop() {
  // Read serial commands from computer/Python GUI
  while (Serial.available()) {
    char cmd = Serial.read();
    
    // Skip newline and carriage return
    if (cmd == '\n' || cmd == '\r') {
      continue;
    }
    
    Serial.print("Command sent: ");
    Serial.println(cmd);
    
    switch (cmd) {
      // Basic movement commands
      case 'w':
        send_command("forward");
        Serial.println("[CMD] Forward");
>>>>>>> Stashed changes
        break;
        
      case 's':
<<<<<<< Updated upstream
        send_command("backward", 0, 0, MANUAL_CONTROL);
=======
        send_command("backward");
        Serial.println("[CMD] Backward");
>>>>>>> Stashed changes
        break;
        
      case 'a':
<<<<<<< Updated upstream
        send_command("turn_left", 0, 0, MANUAL_CONTROL);
=======
        send_command("turn_left");
        Serial.println("[CMD] Turn Left");
>>>>>>> Stashed changes
        break;
        
      case 'd':
<<<<<<< Updated upstream
        send_command("turn_right", 0, 0, MANUAL_CONTROL);
        break;
        
      case 'x':
        send_command("stop", 0, 0, MANUAL_CONTROL);
        break;
      
      // ===== AUTONOMOUS MAPPING =====
      case 'm':
        Serial.println("=== STARTING AUTONOMOUS MAPPING ===");
        current_mode = AUTONOMOUS_MAPPING;
        send_command("start_mapping", 0, 0, AUTONOMOUS_MAPPING);
        break;
        
      case 'r':
        Serial.println("Command: Return to start");
        send_command("return_to_start", 0, 0, AUTONOMOUS_MAPPING);
        break;
        
      case 'p':
        Serial.println("Command: Pause mapping");
        send_command("pause_mapping", 0, 0, AUTONOMOUS_MAPPING);
        break;
        
      case 'c':
        Serial.println("Command: Continue mapping");
        send_command("continue_mapping", 0, 0, AUTONOMOUS_MAPPING);
        break;
      
      // ===== SETTINGS =====
      case '1': case '2': case '3': case '4': case '5':
      case '6': case '7': case '8': case '9':
        {
          int speed = (cmd - '0') * 10; // 10%, 20%, ..., 90%
          Serial.printf("Setting motor speed to %d%%\n", speed);
          send_command("set_speed", speed, 0, current_mode);
        }
        break;
        
      case 't':
        Serial.println("Testing all sensors...");
        send_command("test_sensors", 0, 0, MONITORING);
        break;
        
      case 'h':
        print_menu();
        break;
        
      default:
        if (cmd != '\n' && cmd != '\r') {
          Serial.printf("Unknown command: %c (press 'h' for help)\n", cmd);
        }
=======
        send_command("turn_right");
        Serial.println("[CMD] Turn Right");
        break;
      case 'x':
        send_command("stop");
        Serial.println("[CMD] Stop");
        break;
      
      // Mapping commands
      case 'm':
        send_command("start_mapping");
        Serial.println("[CMD] Start Mapping");
        break;
      case 'M':
        send_command("stop_mapping");
        Serial.println("[CMD] Stop Mapping");
        break;
      
      // Navigation commands
      case 'n':
        send_command("wp_start");
        Serial.println("[CMD] Start Navigation");
        break;
      case 'N':
        send_command("wp_stop");
        Serial.println("[CMD] Stop Navigation");
        break;
      
      // Waypoint management
      case 'c':
        send_command("wp_clear");
        Serial.println("[CMD] Clear Waypoints");
        break;
      case 'l':
        send_command("wp_list");
        Serial.println("[CMD] List Waypoints");
        break;
      
      // Pre-defined paths
      case '1':
        send_command("load_path_1");
        Serial.println("[CMD] Load Competition Path 1");
        break;
      case '2':
        send_command("load_path_2");
        Serial.println("[CMD] Load Competition Path 2");
        break;
      
      // Add sample waypoint
      case '+':
        send_command("wp_add:100,0");
        Serial.println("[CMD] Add waypoint (100,0)");
        break;
      
      // Help
      case 'h':
        print_help();
        break;
      
      // Unknown command
      default:
        Serial.println("[CMD] Unknown command. Press 'h' for help.");
>>>>>>> Stashed changes
        break;
    }
  }
  
<<<<<<< Updated upstream
  // Auto-request status updates during autonomous mode
  static unsigned long last_status_request = 0;
  if (current_mode == AUTONOMOUS_MAPPING && millis() - last_status_request > 5000) {
    last_status_request = millis();
    send_command("get_status", 0, 0, MONITORING);
  }
>>>>>>> Stashed changes
=======
  // Small delay to prevent overwhelming the serial buffer
  delay(10);
>>>>>>> Stashed changes
}