#include <WiFi.h>
#include <esp_now.h>

// ==================== CONFIGURATION ====================
#define SERIAL_BAUD 115200
#define DEBUG_MODE 1

// Robot MAC address
uint8_t robotAddress[] = {0x48, 0xe7, 0x29, 0xc9, 0xdf, 0x68};

// Map configuration
#define MAP_SIZE 30
#define CELL_SIZE 5.0

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
}

// ==================== ESP-NOW FUNCTIONS ====================
void initializeESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.println("📡 Initializing ESP-NOW...");
  Serial.println("Station MAC: " + WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW initialization failed");
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
    Serial.println("❌ Failed to add robot peer");
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
}