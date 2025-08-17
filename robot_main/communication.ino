// communication.ino - ESP-NOW Communication System

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
  
  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
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

// ==================== ESP-NOW CALLBACKS ====================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG_MODE && status != ESP_NOW_SEND_SUCCESS) {
    DEBUG_PRINTLN("📤 Send failed to: " + macToString(mac_addr));
  }
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
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
}