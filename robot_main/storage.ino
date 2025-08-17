// esp_now_controller.ino - ระบบสื่อสาร ESP-NOW แบบสมบูรณ์

// ESP-NOW callback functions
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("✅ Message sent successfully");
  } else {
    Serial.println("❌ Message send failed");
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  
  Serial.print("📨 Received: ");
  Serial.println(incoming.text);
  
  // Process received commands
  processReceivedCommand(incoming.text);
  
  // Send acknowledgment
  char ackMsg[50];
  snprintf(ackMsg, sizeof(ackMsg), "ACK: %s", incoming.text);
  send_message(ackMsg);
}

// Initialize ESP-NOW
void esp_now_begin() {
  // Set device as Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // Print MAC address
  Serial.print("Robot MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }
  
  Serial.println("✅ ESP-NOW initialized");
  
  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  // Add peer (control station)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, stationAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    return;
  }
  
  Serial.println("✅ Peer added successfully");
  Serial.print("Station MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", stationAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
}

// Send message to control station
void send_message(const char *message) {
  snprintf(outgoing.text, sizeof(outgoing.text), "%s", message);
  
  esp_err_t result = esp_now_send(stationAddress, 
                                  (uint8_t *)&outgoing, 
                                  sizeof(outgoing));
  
  if (result != ESP_OK) {
    Serial.print("❌ Error sending message: ");
    Serial.println(result);
  }
}

// Send structured map data
void send_map_data() {
  // Create map data packet
  struct MapPacket {
    float x, y, heading;
    float sensors[4];
    int8_t localMap[25]; // 5x5 local map
    uint8_t mode; // 0=idle, 1=mapping, 2=avoiding
  } mapPacket;
  
  // Fill packet data
  mapPacket.x = robotX;
  mapPacket.y = robotY;
  mapPacket.heading = robotHeading;
  
  for (int i = 0; i < 4; i++) {
    mapPacket.sensors[i] = sensorDistances[i];
  }
  
  // Compress local map to 1D array
  int idx = 0;
  for (int i = 0; i < LOCAL_MAP_SIZE; i++) {
    for (int j = 0; j < LOCAL_MAP_SIZE; j++) {
      mapPacket.localMap[idx++] = localMap[i][j].occupancy;
    }
  }
  
  mapPacket.mode = isAvoiding ? 2 : (isMapping ? 1 : 0);
  
  // Send packet
  esp_err_t result = esp_now_send(stationAddress, 
                                  (uint8_t *)&mapPacket, 
                                  sizeof(mapPacket));
  
  if (result == ESP_OK) {
    Serial.println("📤 Map data sent");
  } else {
    Serial.println("❌ Failed to send map data");
  }
}

// Test ESP-NOW connection
void test_send_message(int ms_interval) {
  static unsigned long lastSend = 0;
  
  if (millis() - lastSend > ms_interval) {
    lastSend = millis();
    
    char testMsg[100];
    snprintf(testMsg, sizeof(testMsg), 
             "Robot alive - Pos:(%.1f,%.1f) H:%.1f°", 
             robotX, robotY, robotHeading);
    
    send_message(testMsg);
  }
}

// Send telemetry data
void send_telemetry() {
  static unsigned long lastTelemetry = 0;
  
  if (millis() - lastTelemetry > 1000) { // Send every second
    lastTelemetry = millis();
    
    char telemetryMsg[200];
    snprintf(telemetryMsg, sizeof(telemetryMsg),
             "[TELEMETRY] Bat:%.1fV | IMU:OK | Motors:%s | Sensors:%d/%d",
             readBatteryVoltage(),
             isMovingStraight ? "ACTIVE" : "IDLE",
             countValidSensors(), 4);
    
    send_message(telemetryMsg);
  }
}

// Helper functions
float readBatteryVoltage() {
  // Assuming voltage divider on ADC pin
  // Adjust these values based on your hardware
  int adcValue = analogRead(36); // GPIO36 (VP)
  float voltage = (adcValue / 4095.0) * 3.3 * 2; // Assuming 2:1 divider
  return voltage;
}

int countValidSensors() {
  int count = 0;
  for (int i = 0; i < 4; i++) {
    if (sensorValid[i]) count++;
  }
  return count;
}