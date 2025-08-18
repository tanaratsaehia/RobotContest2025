/*
 * Control Station for Robot Mapping System
 * FINAL VERSION WITH CORRECT MAC ADDRESSES
 * 
 * Controller MAC: 0x7c,0x87,0xce,0x2f,0xe3,0x20 (this ESP32)
 * Robot MAC:      0x48,0xE7,0x29,0xC9,0xDF,0x68 (target)
 */

#include <WiFi.h>
#include <esp_now.h>

// ====== CORRECT ROBOT MAC ADDRESS ======
uint8_t robotAddress[] = {0x48, 0xE7, 0x29, 0xC9, 0xDF, 0x68};

typedef struct struct_message {
  char text[100];
} struct_message;

struct_message incoming;
struct_message outgoing;

// Callback when data is sent
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Serial.print("[STATION] Send to Robot: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
  
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[DEBUG] Check robot power and distance");
  }
}

// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.print("[ROBOT] ");
  Serial.println(incoming.text);
}

// Initialize ESP-NOW
void esp_now_begin() {
  WiFi.mode(WIFI_STA);
  
  Serial.print("[STATION] Controller MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[STATION] Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[STATION] Failed to add robot peer");
    return;
  }
  
  Serial.print("[STATION] Target Robot MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", robotAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  Serial.println("[STATION] ESP-NOW initialized successfully");
}

// Send a command string over ESP-NOW
void send_command(const char *cmd) {
  snprintf(outgoing.text, sizeof(outgoing.text), "%s", cmd);
  esp_err_t result = esp_now_send(robotAddress, (uint8_t *)&outgoing, sizeof(outgoing));
  
  if (result == ESP_OK) {
    Serial.printf("[STATION] Sent command: %s\n", cmd);
  } else {
    Serial.printf("[STATION] Send failed with error: %d\n", result);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("  Robot Mapping System - CONTROLLER");
  Serial.println("  FINAL VERSION - CORRECT MAC");
  Serial.println("========================================");
  Serial.println("Commands from Python/Serial:");
  Serial.println("  w/W - Forward");
  Serial.println("  s/S - Backward");  
  Serial.println("  a/A - Turn Left");
  Serial.println("  d/D - Turn Right");
  Serial.println("  x/X - Stop");
  Serial.println("  m/M - Start Autonomous Mapping");
  Serial.println("  n/N - Stop Autonomous Mapping");
  Serial.println("  g/G - Get Map Data");
  Serial.println("  test - Connection Test");
  Serial.println("========================================");
  
  esp_now_begin();
  
  Serial.println("[READY] Control Station ready for commands");
  Serial.println("Expected Robot MAC: 48:E7:29:C9:DF:68");
  Serial.println("========================================");
}

void loop() {
  // Read commands from Serial (Python controller)
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.printf("[INPUT] Received: %s\n", command.c_str());
      
      // Process command
      if (command == "w" || command == "W" || command == "forward") {
        send_command("forward");
      }
      else if (command == "s" || command == "S" || command == "backward") {
        send_command("backward");
      }
      else if (command == "a" || command == "A" || command == "turn_left") {
        send_command("turn_left");
      }
      else if (command == "d" || command == "D" || command == "turn_right") {
        send_command("turn_right");
      }
      else if (command == "x" || command == "X" || command == "stop") {
        send_command("stop");
      }
      else if (command == "m" || command == "M" || command == "start_mapping") {
        send_command("start_mapping");
        Serial.println("[STATION] Requesting autonomous mapping start");
      }
      else if (command == "n" || command == "N" || command == "stop_mapping") {
        send_command("stop_mapping");
        Serial.println("[STATION] Requesting autonomous mapping stop");
      }
      else if (command == "g" || command == "G" || command == "get_map") {
        send_command("get_map");
        Serial.println("[STATION] Requesting map data");
      }
      else if (command == "test") {
        // Connection test
        Serial.println("[TEST] Testing ESP-NOW connection...");
        send_command("forward");
        delay(200);
        send_command("stop");
        delay(200);
        send_command("get_map");
        Serial.println("[TEST] Test commands sent");
      }
      else if (command == "info") {
        // System info
        Serial.println("\n=== SYSTEM INFO ===");
        Serial.printf("Controller MAC: %s\n", WiFi.macAddress().c_str());
        Serial.print("Target Robot MAC: ");
        for (int i = 0; i < 6; i++) {
          Serial.printf("%02X", robotAddress[i]);
          if (i < 5) Serial.print(":");
        }
        Serial.println();
        Serial.printf("WiFi Channel: %d\n", WiFi.channel());
        Serial.println("==================");
      }
      else {
        // Forward any other command directly
        send_command(command.c_str());
      }
    }
  }
  
  delay(10);
}