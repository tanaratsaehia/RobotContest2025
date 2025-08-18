/*
 * ESP-NOW Communication Test
 * Upload this to Control Station ESP32 to test ESP-NOW only
 */

#include <WiFi.h>
#include <esp_now.h>

// Robot MAC address
uint8_t robotAddress[] = {0x48, 0xE7, 0x29, 0xC9, 0xDF, 0x68};

typedef struct test_message {
  char msg[32];
  int counter;
} test_message;

test_message testData;
int messageCounter = 0;

void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Serial.print("Message ");
  Serial.print(messageCounter);
  Serial.print(" to ");
  
  // Print target MAC
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", info->peer_addr[i]);
    if (i < 5) Serial.print(":");
  }
  
  Serial.print(" - Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAILED");
  
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Troubleshooting tips:");
    Serial.println("1. Check robot is powered on");
    Serial.println("2. Check distance (< 50m)");
    Serial.println("3. Check robot MAC address");
    Serial.println("4. Try moving ESP32s closer");
  }
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  Serial.print("Received from ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", info->peer_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(": ");
  Serial.println((char*)incomingData);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== ESP-NOW Communication Test ===");
  
  WiFi.mode(WIFI_STA);
  Serial.print("Controller MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  // Add robot as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add robot peer");
    return;
  }
  
  Serial.print("Target Robot MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", robotAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  Serial.println("Starting ESP-NOW test...");
  Serial.println("Will send test messages every 2 seconds");
}

void loop() {
  static unsigned long lastSend = 0;
  
  if (millis() - lastSend >= 2000) {
    lastSend = millis();
    messageCounter++;
    
    snprintf(testData.msg, sizeof(testData.msg), "Test_%d", messageCounter);
    testData.counter = messageCounter;
    
    Serial.printf("\nSending message %d...\n", messageCounter);
    esp_err_t result = esp_now_send(robotAddress, (uint8_t*)&testData, sizeof(testData));
    
    if (result != ESP_OK) {
      Serial.printf("Send error: %d\n", result);
    }
  }
  
  // Check for serial commands
  if (Serial.available()) {
    String cmd = Serial.readString();
    cmd.trim();
    
    if (cmd == "test") {
      Serial.println("Sending immediate test...");
      snprintf(testData.msg, sizeof(testData.msg), "Immediate");
      testData.counter = 999;
      esp_now_send(robotAddress, (uint8_t*)&testData, sizeof(testData));
    } else if (cmd == "info") {
      Serial.println("\n=== System Info ===");
      Serial.printf("Controller MAC: %s\n", WiFi.macAddress().c_str());
      Serial.print("Target Robot MAC: ");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", robotAddress[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println();
      Serial.printf("WiFi Channel: %d\n", WiFi.channel());
      Serial.printf("Messages sent: %d\n", messageCounter);
    }
  }
}