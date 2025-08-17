#include <WiFi.h>
#include <esp_now.h>

// MAC Address ของ Control Station (ตรวจสอบให้ถูกต้อง)
uint8_t stationAddress[] = {0x48, 0xe7, 0x29, 0xc9, 0x57, 0x28};

typedef struct struct_message {
  char text[50];
} struct_message;

struct_message incoming;
struct_message outgoing;

void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.print("Received command: ");
  Serial.println(incoming.text);
  
  // ส่งข้อความยืนยันกลับ
  snprintf(outgoing.text, sizeof(outgoing.text), "ACK: %s", incoming.text);
  esp_now_send(stationAddress, (uint8_t *)&outgoing, sizeof(outgoing));
  
  // แสดงคำสั่งที่ได้รับ
  if (strcmp(incoming.text, "forward") == 0) {
    Serial.println("-> MOVING FORWARD");
  } else if (strcmp(incoming.text, "backward") == 0) {
    Serial.println("-> MOVING BACKWARD");
  } else if (strcmp(incoming.text, "turn_left") == 0) {
    Serial.println("-> TURNING LEFT");
  } else if (strcmp(incoming.text, "turn_right") == 0) {
    Serial.println("-> TURNING RIGHT");
  } else if (strcmp(incoming.text, "stop") == 0) {
    Serial.println("-> STOPPING");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("=== ROBOT TEST SYSTEM ===");
  
  // แสดง MAC address
  WiFi.mode(WIFI_STA);
  Serial.print("Robot MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Target Station MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", stationAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  // เริ่ม ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  // เพิ่ม peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, stationAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("ESP-NOW initialized successfully");
  Serial.println("Robot ready for commands!");
  Serial.println("==========================");
}

void loop() {
  static unsigned long lastSend = 0;
  static int counter = 0;
  
  // ส่ง heartbeat ทุก 5 วินาที
  if (millis() - lastSend > 5000) {
    lastSend = millis();
    counter++;
    
    snprintf(outgoing.text, sizeof(outgoing.text), "Robot alive %d", counter);
    esp_now_send(stationAddress, (uint8_t *)&outgoing, sizeof(outgoing));
    
    Serial.printf("Heartbeat sent: %d\n", counter);
  }
  
  delay(100);
}