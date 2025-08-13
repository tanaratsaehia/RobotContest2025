#include <WiFi.h>
#include <esp_now.h>

// CHANGE THIS to the MAC address of the other ESP32
uint8_t peerAddress[] = {0x48, 0xe7, 0x29, 0xc9, 0x57, 0x28}; 

typedef struct struct_message {
  char text[50];
} struct_message;

struct_message incoming;
struct_message outgoing;

// Callback when data is sent
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Serial.print("Last Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.print("Got message: ");
  Serial.println(incoming.text);
}

void esp_now_begin(){
  // Wi-Fi in station mode
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

static unsigned long lastSend = 0;
void test_send_message(int ms_interval){
  if (millis() - lastSend > ms_interval) {
    lastSend = millis();
    snprintf(outgoing.text, sizeof(outgoing.text), "Hello from Robot %s", WiFi.macAddress().c_str());
    esp_now_send(peerAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  }
}
