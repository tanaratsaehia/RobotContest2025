#include <WiFi.h>
#include <esp_now.h>

// MAC of the robot car ESP32
uint8_t peerAddress[] = {0x48, 0xE7, 0x29, 0xC9, 0xDF, 0x68};

typedef struct struct_message {
  char text[50];
} struct_message;

struct_message incoming;
struct_message outgoing;

// Callback when data is sent
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Serial.printf("[ESP32] Last Send Status: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  // Serial.flush();
}

// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.println("[ESP32] Got message: " + String(incoming.text));
  // Serial.flush();
}

// Initialize ESP-NOW
void esp_now_begin() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP32] Error initializing ESP-NOW");
    Serial.flush();
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
    Serial.flush();
    return;
  }
}

// Send a command string over ESP-NOW
void send_command(const char *cmd) {
  snprintf(outgoing.text, sizeof(outgoing.text), "%s", cmd);
  esp_now_send(peerAddress, (uint8_t *)&outgoing, sizeof(outgoing));
}

void setup() {
  Serial.begin(115200);
  // while (!Serial);
  delay(1000); // Give USB time to enumerate
  Serial.println("[ESP32] Controller ready. Send w/s/a/d via Python GUI.");
  Serial.flush();
  esp_now_begin();
}

void loop() {
  // Read serial from Python GUI
  while (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'w':
        send_command("forward");
        break;
      case 's':
        send_command("backward");
        break;
      case 'a':
        send_command("turn_left");
        break;
      case 'd':
        send_command("turn_right");
        break;
      default:
        send_command("");
        break;
    }
  }
}
