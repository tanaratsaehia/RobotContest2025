/*
 * Minimal Robot Code for Testing
 * Basic functionality only to avoid reset issues
 */

#include <WiFi.h>
#include <esp_now.h>

// MAC Addresses
uint8_t controllerAddress[] = {0x7c, 0x87, 0xce, 0x2f, 0xe3, 0x20};

// Ultrasonic pins
#define TRIG_PIN   32
#define ECHO1_PIN  39  // Front
#define ECHO2_PIN  34  // Right
#define ECHO3_PIN  36  // Back
#define ECHO4_PIN  35  // Left

#define SOUND_SPEED 0.0343

// Motor pins
#define MOTOR_FRONT_A1 19
#define MOTOR_FRONT_A2 18
#define MOTOR_FRONT_B1 33
#define MOTOR_FRONT_B2 23
#define MOTOR_BACK_A1 4
#define MOTOR_BACK_A2 13
#define MOTOR_BACK_B1 16
#define MOTOR_BACK_B2 17

// Global variables
volatile unsigned long echo_start[4] = {0, 0, 0, 0};
volatile unsigned long echo_end[4] = {0, 0, 0, 0};
volatile bool echo_done[4] = {false, false, false, false};
float sensor_distances[4] = {999.0, 999.0, 999.0, 999.0};

unsigned long lastTriggerTime = 0;
unsigned long lastStatusTime = 0;
bool esp_now_initialized = false;

// ESP-NOW message structure
typedef struct struct_message {
  char text[50];
} struct_message;

struct_message incoming;
struct_message outgoing;

// Motor control functions
void stop_motor() {
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 0);
}

void move_forward() {
  Serial.println("[MOTOR] Moving forward");
  analogWrite(MOTOR_FRONT_A1, 150);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 150);
  analogWrite(MOTOR_BACK_A1, 150);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 150);
}

void move_backward() {
  Serial.println("[MOTOR] Moving backward");
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 150);
  analogWrite(MOTOR_FRONT_B1, 150);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 150);
  analogWrite(MOTOR_BACK_B1, 150);
  analogWrite(MOTOR_BACK_B2, 0);
}

void turn_left() {
  Serial.println("[MOTOR] Turning left");
  analogWrite(MOTOR_FRONT_A1, 150);
  analogWrite(MOTOR_FRONT_A2, 0);
  analogWrite(MOTOR_FRONT_B1, 150);
  analogWrite(MOTOR_FRONT_B2, 0);
  analogWrite(MOTOR_BACK_A1, 150);
  analogWrite(MOTOR_BACK_A2, 0);
  analogWrite(MOTOR_BACK_B1, 150);
  analogWrite(MOTOR_BACK_B2, 0);
}

void turn_right() {
  Serial.println("[MOTOR] Turning right");
  analogWrite(MOTOR_FRONT_A1, 0);
  analogWrite(MOTOR_FRONT_A2, 150);
  analogWrite(MOTOR_FRONT_B1, 0);
  analogWrite(MOTOR_FRONT_B2, 150);
  analogWrite(MOTOR_BACK_A1, 0);
  analogWrite(MOTOR_BACK_A2, 150);
  analogWrite(MOTOR_BACK_B1, 0);
  analogWrite(MOTOR_BACK_B2, 150);
}

// Ultrasonic interrupt handlers
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

// ESP-NOW callbacks
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW] Send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAILED");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  String command = String(incoming.text);
  
  Serial.print("[ESP-NOW] Received: ");
  Serial.println(command);
  
  // Handle commands
  if (command == "forward") {
    move_forward();
  } else if (command == "backward") {
    move_backward();
  } else if (command == "turn_left") {
    turn_left();
  } else if (command == "turn_right") {
    turn_right();
  } else if (command == "stop") {
    stop_motor();
  } else {
    Serial.println("[ESP-NOW] Unknown command, stopping");
    stop_motor();
  }
  
  // Auto stop after 2 seconds
  delay(2000);
  stop_motor();
  Serial.println("[MOTOR] Auto stopped");
}

void sendPositionData() {
  if (!esp_now_initialized) return;
  
  snprintf(outgoing.text, sizeof(outgoing.text), 
           "POS:50.0,50.0,0.0|S:%.1f,%.1f,%.1f,%.1f", 
           sensor_distances[0], sensor_distances[1], 
           sensor_distances[2], sensor_distances[3]);
  
  esp_err_t result = esp_now_send(controllerAddress, (uint8_t*)&outgoing, sizeof(outgoing));
  if (result == ESP_OK) {
    Serial.println("[ESP-NOW] Position data sent");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // Give more time for startup
  
  Serial.println("================================");
  Serial.println("  Minimal Robot Test");
  Serial.println("================================");
  
  // Initialize pins
  Serial.println("[SETUP] Initializing pins...");
  
  // Motor pins
  pinMode(MOTOR_FRONT_A1, OUTPUT);
  pinMode(MOTOR_FRONT_A2, OUTPUT);
  pinMode(MOTOR_FRONT_B1, OUTPUT);
  pinMode(MOTOR_FRONT_B2, OUTPUT);
  pinMode(MOTOR_BACK_A1, OUTPUT);
  pinMode(MOTOR_BACK_A2, OUTPUT);
  pinMode(MOTOR_BACK_B1, OUTPUT);
  pinMode(MOTOR_BACK_B2, OUTPUT);
  stop_motor();
  Serial.println("[MOTOR] Initialized");
  
  // Ultrasonic pins
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
  Serial.println("[ULTRASONIC] Initialized");
  
  // Initialize WiFi and ESP-NOW
  Serial.println("[SETUP] Initializing ESP-NOW...");
  WiFi.mode(WIFI_STA);
  Serial.print("[WIFI] Robot MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init failed!");
    esp_now_initialized = false;
  } else {
    Serial.println("[ESP-NOW] Init success");
    
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controllerAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ESP-NOW] Add peer failed!");
      esp_now_initialized = false;
    } else {
      Serial.print("[ESP-NOW] Added controller: ");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", controllerAddress[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println();
      esp_now_initialized = true;
    }
  }
  
  // LED indicator
  pinMode(2, OUTPUT);
  
  Serial.println("================================");
  Serial.println("[READY] Robot ready for commands!");
  Serial.println("Expected controller: 7C:87:CE:2F:E3:20");
  Serial.printf("[STATUS] ESP-NOW: %s\n", esp_now_initialized ? "OK" : "FAILED");
  Serial.println("================================");
}

void loop() {
  unsigned long now = millis();
  
  // Blink LED to show robot is alive
  static bool ledState = false;
  static unsigned long lastBlink = 0;
  if (now - lastBlink >= 1000) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(2, ledState);
  }
  
  // Trigger ultrasonic sensors
  if (now - lastTriggerTime >= 200) {
    lastTriggerTime = now;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
  }
  
  // Process sensor readings
  for (int i = 0; i < 4; i++) {
    if (echo_done[i]) {
      noInterrupts();
      unsigned long duration = echo_end[i] - echo_start[i];
      echo_done[i] = false;
      interrupts();
      
      float distance = (duration * SOUND_SPEED) / 2.0;
      if (distance > 2 && distance < 400) {
        sensor_distances[i] = distance;
      } else {
        sensor_distances[i] = 999.0;
      }
      
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(sensor_distances[i]);
      Serial.println(" cm");
    }
  }
  
  // Send status every 5 seconds
  if (now - lastStatusTime >= 5000) {
    lastStatusTime = now;
    Serial.printf("[STATUS] Uptime: %lu ms, ESP-NOW: %s\n", 
                  now, esp_now_initialized ? "OK" : "FAILED");
    sendPositionData();
  }
  
  delay(10);
}