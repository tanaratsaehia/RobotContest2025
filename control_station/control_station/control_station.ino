/*
 * Real-time Robot Controller
 * Compatible with Real-time Robot Control System
 */

#include <WiFi.h>
#include <esp_now.h>

// ====== MAC ADDRESS ที่จะต้องปรับให้ตรงกับหุ่นยนต์ ======
// ใช้คำสั่ง Serial.println(WiFi.macAddress()); เพื่อหา MAC ของหุ่นยนต์
uint8_t robotAddress[] = {0x48, 0xE7, 0x29, 0xC9, 0xDF, 0x68}; // ต้องแก้ให้ตรงกับ MAC ของหุ่นยนต์

typedef struct struct_message {
  char text[100];
} struct_message;

struct_message incoming;
struct_message outgoing;

// Variables for continuous control
String currentCommand = "stop";
unsigned long commandStartTime = 0;
unsigned long lastHeartbeat = 0;
bool robotConnected = false;

// Callback when data is sent
void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  
  if (status == ESP_NOW_SEND_SUCCESS) {
    robotConnected = true;
    // Print success message only every 1 second to avoid spam
    if (now - lastPrint >= 1000) {
      Serial.println("[✓] Robot connected");
      lastPrint = now;
    }
  } else {
    robotConnected = false;
    Serial.println("[✗] Send FAILED - Check robot power/distance");
  }
}

// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  String response = String(incoming.text);
  
  robotConnected = true;
  lastHeartbeat = millis();
  
  // Parse sensor data and status
  if (response.startsWith("SENSORS:")) {
    // Parse: SENSORS:10.5,25.3,15.2,30.1|CMD:forward|SPEED:120
    int sensorEnd = response.indexOf("|");
    if (sensorEnd > 0) {
      String sensors = response.substring(8, sensorEnd);
      String status = response.substring(sensorEnd + 1);
      
      // แสดงข้อมูลเซนเซอร์แบบเรียลไทม์
      Serial.print("[SENSORS] F:");
      int comma1 = sensors.indexOf(",");
      Serial.print(sensors.substring(0, comma1));
      
      Serial.print(" R:");
      int comma2 = sensors.indexOf(",", comma1 + 1);
      Serial.print(sensors.substring(comma1 + 1, comma2));
      
      Serial.print(" B:");
      int comma3 = sensors.indexOf(",", comma2 + 1);
      Serial.print(sensors.substring(comma2 + 1, comma3));
      
      Serial.print(" L:");
      Serial.print(sensors.substring(comma3 + 1));
      
      // แสดงสถานะ
      if (status.indexOf("CMD:") != -1) {
        int cmdStart = status.indexOf("CMD:") + 4;
        int cmdEnd = status.indexOf("|", cmdStart);
        if (cmdEnd == -1) cmdEnd = status.length();
        String robotCmd = status.substring(cmdStart, cmdEnd);
        Serial.print(" | Status: " + robotCmd);
      }
      
      Serial.println();
    }
  } else {
    Serial.println("[ROBOT] " + response);
  }
}

// Initialize ESP-NOW
void esp_now_begin() {
  WiFi.mode(WIFI_STA);
  
  Serial.print("[CONTROLLER] MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, robotAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Failed to add robot peer");
    return;
  }
  
  Serial.print("[TARGET] Robot MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", robotAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  Serial.println("[✓] ESP-NOW initialized");
}

// Send command to robot
void send_command(String cmd) {
  snprintf(outgoing.text, sizeof(outgoing.text), "%s", cmd.c_str());
  esp_err_t result = esp_now_send(robotAddress, (uint8_t *)&outgoing, sizeof(outgoing));
  
  if (result == ESP_OK) {
    if (cmd != currentCommand) {
      Serial.println("[CMD] " + currentCommand + " -> " + cmd);
      currentCommand = cmd;
      commandStartTime = millis();
    }
  } else {
    Serial.printf("[ERROR] Send failed: %d\n", result);
  }
}

// Continuous command sending for real-time control
void sendContinuousCommand() {
  if (currentCommand != "stop") {
    send_command(currentCommand);
  }
}

void printHelp() {
  Serial.println("\n========================================");
  Serial.println("  🤖 REAL-TIME ROBOT CONTROLLER 🤖");
  Serial.println("========================================");
  Serial.println("📱 MOVEMENT CONTROLS:");
  Serial.println("  w/W/8     - Forward");
  Serial.println("  s/S/2     - Backward");  
  Serial.println("  a/A/4     - Turn Left");
  Serial.println("  d/D/6     - Turn Right");
  Serial.println("  q/Q/7     - Strafe Left");
  Serial.println("  e/E/9     - Strafe Right");
  Serial.println("  x/X/5     - Stop");
  Serial.println("");
  Serial.println("⚡ SPEED CONTROLS:");
  Serial.println("  1         - Slow speed");
  Serial.println("  2         - Normal speed");
  Serial.println("  3         - Fast speed");
  Serial.println("");
  Serial.println("🔧 SYSTEM COMMANDS:");
  Serial.println("  test      - Connection test");
  Serial.println("  info      - System information");
  Serial.println("  help      - Show this help");
  Serial.println("========================================");
  Serial.println("💡 TIP: Commands work in real-time!");
  Serial.println("    Hold key for continuous movement");
  Serial.println("========================================\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  printHelp();
  esp_now_begin();
  
  Serial.println("🚀 [READY] Controller ready!");
  Serial.println("📍 Waiting for robot connection...\n");
  
  // Send initial test command
  delay(1000);
  send_command("stop");
}

void loop() {
  unsigned long now = millis();
  
  // Check robot connection timeout
  if (robotConnected && (now - lastHeartbeat > 3000)) {
    robotConnected = false;
    Serial.println("⚠️  [WARNING] Robot connection lost");
  }
  
  // Read commands from Serial
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command.length() > 0) {
      // Movement commands
      if (command == "w" || command == "8" || command == "forward") {
        send_command("forward");
      }
      else if (command == "s" || command == "2" || command == "backward") {
        send_command("backward");
      }
      else if (command == "a" || command == "4" || command == "left") {
        send_command("left");
      }
      else if (command == "d" || command == "6" || command == "right") {
        send_command("right");
      }
      else if (command == "q" || command == "7" || command == "strafe_left") {
        send_command("strafe_left");
      }
      else if (command == "e" || command == "9" || command == "strafe_right") {
        send_command("strafe_right");
      }
      else if (command == "x" || command == "5" || command == "stop") {
        send_command("stop");
      }
      // Speed commands
      else if (command == "1" || command == "slow") {
        send_command("speed:slow");
        Serial.println("🐌 Speed set to SLOW");
      }
      else if (command == "2" || command == "normal") {
        send_command("speed:normal");
        Serial.println("🚶 Speed set to NORMAL");
      }
      else if (command == "3" || command == "fast") {
        send_command("speed:fast");
        Serial.println("🏃 Speed set to FAST");
      }
      // System commands
      else if (command == "test") {
        Serial.println("🔄 Testing connection...");
        send_command("forward");
        delay(500);
        send_command("stop");
        Serial.println("✅ Test complete");
      }
      else if (command == "info") {
        Serial.println("\n📊 SYSTEM INFO:");
        Serial.printf("Controller MAC: %s\n", WiFi.macAddress().c_str());
        Serial.print("Robot MAC: ");
        for (int i = 0; i < 6; i++) {
          Serial.printf("%02X", robotAddress[i]);
          if (i < 5) Serial.print(":");
        }
        Serial.println();
        Serial.printf("Connection: %s\n", robotConnected ? "✅ Connected" : "❌ Disconnected");
        Serial.printf("Current Command: %s\n", currentCommand.c_str());
        Serial.printf("WiFi Channel: %d\n", WiFi.channel());
        Serial.println();
      }
      else if (command == "help") {
        printHelp();
      }
      else {
        // Forward unknown command directly
        send_command(command);
        Serial.println("🔄 Sent: " + command);
      }
    }
  }
  
  // Send continuous commands every 100ms for real-time control
  static unsigned long lastContinuous = 0;
  if (now - lastContinuous >= 100) {
    lastContinuous = now;
    if (currentCommand != "stop") {
      sendContinuousCommand();
    }
  }
  
  delay(10);
}