#include <WiFi.h>
#include <esp_now.h>

// Mode selection - choose one
#define SIMPLE_MODE 0    // เหมือนโค้ดเดิม (keyboard control เท่านั้น)
#define MAPPING_MODE 1   // ระบบ mapping แบบเต็ม

#if MAPPING_MODE
// Mapping mode configuration
#define MAP_SIZE 100
#define CELL_SIZE 10
#define LOCAL_MAP_SIZE 5

struct Position {
  float x, y, heading;
};

typedef struct {
  float x, y, heading;
  float distances[4];
  int8_t mapData[25]; // 5x5 local map
} MapMessage;

// Global map storage
int8_t globalMap[MAP_SIZE][MAP_SIZE];
Position robotPosition;
float sensorDistances[4];
unsigned long lastDataReceived = 0;
bool newDataAvailable = false;

// Initialize global map
void initGlobalMap() {
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      globalMap[i][j] = -1; // Unknown
    }
  }
}

// Convert world coordinates to grid coordinates
void worldToGrid(float worldX, float worldY, int* gridX, int* gridY) {
  *gridX = constrain((int)(worldX / CELL_SIZE), 0, MAP_SIZE - 1);
  *gridY = constrain((int)(worldY / CELL_SIZE), 0, MAP_SIZE - 1);
}

#endif // MAPPING_MODE

// MAC address of the robot car ESP32
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
  Serial.flush();
}

// Callback when data is received
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#if SIMPLE_MODE
  // Simple mode - just print received messages
  memcpy(&incoming, incomingData, sizeof(incoming));
  Serial.printf("[ESP32] Got message: %s\n", incoming.text);
  Serial.flush();
  
#elif MAPPING_MODE
  // Mapping mode - process map data
  if (len == sizeof(MapMessage)) {
    MapMessage* msg = (MapMessage*)incomingData;
    
    // Update robot position
    robotPosition.x = msg->x;
    robotPosition.y = msg->y;
    robotPosition.heading = msg->heading;
    
    // Update sensor distances
    for (int i = 0; i < 4; i++) {
      sensorDistances[i] = msg->distances[i];
    }
    
    // Update global map with received local map
    int robotGridX, robotGridY;
    worldToGrid(robotPosition.x, robotPosition.y, &robotGridX, &robotGridY);
    
    int idx = 0;
    for (int i = -2; i <= 2; i++) {
      for (int j = -2; j <= 2; j++) {
        int x = robotGridX + i;
        int y = robotGridY + j;
        if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE) {
          if (msg->mapData[idx] != -1) { // Valid data
            globalMap[x][y] = msg->mapData[idx];
          }
        }
        idx++;
      }
    }
    
    lastDataReceived = millis();
    newDataAvailable = true;
    
    Serial.printf("Received: Robot at (%.1f,%.1f) heading %.1f°\n", 
                  robotPosition.x, robotPosition.y, robotPosition.heading);
  } else {
    // Fall back to simple message
    memcpy(&incoming, incomingData, sizeof(incoming));
    Serial.printf("[ESP32] Got message: %s\n", incoming.text);
    Serial.flush();
  }
#endif
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

#if MAPPING_MODE
// Print map in ASCII format
void printMap() {
  Serial.println("\n=== GLOBAL MAP ===");
  Serial.printf("Map size: %dx%d cells, Cell size: %dcm\n", MAP_SIZE, MAP_SIZE, CELL_SIZE);
  Serial.printf("Robot position: (%.1f,%.1f) heading %.1f°\n", 
                robotPosition.x, robotPosition.y, robotPosition.heading);
  Serial.printf("Sensors - F:%.1f R:%.1f B:%.1f L:%.1f\n", 
                sensorDistances[0], sensorDistances[1], sensorDistances[2], sensorDistances[3]);
  
  // Find map boundaries with data
  int minX = MAP_SIZE, maxX = 0, minY = MAP_SIZE, maxY = 0;
  bool hasData = false;
  
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      if (globalMap[i][j] != -1) {
        minX = min(minX, i);
        maxX = max(maxX, i);
        minY = min(minY, j);
        maxY = max(maxY, j);
        hasData = true;
      }
    }
  }
  
  if (!hasData) {
    Serial.println("No map data available yet");
    return;
  }
  
  // Print map with boundaries
  int robotGridX, robotGridY;
  worldToGrid(robotPosition.x, robotPosition.y, &robotGridX, &robotGridY);
  
  // Expand view around robot
  minX = max(0, min(minX, robotGridX - 10));
  maxX = min(MAP_SIZE - 1, max(maxX, robotGridX + 10));
  minY = max(0, min(minY, robotGridY - 10));
  maxY = min(MAP_SIZE - 1, max(maxY, robotGridY + 10));
  
  Serial.printf("Showing area: X(%d-%d) Y(%d-%d)\n", minX, maxX, minY, maxY);
  
  // Print map from top to bottom
  for (int j = maxY; j >= minY; j--) {
    Serial.printf("%3d ", j);
    for (int i = minX; i <= maxX; i++) {
      if (i == robotGridX && j == robotGridY) {
        // Show robot with direction indicator
        char robotChar = 'R';
        if (robotPosition.heading >= -45 && robotPosition.heading < 45) robotChar = '>';
        else if (robotPosition.heading >= 45 && robotPosition.heading < 135) robotChar = '^';
        else if (robotPosition.heading >= 135 && robotPosition.heading < 225) robotChar = '<';
        else robotChar = 'v';
        Serial.printf("%c", robotChar);
      } else if (globalMap[i][j] == 100) {
        Serial.print("#"); // Obstacle
      } else if (globalMap[i][j] == 0) {
        Serial.print("."); // Free space
      } else {
        Serial.print(" "); // Unknown
      }
    }
    Serial.println();
  }
  Serial.println("===================\n");
}

// Calculate map statistics
void printMapStats() {
  int unknownCells = 0, freeCells = 0, occupiedCells = 0;
  
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      if (globalMap[i][j] == -1) unknownCells++;
      else if (globalMap[i][j] == 0) freeCells++;
      else if (globalMap[i][j] == 100) occupiedCells++;
    }
  }
  
  int totalCells = MAP_SIZE * MAP_SIZE;
  float explorationPercent = ((float)(freeCells + occupiedCells) / totalCells) * 100;
  
  Serial.println("=== MAP STATISTICS ===");
  Serial.printf("Total cells: %d\n", totalCells);
  Serial.printf("Unknown: %d (%.1f%%)\n", unknownCells, (float)unknownCells/totalCells*100);
  Serial.printf("Free: %d (%.1f%%)\n", freeCells, (float)freeCells/totalCells*100);
  Serial.printf("Occupied: %d (%.1f%%)\n", occupiedCells, (float)occupiedCells/totalCells*100);
  Serial.printf("Exploration: %.1f%%\n", explorationPercent);
  Serial.println("========================\n");
}
#endif // MAPPING_MODE

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000); // Give USB time to enumerate

#if SIMPLE_MODE
  Serial.println("[ESP32] Controller ready. Send w/s/a/d via Python GUI.");
  Serial.println("=== SIMPLE CONTROL MODE ===");
#elif MAPPING_MODE
  initGlobalMap();
  robotPosition.x = MAP_SIZE * CELL_SIZE / 2;
  robotPosition.y = MAP_SIZE * CELL_SIZE / 2;
  robotPosition.heading = 0;
  
  Serial.println("=== ROBOT MAPPING CONTROL STATION ===");
  Serial.println("Commands:");
  Serial.println("  w - Forward");
  Serial.println("  s - Backward");
  Serial.println("  a - Turn Left");
  Serial.println("  d - Turn Right");
  Serial.println("  x - Stop");
  Serial.println("  m - Print Map");
  Serial.println("  t - Map Statistics");
  Serial.println("=====================================\n");
#endif

  Serial.flush();
  esp_now_begin();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read serial from Python GUI or Serial Monitor
  while (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'w':
        send_command("forward");
        Serial.println("Sent: forward");
        break;
      case 's':
        send_command("backward");
        Serial.println("Sent: backward");
        break;
      case 'a':
        send_command("turn_left");
        Serial.println("Sent: turn_left");
        break;
      case 'd':
        send_command("turn_right");
        Serial.println("Sent: turn_right");
        break;
      case 'x':
        send_command("stop");
        Serial.println("Sent: stop");
        break;
#if MAPPING_MODE
      case 'm':
        printMap();
        break;
      case 't':
        printMapStats();
        break;
#endif
      default:
        send_command("stop");
        break;
    }
  }

#if MAPPING_MODE
  // Check for communication timeout
  if (currentTime - lastDataReceived > 5000 && lastDataReceived > 0) {
    Serial.println("WARNING: No data received from robot for 5 seconds!");
  }
  
  // Update display and print info periodically
  static unsigned long lastUpdate = 0;
  if (newDataAvailable && currentTime - lastUpdate > 1000) {
    lastUpdate = currentTime;
    newDataAvailable = false;
    
    // Print brief status
    Serial.printf("[STATUS] Robot: (%.1f,%.1f) %.1f° | Sensors: F%.0f R%.0f B%.0f L%.0f\n",
                  robotPosition.x, robotPosition.y, robotPosition.heading,
                  sensorDistances[0], sensorDistances[1], sensorDistances[2], sensorDistances[3]);
  }
#endif

  delay(10);
}