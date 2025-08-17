// robot_main.ino - ไฟล์หลักของหุ่นยนต์
#include <WiFi.h>
#include <esp_now.h>
#include <BMI160Gen.h>
#include <ESP32Servo.h>
#include "../config/robot_config.h"  // Fixed include path

// ==================== GLOBAL VARIABLES ====================
// Global variables from config
MapCell localMap[LOCAL_MAP_SIZE][LOCAL_MAP_SIZE];
Position robotPosition;
SensorReading currentSensors;
RobotState currentState = STATE_IDLE;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
bool motorsEnabled = false;
bool systemReady = false;
bool mappingMode = false;
bool obstacleDetected = false;
uint32_t lastHeartbeat = 0;
uint32_t totalDistance = 0;
uint32_t mappingStartTime = 0;
uint16_t obstaclesFound = 0;

// Sensor timing variables
volatile unsigned long echo_start[4] = {0, 0, 0, 0};
volatile unsigned long echo_end[4] = {0, 0, 0, 0};
volatile bool echo_done[4] = {false, false, false, false};

// Hardware objects
Servo servo_left;
Servo servo_right;

// Timing variables
unsigned long lastTriggerTime = 0;
unsigned long lastMotorUpdate = 0;
unsigned long lastMappingUpdate = 0;
unsigned long lastAutoMoveTime = 0;
unsigned long lastHeadingCorrection = 0;

// IMU calibration
float gyro_bias_z = 0.0;
float gyroIntegral = 0.0;
bool isMovingStraight = false;

// Obstacle avoidance
bool isAvoiding = false;
int avoidanceState = 0;
unsigned long avoidanceStartTime = 0;

// ESP-NOW variables
uint8_t stationAddress[] = {0x48, 0xe7, 0x29, 0xc9, 0x57, 0x28};
typedef struct struct_message {
  char text[200];
} struct_message;
struct_message incoming;
struct_message outgoing;

// ==================== SETUP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial);
  
  Serial.println("🤖 Advanced Robot SLAM System Starting...");
  Serial.println("====================================");
  
  // Initialize system
  systemInit();
  
  Serial.println("✅ Robot ready for autonomous mapping!");
  Serial.printf("Initial position: (%.1f, %.1f) heading %.1f°\n", 
                robotPosition.x, robotPosition.y, robotPosition.heading);
}

// ==================== MAIN LOOP ====================
void loop() {
  systemUpdate();
  delay(10);
}

// ==================== SYSTEM FUNCTIONS ====================
void systemInit() {
  // Initialize hardware pins
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_ERROR, OUTPUT);
  
  // Initialize sensors
  sensorsInit();
  
  // Initialize motors
  motorsInit();
  
  // Initialize mapping
  mappingInit();
  
  // Initialize serial communication
  serialInit();
  
  // Initialize IMU
  initializeIMU();
  
  // Initialize ESP-NOW
  esp_now_begin();
  
  // Set initial position at center
  setRobotPosition(MAP_CENTER_X * CELL_SIZE, MAP_CENTER_Y * CELL_SIZE, 0);
  
  currentState = STATE_IDLE;
  systemReady = true;
  
  DEBUG_PRINTLN("System initialization complete");
}

void systemUpdate() {
  uint32_t now = millis();
  
  // Update sensors
  if (now - lastTriggerTime >= UPDATE_INTERVAL_SENSORS) {
    lastTriggerTime = now;
    sensorsUpdate();
    updateIMUData();
  }
  
  // Update motors
  if (now - lastMotorUpdate >= UPDATE_INTERVAL_MOTORS) {
    lastMotorUpdate = now;
    motorsUpdate();
  }
  
  // Update mapping
  if (now - lastMappingUpdate >= UPDATE_INTERVAL_MAPPING) {
    lastMappingUpdate = now;
    mappingUpdate();
  }
  
  // Update serial communication
  serialUpdate();
  
  // Check for emergency conditions
  if (isObstacleDetected()) {
    if (currentState != STATE_ERROR) {
      DEBUG_PRINTLN("Emergency stop - obstacle too close!");
      emergencyStop();
    }
  }
  
  // Handle button press
  handleButtonPress();
  
  // Send heartbeat
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = now;
    sendStatusMessage();
    blinkLED(LED_STATUS);
  }
}

void emergencyStop() {
  stopMotors();
  currentState = STATE_ERROR;
  obstacleDetected = true;
  blinkLED(LED_ERROR, 3);
}

bool isSystemReady() {
  return systemReady;
}

// ==================== SENSOR FUNCTIONS ====================
void sensorsInit() {
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  
  // Set all triggers low
  digitalWrite(TRIG_FRONT, LOW);
  digitalWrite(TRIG_RIGHT, LOW);
  digitalWrite(TRIG_BACK, LOW);
  digitalWrite(TRIG_LEFT, LOW);
  
  DEBUG_PRINTLN("Sensors initialized");
}

void sensorsUpdate() {
  static int currentSensor = 0;
  static unsigned long sensorStartTime = 0;
  
  uint32_t now = millis();
  
  // Trigger sensors sequentially to avoid interference
  if (now - sensorStartTime >= SENSOR_READ_DELAY) {
    // Read previous sensor
    if (currentSensor > 0) {
      int prevSensor = currentSensor - 1;
      currentSensors.distance[prevSensor] = readUltrasonicSensor(prevSensor);
      currentSensors.valid[prevSensor] = (currentSensors.distance[prevSensor] > MIN_DISTANCE && 
                                          currentSensors.distance[prevSensor] < MAX_DISTANCE);
    }
    
    // Trigger next sensor
    triggerUltrasonicSensor(currentSensor);
    
    currentSensor = (currentSensor + 1) % 4;
    sensorStartTime = now;
  }
  
  currentSensors.timestamp = now;
}

float readUltrasonicSensor(int sensorIndex) {
  int trigPin, echoPin;
  
  switch(sensorIndex) {
    case 0: trigPin = TRIG_FRONT; echoPin = ECHO_FRONT; break;
    case 1: trigPin = TRIG_RIGHT; echoPin = ECHO_RIGHT; break;
    case 2: trigPin = TRIG_BACK; echoPin = ECHO_BACK; break;
    case 3: trigPin = TRIG_LEFT; echoPin = ECHO_LEFT; break;
    default: return MAX_DISTANCE;
  }
  
  return readUltrasonic(trigPin, echoPin);
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);
  
  if (duration == 0) {
    return MAX_DISTANCE; // Timeout
  }
  
  float distance = (duration * 0.034) / 2.0; // Convert to cm
  
  return CLAMP(distance, MIN_DISTANCE, MAX_DISTANCE);
}

void triggerUltrasonicSensor(int sensorIndex) {
  int trigPin;
  
  switch(sensorIndex) {
    case 0: trigPin = TRIG_FRONT; break;
    case 1: trigPin = TRIG_RIGHT; break;
    case 2: trigPin = TRIG_BACK; break;
    case 3: trigPin = TRIG_LEFT; break;
    default: return;
  }
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

bool isObstacleDetected() {
  return (currentSensors.valid[0] && currentSensors.distance[0] < EMERGENCY_STOP_DISTANCE);
}

// ==================== MOTOR FUNCTIONS ====================
void motorsInit() {
  servo_left.attach(MOTOR_LEFT_PWM);
  servo_right.attach(MOTOR_RIGHT_PWM);
  
  // Set motors to neutral
  servo_left.writeMicroseconds(1500);
  servo_right.writeMicroseconds(1500);
  
  motorsEnabled = true;
  DEBUG_PRINTLN("Motors initialized");
}

void motorsUpdate() {
  if (!motorsEnabled) return;
  
  // Apply motor speeds with ramping
  static int current_left = 1500;
  static int current_right = 1500;
  
  int target_left = MAP_RANGE(leftMotorSpeed, -100, 100, 1000, 2000);
  int target_right = MAP_RANGE(rightMotorSpeed, -100, 100, 1000, 2000);
  
  // Ramp speeds gradually
  if (current_left < target_left) {
    current_left = min(current_left + 10, target_left);
  } else if (current_left > target_left) {
    current_left = max(current_left - 10, target_left);
  }
  
  if (current_right < target_right) {
    current_right = min(current_right + 10, target_right);
  } else if (current_right > target_right) {
    current_right = max(current_right - 10, target_right);
  }
  
  servo_left.writeMicroseconds(current_left);
  servo_right.writeMicroseconds(current_right);
}

void setMotorSpeeds(int left, int right) {
  if (!motorsEnabled) return;
  
  leftMotorSpeed = CLAMP(left, -100, 100);
  rightMotorSpeed = CLAMP(right, -100, 100);
}

void stopMotors() {
  setMotorSpeeds(0, 0);
  isMovingStraight = false;
}

void moveForward(int speed) {
  setMotorSpeeds(speed, speed);
  isMovingStraight = true;
}

void moveBackward(int speed) {
  setMotorSpeeds(-speed, -speed);
}

void turnLeft(int speed) {
  setMotorSpeeds(-speed, speed);
}

void turnRight(int speed) {
  setMotorSpeeds(speed, -speed);
}

// ==================== MAPPING FUNCTIONS ====================
void mappingInit() {
  clearMap();
  mappingStartTime = millis();
  DEBUG_PRINTLN("Mapping system initialized");
}

void mappingUpdate() {
  if (!mappingMode) return;
  
  // Update robot position based on movement
  updateRobotPosition();
  
  // Update map with current sensor readings
  updateMapFromSensors();
  
  // Perform autonomous navigation
  navigationUpdate();
}

void clearMap() {
  for (int i = 0; i < LOCAL_MAP_SIZE; i++) {
    for (int j = 0; j < LOCAL_MAP_SIZE; j++) {
      localMap[i][j].value = CELL_UNKNOWN;
      localMap[i][j].confidence = 0;
      localMap[i][j].lastUpdate = 0;
    }
  }
  
  obstaclesFound = 0;
  DEBUG_PRINTLN("Map cleared");
}

void updateMapCell(int x, int y, int8_t value, uint8_t confidence) {
  if (!IS_VALID_GRID(x, y)) return;
  
  localMap[x][y].value = value;
  localMap[x][y].confidence = min(255, localMap[x][y].confidence + confidence);
  localMap[x][y].lastUpdate = millis();
  
  if (value == CELL_OCCUPIED) {
    obstaclesFound++;
  }
}

void updateMapFromSensors() {
  int robot_grid_x = WORLD_TO_GRID_X(robotPosition.x);
  int robot_grid_y = WORLD_TO_GRID_Y(robotPosition.y);
  
  // Mark robot position as free
  updateMapCell(robot_grid_x, robot_grid_y, CELL_FREE, CONFIDENCE_STEP);
  
  // Process each sensor
  for (int i = 0; i < 4; i++) {
    if (!currentSensors.valid[i]) continue;
    
    float sensor_angle = robotPosition.heading;
    switch(i) {
      case 0: break; // Front
      case 1: sensor_angle += 90; break; // Right
      case 2: sensor_angle += 180; break; // Back
      case 3: sensor_angle += 270; break; // Left
    }
    
    float angle_rad = DEGREES_TO_RADIANS(sensor_angle);
    float distance = currentSensors.distance[i];
    
    // Mark free space along the ray
    for (int d = CELL_SIZE; d < distance; d += CELL_SIZE) {
      float x = robotPosition.x + d * cos(angle_rad);
      float y = robotPosition.y + d * sin(angle_rad);
      
      int grid_x = WORLD_TO_GRID_X(x);
      int grid_y = WORLD_TO_GRID_Y(y);
      
      updateMapCell(grid_x, grid_y, CELL_FREE, CONFIDENCE_STEP);
    }
    
    // Mark obstacle at the end if within range
    if (distance < MAX_DISTANCE - 10) {
      float obs_x = robotPosition.x + distance * cos(angle_rad);
      float obs_y = robotPosition.y + distance * sin(angle_rad);
      
      int obs_grid_x = WORLD_TO_GRID_X(obs_x);
      int obs_grid_y = WORLD_TO_GRID_Y(obs_y);
      
      updateMapCell(obs_grid_x, obs_grid_y, CELL_OCCUPIED, CONFIDENCE_STEP * 2);
    }
  }
}

void markObstacle(float world_x, float world_y) {
  int grid_x = WORLD_TO_GRID_X(world_x);
  int grid_y = WORLD_TO_GRID_Y(world_y);
  updateMapCell(grid_x, grid_y, CELL_OCCUPIED, CONFIDENCE_STEP * 2);
}

void markFreeSpace(float world_x, float world_y) {
  int grid_x = WORLD_TO_GRID_X(world_x);
  int grid_y = WORLD_TO_GRID_Y(world_y);
  updateMapCell(grid_x, grid_y, CELL_FREE, CONFIDENCE_STEP);
}

// ==================== NAVIGATION FUNCTIONS ====================
void navigationInit() {
  DEBUG_PRINTLN("Navigation system initialized");
}

void navigationUpdate() {
  if (currentState != STATE_MAPPING) return;
  
  // Simple obstacle avoidance and exploration
  if (isObstacleDetected()) {
    performObstacleAvoidance();
  } else {
    performExploration();
  }
}

void performObstacleAvoidance() {
  if (!isAvoiding) {
    isAvoiding = true;
    avoidanceState = 0;
    avoidanceStartTime = millis();
    DEBUG_PRINTLN("Starting obstacle avoidance");
  }
  
  uint32_t elapsed = millis() - avoidanceStartTime;
  
  switch (avoidanceState) {
    case 0: // Stop
      stopMotors();
      if (elapsed > 200) {
        avoidanceState = 1;
        avoidanceStartTime = millis();
      }
      break;
      
    case 1: // Decide turn direction
      if (currentSensors.distance[1] > currentSensors.distance[3]) {
        turnRight(TURN_SPEED);
      } else {
        turnLeft(TURN_SPEED);
      }
      
      if (elapsed > 800) {
        avoidanceState = 2;
        avoidanceStartTime = millis();
      }
      break;
      
    case 2: // Move forward if clear
      if (currentSensors.distance[0] > SAFE_DISTANCE) {
        moveForward(BASE_SPEED);
        if (elapsed > 1000) {
          isAvoiding = false;
          avoidanceState = 0;
          DEBUG_PRINTLN("Obstacle avoidance complete");
        }
      } else {
        // Still blocked, turn more
        avoidanceState = 1;
        avoidanceStartTime = millis();
      }
      break;
  }
}

void performExploration() {
  // Simple wall-following exploration
  if (currentSensors.valid[1] && currentSensors.distance[1] < 40) {
    // Wall on right - follow it
    if (currentSensors.distance[1] < 20) {
      turnLeft(BASE_SPEED / 2);
    } else if (currentSensors.distance[1] > 35) {
      turnRight(BASE_SPEED / 2);
    } else {
      moveForward(BASE_SPEED);
    }
  } else {
    // No wall - explore forward
    moveForward(BASE_SPEED);
  }
}

void setRobotPosition(float x, float y, float heading) {
  robotPosition.x = x;
  robotPosition.y = y;
  robotPosition.heading = normalizeAngle(heading);
  robotPosition.timestamp = millis();
}

void updateRobotPosition() {
  // Simple odometry based on motor commands
  static uint32_t lastUpdateTime = 0;
  uint32_t now = millis();
  
  if (lastUpdateTime == 0) {
    lastUpdateTime = now;
    return;
  }
  
  float dt = (now - lastUpdateTime) / 1000.0;
  lastUpdateTime = now;
  
  // Estimate movement based on motor speeds
  float avgSpeed = (leftMotorSpeed + rightMotorSpeed) / 2.0;
  float speedCmPerSec = MAP_RANGE(abs(avgSpeed), 0, 100, 0, 20);
  
  if (abs(avgSpeed) > 10) {
    float distance = speedCmPerSec * dt;
    float radians = DEGREES_TO_RADIANS(robotPosition.heading);
    
    robotPosition.x += distance * cos(radians) * (avgSpeed > 0 ? 1 : -1);
    robotPosition.y += distance * sin(radians) * (avgSpeed > 0 ? 1 : -1);
    
    totalDistance += distance;
  }
  
  robotPosition.timestamp = now;
}

bool planPath(float target_x, float target_y) {
  // Simple path planning - just move toward target
  float dx = target_x - robotPosition.x;
  float dy = target_y - robotPosition.y;
  float target_heading = RADIANS_TO_DEGREES(atan2(dy, dx));
  
  float heading_error = target_heading - robotPosition.heading;
  heading_error = normalizeAngle(heading_error);
  
  if (abs(heading_error) > 10) {
    if (heading_error > 0) {
      turnLeft(TURN_SPEED);
    } else {
      turnRight(TURN_SPEED);
    }
    return false;
  } else {
    moveForward(BASE_SPEED);
    return true;
  }
}

void executeMovement() {
  // This function is called by the navigation system
  // Motor control is handled in motorsUpdate()
}

// ==================== COMMUNICATION FUNCTIONS ====================
void serialInit() {
  DEBUG_PRINTLN("Serial communication initialized");
}

void serialUpdate() {
  // Process incoming commands
  if (Serial.available()) {
    char cmd = Serial.read();
    processCommand(cmd);
  }
}

void sendStatusMessage() {
  char statusMsg[250];
  
  snprintf(statusMsg, sizeof(statusMsg), 
           "[STATUS] Robot: (%.1f,%.1f) %.1f° | Sensors: F%.0f R%.0f B%.0f L%.0f | Mode: %s",
           robotPosition.x, robotPosition.y, robotPosition.heading,
           currentSensors.valid[0] ? currentSensors.distance[0] : 0,
           currentSensors.valid[1] ? currentSensors.distance[1] : 0,
           currentSensors.valid[2] ? currentSensors.distance[2] : 0,
           currentSensors.valid[3] ? currentSensors.distance[3] : 0,
           getStateName(currentState));
  
  Serial.println(statusMsg);
  
  if (DEBUG_SERIAL) {
    DEBUG_PRINTF("Status: (%.1f,%.1f) %.1f°\n", 
                 robotPosition.x, robotPosition.y, robotPosition.heading);
  }
}

void sendTelemetryMessage() {
  char telemMsg[200];
  float voltage = getBatteryVoltage();
  uint32_t uptime = getUptime();
  
  snprintf(telemMsg, sizeof(telemMsg),
           "[TELEMETRY] Bat:%.1fV Uptime:%lds Obstacles:%d Distance:%.0fcm",
           voltage, uptime/1000, obstaclesFound, totalDistance);
  
  Serial.println(telemMsg);
}

void sendMapData() {
  // Send compressed map data
  Serial.println("[MAP] Sending map data...");
  
  for (int y = 0; y < LOCAL_MAP_SIZE; y++) {
    for (int x = 0; x < LOCAL_MAP_SIZE; x++) {
      if (localMap[x][y].confidence > 50) {
        Serial.printf("MAP:%d,%d,%d\n", x, y, localMap[x][y].value);
      }
    }
  }
}

void processCommand(char cmd) {
  DEBUG_PRINTF("Received command: %c\n", cmd);
  
  switch(cmd) {
    case CMD_CHAR_FORWARD:
      if (currentState == STATE_MANUAL) moveForward(BASE_SPEED);
      break;
      
    case CMD_CHAR_BACKWARD:
      if (currentState == STATE_MANUAL) moveBackward(BASE_SPEED);
      break;
      
    case CMD_CHAR_LEFT:
      if (currentState == STATE_MANUAL) turnLeft(TURN_SPEED);
      break;
      
    case CMD_CHAR_RIGHT:
      if (currentState == STATE_MANUAL) turnRight(TURN_SPEED);
      break;
      
    case CMD_CHAR_STOP:
      stopMotors();
      break;
      
    case CMD_CHAR_MAPPING:
      startMapping();
      break;
      
    case CMD_CHAR_AUTO:
      currentState = STATE_AUTO;
      DEBUG_PRINTLN("Auto mode activated");
      break;
      
    case CMD_CHAR_MANUAL:
      currentState = STATE_MANUAL;
      stopMotors();
      DEBUG_PRINTLN("Manual mode activated");
      break;
      
    case CMD_CHAR_RESET:
      ESP.restart();
      break;
      
    default:
      DEBUG_PRINTF("Unknown command: %c\n", cmd);
  }
}

// ==================== IMU FUNCTIONS ====================
void initializeIMU() {
  BMI160.begin(BMI160GenClass::I2C_MODE, 0x69);
  uint8_t dev_id = BMI160.getDeviceID();
  DEBUG_PRINTF("IMU Device ID: 0x%02X\n", dev_id);
  
  BMI160.setGyroRange(250);
  BMI160.setAccelerometerRange(2);
  
  // Calibrate gyro
  DEBUG_PRINTLN("Calibrating IMU...");
  delay(1000);
  calibrateGyro();
}

void calibrateGyro() {
  float sum_z = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    int gx, gy, gz;
    BMI160.readGyro(gx, gy, gz);
    sum_z += gz;
    delay(10);
  }
  
  gyro_bias_z = sum_z / samples;
  DEBUG_PRINTF("Gyro Z bias: %.2f\n", gyro_bias_z);
}

void updateIMUData() {
  static uint32_t lastIMUTime = 0;
  
  int gx, gy, gz;
  BMI160.readGyro(gx, gy, gz);
  
  uint32_t currentTime = millis();
  if (lastIMUTime > 0) {
    float deltaTime = (currentTime - lastIMUTime) / 1000.0;
    
    // Remove bias and convert to degrees per second
    float gyro_z_dps = (gz - gyro_bias_z) / 131.0;
    
    // Integrate to get heading change
    robotPosition.heading += gyro_z_dps * deltaTime;
    robotPosition.heading = normalizeAngle(robotPosition.heading);
    
    // Accumulate for straight line correction
    if (isMovingStraight) {
      gyroIntegral += gyro_z_dps * deltaTime;
    }
  }
  lastIMUTime = currentTime;
}

// ==================== UTILITY FUNCTIONS ====================
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

float calculateDistance(float x1, float y1, float x2, float y2) {
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

uint32_t getUptime() {
  return millis();
}

float getBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  float voltage = (adcValue / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

void blinkLED(int pin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(100);
    digitalWrite(pin, LOW);
    delay(100);
  }
}

const char* getStateName(RobotState state) {
  switch(state) {
    case STATE_IDLE: return "IDLE";
    case STATE_MANUAL: return "MANUAL";
    case STATE_AUTO: return "AUTO";
    case STATE_MAPPING: return "MAPPING";
    case STATE_RETURNING: return "RETURNING";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void handleButtonPress() {
  static bool lastButtonState = HIGH;
  static uint32_t buttonPressTime = 0;
  
  bool currentButtonState = digitalRead(BTN_START);
  
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    buttonPressTime = millis();
  } else if (currentButtonState == HIGH && lastButtonState == LOW) {
    uint32_t pressDuration = millis() - buttonPressTime;
    
    if (pressDuration > 50 && pressDuration < 1000) {
      // Short press - toggle mapping
      if (currentState == STATE_MAPPING) {
        stopMapping();
      } else {
        startMapping();
      }
    } else if (pressDuration >= 1000) {
      // Long press - emergency stop
      emergencyStop();
      DEBUG_PRINTLN("Emergency stop activated!");
    }
  }
  
  lastButtonState = currentButtonState;
}

void startMapping() {
  mappingMode = true;
  currentState = STATE_MAPPING;
  mappingStartTime = millis();
  DEBUG_PRINTLN("🗺️ Mapping started");
  
  // Beep to confirm
  tone(LED_STATUS, 1000, 200);
}

void stopMapping() {
  mappingMode = false;
  currentState = STATE_IDLE;
  stopMotors();
  DEBUG_PRINTLN("⏹️ Mapping stopped");
  
  // Double beep to confirm
  tone(LED_STATUS, 1000, 100);
  delay(150);
  tone(LED_STATUS, 1000, 100);
}

// ==================== ESP-NOW FUNCTIONS ====================
void esp_now_begin() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    DEBUG_PRINTLN("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, stationAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    DEBUG_PRINTLN("Failed to add peer");
    return;
  }
  
  DEBUG_PRINTLN("ESP-NOW initialized");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  DEBUG_PRINTF("ESP-NOW Send Status: %s\n", (status == ESP_NOW_SEND_SUCCESS) ? "Success" : "Fail");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  DEBUG_PRINTF("ESP-NOW Received: %s\n", incoming.text);
  
  // Process received commands
  processReceivedCommand(incoming.text);
}

void send_message(const char* message) {
  strcpy(outgoing.text, message);
  esp_err_t result = esp_now_send(stationAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  
  if (result != ESP_OK) {
    DEBUG_PRINTF("ESP-NOW Send Error: %d\n", result);
  }
}

void processReceivedCommand(const char* command) {
  DEBUG_PRINTF("Processing command: %s\n", command);
  
  if (strcmp(command, "start_mapping") == 0) {
    startMapping();
  } else if (strcmp(command, "stop_mapping") == 0) {
    stopMapping();
  } else if (strcmp(command, "forward") == 0) {
    if (currentState == STATE_MANUAL) {
      moveForward(BASE_SPEED);
      updateRobotPosition();
    }
  } else if (strcmp(command, "backward") == 0) {
    if (currentState == STATE_MANUAL) {
      moveBackward(BASE_SPEED);
      updateRobotPosition();
    }
  } else if (strcmp(command, "turn_left") == 0) {
    if (currentState == STATE_MANUAL) {
      turnLeft(TURN_SPEED);
      robotPosition.heading = normalizeAngle(robotPosition.heading - 15);
    }
  } else if (strcmp(command, "turn_right") == 0) {
    if (currentState == STATE_MANUAL) {
      turnRight(TURN_SPEED);
      robotPosition.heading = normalizeAngle(robotPosition.heading + 15);
    }
  } else if (strcmp(command, "stop") == 0) {
    stopMotors();
  } else if (strcmp(command, "get_status") == 0) {
    sendStatusMessage();
    sendTelemetryMessage();
  } else if (strcmp(command, "get_map") == 0) {
    sendMapData();
  } else if (strcmp(command, "reset") == 0) {
    ESP.restart();
  } else if (strcmp(command, "emergency_stop") == 0) {
    emergencyStop();
  } else {
    DEBUG_PRINTF("Unknown ESP-NOW command: %s\n", command);
  }
}

// ==================== ADVANCED MAPPING FUNCTIONS ====================
void performAdvancedMapping() {
  static uint32_t lastExplorationUpdate = 0;
  uint32_t now = millis();
  
  if (now - lastExplorationUpdate < 200) return;
  lastExplorationUpdate = now;
  
  // Advanced exploration strategy
  if (isAvoiding) {
    performObstacleAvoidance();
  } else {
    // Choose exploration strategy based on environment
    if (isInCorner()) {
      performCornerExploration();
    } else if (hasWallToFollow()) {
      performWallFollowing();
    } else {
      performFrontierExploration();
    }
  }
}

bool isInCorner() {
  int wallCount = 0;
  for (int i = 0; i < 4; i++) {
    if (currentSensors.valid[i] && currentSensors.distance[i] < TURN_DISTANCE) {
      wallCount++;
    }
  }
  return wallCount >= 2;
}

bool hasWallToFollow() {
  return (currentSensors.valid[1] && currentSensors.distance[1] < 60) ||
         (currentSensors.valid[3] && currentSensors.distance[3] < 60);
}

void performCornerExploration() {
  DEBUG_PRINTLN("Corner exploration mode");
  
  // Turn to find the most open direction
  float maxDistance = 0;
  int bestDirection = 0;
  
  for (int i = 0; i < 4; i++) {
    if (currentSensors.valid[i] && currentSensors.distance[i] > maxDistance) {
      maxDistance = currentSensors.distance[i];
      bestDirection = i;
    }
  }
  
  // Turn toward the most open direction
  switch (bestDirection) {
    case 0: moveForward(BASE_SPEED); break;
    case 1: turnRight(TURN_SPEED); break;
    case 2: 
      // Turn around
      turnRight(TURN_SPEED);
      delay(1600); // Approximate 180-degree turn
      break;
    case 3: turnLeft(TURN_SPEED); break;
  }
}

void performWallFollowing() {
  DEBUG_PRINTLN("Wall following mode");
  
  float rightDist = currentSensors.valid[1] ? currentSensors.distance[1] : MAX_DISTANCE;
  float leftDist = currentSensors.valid[3] ? currentSensors.distance[3] : MAX_DISTANCE;
  
  // Prefer right wall following
  if (rightDist < 80) {
    followRightWall(rightDist);
  } else if (leftDist < 80) {
    followLeftWall(leftDist);
  } else {
    moveForward(BASE_SPEED);
  }
}

void followRightWall(float wallDistance) {
  const float TARGET_WALL_DISTANCE = 25.0;
  const float TOLERANCE = 5.0;
  
  if (wallDistance < TARGET_WALL_DISTANCE - TOLERANCE) {
    // Too close to wall, turn left slightly
    turnLeft(BASE_SPEED / 3);
    delay(100);
    moveForward(BASE_SPEED);
  } else if (wallDistance > TARGET_WALL_DISTANCE + TOLERANCE) {
    // Too far from wall, turn right slightly
    turnRight(BASE_SPEED / 3);
    delay(100);
    moveForward(BASE_SPEED);
  } else {
    // Good distance, move forward
    moveForward(BASE_SPEED);
  }
}

void followLeftWall(float wallDistance) {
  const float TARGET_WALL_DISTANCE = 25.0;
  const float TOLERANCE = 5.0;
  
  if (wallDistance < TARGET_WALL_DISTANCE - TOLERANCE) {
    // Too close to wall, turn right slightly
    turnRight(BASE_SPEED / 3);
    delay(100);
    moveForward(BASE_SPEED);
  } else if (wallDistance > TARGET_WALL_DISTANCE + TOLERANCE) {
    // Too far from wall, turn left slightly
    turnLeft(BASE_SPEED / 3);
    delay(100);
    moveForward(BASE_SPEED);
  } else {
    // Good distance, move forward
    moveForward(BASE_SPEED);
  }
}

void performFrontierExploration() {
  DEBUG_PRINTLN("Frontier exploration mode");
  
  // Simple frontier-based exploration
  // Move toward areas with the least explored cells
  
  static int explorationDirection = 0;
  static uint32_t lastDirectionChange = 0;
  uint32_t now = millis();
  
  // Change direction every few seconds if no obstacles
  if (now - lastDirectionChange > 3000) {
    explorationDirection = (explorationDirection + 1) % 4;
    lastDirectionChange = now;
    
    switch (explorationDirection) {
      case 0: DEBUG_PRINTLN("Exploring North"); break;
      case 1: DEBUG_PRINTLN("Exploring East"); break;
      case 2: DEBUG_PRINTLN("Exploring South"); break;
      case 3: DEBUG_PRINTLN("Exploring West"); break;
    }
  }
  
  // Move in the chosen direction if path is clear
  bool pathClear = currentSensors.valid[0] && currentSensors.distance[0] > SAFE_DISTANCE;
  
  if (pathClear) {
    // Adjust heading toward exploration direction
    float targetHeading = explorationDirection * 90.0;
    float headingError = targetHeading - robotPosition.heading;
    headingError = normalizeAngle(headingError);
    
    if (abs(headingError) > 15) {
      if (headingError > 0) {
        turnRight(TURN_SPEED);
      } else {
        turnLeft(TURN_SPEED);
      }
    } else {
      moveForward(BASE_SPEED);
    }
  } else {
    // Path blocked, choose new direction
    lastDirectionChange = 0;
  }
}

// ==================== ENHANCED SENSOR PROCESSING ====================
void processSensorReadingsAdvanced() {
  static float sensorHistory[4][5] = {{0}}; // Keep last 5 readings
  static int historyIndex = 0;
  
  // Store current readings in history
  for (int i = 0; i < 4; i++) {
    sensorHistory[i][historyIndex] = currentSensors.distance[i];
  }
  
  historyIndex = (historyIndex + 1) % 5;
  
  // Apply median filter to reduce noise
  for (int i = 0; i < 4; i++) {
    float sortedValues[5];
    memcpy(sortedValues, sensorHistory[i], sizeof(sortedValues));
    
    // Simple bubble sort
    for (int j = 0; j < 4; j++) {
      for (int k = j + 1; k < 5; k++) {
        if (sortedValues[j] > sortedValues[k]) {
          float temp = sortedValues[j];
          sortedValues[j] = sortedValues[k];
          sortedValues[k] = temp;
        }
      }
    }
    
    // Use median value
    currentSensors.distance[i] = sortedValues[2];
    currentSensors.valid[i] = (currentSensors.distance[i] > MIN_DISTANCE && 
                               currentSensors.distance[i] < MAX_DISTANCE);
  }
}

// ==================== SYSTEM DIAGNOSTICS ====================
void performSystemDiagnostics() {
  static uint32_t lastDiagnostic = 0;
  uint32_t now = millis();
  
  if (now - lastDiagnostic < 10000) return; // Every 10 seconds
  lastDiagnostic = now;
  
  DEBUG_PRINTLN("=== System Diagnostics ===");
  
  // Check battery voltage
  float voltage = getBatteryVoltage();
  DEBUG_PRINTF("Battery: %.1fV ", voltage);
  if (voltage < BATTERY_CRITICAL) {
    DEBUG_PRINTLN("(CRITICAL!)");
    emergencyStop();
  } else if (voltage < BATTERY_MIN_VOLTAGE) {
    DEBUG_PRINTLN("(LOW)");
  } else {
    DEBUG_PRINTLN("(OK)");
  }
  
  // Check sensor status
  DEBUG_PRINT("Sensors: ");
  for (int i = 0; i < 4; i++) {
    DEBUG_PRINTF("%c:%.0f ", "FRBL"[i], 
                 currentSensors.valid[i] ? currentSensors.distance[i] : -1);
  }
  DEBUG_PRINTLN("");
  
  // Check motor status
  DEBUG_PRINTF("Motors: L:%d R:%d %s\n", 
               leftMotorSpeed, rightMotorSpeed, 
               motorsEnabled ? "ON" : "OFF");
  
  // Check map coverage
  int knownCells = 0;
  for (int i = 0; i < LOCAL_MAP_SIZE; i++) {
    for (int j = 0; j < LOCAL_MAP_SIZE; j++) {
      if (localMap[i][j].confidence > 0) {
        knownCells++;
      }
    }
  }
  
  float coverage = (float)knownCells / (LOCAL_MAP_SIZE * LOCAL_MAP_SIZE) * 100;
  DEBUG_PRINTF("Map coverage: %.1f%% (%d/%d cells)\n", 
               coverage, knownCells, LOCAL_MAP_SIZE * LOCAL_MAP_SIZE);
  
  // System uptime and statistics
  uint32_t uptime = getUptime();
  DEBUG_PRINTF("Uptime: %ld.%lds\n", uptime/1000, (uptime%1000)/100);
  DEBUG_PRINTF("Total distance: %.0fcm\n", totalDistance);
  DEBUG_PRINTF("Obstacles found: %d\n", obstaclesFound);
  
  DEBUG_PRINTLN("=== End Diagnostics ===");
}

// ==================== EMERGENCY RECOVERY ====================
void performEmergencyRecovery() {
  DEBUG_PRINTLN("=== Emergency Recovery Mode ===");
  
  // Step 1: Stop all movement
  stopMotors();
  delay(500);
  
  // Step 2: Check sensors
  sensorsUpdate();
  delay(100);
  
  // Step 3: Try to back away from obstacle
  if (currentSensors.valid[2]) { // Check if back is clear
    DEBUG_PRINTLN("Backing away from obstacle");
    moveBackward(BASE_SPEED / 2);
    delay(1000);
    stopMotors();
  }
  
  // Step 4: Try to turn to find clear path
  float bestDistance = 0;
  int bestDirection = -1;
  
  for (int dir = 0; dir < 4; dir++) {
    // Turn 90 degrees
    turnRight(TURN_SPEED);
    delay(800); // Approximate 90-degree turn
    
    // Check distance
    sensorsUpdate();
    delay(200);
    
    if (currentSensors.valid[0] && currentSensors.distance[0] > bestDistance) {
      bestDistance = currentSensors.distance[0];
      bestDirection = dir;
    }
  }
  
  // Turn to best direction
  if (bestDirection >= 0 && bestDistance > SAFE_DISTANCE) {
    DEBUG_PRINTF("Found clear path in direction %d, distance %.0f\n", 
                 bestDirection, bestDistance);
    
    // Turn to that direction (we're already facing it from the loop above)
    currentState = STATE_MAPPING; // Resume mapping
    obstacleDetected = false;
  } else {
    DEBUG_PRINTLN("No clear path found - staying in error state");
    blinkLED(LED_ERROR, 5);
  }
  
  DEBUG_PRINTLN("=== Recovery Complete ===");
}

// ==================== MAIN STATE MACHINE ====================
void executeStateMachine() {
  static uint32_t lastStateUpdate = 0;
  uint32_t now = millis();
  
  if (now - lastStateUpdate < 100) return; // Update every 100ms
  lastStateUpdate = now;
  
  switch (currentState) {
    case STATE_IDLE:
      // Just monitor sensors and wait for commands
      break;
      
    case STATE_MANUAL:
      // Manual control - commands processed in serialUpdate()
      break;
      
    case STATE_AUTO:
      // Simple autonomous mode without mapping
      if (isObstacleDetected()) {
        performObstacleAvoidance();
      } else {
        moveForward(BASE_SPEED);
      }
      break;
      
    case STATE_MAPPING:
      // Advanced mapping mode
      performAdvancedMapping();
      break;
      
    case STATE_RETURNING:
      // Return to start position (future implementation)
      // planPath(MAP_CENTER_X * CELL_SIZE, MAP_CENTER_Y * CELL_SIZE);
      break;
      
    case STATE_ERROR:
      // Try to recover from error
      performEmergencyRecovery();
      break;
      
    default:
      currentState = STATE_IDLE;
      DEBUG_PRINTLN("Unknown state - resetting to IDLE");
  }
}

// Update the main loop to use the state machine
// Replace the navigation section in systemUpdate() with:
// executeStateMachine();

// ==================== FINAL ADDITIONS ====================
// Add these functions to complete the implementation

void tone(int pin, int frequency, int duration) {
  // Simple tone generation for ESP32
  // Note: This is a simplified version
  // In a real implementation, you might want to use the ESP32's built-in tone library
  
  pinMode(pin, OUTPUT);
  
  int period = 1000000 / frequency; // Period in microseconds
  int cycles = (duration * 1000) / period;
  
  for (int i = 0; i < cycles; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(pin, LOW);
    delayMicroseconds(period / 2);
  }
}