#include <WiFi.h>
#include <esp_now.h>
#include <EEPROM.h>
#include "config.h"

// ==================== GLOBAL VARIABLES ====================
MapCell globalMap[MAP_SIZE][MAP_SIZE];
Position robotPos = {0, 0, 0, 0};
SensorData sensors;
Mission currentMission;
SystemState systemState = STATE_IDLE;
MovementState moveState = MOVE_STOP;

// Sensor interrupt variables
volatile unsigned long echo_start[NUM_SENSORS] = {0, 0, 0, 0};
volatile unsigned long echo_end[NUM_SENSORS] = {0, 0, 0, 0};
volatile bool echo_done[NUM_SENSORS] = {false, false, false, false};

// Timing variables
unsigned long lastSensorUpdate = 0;
unsigned long lastMapUpdate = 0;
unsigned long lastStatusSend = 0;
unsigned long lastNavigationUpdate = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial);
  
  DEBUG_PRINTLN("🤖 Emergency Robot Mapping System Starting...");
  DEBUG_PRINTLN("==========================================");
  
  // Initialize hardware
  initializePins();
  initializeSensors();
  initializeMotors();
  
  // Initialize systems
  initializeMap();
  initializeEEPROM();
  initializeESPNow();
  
  // Set initial position at map center
  robotPos.x = 0;
  robotPos.y = 0;
  robotPos.heading = 0;
  robotPos.timestamp = millis();
  
  systemState = STATE_IDLE;
  
  DEBUG_PRINTLN("✅ Robot ready!");
  DEBUG_PRINTLN("Commands: M=Mapping, S=Stop, R=Reset");
  
  // Startup beep
  tone(BUZZER_PIN, 1000, 200);
  delay(300);
  tone(BUZZER_PIN, 1500, 200);
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long now = millis();
  
  // Update sensors (every 100ms)
  if (now - lastSensorUpdate >= 100) {
    lastSensorUpdate = now;
    updateSensors();
  }
  
  // Update mapping (every 200ms)
  if (now - lastMapUpdate >= 200) {
    lastMapUpdate = now;
    updateMapping();
  }
  
  // Update navigation (every 150ms)
  if (now - lastNavigationUpdate >= 150) {
    lastNavigationUpdate = now;
    updateNavigation();
  }
  
  // Send status (every 1000ms)
  if (now - lastStatusSend >= 1000) {
    lastStatusSend = now;
    sendStatus();
  }
  
  // Handle serial commands
  handleSerialCommands();
  
  // Handle button press
  handleButtonPress();
  
  // State machine
  executeStateMachine();
  
  delay(10);
}

// ==================== HARDWARE INITIALIZATION ====================
void initializePins() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  
  DEBUG_PRINTLN("✅ Pins initialized");
}

// ==================== SERIAL COMMAND HANDLER ====================
void handleSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'M':
      case 'm':
        if (systemState == STATE_IDLE) {
          startMapping();
        } else {
          stopMapping();
        }
        break;
        
      case 'S':
      case 's':
        emergencyStop();
        break;
        
      case 'R':
      case 'r':
        resetSystem();
        break;
        
      case 'T':
      case 't':
        printSystemStatus();
        break;
        
      case 'C':
      case 'c':
        clearMap();
        break;
        
      default:
        DEBUG_PRINTLN("Unknown command: " + String(cmd));
    }
  }
}

// ==================== BUTTON HANDLER ====================
void handleButtonPress() {
  static bool lastButtonState = HIGH;
  static unsigned long buttonPressTime = 0;
  
  bool currentState = digitalRead(BUTTON_PIN);
  
  if (currentState == LOW && lastButtonState == HIGH) {
    buttonPressTime = millis();
  } else if (currentState == HIGH && lastButtonState == LOW) {
    unsigned long pressDuration = millis() - buttonPressTime;
    
    if (pressDuration > 50 && pressDuration < 1000) {
      // Short press - toggle mapping
      if (systemState == STATE_MAPPING) {
        stopMapping();
      } else if (systemState == STATE_IDLE) {
        startMapping();
      }
    } else if (pressDuration >= 1000) {
      // Long press - emergency stop
      emergencyStop();
    }
  }
  
  lastButtonState = currentState;
}

// ==================== SYSTEM CONTROL ====================
void startMapping() {
  systemState = STATE_MAPPING;
  clearMap();
  DEBUG_PRINTLN("🗺️ Mapping started");
  tone(BUZZER_PIN, 1000, 100);
}

void stopMapping() {
  systemState = STATE_IDLE;
  stopMotors();
  saveMapToEEPROM();
  DEBUG_PRINTLN("⏹️ Mapping stopped and saved");
  tone(BUZZER_PIN, 800, 100);
  delay(150);
  tone(BUZZER_PIN, 600, 100);
}

void emergencyStop() {
  systemState = STATE_ERROR;
  stopMotors();
  DEBUG_PRINTLN("🛑 EMERGENCY STOP!");
  
  // Emergency beep pattern
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, 2000, 100);
    delay(150);
  }
}

void resetSystem() {
  systemState = STATE_IDLE;
  stopMotors();
  clearMap();
  currentMission.active = false;
  currentMission.waypointCount = 0;
  DEBUG_PRINTLN("🔄 System reset");
}

// ==================== STATE MACHINE ====================
void executeStateMachine() {
  static unsigned long lastStateUpdate = 0;
  unsigned long now = millis();
  
  if (now - lastStateUpdate < 100) return;
  lastStateUpdate = now;
  
  switch (systemState) {
    case STATE_IDLE:
      // Just monitor and wait for commands
      break;
      
    case STATE_MAPPING:
      performMapping();
      break;
      
    case STATE_MISSION_READY:
      // Wait for start mission command
      break;
      
    case STATE_MISSION_ACTIVE:
      executeMission();
      break;
      
    case STATE_GOTO_WAYPOINT:
      navigateToCurrentWaypoint();
      break;
      
    case STATE_AT_WAYPOINT:
      handleWaypointArrival();
      break;
      
    case STATE_MISSION_COMPLETE:
      stopMotors();
      DEBUG_PRINTLN("🎉 Mission completed!");
      systemState = STATE_IDLE;
      break;
      
    case STATE_ERROR:
      // Stay in error state until manual reset
      break;
  }
}

// ==================== STATUS REPORTING ====================
void printSystemStatus() {
  DEBUG_PRINTLN("\n=== SYSTEM STATUS ===");
  DEBUG_PRINTLN("State: " + String(getStateName()));
  DEBUG_PRINTLN("Position: (" + String(robotPos.x, 1) + ", " + String(robotPos.y, 1) + ")");
  DEBUG_PRINTLN("Heading: " + String(robotPos.heading, 1) + "°");
  
  DEBUG_PRINT("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensors.valid[i]) {
      DEBUG_PRINT(String(sensors.distances[i], 1) + " ");
    } else {
      DEBUG_PRINT("-- ");
    }
  }
  DEBUG_PRINTLN("");
  
  if (currentMission.active) {
    DEBUG_PRINTLN("Mission: " + String(currentMission.currentWaypoint + 1) + 
                  "/" + String(currentMission.waypointCount));
  }
  
  DEBUG_PRINTLN("====================\n");
}

const char* getStateName() {
  switch (systemState) {
    case STATE_IDLE: return "IDLE";
    case STATE_MAPPING: return "MAPPING";
    case STATE_MISSION_READY: return "MISSION_READY";
    case STATE_MISSION_ACTIVE: return "MISSION_ACTIVE";
    case STATE_GOTO_WAYPOINT: return "GOTO_WAYPOINT";
    case STATE_AT_WAYPOINT: return "AT_WAYPOINT";
    case STATE_MISSION_COMPLETE: return "MISSION_COMPLETE";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// ==================== UTILITY FUNCTIONS ====================
void tone(int pin, int frequency, int duration) {
  // Simple tone generation for ESP32
  pinMode(pin, OUTPUT);
  
  if (frequency <= 0 || duration <= 0) return;
  
  int period = 1000000 / frequency;
  int cycles = (duration * 1000) / period;
  
  for (int i = 0; i < cycles; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(period / 2);
    digitalWrite(pin, LOW);
    delayMicroseconds(period / 2);
  }
}