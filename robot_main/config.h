#ifndef CONFIG_H
#define CONFIG_H

// ==================== SYSTEM CONFIGURATION ====================
#define SERIAL_BAUD 115200
#define DEBUG_MODE 1

// ==================== HARDWARE PINS ====================
// Ultrasonic Sensors
#define TRIG_PIN 32
#define ECHO_FRONT 39  // Sensor 0
#define ECHO_RIGHT 34  // Sensor 1  
#define ECHO_BACK 36   // Sensor 2
#define ECHO_LEFT 35   // Sensor 3

// Motors
#define MOTOR_FRONT_A1 19  // Right motor +
#define MOTOR_FRONT_A2 18  // Right motor -
#define MOTOR_FRONT_B1 33  // Left motor +
#define MOTOR_FRONT_B2 23  // Left motor -
#define MOTOR_BACK_A1 4    // Right motor +
#define MOTOR_BACK_A2 13   // Right motor -
#define MOTOR_BACK_B1 16   // Left motor +
#define MOTOR_BACK_B2 17   // Left motor -

// Control pins
#define BUTTON_PIN 27
#define BUZZER_PIN 14
#define LED_STATUS 2

// ==================== MAP CONFIGURATION ====================
#define MAP_SIZE 30            // 30x30 grid
#define CELL_SIZE 5.0         // 5cm per cell = 1.5m x 1.5m total
#define MAP_CENTER_X 15
#define MAP_CENTER_Y 15

// Map cell values
#define CELL_UNKNOWN -1
#define CELL_FREE 0
#define CELL_OBSTACLE 1

// ==================== SENSOR CONFIGURATION ====================
#define NUM_SENSORS 4
#define MIN_DISTANCE 2.0
#define MAX_DISTANCE 200.0
#define SENSOR_TIMEOUT 30000
#define SOUND_SPEED 0.0343

// ==================== NAVIGATION CONFIGURATION ====================
#define BASE_SPEED 60
#define TURN_SPEED 50
#define SAFE_DISTANCE 20.0
#define WAYPOINT_TOLERANCE 10.0  // 10cm tolerance
#define MAX_WAYPOINTS 10

// ==================== MAC ADDRESSES ====================
uint8_t stationAddress[] = {0x48, 0xe7, 0x29, 0xc9, 0x57, 0x28};

// ==================== DATA STRUCTURES ====================
struct Position {
  float x, y;
  float heading;
  unsigned long timestamp;
};

struct SensorData {
  float distances[NUM_SENSORS];
  bool valid[NUM_SENSORS];
  unsigned long timestamp;
};

struct MapCell {
  int8_t value;      // -1=unknown, 0=free, 1=obstacle
  uint8_t confidence;
  unsigned long lastUpdate;
};

struct Waypoint {
  float x, y;
  bool visited;
  unsigned long arrivalTime;
};

struct Mission {
  Waypoint waypoints[MAX_WAYPOINTS];
  int waypointCount;
  int currentWaypoint;
  bool active;
  unsigned long startTime;
};

// ==================== SYSTEM STATES ====================
enum SystemState {
  STATE_IDLE,
  STATE_MAPPING, 
  STATE_MISSION_READY,
  STATE_MISSION_ACTIVE,
  STATE_GOTO_WAYPOINT,
  STATE_AT_WAYPOINT,
  STATE_MISSION_COMPLETE,
  STATE_ERROR
};

enum MovementState {
  MOVE_STOP,
  MOVE_FORWARD,
  MOVE_BACKWARD,
  MOVE_TURN_LEFT,
  MOVE_TURN_RIGHT,
  MOVE_OBSTACLE_AVOID
};

// ==================== ESP-NOW MESSAGE STRUCTURE ====================
struct ESPNowMessage {
  char type[10];     // "STATUS", "MAP", "MISSION", "ACK"
  char data[200];    // JSON-like data
  uint8_t checksum;
};

// ==================== GLOBAL VARIABLES ====================
extern MapCell globalMap[MAP_SIZE][MAP_SIZE];
extern Position robotPos;
extern SensorData sensors;
extern Mission currentMission;
extern SystemState systemState;
extern MovementState moveState;

// Sensor timing for interrupts
extern volatile unsigned long echo_start[NUM_SENSORS];
extern volatile unsigned long echo_end[NUM_SENSORS];
extern volatile bool echo_done[NUM_SENSORS];

// ==================== UTILITY MACROS ====================
#define DEBUG_PRINT(x) if(DEBUG_MODE) Serial.print(x)
#define DEBUG_PRINTLN(x) if(DEBUG_MODE) Serial.println(x)
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define DEGREES_TO_RADIANS(deg) ((deg) * PI / 180.0)
#define RADIANS_TO_DEGREES(rad) ((rad) * 180.0 / PI)

// World to grid conversion
#define WORLD_TO_GRID_X(world_x) ((int)((world_x) / CELL_SIZE + MAP_CENTER_X))
#define WORLD_TO_GRID_Y(world_y) ((int)((world_y) / CELL_SIZE + MAP_CENTER_Y))
#define GRID_TO_WORLD_X(grid_x) (((grid_x) - MAP_CENTER_X) * CELL_SIZE)
#define GRID_TO_WORLD_Y(grid_y) (((grid_y) - MAP_CENTER_Y) * CELL_SIZE)

// Validation macros
#define IS_VALID_GRID(x, y) ((x) >= 0 && (x) < MAP_SIZE && (y) >= 0 && (y) < MAP_SIZE)
#define IS_VALID_DISTANCE(d) ((d) >= MIN_DISTANCE && (d) <= MAX_DISTANCE)

#endif // CONFIG_H