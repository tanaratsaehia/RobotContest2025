/*
 * Robot SLAM System Configuration
 * กำหนดค่าต่างๆ ของระบบ mapping ไว้ในไฟล์เดียว
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// ================================
// SYSTEM CONFIGURATION
// ================================

// Map Configuration
#define MAP_SIZE 100              // จำนวน cells ในแต่ละด้าน (100x100)
#define CELL_SIZE 10              // ขนาด cell ใน cm (10cm = 1 cell)
#define MAP_AREA_SIZE (MAP_SIZE * CELL_SIZE)  // พื้นที่รวม 10m x 10m

// Sensor Configuration
#define MAX_SENSOR_RANGE 400      // ระยะไกลสุดของ ultrasonic (4 เมตร)
#define MIN_SENSOR_RANGE 2        // ระยะใกล้สุดที่ถือว่าน่าเชื่อถือ (2 cm)
#define SENSOR_TIMEOUT 30000      // timeout ใน microseconds
#define NUM_SENSORS 4             // จำนวน ultrasonic sensors

// Update Intervals
#define MAP_UPDATE_INTERVAL 200   // อัพเดท map ทุก 200ms
#define SENSOR_READ_INTERVAL 100  // อ่าน sensors ทุก 100ms
#define COMM_SEND_INTERVAL 200    // ส่งข้อมูลทุก 200ms
#define DISPLAY_UPDATE_INTERVAL 1000  // แสดงผลทุก 1 วินาที

// Communication Configuration
#define LOCAL_MAP_SIZE 5          // ส่ง local map ขนาด 5x5 cells
#define MESSAGE_BUFFER_SIZE 50    // ขนาด buffer สำหรับข้อความ

// ================================
// HARDWARE PIN DEFINITIONS
// ================================

// Ultrasonic Sensor Pins
#define TRIG_PIN        32        // Trigger pin ร่วมกันสำหรับ sensors ทั้งหมด
#define ECHO1_PIN       39        // Front sensor echo pin
#define ECHO2_PIN       34        // Right sensor echo pin  
#define ECHO3_PIN       36        // Back sensor echo pin
#define ECHO4_PIN       35        // Left sensor echo pin

// Motor Control Pins
#define MOTOR_FRONT_A1  19        // Front right motor +
#define MOTOR_FRONT_A2  18        // Front right motor -
#define MOTOR_FRONT_B1  33        // Front left motor +
#define MOTOR_FRONT_B2  23        // Front left motor -
#define MOTOR_BACK_A1   4         // Back right motor +
#define MOTOR_BACK_A2   13        // Back right motor -
#define MOTOR_BACK_B1   16        // Back left motor +
#define MOTOR_BACK_B2   17        // Back left motor -

// Servo Pins (optional)
#define SERVO_PIN1      26        // Left servo
#define SERVO_PIN2      25        // Right servo

// Control Pins
#define BUTTON_PIN      27        // Control button
#define BUZZER_PIN      14        // Buzzer output
#define LED_PIN         2         // Status LED

// IMU Configuration
#define IMU_I2C_ADDR    0x69      // BMI160 I2C address

// ================================
// MAC ADDRESSES
// ================================
// อัพเดท MAC addresses ตาม ESP32 จริงของคุณ

// Robot ESP32 MAC Address
static const uint8_t ROBOT_MAC_ADDR[] = {0x48, 0xe7, 0x29, 0xc9, 0xdf, 0x68};

// Control Station ESP32 MAC Address  
static const uint8_t STATION_MAC_ADDR[] = {0x48, 0xe7, 0x29, 0xc9, 0x57, 0x28};

// ================================
// MOTOR CONFIGURATION
// ================================

#define MOTOR_SPEED_DEFAULT  90   // ความเร็วมาตรฐาน (%)
#define MOTOR_ANALOG_VALUE   105  // ค่า analog สำหรับ PWM (0-255)
#define TURN_SPEED_RATIO     0.7  // อัตราส่วนความเร็วในการเลี้ยว

// Robot Physical Parameters
#define WHEEL_DIAMETER       65   // เส้นผ่านศูนย์กลางล้อ (mm)
#define WHEEL_BASE          150   // ระยะห่างระหว่างล้อซ้าย-ขวา (mm)
#define ROBOT_LENGTH        200   // ความยาวหุ่นยนต์ (mm)
#define ROBOT_WIDTH         150   // ความกว้างหุ่นยนต์ (mm)

// ================================
// SENSOR ANGLES AND POSITIONS
// ================================

// มุมของ sensors เทียบกับด้านหน้าหุ่นยนต์ (องศา)
static const float SENSOR_ANGLES[NUM_SENSORS] = {
  0,    // Front sensor
  90,   // Right sensor  
  180,  // Back sensor
  270   // Left sensor
};

// ตำแหน่งของ sensors บนหุ่นยนต์ (mm จากจุดศูนย์กลาง)
static const struct SensorPosition {
  float x, y;
} SENSOR_POSITIONS[NUM_SENSORS] = {
  {ROBOT_LENGTH/2, 0},      // Front: หน้าหุ่นยนต์
  {0, -ROBOT_WIDTH/2},      // Right: ขวาหุ่นยนต์
  {-ROBOT_LENGTH/2, 0},     // Back: หลังหุ่นยนต์
  {0, ROBOT_WIDTH/2}        // Left: ซ้ายหุ่นยนต์
};

// ================================
// ALGORITHM PARAMETERS
// ================================

// SLAM Parameters
#define CONFIDENCE_INCREMENT  10   // เพิ่ม confidence เมื่อเจอข้อมูลใหม่
#define CONFIDENCE_MAX       255   // ค่า confidence สูงสุด
#define CONFIDENCE_THRESHOLD  50   // threshold สำหรับถือว่าข้อมูลน่าเชื่อถือ

// Dead Reckoning Parameters
#define GYRO_SENSITIVITY     0.01  // ความไวของ gyroscope
#define POSITION_TOLERANCE   5.0   // ความคลาดเคลื่อนที่ยอมรับได้ (cm)
#define HEADING_TOLERANCE    2.0   // ความคลาดเคลื่อนมุมที่ยอมรับได้ (องศา)

// Obstacle Detection
#define OBSTACLE_THRESHOLD   10    // ระยะขั้นต่ำที่ถือว่าเป็นสิ่งกีดขวาง (cm)
#define FREE_SPACE_INCREMENT 5     // การเพิ่ม confidence สำหรับพื้นที่ว่าง

// ================================
// DEBUGGING AND LOGGING
// ================================

// Debug Levels
#define DEBUG_NONE    0
#define DEBUG_ERROR   1  
#define DEBUG_WARN    2
#define DEBUG_INFO    3
#define DEBUG_VERBOSE 4

// Set current debug level
#define DEBUG_LEVEL DEBUG_INFO

// Debug macros
#if DEBUG_LEVEL >= DEBUG_ERROR
  #define DEBUG_ERROR_PRINT(x) Serial.println("[ERROR] " + String(x))
#else
  #define DEBUG_ERROR_PRINT(x)
#endif

#if DEBUG_LEVEL >= DEBUG_WARN
  #define DEBUG_WARN_PRINT(x) Serial.println("[WARN] " + String(x))
#else
  #define DEBUG_WARN_PRINT(x)
#endif

#if DEBUG_LEVEL >= DEBUG_INFO
  #define DEBUG_INFO_PRINT(x) Serial.println("[INFO] " + String(x))
#else
  #define DEBUG_INFO_PRINT(x)
#endif

#if DEBUG_LEVEL >= DEBUG_VERBOSE
  #define DEBUG_VERBOSE_PRINT(x) Serial.println("[VERBOSE] " + String(x))
#else
  #define DEBUG_VERBOSE_PRINT(x)
#endif

// ================================
// SYSTEM STATUS CODES
// ================================

enum SystemStatus {
  STATUS_OK = 0,
  STATUS_ERROR_SENSOR = 1,
  STATUS_ERROR_COMM = 2,
  STATUS_ERROR_MOTOR = 3,
  STATUS_ERROR_IMU = 4,
  STATUS_ERROR_MEMORY = 5
};

enum MovementCommand {
  CMD_STOP = 0,
  CMD_FORWARD = 1,
  CMD_BACKWARD = 2,
  CMD_TURN_LEFT = 3,
  CMD_TURN_RIGHT = 4,
  CMD_ROTATE_CW = 5,
  CMD_ROTATE_CCW = 6
};

// ================================
// DATA STRUCTURES
// ================================

struct RobotPose {
  float x, y;        // ตำแหน่ง (cm)
  float heading;     // ทิศทาง (องศา)
  unsigned long timestamp;
};

struct SensorReading {
  float distances[NUM_SENSORS];
  bool valid[NUM_SENSORS];
  unsigned long timestamp;
};

struct MapCell {
  int8_t occupancy;     // -1=unknown, 0=free, 100=occupied  
  uint8_t confidence;   // 0-255
  unsigned long lastUpdate;
};

struct MapMessage {
  RobotPose pose;
  SensorReading sensors;
  int8_t localMap[LOCAL_MAP_SIZE * LOCAL_MAP_SIZE];
  uint8_t checksum;
};

// ================================
// UTILITY FUNCTIONS
// ================================

// Convert degrees to radians
inline float degToRad(float degrees) {
  return degrees * PI / 180.0;
}

// Convert radians to degrees  
inline float radToDeg(float radians) {
  return radians * 180.0 / PI;
}

// Normalize angle to 0-360 degrees
inline float normalizeAngle(float angle) {
  while (angle < 0) angle += 360;
  while (angle >= 360) angle -= 360;
  return angle;
}

// Convert world coordinates to grid coordinates
inline void worldToGrid(float worldX, float worldY, int* gridX, int* gridY) {
  *gridX = constrain((int)(worldX / CELL_SIZE), 0, MAP_SIZE - 1);
  *gridY = constrain((int)(worldY / CELL_SIZE), 0, MAP_SIZE - 1);
}

// Convert grid coordinates to world coordinates  
inline void gridToWorld(int gridX, int gridY, float* worldX, float* worldY) {
  *worldX = gridX * CELL_SIZE + CELL_SIZE / 2.0;
  *worldY = gridY * CELL_SIZE + CELL_SIZE / 2.0;
}

// Calculate distance between two points
inline float distance(float x1, float y1, float x2, float y2) {
  return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

// Calculate angle between two points
inline float angleTo(float x1, float y1, float x2, float y2) {
  return atan2(y2-y1, x2-x1) * 180.0 / PI;
}

// ================================
// SYSTEM INFORMATION
// ================================

#define SYSTEM_VERSION_MAJOR  1
#define SYSTEM_VERSION_MINOR  0  
#define SYSTEM_VERSION_PATCH  0

#define SYSTEM_NAME          "Robot SLAM Mapping System"
#define SYSTEM_AUTHOR        "Robot Contest 2025 Team"
#define SYSTEM_BUILD_DATE    __DATE__ " " __TIME__

// Print system information
inline void printSystemInfo() {
  Serial.println("================================");
  Serial.println(SYSTEM_NAME);
  Serial.printf("Version: %d.%d.%d\n", SYSTEM_VERSION_MAJOR, SYSTEM_VERSION_MINOR, SYSTEM_VERSION_PATCH);
  Serial.println("Author: " + String(SYSTEM_AUTHOR));
  Serial.println("Built: " + String(SYSTEM_BUILD_DATE));
  Serial.println("================================");
  Serial.printf("Map: %dx%d cells, %dcm per cell\n", MAP_SIZE, MAP_SIZE, CELL_SIZE);
  Serial.printf("Area: %.1fm x %.1fm\n", MAP_AREA_SIZE/100.0, MAP_AREA_SIZE/100.0);
  Serial.printf("Sensors: %d ultrasonic, range %d-%dcm\n", NUM_SENSORS, MIN_SENSOR_RANGE, MAX_SENSOR_RANGE);
  Serial.println("================================");
}

#endif // ROBOT_CONFIG_H