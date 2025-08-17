// mapping.ino - SLAM Mapping System

// ==================== MAP INITIALIZATION ====================
void initializeMap() {
  clearMap();
  DEBUG_PRINTLN("✅ Map initialized (" + String(MAP_SIZE) + "x" + String(MAP_SIZE) + ")");
}

void clearMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      globalMap[x][y].value = CELL_UNKNOWN;
      globalMap[x][y].confidence = 0;
      globalMap[x][y].lastUpdate = 0;
    }
  }
  DEBUG_PRINTLN("🗺️ Map cleared");
}

// ==================== MAP UPDATING ====================
void updateMapping() {
  if (systemState != STATE_MAPPING) return;
  
  // Update robot position in map
  updateRobotPositionInMap();
  
  // Update map based on sensor readings
  updateMapFromSensors();
  
  // Clean old data
  cleanOldMapData();
}

void updateRobotPositionInMap() {
  int gridX = WORLD_TO_GRID_X(robotPos.x);
  int gridY = WORLD_TO_GRID_Y(robotPos.y);
  
  if (IS_VALID_GRID(gridX, gridY)) {
    // Mark robot position as free space
    setMapCell(gridX, gridY, CELL_FREE, 100);
  }
}

void updateMapFromSensors() {
  int robotGridX = WORLD_TO_GRID_X(robotPos.x);
  int robotGridY = WORLD_TO_GRID_Y(robotPos.y);
  
  if (!IS_VALID_GRID(robotGridX, robotGridY)) return;
  
  // Process each sensor
  float sensorAngles[] = {0, 90, 180, 270}; // Front, Right, Back, Left
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensors.valid[i]) continue;
    
    float distance = sensors.distances[i];
    float absoluteAngle = robotPos.heading + sensorAngles[i];
    
    // Normalize angle
    while (absoluteAngle >= 360) absoluteAngle -= 360;
    while (absoluteAngle < 0) absoluteAngle += 360;
    
    // Convert to radians
    float angleRad = DEGREES_TO_RADIANS(absoluteAngle);
    
    // Mark free space along the ray
    markFreeSpaceAlongRay(robotPos.x, robotPos.y, angleRad, distance);
    
    // Mark obstacle at the end if within reasonable range
    if (distance < MAX_DISTANCE - 10) {
      markObstacleAtDistance(robotPos.x, robotPos.y, angleRad, distance);
    }
  }
}

void markFreeSpaceAlongRay(float startX, float startY, float angleRad, float maxDistance) {
  float stepSize = CELL_SIZE / 2.0; // Half cell size for better resolution
  int steps = (int)(maxDistance / stepSize);
  
  for (int step = 1; step < steps; step++) {
    float x = startX + step * stepSize * cos(angleRad);
    float y = startY + step * stepSize * sin(angleRad);
    
    int gridX = WORLD_TO_GRID_X(x);
    int gridY = WORLD_TO_GRID_Y(y);
    
    if (IS_VALID_GRID(gridX, gridY)) {
      // Only mark as free if not already marked as obstacle
      if (globalMap[gridX][gridY].value != CELL_OBSTACLE) {
        setMapCell(gridX, gridY, CELL_FREE, 5);
      }
    }
  }
}

void markObstacleAtDistance(float startX, float startY, float angleRad, float distance) {
  float obstacleX = startX + distance * cos(angleRad);
  float obstacleY = startY + distance * sin(angleRad);
  
  int gridX = WORLD_TO_GRID_X(obstacleX);
  int gridY = WORLD_TO_GRID_Y(obstacleY);
  
  if (IS_VALID_GRID(gridX, gridY)) {
    setMapCell(gridX, gridY, CELL_OBSTACLE, 20);
  }
}

void setMapCell(int x, int y, int8_t value, uint8_t confidenceIncrease) {
  if (!IS_VALID_GRID(x, y)) return;
  
  globalMap[x][y].value = value;
  globalMap[x][y].confidence = min(255, globalMap[x][y].confidence + confidenceIncrease);
  globalMap[x][y].lastUpdate = millis();
}

// ==================== MAP ANALYSIS ====================
bool isMapCellObstacle(int x, int y) {
  if (!IS_VALID_GRID(x, y)) return true; // Treat out-of-bounds as obstacles
  
  return (globalMap[x][y].value == CELL_OBSTACLE && 
          globalMap[x][y].confidence > 50);
}

bool isMapCellFree(int x, int y) {
  if (!IS_VALID_GRID(x, y)) return false;
  
  return (globalMap[x][y].value == CELL_FREE && 
          globalMap[x][y].confidence > 30);
}

bool isPathClearInMap(float startX, float startY, float endX, float endY) {
  float dx = endX - startX;
  float dy = endY - startY;
  float distance = sqrt(dx * dx + dy * dy);
  
  if (distance < 1.0) return true; // Very short distance
  
  int steps = (int)(distance / (CELL_SIZE / 2.0));
  
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float x = startX + t * dx;
    float y = startY + t * dy;
    
    int gridX = WORLD_TO_GRID_X(x);
    int gridY = WORLD_TO_GRID_Y(y);
    
    if (isMapCellObstacle(gridX, gridY)) {
      return false;
    }
  }
  
  return true;
}

// ==================== MAP CLEANING ====================
void cleanOldMapData() {
  static unsigned long lastCleanup = 0;
  unsigned long now = millis();
  
  // Clean old data every 30 seconds
  if (now - lastCleanup < 30000) return;
  lastCleanup = now;
  
  int cleanedCells = 0;
  
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      // Reduce confidence of old data
      if (globalMap[x][y].lastUpdate > 0 && 
          now - globalMap[x][y].lastUpdate > 60000) { // 1 minute old
        
        if (globalMap[x][y].confidence > 0) {
          globalMap[x][y].confidence = max(0, globalMap[x][y].confidence - 10);
          cleanedCells++;
          
          // If confidence drops too low, mark as unknown
          if (globalMap[x][y].confidence < 20) {
            globalMap[x][y].value = CELL_UNKNOWN;
          }
        }
      }
    }
  }
  
  if (cleanedCells > 0) {
    DEBUG_PRINTLN("🧹 Cleaned " + String(cleanedCells) + " old map cells");
  }
}

// ==================== MAP EXPLORATION ====================
void performMapping() {
  static unsigned long lastExplorationUpdate = 0;
  unsigned long now = millis();
  
  if (now - lastExplorationUpdate < 200) return;
  lastExplorationUpdate = now;
  
  // Check for emergency conditions first
  if (checkEmergencyConditions()) {
    emergencyStop();
    return;
  }
  
  // Perform exploration strategy
  if (moveState == MOVE_OBSTACLE_AVOID) {
    // Let obstacle avoidance handle movement
    return;
  }
  
  // Choose exploration strategy
  if (isObstacleDetected()) {
    performObstacleAvoidance();
  } else {
    performWallFollowing();
  }
}

void performWallFollowing() {
  float rightDist = sensors.valid[1] ? sensors.distances[1] : MAX_DISTANCE;
  float leftDist = sensors.valid[3] ? sensors.distances[3] : MAX_DISTANCE;
  float frontDist = sensors.valid[0] ? sensors.distances[0] : MAX_DISTANCE;
  
  const float IDEAL_WALL_DISTANCE = 25.0;
  const float WALL_TOLERANCE = 8.0;
  
  // Prefer right wall following
  if (rightDist < 60) {
    // Wall detected on right
    if (frontDist < SAFE_DISTANCE) {
      // Front blocked, turn left
      turnLeft(TURN_SPEED);
    } else if (rightDist < IDEAL_WALL_DISTANCE - WALL_TOLERANCE) {
      // Too close to right wall, turn left slightly
      turnLeft(BASE_SPEED / 2);
    } else if (rightDist > IDEAL_WALL_DISTANCE + WALL_TOLERANCE) {
      // Too far from right wall, turn right slightly
      turnRight(BASE_SPEED / 2);
    } else {
      // Good distance, move forward
      moveForward(BASE_SPEED);
    }
  } else if (leftDist < 60) {
    // Wall detected on left
    if (frontDist < SAFE_DISTANCE) {
      // Front blocked, turn right
      turnRight(TURN_SPEED);
    } else if (leftDist < IDEAL_WALL_DISTANCE - WALL_TOLERANCE) {
      // Too close to left wall, turn right slightly
      turnRight(BASE_SPEED / 2);
    } else if (leftDist > IDEAL_WALL_DISTANCE + WALL_TOLERANCE) {
      // Too far from left wall, turn left slightly
      turnLeft(BASE_SPEED / 2);
    } else {
      // Good distance, move forward
      moveForward(BASE_SPEED);
    }
  } else {
    // No walls detected, explore forward
    if (frontDist > SAFE_DISTANCE) {
      moveForward(BASE_SPEED);
    } else {
      // Front blocked, turn to find open space
      if (rightDist > leftDist) {
        turnRight(TURN_SPEED);
      } else {
        turnLeft(TURN_SPEED);
      }
    }
  }
}

// ==================== MAP STATISTICS ====================
void getMapStatistics(int* knownCells, int* freeCells, int* obstacleCells) {
  *knownCells = 0;
  *freeCells = 0;
  *obstacleCells = 0;
  
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      if (globalMap[x][y].confidence > 20) {
        (*knownCells)++;
        
        if (globalMap[x][y].value == CELL_FREE) {
          (*freeCells)++;
        } else if (globalMap[x][y].value == CELL_OBSTACLE) {
          (*obstacleCells)++;
        }
      }
    }
  }
}

float getMapCompletionPercentage() {
  int knownCells, freeCells, obstacleCells;
  getMapStatistics(&knownCells, &freeCells, &obstacleCells);
  
  return (float)knownCells / (MAP_SIZE * MAP_SIZE) * 100.0;
}

void printMapStatistics() {
  int knownCells, freeCells, obstacleCells;
  getMapStatistics(&knownCells, &freeCells, &obstacleCells);
  
  float completion = getMapCompletionPercentage();
  
  DEBUG_PRINTLN("\n=== MAP STATISTICS ===");
  DEBUG_PRINTLN("Map size: " + String(MAP_SIZE) + "x" + String(MAP_SIZE) + " cells");
  DEBUG_PRINTLN("Cell size: " + String(CELL_SIZE) + " cm");
  DEBUG_PRINTLN("Total area: " + String(MAP_SIZE * CELL_SIZE) + "x" + String(MAP_SIZE * CELL_SIZE) + " cm");
  DEBUG_PRINTLN("Known cells: " + String(knownCells) + "/" + String(MAP_SIZE * MAP_SIZE));
  DEBUG_PRINTLN("Free space: " + String(freeCells) + " cells");
  DEBUG_PRINTLN("Obstacles: " + String(obstacleCells) + " cells");
  DEBUG_PRINTLN("Completion: " + String(completion, 1) + "%");
  DEBUG_PRINTLN("======================\n");
}