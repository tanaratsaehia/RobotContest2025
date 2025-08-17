// navigation.ino - Navigation and Mission System

// ==================== NAVIGATION UPDATE ====================
void updateNavigation() {
  switch (systemState) {
    case STATE_GOTO_WAYPOINT:
      navigateToCurrentWaypoint();
      break;
    case STATE_MISSION_ACTIVE:
      executeMission();
      break;
    default:
      // No navigation needed for other states
      break;
  }
}

// ==================== MISSION MANAGEMENT ====================
void startMission() {
  if (currentMission.waypointCount == 0) {
    DEBUG_PRINTLN("❌ No waypoints defined for mission");
    return;
  }
  
  currentMission.active = true;
  currentMission.currentWaypoint = 0;
  currentMission.startTime = millis();
  systemState = STATE_MISSION_ACTIVE;
  
  DEBUG_PRINTLN("🚀 Mission started with " + String(currentMission.waypointCount) + " waypoints");
  
  // Reset waypoint visited flags
  for (int i = 0; i < currentMission.waypointCount; i++) {
    currentMission.waypoints[i].visited = false;
    currentMission.waypoints[i].arrivalTime = 0;
  }
  
  tone(BUZZER_PIN, 1200, 100);
  delay(150);
  tone(BUZZER_PIN, 1500, 100);
}

void stopMission() {
  currentMission.active = false;
  systemState = STATE_IDLE;
  stopMotors();
  
  DEBUG_PRINTLN("⏹️ Mission stopped");
  tone(BUZZER_PIN, 800, 200);
}

void executeMission() {
  if (!currentMission.active || currentMission.currentWaypoint >= currentMission.waypointCount) {
    // Mission complete
    systemState = STATE_MISSION_COMPLETE;
    return;
  }
  
  systemState = STATE_GOTO_WAYPOINT;
}

// ==================== WAYPOINT NAVIGATION ====================
void navigateToCurrentWaypoint() {
  if (!currentMission.active || currentMission.currentWaypoint >= currentMission.waypointCount) {
    return;
  }
  
  Waypoint* target = &currentMission.waypoints[currentMission.currentWaypoint];
  
  // Calculate distance to waypoint
  float distance = calculateDistance(robotPos.x, robotPos.y, target->x, target->y);
  
  if (distance < WAYPOINT_TOLERANCE) {
    // Arrived at waypoint
    arriveAtWaypoint();
  } else {
    // Navigate toward waypoint
    navigateToPoint(target->x, target->y);
  }
}

void navigateToPoint(float targetX, float targetY) {
  // Check for obstacles first
  if (isObstacleDetected()) {
    performObstacleAvoidance();
    return;
  }
  
  float dx = targetX - robotPos.x;
  float dy = targetY - robotPos.y;
  float distance = sqrt(dx * dx + dy * dy);
  
  if (distance < WAYPOINT_TOLERANCE) {
    stopMotors();
    return;
  }
  
  // Calculate desired heading
  float targetHeading = RADIANS_TO_DEGREES(atan2(dy, dx));
  float headingError = targetHeading - robotPos.heading;
  
  // Normalize heading error to -180 to 180
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  // Determine movement strategy
  if (abs(headingError) > 20) {
    // Large heading error, turn in place
    if (headingError > 0) {
      rotateLeft(TURN_SPEED);
    } else {
      rotateRight(TURN_SPEED);
    }
  } else if (abs(headingError) > 10) {
    // Medium heading error, turn while moving
    if (headingError > 0) {
      turnLeft(TURN_SPEED);
    } else {
      turnRight(TURN_SPEED);
    }
  } else {
    // Small heading error, move forward with speed adjustment
    int speed = BASE_SPEED;
    
    // Slow down as we approach the target
    if (distance < 50) {
      speed = map(distance, 0, 50, 25, BASE_SPEED);
    }
    
    moveForward(speed);
  }
}

void arriveAtWaypoint() {
  Waypoint* waypoint = &currentMission.waypoints[currentMission.currentWaypoint];
  
  waypoint->visited = true;
  waypoint->arrivalTime = millis();
  systemState = STATE_AT_