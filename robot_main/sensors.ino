// sensors.ino - Ultrasonic Sensor Management with Interrupts

// ==================== INTERRUPT SERVICE ROUTINES ====================
void IRAM_ATTR echoFrontISR() {
  if (digitalRead(ECHO_FRONT)) {
    echo_start[0] = micros();
  } else {
    echo_end[0] = micros();
    echo_done[0] = true;
  }
}

void IRAM_ATTR echoRightISR() {
  if (digitalRead(ECHO_RIGHT)) {
    echo_start[1] = micros();
  } else {
    echo_end[1] = micros();
    echo_done[1] = true;
  }
}

void IRAM_ATTR echoBackISR() {
  if (digitalRead(ECHO_BACK)) {
    echo_start[2] = micros();
  } else {
    echo_end[2] = micros();
    echo_done[2] = true;
  }
}

void IRAM_ATTR echoLeftISR() {
  if (digitalRead(ECHO_LEFT)) {
    echo_start[3] = micros();
  } else {
    echo_end[3] = micros();
    echo_done[3] = true;
  }
}

// ==================== SENSOR INITIALIZATION ====================
void initializeSensors() {
  // Configure trigger pin
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  // Configure echo pins
  pinMode(ECHO_FRONT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(ECHO_BACK, INPUT);
  pinMode(ECHO_LEFT, INPUT);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ECHO_FRONT), echoFrontISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_RIGHT), echoRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_BACK), echoBackISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_LEFT), echoLeftISR, CHANGE);
  
  // Initialize sensor data
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensors.distances[i] = MAX_DISTANCE;
    sensors.valid[i] = false;
    echo_start[i] = 0;
    echo_end[i] = 0;
    echo_done[i] = false;
  }
  
  sensors.timestamp = millis();
  
  DEBUG_PRINTLN("✅ Sensors initialized");
  DEBUG_PRINTLN("   Front: Pin " + String(ECHO_FRONT));
  DEBUG_PRINTLN("   Right: Pin " + String(ECHO_RIGHT));
  DEBUG_PRINTLN("   Back:  Pin " + String(ECHO_BACK));
  DEBUG_PRINTLN("   Left:  Pin " + String(ECHO_LEFT));
}

// ==================== SENSOR UPDATE ====================
void updateSensors() {
  static int currentSensor = 0;
  static unsigned long lastTrigger = 0;
  unsigned long now = millis();
  
  // Process completed measurements first
  processSensorReadings();
  
  // Trigger next sensor every 25ms to avoid interference
  if (now - lastTrigger >= 25) {
    triggerSensor(currentSensor);
    currentSensor = (currentSensor + 1) % NUM_SENSORS;
    lastTrigger = now;
  }
  
  sensors.timestamp = now;
}

void triggerSensor(int sensorIndex) {
  // Reset flags for this sensor
  echo_done[sensorIndex] = false;
  echo_start[sensorIndex] = 0;
  echo_end[sensorIndex] = 0;
  
  // Send trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

void processSensorReadings() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (echo_done[i]) {
      noInterrupts();
      unsigned long duration = echo_end[i] - echo_start[i];
      echo_done[i] = false;
      interrupts();
      
      // Calculate distance
      float distance = (duration * SOUND_SPEED) / 2.0;
      
      // Validate reading
      if (IS_VALID_DISTANCE(distance)) {
        sensors.distances[i] = distance;
        sensors.valid[i] = true;
      } else {
        sensors.distances[i] = MAX_DISTANCE;
        sensors.valid[i] = false;
      }
    }
  }
}

// ==================== SENSOR UTILITIES ====================
bool isObstacleDetected() {
  // Check front sensor for immediate obstacle
  return (sensors.valid[0] && sensors.distances[0] < SAFE_DISTANCE);
}

bool isPathClear(int direction) {
  // direction: 0=front, 1=right, 2=back, 3=left
  if (direction < 0 || direction >= NUM_SENSORS) return false;
  
  return (sensors.valid[direction] && sensors.distances[direction] > SAFE_DISTANCE);
}

float getMinDistance() {
  float minDist = MAX_DISTANCE;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensors.valid[i] && sensors.distances[i] < minDist) {
      minDist = sensors.distances[i];
    }
  }
  return minDist;
}

int getValidSensorCount() {
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensors.valid[i]) count++;
  }
  return count;
}

void printSensorStatus() {
  DEBUG_PRINTLN("\n=== SENSOR STATUS ===");
  const char* sensorNames[] = {"Front", "Right", "Back", "Left"};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    DEBUG_PRINT(sensorNames[i]);
    DEBUG_PRINT(": ");
    if (sensors.valid[i]) {
      DEBUG_PRINT(String(sensors.distances[i], 1));
      DEBUG_PRINTLN(" cm");
    } else {
      DEBUG_PRINTLN("INVALID");
    }
  }
  
  DEBUG_PRINTLN("Valid sensors: " + String(getValidSensorCount()) + "/" + String(NUM_SENSORS));
  DEBUG_PRINTLN("Min distance: " + String(getMinDistance(), 1) + " cm");
  DEBUG_PRINTLN("===================\n");
}

// ==================== EMERGENCY DETECTION ====================
bool checkEmergencyConditions() {
  // Check if front sensor detects very close obstacle
  if (sensors.valid[0] && sensors.distances[0] < 8.0) {
    DEBUG_PRINTLN("⚠️ EMERGENCY: Front obstacle too close!");
    return true;
  }
  
  // Check if all sensors show obstacles (stuck situation)
  int blockedSensors = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensors.valid[i] && sensors.distances[i] < 15.0) {
      blockedSensors++;
    }
  }
  
  if (blockedSensors >= 3) {
    DEBUG_PRINTLN("⚠️ EMERGENCY: Robot appears stuck!");
    return true;
  }
  
  return false;
}

// ==================== SENSOR CALIBRATION ====================
void calibrateSensors() {
  DEBUG_PRINTLN("📏 Calibrating sensors...");
  
  // Take multiple readings to establish baseline
  float readings[NUM_SENSORS][10];
  
  for (int sample = 0; sample < 10; sample++) {
    // Trigger all sensors
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    delay(100); // Wait for readings
    
    // Store readings
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (echo_done[i]) {
        unsigned long duration = echo_end[i] - echo_start[i];
        readings[i][sample] = (duration * SOUND_SPEED) / 2.0;
        echo_done[i] = false;
      } else {
        readings[i][sample] = MAX_DISTANCE;
      }
    }
  }
  
  // Calculate averages and detect faulty sensors
  const char* sensorNames[] = {"Front", "Right", "Back", "Left"};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    float sum = 0;
    int validReadings = 0;
    
    for (int j = 0; j < 10; j++) {
      if (IS_VALID_DISTANCE(readings[i][j])) {
        sum += readings[i][j];
        validReadings++;
      }
    }
    
    if (validReadings >= 5) {
      float average = sum / validReadings;
      DEBUG_PRINTLN(sensorNames[i] + " sensor: " + String(average, 1) + " cm (OK)");
    } else {
      DEBUG_PRINTLN(sensorNames[i] + " sensor: FAULTY - " + String(validReadings) + "/10 valid readings");
    }
  }
  
  DEBUG_PRINTLN("✅ Sensor calibration complete");
}