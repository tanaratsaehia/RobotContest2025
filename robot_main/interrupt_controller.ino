// interrupt_controller.ino - ระบบ Interrupt สำหรับ Ultrasonic Sensors

#define ECHO1_PIN  39  // Front sensor
#define ECHO2_PIN  34  // Right sensor
#define ECHO3_PIN  36  // Back sensor
#define ECHO4_PIN  35  // Left sensor

// Interrupt Service Routines (ISR)
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

// Initialize ultrasonic sensors
void ultrasonic_begin() {
  // Configure trigger pin
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  // Configure echo pins
  pinMode(ECHO1_PIN, INPUT);
  pinMode(ECHO2_PIN, INPUT);
  pinMode(ECHO3_PIN, INPUT);
  pinMode(ECHO4_PIN, INPUT);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ECHO1_PIN), echo1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO2_PIN), echo2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO3_PIN), echo3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO4_PIN), echo4ISR, CHANGE);
  
  Serial.println("✅ Ultrasonic sensors initialized");
  Serial.println("   Front: Pin 39");
  Serial.println("   Right: Pin 34");
  Serial.println("   Back:  Pin 36");
  Serial.println("   Left:  Pin 35");
}

// Get sensor name by index
const char* getSensorNameByIndex(int index) {
  switch(index) {
    case 0: return "Front";
    case 1: return "Right";
    case 2: return "Back";
    case 3: return "Left";
    default: return "Unknown";
  }
}

// Print sensor status
void printSensorStatus() {
  Serial.println("\n=== Sensor Status ===");
  for (int i = 0; i < 4; i++) {
    Serial.print(getSensorNameByIndex(i));
    Serial.print(": ");
    if (sensorValid[i]) {
      Serial.print(sensorDistances[i]);
      Serial.println(" cm");
    } else {
      Serial.println("Invalid/No reading");
    }
  }
  Serial.println("====================\n");
}

// Test ultrasonic sensors
void test_ultrasonic_sensors() {
  Serial.println("Testing ultrasonic sensors...");
  
  // Trigger all sensors
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Wait for measurements
  delay(100);
  
  // Process and display results
  processSensorReadings();
  printSensorStatus();
}

// Check for sensor failures
bool checkSensorHealth() {
  int failedSensors = 0;
  
  for (int i = 0; i < 4; i++) {
    if (!sensorValid[i]) {
      failedSensors++;
      Serial.print("⚠️ Sensor ");
      Serial.print(getSensorNameByIndex(i));
      Serial.println(" may be disconnected or faulty");
    }
  }
  
  if (failedSensors == 0) {
    return true;
  } else if (failedSensors < 3) {
    Serial.println("⚠️ Some sensors failed but system can continue");
    return true;
  } else {
    Serial.println("❌ Too many sensor failures - check connections!");
    return false;
  }
}

// Emergency stop if all front sensors detect close obstacle
void checkEmergencyStop() {
  if (sensorValid[0] && sensorDistances[0] < 10) { // Front sensor < 10cm
    Serial.println("🛑 EMERGENCY STOP - Obstacle too close!");
    stop_motor();
    isMovingStraight = false;
    isMapping = false;
    
    // Sound alarm
    for (int i = 0; i < 3; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
    }
  }
}