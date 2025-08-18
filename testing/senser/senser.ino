/*
 * Ultrasonic Sensor Test Code
 * Upload this to Robot ESP32 to test sensors only
 */

#define TRIG_PIN   32
#define ECHO1_PIN  39  // Front
#define ECHO2_PIN  34  // Right
#define ECHO3_PIN  36  // Back
#define ECHO4_PIN  35  // Left

#define SOUND_SPEED 0.0343

// Ultrasonic data
volatile unsigned long echo_start[4] = {0, 0, 0, 0};
volatile unsigned long echo_end[4] = {0, 0, 0, 0};
volatile bool echo_done[4] = {false, false, false, false};
float sensor_distances[4] = {999.0, 999.0, 999.0, 999.0};

unsigned long lastTriggerTime = 0;

// Interrupt handlers
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Ultrasonic Sensor Test ===");
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(ECHO2_PIN, INPUT);
  pinMode(ECHO3_PIN, INPUT);
  pinMode(ECHO4_PIN, INPUT);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ECHO1_PIN), echo1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO2_PIN), echo2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO3_PIN), echo3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO4_PIN), echo4ISR, CHANGE);
  
  Serial.println("Sensors initialized. Testing...");
  Serial.println("Wiring check:");
  Serial.println("TRIG_PIN: 32 (shared)");
  Serial.println("ECHO1_PIN: 39 (Front)");
  Serial.println("ECHO2_PIN: 34 (Right)");
  Serial.println("ECHO3_PIN: 36 (Back)");
  Serial.println("ECHO4_PIN: 35 (Left)");
  Serial.println("VCC: 5V, GND: GND");
  Serial.println("============================");
}

void loop() {
  unsigned long now = millis();
  
  // Trigger sensors every 200ms
  if (now - lastTriggerTime >= 200) {
    lastTriggerTime = now;
    
    Serial.println("\n--- Triggering Sensors ---");
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    Serial.println("Trigger pulse sent");
  }
  
  // Process sensor responses
  for (int i = 0; i < 4; i++) {
    if (echo_done[i]) {
      noInterrupts();
      unsigned long duration = echo_end[i] - echo_start[i];
      echo_done[i] = false;
      interrupts();
      
      float distance = (duration * SOUND_SPEED) / 2.0;
      
      // Validate reading
      if (distance > 2 && distance < 400) {
        sensor_distances[i] = distance;
      } else {
        sensor_distances[i] = 999.0;  // Invalid
      }
      
      // Print detailed info
      String sensor_names[] = {"Front", "Right", "Back", "Left"};
      Serial.printf("Sensor %d (%s):\n", i+1, sensor_names[i].c_str());
      Serial.printf("  Duration: %lu microseconds\n", duration);
      Serial.printf("  Distance: %.1f cm\n", distance);
      Serial.printf("  Valid: %s\n", (distance > 2 && distance < 400) ? "YES" : "NO");
      
      // Standard format for Python
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(sensor_distances[i]);
      Serial.println(" cm");
    }
  }
  
  // Print status every 5 seconds
  static unsigned long lastStatus = 0;
  if (now - lastStatus >= 5000) {
    lastStatus = now;
    Serial.println("\n=== Current Readings ===");
    String sensor_names[] = {"Front", "Right", "Back", "Left"};
    for (int i = 0; i < 4; i++) {
      Serial.printf("%s: %.1f cm\n", sensor_names[i].c_str(), sensor_distances[i]);
    }
    Serial.println("========================");
    
    // Send position format for Python testing
    Serial.printf("POS:50.0,50.0,0.0|S:%.1f,%.1f,%.1f,%.1f\n", 
                  sensor_distances[0], sensor_distances[1], 
                  sensor_distances[2], sensor_distances[3]);
  }
  
  delay(10);
}