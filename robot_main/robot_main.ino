#include <WiFi.h>
#include <esp_now.h>
#include <BMI160Gen.h>
#include <ESP32Servo.h>

// ====== Heading-hold (BMI160) ======
static const float GYRO_LSB_PER_DPS = 131.2f;  // BMI160 @ ±250 dps ≈ 131.2 LSB/(°/s)
static const float KP = 1.8f;                  // Proportional gain (tune 0.8–3.0)
static const float KD = 0.0f;                  // Derivative gain (start at 0)
static const uint8_t MAX_PWM = 255;

volatile float yaw_deg = 0.0f;     // integrated heading
volatile float gyro_z_bias = 0.0f; // stationary bias
unsigned long last_imu_us = 0;

bool   straight_active   = false;
float  target_yaw_deg    = 0.0f;
uint8_t base_forward_pwm = 0;
float  prev_err = 0.0f;

// Convert % to 0..255 PWM
static inline uint8_t pct_to_pwm(int p) { p = constrain(p, 0, 100); return (uint8_t)(p * 255 / 100); }


// Pin definitions
#define TRIG_PIN   32
#define BUTTON_PIN 27
#define BUZZER_PIN 14

#define IMU_I2C_ADDR 0x69

// Speed of sound (cm/µs)
#define SOUND_SPEED 0.0343

// Store timing for each sensor
volatile unsigned long echo_start[4] = {0, 0, 0, 0};
volatile unsigned long echo_end[4]   = {0, 0, 0, 0};
volatile bool echo_done[4]           = {false, false, false, false};

bool servo_state = false;
int motor_state = 0;

// Last measurement times
unsigned long lastTriggerTime = 0;
unsigned long lastMotorUpdate = 0;

unsigned long lastCommandTime = 0;

Servo servo_left;
Servo servo_right;

void calibrate_gyro_bias(uint16_t samples = 800) {
  // Keep robot still during this!
  delay(200);
  long sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    int gx, gy, gz;
    BMI160.readGyro(gx, gy, gz);
    sum += gz;
    delay(2); // ~500 Hz sampling during calibration
  }
  gyro_z_bias = sum / (float)samples;
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  BMI160.begin(BMI160GenClass::I2C_MODE, IMU_I2C_ADDR);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("IMU DEVICE ID: ");
  Serial.println(dev_id, HEX);
  BMI160.setGyroRange(250);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT);

  motor_begin();
  ultrasonic_begin();
  esp_now_begin();

  calibrate_gyro_bias();      // <— add this
  last_imu_us = micros();     // initialize timer for integration
}

void loop() {
  unsigned long now = millis();
  int button_status = digitalRead(BUTTON_PIN);

  // Trigger pulse every 50 ms
  if (now - lastTriggerTime >= 100) {
    lastTriggerTime = now;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    Serial.println("Button state: " + String(digitalRead(BUTTON_PIN)));

    int gx, gy, gz;
    int ax, ay, az;
    BMI160.readGyro(gx, gy, gz);
    BMI160.readAccelerometer(ax, ay, az);
    Serial.print("gyro:\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);

    Serial.print("\tAccel:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.println("\n----------------------------------------------------\n");
  }

  // Process completed measurements
  for (int i = 0; i < 4; i++) {
    if (echo_done[i]) {
      noInterrupts();
      unsigned long duration = echo_end[i] - echo_start[i];
      echo_done[i] = false;
      interrupts();

      float distance_cm = (duration * SOUND_SPEED) / 2.0;
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(distance_cm);
      Serial.println(" cm");
      String msg = "Sensor " + String(i + 1) + ": " + String(distance_cm, 1) + " cm";
      send_message(msg.c_str());
    }
  }

  // motor testing
  if (!button_status && now - lastMotorUpdate >= 500){
    lastMotorUpdate = now;
    if (servo_state) {
      servo_left.write(180);
      servo_right.write(180);
    }else{
      servo_left.write(0);
      servo_right.write(0);
    }
    servo_state = !servo_state;
  }
  digitalWrite(BUZZER_PIN, !button_status);

  // imu testing
  straight_update();  // keeps yaw updated and trims motors when active
}
