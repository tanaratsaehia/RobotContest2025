#include <WiFi.h>
#include <esp_now.h>
#include <BMI160Gen.h>
#include <ESP32Servo.h>

// Pin definitions
#define TRIG_PIN   32

#define BUTTON_PIN 27
#define BUZZER_PIN 14

#define MOTOR_FRONT_A1 19 // right
#define MOTOR_FRONT_A2 18 // right
#define MOTOR_FRONT_B1 33 // left
#define MOTOR_FRONT_B2 23 // left
#define MOTOR_BACK_A1 4  // right
#define MOTOR_BACK_A2 13 // right
#define MOTOR_BACK_B1 16 // left
#define MOTOR_BACK_B2 17 // left

#define SERVO_PIN1 26
#define SERVO_PIN2 25

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

Servo servo_left;
Servo servo_right;


void move_motor(int action){
  if (action == 0){
    digitalWrite(MOTOR_FRONT_A1, LOW);
    digitalWrite(MOTOR_FRONT_A2, LOW);
    digitalWrite(MOTOR_FRONT_B1, LOW);
    digitalWrite(MOTOR_FRONT_B2, LOW);
    digitalWrite(MOTOR_BACK_A1, LOW);
    digitalWrite(MOTOR_BACK_A2, LOW);
    digitalWrite(MOTOR_BACK_B1, LOW);
    digitalWrite(MOTOR_BACK_B2, LOW);
  }else if (action == 1){
    digitalWrite(MOTOR_FRONT_A1, HIGH);
    digitalWrite(MOTOR_FRONT_A2, LOW);
    digitalWrite(MOTOR_FRONT_B1, LOW);
    digitalWrite(MOTOR_FRONT_B2, HIGH);
    digitalWrite(MOTOR_BACK_A1, HIGH);
    digitalWrite(MOTOR_BACK_A2, LOW);
    digitalWrite(MOTOR_BACK_B1, LOW);
    digitalWrite(MOTOR_BACK_B2, HIGH);
  }else if (action == 2){
    digitalWrite(MOTOR_FRONT_A1, LOW);
    digitalWrite(MOTOR_FRONT_A2, HIGH);
    digitalWrite(MOTOR_FRONT_B1, HIGH);
    digitalWrite(MOTOR_FRONT_B2, LOW);
    digitalWrite(MOTOR_BACK_A1, LOW);
    digitalWrite(MOTOR_BACK_A2, HIGH);
    digitalWrite(MOTOR_BACK_B1, HIGH);
    digitalWrite(MOTOR_BACK_B2, LOW);
  }else{
    Serial.println("Motor wrong action: " + String(action));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  BMI160.begin(BMI160GenClass::I2C_MODE, IMU_I2C_ADDR);
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("IMU DEVICE ID: ");
  Serial.println(dev_id, HEX);
  BMI160.setGyroRange(250);

  servo_left.attach(SERVO_PIN1);
  servo_right.attach(SERVO_PIN2);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT);

  pinMode(MOTOR_FRONT_A1, OUTPUT);
  pinMode(MOTOR_FRONT_A2, OUTPUT);
  pinMode(MOTOR_FRONT_B1, OUTPUT);
  pinMode(MOTOR_FRONT_B2, OUTPUT);
  pinMode(MOTOR_BACK_A1, OUTPUT);
  pinMode(MOTOR_BACK_A2, OUTPUT);
  pinMode(MOTOR_BACK_B1, OUTPUT);
  pinMode(MOTOR_BACK_B2, OUTPUT);

  ultrasonic_begin();
  delay(1000);
  esp_now_begin();
}

void loop() {
  unsigned long now = millis();

  // Trigger pulse every 50 ms
  if (now - lastTriggerTime >= 500) {
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
    }
  }

  // motor testing
  if (!digitalRead(BUTTON_PIN) && now - lastMotorUpdate >= 500){
    lastMotorUpdate = now;
    if (motor_state < 2){
      motor_state += 1;
    }else{
      motor_state = 0;
    }
    Serial.println("Change Motor state to: " + String(motor_state));
    move_motor(motor_state);

    if (servo_state) {
      servo_left.write(180);
      servo_right.write(180);
    }else{
      servo_left.write(0);
      servo_right.write(0);
    }
    servo_state = !servo_state;
  }

  // buzzer testing
  // Serial.println("Button state: " + String(digitalRead(BUTTON_PIN)));
  digitalWrite(BUZZER_PIN, !digitalRead(BUTTON_PIN));

  // test esp-now
  test_send_message(1000);
}
