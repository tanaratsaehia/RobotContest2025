
// Update yaw from gyro (call as often as you can)
inline void imu_update_yaw() {
  int gx, gy, gz;
  BMI160.readGyro(gx, gy, gz);

  unsigned long now = micros();
  if (last_imu_us == 0) { last_imu_us = now; return; }
  float dt = (now - last_imu_us) * 1e-6f;  // seconds
  last_imu_us = now;

  // Convert to deg/s; flip sign here if your yaw grows in the opposite direction
  float rate_dps = (gz - gyro_z_bias) / GYRO_LSB_PER_DPS;
  yaw_deg += rate_dps * dt;
}

void straight_start(int speed_percent) {
  base_forward_pwm = pct_to_pwm(speed_percent);
  target_yaw_deg   = yaw_deg;   // lock current heading
  prev_err         = 0.0f;
  straight_active  = true;
}

void straight_stop() {
  straight_active = false;
  stop_motor();
}

void straight_update() {
  if (!straight_active) return;

  imu_update_yaw();  // keep yaw fresh

  // Heading error (deg). If it steers the wrong way, change to (yaw_deg - target_yaw_deg)
  float err  = (target_yaw_deg - yaw_deg);
  float derr = (err - prev_err); // crude derivative on sample
  prev_err = err;

  // Controller output is a PWM delta we add/subtract per side
  float corr = KP * err + KD * derr;

  // Clamp how much we can trim each side (protects from overcorrection)
  float max_delta = base_forward_pwm * 0.6f; // allow up to ±60% trim
  corr = constrain(corr, -max_delta, max_delta);

  int left_pwm  = constrain((int)base_forward_pwm + (int)corr, 0, MAX_PWM);
  int right_pwm = constrain((int)base_forward_pwm - (int)corr, 0, MAX_PWM);

  set_forward_side_pwm((uint8_t)left_pwm, (uint8_t)right_pwm);
}
