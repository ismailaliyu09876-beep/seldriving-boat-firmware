/*
 * ============================================================
 * AUTONOMOUS HEADING-CONTROLLED BOAT
 * FINAL EXHIBITION / VALIDATION VERSION
 * ============================================================
 *
 * ESP32 + MPU6050 + QMC5883P + HC-SR04 + L298N
 *
 * IMPORTANT:
 * 1. QMC5883P and MPU6050 axes must be mounted consistently.
 * 2. Battery monitoring REQUIRES an external voltage divider on
 *    GPIO34 - see comments at BATTERY_ADC_PIN below.
 *
 * WEB: 
 *   Connect to WiFi AP: ASV
 *   Password: U21CO2015
 *   Open: http://192.168.4.1
 * SERIAL: 115200 baud
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <math.h>

// ============================================================
// QMC5883P NATIVE DRIVER
// ============================================================
#define QMC_ADDR        0x2C
#define QMC_REG_DATA    0x01
#define QMC_REG_CTRL1   0x0A
#define QMC_REG_CTRL2   0x0B
#define QMC_REG_MAGIC1  0x0D
#define QMC_REG_MAGIC2  0x29
#define QMC_REG_CHIPID  0xFF

const float QMC_UT_PER_LSB = 200.0f / 32768.0f;

float lastMagRawX_wd = 0.0f, lastMagRawY_wd = 0.0f, lastMagRawZ_wd = 0.0f;
unsigned long lastMagChangeTime = 0;
const unsigned long MAG_STALE_TIMEOUT_MS = 3000;
bool magWatchdogPrimed = false;

bool qmcWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

uint8_t qmcReadReg(uint8_t reg, bool &ok) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(reg);
  ok = (Wire.endTransmission(false) == 0);
  if (!ok) return 0;
  uint8_t n = Wire.requestFrom((int)QMC_ADDR, 1);
  if (n != 1) { ok = false; return 0; }
  return Wire.read();
}

bool qmcInit() {
  bool ok = true;
  ok &= qmcWriteReg(QMC_REG_MAGIC1, 0x40);
  ok &= qmcWriteReg(QMC_REG_MAGIC2, 0x06);
  ok &= qmcWriteReg(QMC_REG_CTRL1, 0xCF);
  ok &= qmcWriteReg(QMC_REG_CTRL2, 0x0C);
  ok &= qmcWriteReg(QMC_REG_CTRL1, 0x09);
  delay(10);
  bool idOk = false;
  uint8_t chipId = qmcReadReg(QMC_REG_CHIPID, idOk);
  if (idOk) {
    Serial.printf("[INFO] QMC5883P chip ID register = 0x%02X (expect 0x80)\n", chipId);
  } else {
    Serial.println("[INFO] QMC5883P chip ID register unreadable (non-fatal)");
  }
  return ok;
}

bool qmcRead(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(QMC_REG_DATA);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom((int)QMC_ADDR, 6);
  if (n != 6) return false;
  uint8_t xl = Wire.read(); uint8_t xh = Wire.read();
  uint8_t yl = Wire.read(); uint8_t yh = Wire.read();
  uint8_t zl = Wire.read(); uint8_t zh = Wire.read();
  x = (int16_t)((xh << 8) | xl);
  y = (int16_t)((yh << 8) | yl);
  z = (int16_t)((zh << 8) | zl);
  return true;
}

// ============================================================
// MPU6050 NATIVE DRIVER
// ============================================================
#define MPU_ADDR              0x68
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_CONFIG        0x1A
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_WHO_AM_I      0x75

const float MPU_ACCEL_LSB_PER_G   = 16384.0f;
const float MPU_GYRO_LSB_PER_DPS  = 131.0f;
const float DEG_TO_RAD_F          = 0.0174532925f;

bool mpuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

uint8_t mpuReadReg(uint8_t reg, bool &ok) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  ok = (Wire.endTransmission(false) == 0);
  if (!ok) return 0;
  uint8_t n = Wire.requestFrom((int)MPU_ADDR, 1);
  if (n != 1) { ok = false; return 0; }
  return Wire.read();
}

bool mpuInit() {
  bool ok = true;
  ok &= mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00);
  delay(10);
  ok &= mpuWriteReg(MPU_REG_CONFIG, 0x04);
  ok &= mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);
  ok &= mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00);
  bool idOk = false;
  uint8_t whoAmI = mpuReadReg(MPU_REG_WHO_AM_I, idOk);
  if (idOk) {
    Serial.printf("[INFO] MPU6050 WHO_AM_I register = 0x%02X (expect 0x68)\n", whoAmI);
  } else {
    Serial.println("[INFO] MPU6050 WHO_AM_I register unreadable (non-fatal)");
  }
  return ok;
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom((int)MPU_ADDR, 14);
  if (n != 14) return false;
  uint8_t buf[14];
  for (int i = 0; i < 14; i++) buf[i] = Wire.read();
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8]  << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
  return true;
}

// ===================== WiFi =====================
const char* ssid = "ASV";
const char* wifiPassword = "U21CO2015";
const char* hostname = "myboat";

// ===================== L298N =====================
// Left motor pins unchanged.
#define ENA 14
#define IN1 27
#define IN2 26

// Right motor pins REASSIGNED (previously ENB=25, IN3=13, IN4=33)
// to avoid the DAC-shared pins (25/26) and the JTAG/strapping group
// (12-15). GPIO4/32/19 are general-purpose, PWM-capable, not
// input-only, not strapping pins, and free on every ESP32 module
// variant (WROOM or WROVER).
#define ENB 4
#define IN3 32
#define IN4 19

// ===================== HC-SR04 ===================
#define TRIG_PIN 5
#define ECHO_PIN 18

// ===================== Battery Monitoring ===================
// Requires an external 2-resistor voltage divider from battery+ to
// GND, midpoint wired to this pin. Raw battery voltage direct to a
// GPIO WILL damage it. GPIO34 is input-only, ADC1 (WiFi-safe).
#define BATTERY_ADC_PIN 34

const float BATTERY_MAX_VOLTAGE = 8.4f;
const float BATTERY_MIN_VOLTAGE = 6.4f;
const float BATTERY_DIVIDER_RATIO = 3.13f;
const float ADC_REF_VOLTAGE = 3.3f;
const int ADC_RESOLUTION = 4095;

// ===================== PWM =======================
#define PWM_FREQ 5000
#define PWM_RES 8

// ===================== Commands =================
#define CMD_FORWARD      'F'
#define CMD_BACKWARD     'B'
#define CMD_LEFT         'L'
#define CMD_RIGHT        'R'
#define CMD_PIVOT_LEFT   'P'
#define CMD_PIVOT_RIGHT  'Q'
#define CMD_STOP         'S'
#define CMD_AUTO         'A'
#define CMD_MANUAL       'M'
#define CMD_SPEED_UP     '+'
#define CMD_SPEED_DOWN   '-'

// ===================== Sensors ===================
bool imuOK = false;
bool magOK = false;
uint8_t mpuAddress = 0x00;

WebServer server(80);

// ===================== Magnetometer Calibration =====================
float MAG_OFFSET_X = -17.03f;
float MAG_OFFSET_Y = -30.85f;
float MAG_OFFSET_Z = 2.22f;

float MAG_SCALE_X = 1.0f;
float MAG_SCALE_Y = 1.0f;
float MAG_SCALE_Z = 1.0f;

// ===================== Madgwick ==================
float beta = 0.08f;

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;
float headingDeg = 0.0f;
float inputHeading = 0.0f;

float ax_g = 0, ay_g = 0, az_g = 0;
float gx_dps = 0, gy_dps = 0, gz_dps = 0;

float magRawX = 0, magRawY = 0, magRawZ = 0;
float magX = 0, magY = 0, magZ = 0;
float magMagnitude = 0;

// ===================== Control ===================
int baseSpeed = 150;
float setpoint = 90.0f;

float pidError = 0;
float pidOutput = 0;
float integral = 0;
float derivative = 0;

float Kp = 1.0f;
float Ki = 0.8f;
float Kd = 0.1f;

float lastError = 0;
unsigned long lastPIDTime = 0;

int leftPWM = 0;
int rightPWM = 0;

bool autoMode = false;

// ===================== Obstacle ==================
const float OBSTACLE_THRESHOLD = 30.0f;
float lastDistance = -1.0f;
bool obstacleDetected = false;

// ===================== Battery ==================
float batteryVoltage = 0.0f;
float batteryPercent = 0.0f;

// ===================== Timing ====================
unsigned long lastIMUTime = 0;
unsigned long lastTelemetry = 0;
unsigned long lastObstacleCheck = 0;
unsigned long lastBatteryCheck = 0;

// ===================== Motor Kickstart ====================
const unsigned long MOTOR_KICK_DURATION_MS = 150;
const int MOTOR_KICK_PWM = 255;

bool leftKicking = false;
unsigned long leftKickEndTime = 0;
int leftKickTarget = 0;

bool rightKicking = false;
unsigned long rightKickEndTime = 0;
int rightKickTarget = 0;

// ===================== Motor Balance Compensation ====================
const int LEFT_MOTOR_OFFSET = 0;
const int RIGHT_MOTOR_OFFSET = 45; // starting estimate - tune on water

// ============================================================
// Utility
// ============================================================
float wrap360(float angle) {
  while (angle < 0) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// ============================================================
// Motor Control
// ============================================================
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    leftKicking = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
    leftPWM = 0;
    return;
  }

  bool wasStopped = (leftPWM == 0);

  if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    speed = -speed;
  }

  speed = constrain(speed, 0, 255);

  int compensatedSpeed = speed + LEFT_MOTOR_OFFSET;
  compensatedSpeed = constrain(compensatedSpeed, 0, 255);

  if (wasStopped && compensatedSpeed > 0) {
    leftKicking = true;
    leftKickEndTime = millis() + MOTOR_KICK_DURATION_MS;
    leftKickTarget = compensatedSpeed;
    ledcWrite(ENA, MOTOR_KICK_PWM);
  } else {
    leftKicking = false;
    ledcWrite(ENA, compensatedSpeed);
  }

  leftPWM = (digitalRead(IN2) == HIGH) ? compensatedSpeed : -compensatedSpeed;
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    rightKicking = false;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, 0);
    rightPWM = 0;
    return;
  }

  bool wasStopped = (rightPWM == 0);

  if (speed > 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    speed = -speed;
  }

  speed = constrain(speed, 0, 255);

  int compensatedSpeed = speed + RIGHT_MOTOR_OFFSET;
  compensatedSpeed = constrain(compensatedSpeed, 0, 255);

  if (wasStopped && compensatedSpeed > 0) {
    rightKicking = true;
    rightKickEndTime = millis() + MOTOR_KICK_DURATION_MS;
    rightKickTarget = compensatedSpeed;
    ledcWrite(ENB, MOTOR_KICK_PWM);
  } else {
    rightKicking = false;
    ledcWrite(ENB, compensatedSpeed);
  }

  rightPWM = (digitalRead(IN4) == HIGH) ? compensatedSpeed : -compensatedSpeed;
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

// ============================================================
// Magnetometer calibration
// ============================================================
void applyMagCalibration() {
  magX = (magRawX - MAG_OFFSET_X) * MAG_SCALE_X;
  magY = (magRawY - MAG_OFFSET_Y) * MAG_SCALE_Y;
  magZ = (magRawZ - MAG_OFFSET_Z) * MAG_SCALE_Z;
  magMagnitude = sqrtf(magX * magX + magY * magY + magZ * magZ);
}

// ============================================================
// Madgwick AHRS
// ============================================================
void madgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float dt) {

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;

  float hx, hy, _2bx, _2bz;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx;
  float _2q0 = 2.0f * q0;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q0q2 = 2.0f * q0 * q2;
  float _2q2q3 = 2.0f * q2 * q3;
  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  recipNorm = sqrtf(ax * ax + ay * ay + az * az);
  if (recipNorm < 0.000001f) return;
  recipNorm = 1.0f / recipNorm;
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

  recipNorm = sqrtf(mx * mx + my * my + mz * mz);
  if (recipNorm < 0.000001f) return;
  recipNorm = 1.0f / recipNorm;
  mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

  _2q0mx = 2.0f * q0 * mx;
  _2q0my = 2.0f * q0 * my;
  _2q0mz = 2.0f * q0 * mz;
  _2q1mx = 2.0f * q1 * mx;

  hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2
     + mx * q1q1 + _2q1 * my * q2
     + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;

  hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1
     + _2q1mx * q2 - my * q1q1
     + my * q2q2 + _2q2 * mz * q3 - my * q3q3;

  _2bx = sqrtf(hx * hx + hy * hy);

  _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0
       + _2q1mx * q3 - mz * q1q1
       + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;

  s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
       - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3)
       + _2bz * (q1q3 - q0q2) - mx)
       + (-_2bx * q3 + _2bz * q1)
       * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
       + _2bx * q2
       * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

  s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
       - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
       + _2bz * q3
       * (_2bx * (0.5f - q2q2 - q3q3)
       + _2bz * (q1q3 - q0q2) - mx)
       + (_2bx * q2 + _2bz * q0)
       * (_2bx * (q1q2 - q0q3)
       + _2bz * (q0q1 + q2q3) - my)
       + (_2bx * q3 - 4.0f * _2bz * q1)
       * (_2bx * (q0q2 + q1q3)
       + _2bz * (0.5f - q1q1 - q2q2) - mz);

  s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
       - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
       + (-4.0f * _2bx * q2 - 2.0f * _2bz * q0)
       * (_2bx * (0.5f - q2q2 - q3q3)
       + _2bz * (q1q3 - q0q2) - mx)
       + (_2bx * q1 + _2bz * q3)
       * (_2bx * (q1q2 - q0q3)
       + _2bz * (q0q1 + q2q3) - my)
       + (_2bx * q0 - 4.0f * _2bz * q2)
       * (_2bx * (q0q2 + q1q3)
       + _2bz * (0.5f - q1q1 - q2q2) - mz);

  s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
       + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
       + (-4.0f * _2bx * q3 + 2.0f * _2bz * q1)
       * (_2bx * (0.5f - q2q2 - q3q3)
       + _2bz * (q1q3 - q0q2) - mx)
       + (-_2bx * q0 + _2bz * q2)
       * (_2bx * (q1q2 - q0q3)
       + _2bz * (q0q1 + q2q3) - my)
       + _2bx * q1
       * (_2bx * (q0q2 + q1q3)
       + _2bz * (0.5f - q1q1 - q2q2) - mz);

  recipNorm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
  if (recipNorm > 0.000001f) {
    recipNorm = 1.0f / recipNorm;
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
  }

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (recipNorm < 0.000001f) {
    q0 = 1; q1 = q2 = q3 = 0;
    return;
  }

  recipNorm = 1.0f / recipNorm;
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// ============================================================
// Orientation
// ============================================================
void updateOrientation() {
  if (!imuOK || !magOK) return;

  int16_t rax = 0, ray = 0, raz = 0, rgx = 0, rgy = 0, rgz = 0;
  if (mpuReadRaw(rax, ray, raz, rgx, rgy, rgz)) {
    ax_g = rax / MPU_ACCEL_LSB_PER_G;
    ay_g = ray / MPU_ACCEL_LSB_PER_G;
    az_g = raz / MPU_ACCEL_LSB_PER_G;
    gx_dps = rgx / MPU_GYRO_LSB_PER_DPS;
    gy_dps = rgy / MPU_GYRO_LSB_PER_DPS;
    gz_dps = rgz / MPU_GYRO_LSB_PER_DPS;
  }

  int16_t qx = 0, qy = 0, qz = 0;
  if (qmcRead(qx, qy, qz)) {
    magRawX = qx * QMC_UT_PER_LSB;
    magRawY = qy * QMC_UT_PER_LSB;
    magRawZ = qz * QMC_UT_PER_LSB;
  }

  if (!magWatchdogPrimed) {
    lastMagRawX_wd = magRawX;
    lastMagRawY_wd = magRawY;
    lastMagRawZ_wd = magRawZ;
    lastMagChangeTime = millis();
    magWatchdogPrimed = true;
  } else if (magRawX != lastMagRawX_wd ||
             magRawY != lastMagRawY_wd ||
             magRawZ != lastMagRawZ_wd) {
    lastMagRawX_wd = magRawX;
    lastMagRawY_wd = magRawY;
    lastMagRawZ_wd = magRawZ;
    lastMagChangeTime = millis();
  } else if (millis() - lastMagChangeTime > MAG_STALE_TIMEOUT_MS) {
    Serial.println("[WARN] Magnetometer reading frozen - reinitializing QMC5883P");
    qmcInit();
    lastMagChangeTime = millis();
  }

  applyMagCalibration();

  unsigned long now = micros();
  float dt = (now - lastIMUTime) / 1000000.0f;
  lastIMUTime = now;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  float gx_rad = gx_dps * DEG_TO_RAD_F;
  float gy_rad = gy_dps * DEG_TO_RAD_F;
  float gz_rad = gz_dps * DEG_TO_RAD_F;

  madgwickUpdate(gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, magX, magY, magZ, dt);

  rollDeg = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / PI;

  float pitchArg = 2.0f * (q0*q2 - q3*q1);
  pitchArg = constrain(pitchArg, -1.0f, 1.0f);
  pitchDeg = asinf(pitchArg) * 180.0f / PI;

  yawDeg = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / PI;

  headingDeg = wrap360(yawDeg);
  inputHeading = headingDeg;
}

// ============================================================
// Ultrasonic
// ============================================================
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0f;
  return duration / 58.2f;
}

void updateObstacle() {
  float d = readDistance();
  lastDistance = d;
  bool newObstacle = (d > 0 && d < OBSTACLE_THRESHOLD);
  if (newObstacle && !obstacleDetected) {
    stopMotors();
    Serial.printf("[OBSTACLE] STOP - %.1f cm\n", d);
  }
  obstacleDetected = newObstacle;
}

// ============================================================
// Battery
// ============================================================
void updateBatteryStatus() {
  int raw = analogRead(BATTERY_ADC_PIN);
  float pinVoltage = (raw / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE;
  batteryVoltage = pinVoltage * BATTERY_DIVIDER_RATIO;
  float pct = (batteryVoltage - BATTERY_MIN_VOLTAGE) /
              (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100.0f;
  batteryPercent = constrain(pct, 0.0f, 100.0f);
}

// ============================================================
// PID
// ============================================================
void resetPID() {
  integral = 0;
  derivative = 0;
  lastError = 0;
  pidOutput = 0;
  lastPIDTime = millis();
}

void runPID() {
  if (obstacleDetected) {
    stopMotors();
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  if (dt < 0.005f) return;
  if (dt > 0.2f) dt = 0.02f;
  lastPIDTime = now;

  pidError = wrap180(setpoint - inputHeading);
  float absError = fabsf(pidError);

  if (absError > 30.0f) { Kp = 3.0f; Ki = 0.2f; Kd = 0.3f; }
  else if (absError > 10.0f) { Kp = 2.0f; Ki = 0.5f; Kd = 0.2f; }
  else { Kp = 1.0f; Ki = 0.8f; Kd = 0.1f; }

  if (absError < 0.5f) {
    integral = 0;
  } else {
    integral += pidError * dt;
    integral = constrain(integral, -100.0f, 100.0f);
  }

  derivative = (pidError - lastError) / dt;

  float P = Kp * pidError;
  float I = Ki * integral;
  float D = Kd * derivative;

  pidOutput = P + I + D;
  pidOutput = constrain(pidOutput, -100.0f, 100.0f);
  lastError = pidError;

  int left = constrain((int)roundf(baseSpeed + pidOutput), 0, 255);
  int right = constrain((int)roundf(baseSpeed - pidOutput), 0, 255);

  setLeftMotor(left);
  setRightMotor(right);
}

// ============================================================
// Commands
// ============================================================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = cmd.charAt(0);

  if (c == CMD_SPEED_UP) { baseSpeed = constrain(baseSpeed + 10, 50, 255); return; }
  if (c == CMD_SPEED_DOWN) { baseSpeed = constrain(baseSpeed - 10, 50, 255); return; }

  if (c == CMD_AUTO || c == 'a') { autoMode = true; resetPID(); return; }

  if (c == CMD_MANUAL || c == 'm') {
    autoMode = false;
    stopMotors();
    resetPID();
    return;
  }

  if ((c == 's') && cmd.length() > 1) {
    float val = cmd.substring(1).toFloat();
    if (val >= 0 && val < 360) {
      setpoint = val;
      resetPID();
      Serial.printf("[SETPOINT] %.1f deg\n", setpoint);
    }
    return;
  }

  if (autoMode) return;

  switch (c) {
    case CMD_FORWARD:
    case 'f':
      if (obstacleDetected) { stopMotors(); return; }
      setLeftMotor(baseSpeed);
      setRightMotor(baseSpeed);
      break;

    case CMD_BACKWARD:
    case 'b':
      setLeftMotor(-baseSpeed);
      setRightMotor(-baseSpeed);
      break;

    case CMD_LEFT:
    case 'l':
    case CMD_PIVOT_LEFT:
    case 'p':
      setLeftMotor(baseSpeed);
      setRightMotor(0);
      break;

    case CMD_RIGHT:
    case 'r':
    case CMD_PIVOT_RIGHT:
    case 'q':
      setLeftMotor(0);
      setRightMotor(baseSpeed);
      break;

    case CMD_STOP:
      stopMotors();
      break;

    default:
      break;
  }
}

// ============================================================
// Telemetry JSON
// ============================================================
String makeTelemetryJSON() {
  String json = "{";
  json += "\"imuOK\":"; json += imuOK ? "true" : "false";
  json += ",\"mpuAddress\":"; json += String((unsigned int)mpuAddress);
  json += ",\"magOK\":"; json += magOK ? "true" : "false";
  json += ",\"madgwick\":"; json += (imuOK && magOK) ? "true" : "false";
  json += ",\"mode\":\""; json += autoMode ? "AUTO" : "MANUAL"; json += "\"";
  json += ",\"heading\":"; json += String(inputHeading, 2);
  json += ",\"setpoint\":"; json += String(setpoint, 2);
  json += ",\"error\":"; json += String(pidError, 2);
  json += ",\"roll\":"; json += String(rollDeg, 2);
  json += ",\"pitch\":"; json += String(pitchDeg, 2);
  json += ",\"yaw\":"; json += String(yawDeg, 2);
  json += ",\"ax\":"; json += String(ax_g, 3);
  json += ",\"ay\":"; json += String(ay_g, 3);
  json += ",\"az\":"; json += String(az_g, 3);
  json += ",\"gx\":"; json += String(gx_dps, 2);
  json += ",\"gy\":"; json += String(gy_dps, 2);
  json += ",\"gz\":"; json += String(gz_dps, 2);
  json += ",\"mxRaw\":"; json += String(magRawX, 2);
  json += ",\"myRaw\":"; json += String(magRawY, 2);
  json += ",\"mzRaw\":"; json += String(magRawZ, 2);
  json += ",\"mx\":"; json += String(magX, 2);
  json += ",\"my\":"; json += String(magY, 2);
  json += ",\"mz\":"; json += String(magZ, 2);
  json += ",\"magField\":"; json += String(magMagnitude, 2);
  json += ",\"kp\":"; json += String(Kp, 2);
  json += ",\"ki\":"; json += String(Ki, 2);
  json += ",\"kd\":"; json += String(Kd, 2);
  json += ",\"pid\":"; json += String(pidOutput, 2);
  json += ",\"leftPWM\":"; json += String(leftPWM);
  json += ",\"rightPWM\":"; json += String(rightPWM);
  json += ",\"speed\":"; json += String(baseSpeed);
  json += ",\"distance\":"; json += String(lastDistance, 1);
  json += ",\"obstacle\":"; json += obstacleDetected ? "true" : "false";
  json += ",\"beta\":"; json += String(beta, 3);
  json += ",\"magOffsetX\":"; json += String(MAG_OFFSET_X, 2);
  json += ",\"magOffsetY\":"; json += String(MAG_OFFSET_Y, 2);
  json += ",\"magOffsetZ\":"; json += String(MAG_OFFSET_Z, 2);
  json += ",\"batteryVoltage\":"; json += String(batteryVoltage, 2);
  json += ",\"batteryPercent\":"; json += String(batteryPercent, 1);
  json += "}";
  return json;
}

// ============================================================
// Serial Exhibition Telemetry
// ============================================================
void printTelemetry() {
  Serial.println();
  Serial.println("============================================================");
  Serial.println("                 AUTONOMOUS BOAT TELEMETRY");
  Serial.println("============================================================");

  Serial.printf("SYSTEM     IMU:%s  MAG:%s  MADGWICK:%s  MODE:%s\n",
                imuOK ? "OK" : "FAIL",
                magOK ? "OK" : "FAIL",
                (imuOK && magOK) ? "ACTIVE" : "OFF",
                autoMode ? "AUTO" : "MANUAL");

  Serial.println();
  Serial.println("ORIENTATION");
  Serial.printf("  Roll       : %7.2f deg\n", rollDeg);
  Serial.printf("  Pitch      : %7.2f deg\n", pitchDeg);
  Serial.printf("  Yaw        : %7.2f deg\n", yawDeg);
  Serial.printf("  Heading    : %7.2f deg\n", inputHeading);

  Serial.println();
  Serial.println("ACCELEROMETER (g)");
  Serial.printf("  X          : %7.3f\n", ax_g);
  Serial.printf("  Y          : %7.3f\n", ay_g);
  Serial.printf("  Z          : %7.3f\n", az_g);

  Serial.println();
  Serial.println("GYROSCOPE (deg/s)");
  Serial.printf("  X          : %7.2f\n", gx_dps);
  Serial.printf("  Y          : %7.2f\n", gy_dps);
  Serial.printf("  Z          : %7.2f\n", gz_dps);

  Serial.println();
  Serial.println("MAGNETOMETER (uT)");
  Serial.printf("  Raw X      : %7.2f\n", magRawX);
  Serial.printf("  Raw Y      : %7.2f\n", magRawY);
  Serial.printf("  Raw Z      : %7.2f\n", magRawZ);
  Serial.printf("  Corr X     : %7.2f\n", magX);
  Serial.printf("  Corr Y     : %7.2f\n", magY);
  Serial.printf("  Corr Z     : %7.2f\n", magZ);
  Serial.printf("  |B|        : %7.2f uT\n", magMagnitude);

  Serial.println();
  Serial.println("HEADING CONTROL");
  Serial.printf("  Setpoint   : %7.2f deg\n", setpoint);
  Serial.printf("  Error      : %7.2f deg\n", pidError);
  Serial.printf("  Kp Ki Kd   : %.2f  %.2f  %.2f\n", Kp, Ki, Kd);
  Serial.printf("  PID Output : %7.2f\n", pidOutput);

  Serial.println();
  Serial.println("ACTUATORS");
  Serial.printf("  Base Speed : %d\n", baseSpeed);
  Serial.printf("  Left PWM   : %d\n", leftPWM);
  Serial.printf("  Right PWM  : %d\n", rightPWM);

  Serial.println();
  Serial.println("OBSTACLE");
  Serial.printf("  Distance   : %.1f cm\n", lastDistance);
  Serial.printf("  Status     : %s\n", obstacleDetected ? "STOP" : "CLEAR");

  Serial.println();
  Serial.println("BATTERY");
  Serial.printf("  Voltage    : %.2f V\n", batteryVoltage);
  Serial.printf("  Percent    : %.1f %%\n", batteryPercent);

  Serial.println("============================================================");
}

// ============================================================
// Web Dashboard
// ============================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ASV Autonomous Boat</title>
<style>
body{font-family:Arial,sans-serif;margin:0;background:#101820;color:#eef2f3}
.wrap{max-width:1000px;margin:auto;padding:14px}
h1{text-align:center;margin:8px 0}
.sub{text-align:center;color:#aab6bd;margin-bottom:15px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
.card{background:#1b2730;border-radius:12px;padding:14px;box-shadow:0 3px 12px #0005}
.card h2{font-size:17px;margin:0 0 10px}
.big{font-size:34px;font-weight:bold;text-align:center}
.row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #ffffff12}
.ok{color:#57e389}.bad{color:#ff6868}.warn{color:#ffd166}
button{font-size:16px;padding:11px;margin:4px;border:0;border-radius:8px;cursor:pointer}
.ctrl{background:#2f81f7;color:white}.stop{background:#e63946;color:white}
.auto{background:#1f9d55;color:white}.manual{background:#6c757d;color:white}
input{padding:10px;width:70px;border-radius:6px;border:0}
.bar{height:13px;background:#303b43;border-radius:8px;overflow:hidden}
.fill{height:100%;background:#2f81f7;width:0}
#compass{width:170px;height:170px;border:3px solid #8fa3ad;border-radius:50%;
margin:10px auto;position:relative;display:flex;align-items:center;justify-content:center}
#needle{position:absolute;width:4px;height:65px;background:#ff4d4d;top:20px;left:83px;
transform-origin:50% 65px;border-radius:4px}
.small{font-size:12px;color:#aab6bd}
</style>
</head>
<body>
<div class="wrap">
<h1>Autonomous Heading-Controlled Boat</h1>
<div class="sub">U21C02015 &bull; ESP32 &bull; Madgwick AHRS &bull; Gain-Scheduled PID &bull; Differential Thrust</div>

<div class="grid">

<div class="card">
<h2>System Health</h2>
<div class="row"><span>MPU6050</span><b id="imu">--</b></div>
<div class="row"><span>QMC5883P</span><b id="mag">--</b></div>
<div class="row"><span>Madgwick</span><b id="mad">--</b></div>
<div class="row"><span>Mode</span><b id="mode">--</b></div>
</div>

<div class="card">
<h2>Heading</h2>
<div id="compass"><div id="needle"></div><b>N<br><span id="head">--</span>&deg;</b></div>
<div class="row"><span>Setpoint</span><b id="sp">--</b></div>
<div class="row"><span>Error</span><b id="err">--</b></div>
</div>

<div class="card">
<h2>Attitude</h2>
<div class="row"><span>Roll</span><b id="roll">--</b></div>
<div class="row"><span>Pitch</span><b id="pitch">--</b></div>
<div class="row"><span>Yaw</span><b id="yaw">--</b></div>
</div>

<div class="card">
<h2>Magnetometer</h2>
<div class="row"><span>Raw X</span><b id="mxr">--</b></div>
<div class="row"><span>Raw Y</span><b id="myr">--</b></div>
<div class="row"><span>Raw Z</span><b id="mzr">--</b></div>
<div class="row"><span>|B|</span><b id="mf">--</b></div>
<div class="small">Use |B| to observe magnetic interference when motors start.</div>
</div>

<div class="card">
<h2>IMU</h2>
<div class="row"><span>Accel X/Y/Z</span><b id="acc">--</b></div>
<div class="row"><span>Gyro X/Y/Z</span><b id="gyro">--</b></div>
</div>

<div class="card">
<h2>PID Controller</h2>
<div class="row"><span>Kp</span><b id="kp">--</b></div>
<div class="row"><span>Ki</span><b id="ki">--</b></div>
<div class="row"><span>Kd</span><b id="kd">--</b></div>
<div class="row"><span>PID Output</span><b id="pid">--</b></div>
</div>

<div class="card">
<h2>Differential Thrust</h2>
<div>LEFT <b id="lp">--</b></div>
<div class="bar"><div id="lb" class="fill"></div></div>
<br>
<div>RIGHT <b id="rp">--</b></div>
<div class="bar"><div id="rb" class="fill"></div></div>
</div>

<div class="card">
<h2>Obstacle</h2>
<div class="big" id="dist">--</div>
<div style="text-align:center">cm</div>
<div class="big" id="obs">--</div>
</div>

<div class="card">
<h2>Mag Offsets</h2>
<div class="row"><span>Offset X</span><b id="offX">--</b></div>
<div class="row"><span>Offset Y</span><b id="offY">--</b></div>
<div class="row"><span>Offset Z</span><b id="offZ">--</b></div>
<div class="small">Fixed hard-iron offsets from manual calibration.</div>
</div>

<div class="card">
<h2>Battery</h2>
<div class="big" id="battPct">--</div>
<div style="text-align:center">percent</div>
<div class="row"><span>Voltage</span><b id="battV">--</b></div>
<div class="small">Requires an external voltage divider on GPIO34 - see firmware comments.</div>
</div>

</div>

<div class="card" style="margin-top:12px;text-align:center">
<h2>Control</h2>
<button class="ctrl" onclick="cmd('F')">FWD</button>
<button class="ctrl" onclick="cmd('B')">REV</button>
<button class="ctrl" onclick="cmd('L')">LEFT</button>
<button class="ctrl" onclick="cmd('R')">RIGHT</button>
<button class="ctrl" onclick="cmd('P')">PIVOT L</button>
<button class="ctrl" onclick="cmd('Q')">PIVOT R</button>
<button class="stop" onclick="cmd('S')">STOP</button>
<br>
<button class="auto" onclick="cmd('A')">AUTO</button>
<button class="manual" onclick="cmd('M')">MANUAL</button>
<br>
<input id="headingSet" type="number" min="0" max="359" value="90">
<button class="auto" onclick="cmd('s'+document.getElementById('headingSet').value)">SET HEADING</button>
<br>
<button onclick="cmd('+')">SPEED +</button>
<button onclick="cmd('-')">SPEED -</button>
<div class="small">Live telemetry refresh: 500 ms</div>
</div>
</div>

<script>
function cmd(c){fetch('/cmd?value='+encodeURIComponent(c)).then(()=>update())}

function setText(id,v){
  const e=document.getElementById(id);
  if(e)e.textContent=v;
}

function health(id,ok){
  const e=document.getElementById(id);
  e.textContent=ok?'OK':'FAIL';
  e.className=ok?'ok':'bad';
}

async function update(){
 try{
  const r=await fetch('/telemetry',{cache:'no-store'});
  const d=await r.json();

  health('imu',d.imuOK);
  health('mag',d.magOK);
  health('mad',d.madgwick);

  setText('mode',d.mode);
  setText('head',d.heading.toFixed(1));
  setText('sp',d.setpoint.toFixed(1));
  setText('err',d.error.toFixed(1));
  setText('roll',d.roll.toFixed(1)+'\u00B0');
  setText('pitch',d.pitch.toFixed(1)+'\u00B0');
  setText('yaw',d.yaw.toFixed(1)+'\u00B0');

  setText('mxr',d.mxRaw.toFixed(2)+' uT');
  setText('myr',d.myRaw.toFixed(2)+' uT');
  setText('mzr',d.mzRaw.toFixed(2)+' uT');
  setText('mf',d.magField.toFixed(2)+' uT');

  setText('acc',d.ax.toFixed(2)+' / '+d.ay.toFixed(2)+' / '+d.az.toFixed(2)+' g');
  setText('gyro',d.gx.toFixed(1)+' / '+d.gy.toFixed(1)+' / '+d.gz.toFixed(1)+' \u00B0/s');

  setText('kp',d.kp.toFixed(2));
  setText('ki',d.ki.toFixed(2));
  setText('kd',d.kd.toFixed(2));
  setText('pid',d.pid.toFixed(2));

  setText('lp',d.leftPWM);
  setText('rp',d.rightPWM);
  document.getElementById('lb').style.width=Math.min(100,Math.abs(d.leftPWM)/255*100)+'%';
  document.getElementById('rb').style.width=Math.min(100,Math.abs(d.rightPWM)/255*100)+'%';

  setText('dist',d.distance<0?'--':d.distance.toFixed(1));
  const o=document.getElementById('obs');
  o.textContent=d.obstacle?'STOP':'CLEAR';
  o.className=d.obstacle?'bad':'ok';

  setText('offX', d.magOffsetX.toFixed(2)+' uT');
  setText('offY', d.magOffsetY.toFixed(2)+' uT');
  setText('offZ', d.magOffsetZ.toFixed(2)+' uT');

  const bp=document.getElementById('battPct');
  bp.textContent=d.batteryPercent.toFixed(0);
  bp.className='big '+(d.batteryPercent<20?'bad':(d.batteryPercent<50?'warn':'ok'));
  setText('battV', d.batteryVoltage.toFixed(2)+' V');

  document.getElementById('needle').style.transform='rotate('+d.heading+'deg)';
 }catch(e){}
}

setInterval(update,500);
update();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("============================================================");
  Serial.println("        AUTONOMOUS BOAT - EXHIBITION VERSION");
  Serial.println("============================================================");

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  if (!ledcAttach(ENA, PWM_FREQ, PWM_RES)) {
    Serial.println("[ERROR] Left PWM attach failed");
  }
  if (!ledcAttach(ENB, PWM_FREQ, PWM_RES)) {
    Serial.println("[ERROR] Right PWM attach failed");
  }

  stopMotors();

  Serial.println("[INIT] Starting I2C on SDA=21, SCL=23");
  Wire.begin(21, 23);
  Wire.setClock(400000);

  delay(100);

  Serial.println("[INIT] Initializing MPU6050...");
  imuOK = false;
  for (int attempt = 1; attempt <= 5 && !imuOK; attempt++) {
    if (mpuInit()) {
      imuOK = true;
      mpuAddress = 0x68;
      Serial.printf("[OK] MPU6050 detected (attempt %d)\n", attempt);
    } else {
      Serial.printf("[WARN] MPU6050 init attempt %d failed, retrying...\n", attempt);
      delay(150);
    }
  }
  if (!imuOK) {
    Serial.println("[ERROR] MPU6050 not detected after 5 attempts");
  }

  Serial.println("[INIT] Initializing QMC5883P magnetometer...");
  magOK = false;
  for (int attempt = 1; attempt <= 5 && !magOK; attempt++) {
    if (qmcInit()) {
      magOK = true;
      Serial.printf("[OK] QMC5883P detected and configured (attempt %d)\n", attempt);
    } else {
      Serial.printf("[WARN] QMC5883P init attempt %d failed, retrying...\n", attempt);
      delay(150);
    }
  }
  if (!magOK) {
    Serial.println("[ERROR] QMC5883P not detected/configured after 5 attempts");
  }

  if (imuOK && magOK) {
    Serial.println("[OK] Madgwick AHRS ready");
  } else {
    Serial.println("[WARN] Madgwick disabled until both sensors are available");
  }

  lastIMUTime = micros();
  lastPIDTime = millis();

  Serial.println("[INIT] Starting WiFi access point...");
  WiFi.mode(WIFI_AP);
  bool apOK = WiFi.softAP(ssid, wifiPassword);

  if (apOK) {
    Serial.printf("[OK] WiFi AP: %s\n", ssid);
    Serial.printf("[OK] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WARN] WiFi AP failed");
  }

  if (MDNS.begin(hostname)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[OK] mDNS: http://%s.local/\n", hostname);
  } else {
    Serial.println("[INFO] mDNS unavailable - use 192.168.4.1");
  }

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html; charset=UTF-8", index_html);
  });

  server.on("/cmd", HTTP_GET, []() {
    if (!server.hasArg("value")) {
      server.send(400, "text/plain", "Missing command");
      return;
    }
    processCommand(server.arg("value"));
    server.send(200, "text/plain", "OK");
  });

  server.on("/telemetry", HTTP_GET, []() {
    server.send(200, "application/json", makeTelemetryJSON());
  });

  server.on("/health", HTTP_GET, []() {
    String s = "{";
    s += "\"imuOK\":" + String(imuOK ? "true" : "false");
    s += ",\"magOK\":" + String(magOK ? "true" : "false");
    s += ",\"madgwick\":" + String((imuOK && magOK) ? "true" : "false");
    s += ",\"mpuAddress\":" + String((unsigned int)mpuAddress);
    s += ",\"wifiIP\":\"" + WiFi.softAPIP().toString() + "\"";
    s += "}";
    server.send(200, "application/json", s);
  });

  server.begin();

  Serial.println("[OK] HTTP server started");
  Serial.println();
  Serial.println("CONNECT TO WIFI:");
  Serial.printf("  SSID     : %s\n", ssid);
  Serial.printf("  Password : %s\n", wifiPassword);
  Serial.printf("  WEB      : http://%s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("  mDNS     : http://myboat.local/");
  Serial.println();
  Serial.println("IMPORTANT: Use the IP address first if mDNS does not resolve.");
  Serial.println("============================================================");
}

// ============================================================
// Loop
// ============================================================
void loop() {
  server.handleClient();

  if (leftKicking && millis() >= leftKickEndTime) {
    leftKicking = false;
    ledcWrite(ENA, leftKickTarget);
  }
  if (rightKicking && millis() >= rightKickEndTime) {
    rightKicking = false;
    ledcWrite(ENB, rightKickTarget);
  }

  if (imuOK && magOK) {
    updateOrientation();
  }

  if (millis() - lastObstacleCheck >= 100) {
    updateObstacle();
    lastObstacleCheck = millis();
  }

  if (millis() - lastBatteryCheck >= 2000) {
    updateBatteryStatus();
    lastBatteryCheck = millis();
  }

  if (autoMode && imuOK && magOK) {
    runPID();
  }

  if (millis() - lastTelemetry >= 1000) {
    printTelemetry();
    lastTelemetry = millis();
  }

  delay(2);
}
