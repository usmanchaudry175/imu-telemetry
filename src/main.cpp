#include <Arduino.h>
#include <Wire.h>
#include "mpu6050_registers.h"   // your header from Step 3

bool readRegister(uint8_t deviceAddr, uint8_t reg, uint8_t* value) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return false;

  Wire.requestFrom(deviceAddr, (uint8_t)1);
  uint32_t start = millis();
  while (!Wire.available()) {
    if (millis() - start > 100) return false;
  }

  *value = Wire.read();
  return true;
}

bool writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission(true); // true to send a stop
  return err == 0;
}

bool readBytes(uint8_t deviceAddr, uint8_t startReg, uint8_t* buffer, uint8_t count) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(startReg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return false;   // bus write failed — bail out now

  Wire.requestFrom(deviceAddr, count);
  for (uint8_t i = 0; i < count; i++) {
    uint32_t start = millis();
    while (!Wire.available()) {
      if (millis() - start > 100) return false;   // timeout — sensor stopped responding
    }
    buffer[i] = Wire.read();
  }
  return true;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  // --- WHO_AM_I check ---
  uint8_t whoAmI;
  bool whoAmIOk = readRegister(MPU6050_ADDR_AD0_LOW, REG_WHO_AM_I, &whoAmI);

  if (!whoAmIOk || whoAmI != WHO_AM_I_EXPECTED) {
    Serial.println("FATAL: MPU-6050 not responding correctly. Halting.");
    while (1);   // stop here — nothing downstream is safe to run
  }

  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  // --- Wake the sensor ---
  bool wakeOk = writeRegister(MPU6050_ADDR_AD0_LOW, REG_PWR_MGMT_1, 0x00);
  if (!wakeOk) {
    Serial.println("FATAL: failed to wake sensor. Halting.");
    while (1);
  }

  // --- Confirm PWR_MGMT_1 ---
  uint8_t pwrMgmt;
  bool pwrOk = readRegister(MPU6050_ADDR_AD0_LOW, REG_PWR_MGMT_1, &pwrMgmt);
  if (!pwrOk) {
    Serial.println("PWR_MGMT_1 read failed");
  } else {
    Serial.print("PWR_MGMT_1: 0x");
    Serial.println(pwrMgmt, HEX);
  }

  // --- Burst read accel + gyro ---
  uint8_t buffer[14];
  bool readOk = readBytes(MPU6050_ADDR_AD0_LOW, REG_ACCEL_XOUT_H, buffer, SENSOR_DATA_LENGTH);

  if (!readOk) {
    Serial.println("Sensor data read failed - skipping this reading.");
  } else {
    uint16_t accelX_raw = (uint16_t)((buffer[0] << 8) | buffer[1]);
    uint16_t accelY_raw = (uint16_t)((buffer[2] << 8) | buffer[3]);
    uint16_t accelZ_raw = (uint16_t)((buffer[4] << 8) | buffer[5]);
    uint16_t gyroX_raw  = (uint16_t)((buffer[8] << 8) | buffer[9]);
    uint16_t gyroY_raw  = (uint16_t)((buffer[10] << 8) | buffer[11]);
    uint16_t gyroZ_raw  = (uint16_t)((buffer[12] << 8) | buffer[13]);

    int16_t accelX_signed = (int16_t)accelX_raw;
    int16_t accelY_signed = (int16_t)accelY_raw;
    int16_t accelZ_signed = (int16_t)accelZ_raw;
    int16_t gyroX_signed  = (int16_t)gyroX_raw;
    int16_t gyroY_signed  = (int16_t)gyroY_raw;
    int16_t gyroZ_signed  = (int16_t)gyroZ_raw;

    Serial.print("ACCEL_X_SIGNED: "); Serial.println(accelX_signed);
    Serial.print("ACCEL_Y_SIGNED: "); Serial.println(accelY_signed);
    Serial.print("ACCEL_Z_SIGNED: "); Serial.println(accelZ_signed);
    Serial.print("GYRO_X_SIGNED: ");  Serial.println(gyroX_signed);
    Serial.print("GYRO_Y_SIGNED: ");  Serial.println(gyroY_signed);
    Serial.print("GYRO_Z_SIGNED: ");  Serial.println(gyroZ_signed);

    float accelX_g = accelX_signed / 16384.0;
    float accelY_g = accelY_signed / 16384.0;
    float accelZ_g = accelZ_signed / 16384.0;
    float magnitude = sqrt(accelX_g * accelX_g + accelY_g * accelY_g + accelZ_g * accelZ_g);
    Serial.print("MAGNITUDE (should be ~1.0g): ");
    Serial.println(magnitude, 3);
  }
}

void loop() {}