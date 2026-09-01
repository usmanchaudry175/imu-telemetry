#include <Arduino.h>
#include <Wire.h>
#include "mpu6050_registers.h"   // your header from Step 3

uint8_t readRegister(uint8_t deviceAddr, uint8_t reg) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(reg);
  Wire.endTransmission(false); // false to send a restart, not a stop

  Wire.requestFrom(deviceAddr, (uint8_t)1);
  while (!Wire.available());
  return Wire.read();
}
void writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true); // true to send a stop
}
void readBytes(uint8_t deviceAddr, uint8_t startReg, uint8_t* buffer, uint8_t count) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(startReg);
  Wire.endTransmission(false);

  Wire.requestFrom(deviceAddr, count);
  for (uint8_t i = 0; i < count; i++) {
    while (!Wire.available());
    buffer[i] = Wire.read();
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  uint8_t whoAmI = readRegister(MPU6050_ADDR_AD0_LOW, REG_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  writeRegister(MPU6050_ADDR_AD0_LOW, REG_PWR_MGMT_1, 0x00);
  uint8_t pwrMgmt = readRegister(MPU6050_ADDR_AD0_LOW, REG_PWR_MGMT_1);
  Serial.print("PWR_MGMT_1: 0x");
  Serial.println(pwrMgmt, HEX);

  uint8_t buffer[14];
  readBytes(MPU6050_ADDR_AD0_LOW, REG_ACCEL_XOUT_H, buffer, SENSOR_DATA_LENGTH);
  uint16_t accelX_raw = (uint16_t)((buffer[0] << 8) | buffer[1]);
  uint16_t accelY_raw = (uint16_t)((buffer[2] << 8) | buffer[3]);
  uint16_t accelZ_raw = (uint16_t)((buffer[4] << 8) | buffer[5]);
  uint16_t gyroX_raw = (uint16_t)((buffer[8] << 8) | buffer[9]);
  uint16_t gyroY_raw = (uint16_t)((buffer[10] << 8) | buffer[11]);
  uint16_t gyroZ_raw = (uint16_t)((buffer[12] << 8) | buffer[13]);
  int16_t accelX_signed = (int16_t)accelX_raw;
  int16_t accelY_signed = (int16_t)accelY_raw;
  int16_t accelZ_signed = (int16_t)accelZ_raw;
  int16_t gyroX_signed = (int16_t)gyroX_raw;
  int16_t gyroY_signed = (int16_t)gyroY_raw;
  int16_t gyroZ_signed = (int16_t)gyroZ_raw;
  Serial.print("ACCEL_X_RAW: ");
  Serial.println(accelX_raw);
  Serial.print("ACCEL_X_SIGNED: ");
  Serial.println(accelX_signed);
  Serial.print("ACCEL_Y_RAW: ");
  Serial.println(accelY_raw);
  Serial.print("ACCEL_Y_SIGNED: ");
  Serial.println(accelY_signed);
  Serial.print("ACCEL_Z_RAW: ");
  Serial.println(accelZ_raw);
  Serial.print("ACCEL_Z_SIGNED: ");
  Serial.println(accelZ_signed);
  Serial.print("GYRO_X_RAW: ");
  Serial.println(gyroX_raw);
  Serial.print("GYRO_X_SIGNED: ");
  Serial.println(gyroX_signed);
  Serial.print("GYRO_Y_RAW: ");
  Serial.println(gyroY_raw);
  Serial.print("GYRO_Y_SIGNED: ");
  Serial.println(gyroY_signed);
  Serial.print("GYRO_Z_RAW: ");
  Serial.println(gyroZ_raw);
  Serial.print("GYRO_Z_SIGNED: ");
  Serial.println(gyroZ_signed);
  float accelX_g = accelX_signed / 16384.0;
  float accelY_g = accelY_signed / 16384.0;
  float accelZ_g = accelZ_signed / 16384.0;
  float magnitude = sqrt(accelX_g*accelX_g + accelY_g*accelY_g + accelZ_g*accelZ_g);
  Serial.print("MAGNITUDE (should be ~1.0g): ");
  Serial.println(magnitude, 3);
}

void loop() {}