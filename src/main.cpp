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

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  uint8_t whoAmI = readRegister(MPU6050_ADDR_AD0_LOW, REG_WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);
}

void loop() {}