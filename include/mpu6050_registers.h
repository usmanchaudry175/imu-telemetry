#ifndef MPU6050_REGISTERS_H
#define MPU6050_REGISTERS_H

#include <stdint.h>

// I2C device address
// Find: what is the 7-bit address? What does the AD0 pin change it to?
constexpr uint8_t MPU6050_ADDR_AD0_LOW  = 0x68;
constexpr uint8_t MPU6050_ADDR_AD0_HIGH = 0x69;

// Identity check
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t WHO_AM_I_EXPECTED = 0x68;

// Power management
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
// Which bit position holds the device asleep at power-on?
constexpr uint8_t PWR_MGMT_1_SLEEP_BIT = 6;

// Sensor data block — where does the burst read start, how many bytes?
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t SENSOR_DATA_LENGTH = 14; // bytes, covers accel+temp+gyro

// Configuration registers
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;

// Sensitivity values for default power-on ranges
// Accel default: ±2g → 16384 LSB/g
// Gyro default: ±250°/s → 131 LSB/(°/s)
constexpr float ACCEL_SENSITIVITY_DEFAULT = 16384.0;
constexpr float GYRO_SENSITIVITY_DEFAULT = 131.0;
#endif