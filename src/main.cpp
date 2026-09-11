#include <Arduino.h>
#include <Wire.h>
#include "mpu6050_registers.h"   // your header from Step 3
#include <SoftwareSerial.h>
//#include <SdFat.h>

SoftwareSerial btSerial(2, 3); // RX, TX

struct __attribute__((packed)) ImuSample {
  uint32_t timestamp_us;
  int16_t ax, ay, az;      // raw sensor units
  int16_t gx, gy, gz;      // raw sensor units
  int16_t pitch_x100;      // pitch * 100, fixed-point (0.01° resolution)
  int16_t roll_x100;       // roll * 100, fixed-point
};
uint8_t btDownsampleCounter = 0;
const uint8_t BT_DOWNSAMPLE = 5; // send every 5th sample over Bluetooth

void sendTelemetryBT(uint32_t timestamp_us, int16_t ax, int16_t ay, int16_t az,
                     int16_t gx, int16_t gy, int16_t gz,
                     float pitch, float roll) {
  ImuSample sample = {timestamp_us, ax, ay, az, gx, gy, gz, (int16_t)(pitch * 100), (int16_t)(roll * 100)};
  btSerial.write((uint8_t*)&sample, sizeof(sample));
}
/*#define CS_PIN 10
#define BUFFER_SIZE 8
SdFat32 sd;
File32 logFile;
ImuSample sampleBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
*/
/*void setupSD() {
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  Serial.println(F("Waking up Onyx Card Controller... Please wait."));

  bool cardConnected = false;
  uint8_t retryCount = 0;
  
  // Ping the Onyx card up to 5 consecutive times to give the controller time to boot
  while (!cardConnected && retryCount < 5) {
    if (sd.begin(SdSpiConfig(10, SHARED_SPI, SD_SCK_MHZ(1)))) { // Run at stable 1MHz
      cardConnected = true;
    } else {
      retryCount++;
      Serial.print(F("Timeout fallback... Retry attempt "));
      Serial.println(retryCount);
      delay(200); // Give the flash controller 200ms to clear its busy bit
    }
  }

  if (!cardConnected) {
    Serial.println(F("\n================================="));
    Serial.println(F("CRITICAL: Onyx Card completely timed out!"));
    Serial.print(F("SD Error Code: 0x"));
    Serial.println(sd.card()->errorCode(), HEX);
    Serial.println(F("================================="));
    while (1);
  }
  
  if (!logFile.open("imu_log.bin", O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println(F("FATAL: Could not open bin file layer."));
    while (1);
  }
  Serial.println(F("Onyx SD Card successfully linked and mounted!"));
}*/


uint32_t lastTime = 0;
float pitch, roll;
float biasX = 0.0, biasY = 0.0, biasZ = 0.0;

volatile bool sampleReady = false;

float kalmanAngle = 0;
float kalmanUncertainty = 4;

const float Q_angle = 0.0000013;
const float R_measure = 0.0379;


ISR(TIMER1_COMPA_vect) {
  sampleReady = true;
}

void setupTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 624;              // 16MHz / 256 prescaler / (624+1) = 100 Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

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
  if (err != 0) return false;

  Wire.requestFrom(deviceAddr, count);

  uint32_t start = micros();
  for (uint8_t i = 0; i < count; i++) {
    while (!Wire.available()) {
      if (micros() - start > 5000) return false;   // 5ms for the whole transfer
    }
    buffer[i] = Wire.read();
  }
  return true;
}
/*void logSample(uint32_t timestamp, int16_t ax, int16_t ay, int16_t az,
               int16_t gx, int16_t gy, int16_t gz, float p, float r) {
  sampleBuffer[bufferIndex] = {timestamp, ax, ay, az, gx, gy, gz, (int16_t)(p * 100), (int16_t)(r * 100)};
  bufferIndex++;

  if (bufferIndex >= BUFFER_SIZE) {
    logFile.write((uint8_t*)sampleBuffer, sizeof(sampleBuffer));
    logFile.flush();
    bufferIndex = 0;
  }
}*/

void calibrateGyro(float* biasX, float* biasY, float* biasZ) {
  const int numSamples = 1000;
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int successfulReads = 0;
  for (int i = 0; i < numSamples; i++) {
    uint8_t buffer[14];
    bool readOk = readBytes(MPU6050_ADDR_AD0_LOW, REG_ACCEL_XOUT_H, buffer, SENSOR_DATA_LENGTH);
    if (!readOk) continue;
      
    int16_t gyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);

    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    successfulReads++;
    delay(1); // small delay to avoid overwhelming the sensor
  }
  const float SENSITIVITY = GYRO_SENSITIVITY_DEFAULT;

  *biasX = (sumX / (float)successfulReads)/SENSITIVITY;
  *biasY = (sumY / (float)successfulReads)/SENSITIVITY;
  *biasZ = (sumZ / (float)successfulReads)/SENSITIVITY;
}
  void captureAccelPosition(const char* label, int16_t* avgX, int16_t* avgY, int16_t* avgZ) {
  const int N = 50;
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int successfulReads = 0;

  for (int i = 0; i < N; i++) {
    uint8_t buffer[14];
    bool ok = readBytes(MPU6050_ADDR_AD0_LOW, REG_ACCEL_XOUT_H, buffer, SENSOR_DATA_LENGTH);
    if (!ok) continue;

    sumX += (int16_t)((buffer[0] << 8) | buffer[1]);
    sumY += (int16_t)((buffer[2] << 8) | buffer[3]);
    sumZ += (int16_t)((buffer[4] << 8) | buffer[5]);
    successfulReads++;

    delay(2);
  }

  if (successfulReads == 0) {
    Serial.print(label);
    Serial.println(F(" — all reads failed"));
    *avgX = *avgY = *avgZ = 0;
    return;
  }

  *avgX = sumX / successfulReads;
  *avgY = sumY / successfulReads;
  *avgZ = sumZ / successfulReads;

  Serial.print(label);
  Serial.print(F(" — X: ")); Serial.print(*avgX);
  Serial.print(F(" (")); Serial.print(*avgX / ACCEL_SENSITIVITY_DEFAULT, 3); Serial.print(F("g)"));
  Serial.print(F("  Y: ")); Serial.print(*avgY);
  Serial.print(F(" (")); Serial.print(*avgY / ACCEL_SENSITIVITY_DEFAULT, 3); Serial.print(F("g)"));
  Serial.print(F("  Z: ")); Serial.print(*avgZ);
  Serial.print(F(" (")); Serial.print(*avgZ / ACCEL_SENSITIVITY_DEFAULT, 3); Serial.println(F("g)"));
}


void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C speed
  Serial.begin(115200);
  while (!Serial);

    //setupSD();
  btSerial.begin(38400);

  // --- WHO_AM_I check ---
  uint8_t whoAmI;
  bool whoAmIOk = readRegister(MPU6050_ADDR_AD0_LOW, REG_WHO_AM_I, &whoAmI);

  if (!whoAmIOk || whoAmI != WHO_AM_I_EXPECTED) {
    Serial.println(F("FATAL: MPU-6050 not responding correctly. Halting."));
    while (1);   // stop here — nothing downstream is safe to run
  }

  Serial.print(F("WHO_AM_I: 0x"));
  Serial.println(whoAmI, HEX);

  // --- Wake the sensor ---
  bool wakeOk = writeRegister(MPU6050_ADDR_AD0_LOW, REG_PWR_MGMT_1, 0x00);
  if (!wakeOk) {
    Serial.println(F("FATAL: failed to wake sensor. Halting."));
    while (1);
  }

  // --- Confirm PWR_MGMT_1 ---
  uint8_t pwrMgmt;
  bool pwrOk = readRegister(MPU6050_ADDR_AD0_LOW, REG_PWR_MGMT_1, &pwrMgmt);
  if (!pwrOk) {
    Serial.println(F("PWR_MGMT_1 read failed"));
  } else {
    Serial.print(F("PWR_MGMT_1: 0x"));
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

    float accelX_g = (accelX_signed - ACCEL_OFFSET_X) / ACCEL_SCALE_X;
    float accelY_g = (accelY_signed - ACCEL_OFFSET_Y) / ACCEL_SCALE_Y;
    float accelZ_g = (accelZ_signed - ACCEL_OFFSET_Z) / ACCEL_SCALE_Z;
    float magnitude = sqrt(accelX_g*accelX_g + accelY_g*accelY_g + accelZ_g*accelZ_g);

    Serial.print("MAGNITUDE (calibrated, should be ~1.0g): ");
    Serial.println(magnitude, 3);
  }
  calibrateGyro(&biasX, &biasY, &biasZ);
  Serial.print("Gyro Bias X: "); Serial.println(biasX, 3);
  Serial.print("Gyro Bias Y: "); Serial.println(biasY, 3);
  Serial.print("Gyro Bias Z: "); Serial.println(biasZ, 3);

// Take one more reading and apply the correction
  uint8_t buffer2[14];
  bool readOk2 = readBytes(MPU6050_ADDR_AD0_LOW, REG_ACCEL_XOUT_H, buffer2, SENSOR_DATA_LENGTH);
  if (readOk2) {
    int16_t gyroX_signed = (int16_t)((buffer2[8] << 8) | buffer2[9]);
    int16_t gyroY_signed = (int16_t)((buffer2[10] << 8) | buffer2[11]);
    int16_t gyroZ_signed = (int16_t)((buffer2[12] << 8) | buffer2[13]);

    float gyroX_dps_raw = gyroX_signed / GYRO_SENSITIVITY_DEFAULT;
    float gyroY_dps_raw = gyroY_signed / GYRO_SENSITIVITY_DEFAULT;
    float gyroZ_dps_raw = gyroZ_signed / GYRO_SENSITIVITY_DEFAULT;

    float gyroX_dps_corrected = gyroX_dps_raw - biasX;
    float gyroY_dps_corrected = gyroY_dps_raw - biasY;
    float gyroZ_dps_corrected = gyroZ_dps_raw - biasZ;

    Serial.print("Gyro X — raw: "); Serial.print(gyroX_dps_raw, 3);
    Serial.print("  corrected: "); Serial.println(gyroX_dps_corrected, 3);

    Serial.print("Gyro Y — raw: "); Serial.print(gyroY_dps_raw, 3);
    Serial.print("  corrected: "); Serial.println(gyroY_dps_corrected, 3);

    Serial.print("Gyro Z — raw: "); Serial.print(gyroZ_dps_raw, 3);
    Serial.print("  corrected: "); Serial.println(gyroZ_dps_corrected, 3);
}
  int16_t x1, y1, z1;
  captureAccelPosition("side (x up)", &x1, &y1, &z1);

  lastTime = micros();
  setupTimer();
}
void loop() {
  if (sampleReady) {
    sampleReady = false;

    uint32_t now = micros();

  

    uint32_t dt_us = now - lastTime;             // wrap-safe
    float dt = dt_us / 1000000.0f;
    lastTime = now;

    uint8_t buffer[14];
    bool ok = readBytes(MPU6050_ADDR_AD0_LOW, REG_ACCEL_XOUT_H, buffer, SENSOR_DATA_LENGTH);


    if (!ok) {
      Serial.println("Sensor read failed, skipping this cycle");
      return;
    }

    int16_t accelX_signed = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t accelY_signed = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t accelZ_signed = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t gyroX_signed  = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gyroY_signed  = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t gyroZ_signed  = (int16_t)((buffer[12] << 8) | buffer[13]);

    float accelX_g = (accelX_signed - ACCEL_OFFSET_X) / ACCEL_SCALE_X;
    float accelY_g = (accelY_signed - ACCEL_OFFSET_Y) / ACCEL_SCALE_Y;
    float accelZ_g = (accelZ_signed - ACCEL_OFFSET_Z) / ACCEL_SCALE_Z;
    float gyroX_dps = (gyroX_signed / GYRO_SENSITIVITY_DEFAULT) - biasX;
    float gyroY_dps = (gyroY_signed / GYRO_SENSITIVITY_DEFAULT) - biasY;
  

    float pitch_accel = atan2(accelY_g, accelZ_g) * 180.0 / PI;
    float pitch_gyro  = pitch + gyroX_dps * dt;

    const float alpha = 0.98;
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_accel;

    float roll_accel = atan2(accelX_g, accelZ_g) * 180.0 / PI;
    float roll_gyro  = roll + gyroY_dps * dt;
    roll = alpha * roll_gyro + (1 - alpha) * roll_accel;

    kalmanAngle += gyroX_dps * dt;
    kalmanUncertainty += Q_angle;
    float kalmanGain = kalmanUncertainty / (kalmanUncertainty + R_measure);
    kalmanAngle += kalmanGain * (pitch_accel - kalmanAngle);
    kalmanUncertainty *= (1 - kalmanGain);

    //logSample(now, accelX_signed, accelY_signed, accelZ_signed, gyroX_signed, gyroY_signed, gyroZ_signed, pitch, roll);
    if (++btDownsampleCounter >= BT_DOWNSAMPLE) {
      btDownsampleCounter = 0;
      sendTelemetryBT(now, accelX_signed, accelY_signed, accelZ_signed,
                   gyroX_signed, gyroY_signed, gyroZ_signed, pitch, roll);
}
    btSerial.println("BT alive");
    static int printCounter = 0;
    if (++printCounter >= 10) {
      printCounter = 0;
      Serial.print("Pitch (comp): "); Serial.println(pitch, 2);
      Serial.print("Kalman Angle: ");  Serial.println(kalmanAngle, 2);
    }
    // no delay here — the ISR paces the loop
  }
}