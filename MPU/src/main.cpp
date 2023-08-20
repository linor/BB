#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "package.h"
#include "SerialTransfer.h"
#include "config.h"

#define MPU_INT 2
#define PACKAGE_SEND_INTERVAL 100

#undef DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

const uint8_t mpuAddress = 0x68;
MPU6050 mpu(mpuAddress);

MpuPackage mpuPackage;

SerialTransfer packageTransfer;
unsigned long packageTransferTime;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Serial.begin(MpuSerialSpeed);
  packageTransfer.begin(Serial);

  Wire.begin();
  Wire.setClock(100000);

  DEBUG_PRINTLN(F("Initializing I2C devices..."));
  mpu.initialize();

  DEBUG_PRINTLN(F("Testing device connections..."));
  if (!mpu.testConnection()) {
    DEBUG_PRINTLN(F("MPU6050 connection failed"));

    mpuPackage.status = MPU_NOT_FOUND;
  } else {
    DEBUG_PRINTLN(F("MPU6050 connection successful"));
    DEBUG_PRINTLN(F("Initializing DMP..."));

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(X_GYRO_OFFSET);
    mpu.setYGyroOffset(Y_GYRO_OFFSET);
    mpu.setZGyroOffset(Z_GYRO_OFFSET);
    mpu.setZAccelOffset(Z_ACCEL_OFFSET); 

    if (devStatus == 0) {
      DEBUG_PRINTLN(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      DEBUG_PRINTLN(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      DEBUG_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      if (devStatus == 1) {
        mpuPackage.status = MEMORY_LOAD_FAILED;
      } else {
        mpuPackage.status = DMP_CONFIGURATION_FAILED;
      }
    }
  }
}

void transferPackage() {
  uint16_t sendSize = 0;
  sendSize = packageTransfer.txObj(mpuPackage, sendSize);
  packageTransfer.sendData(sendSize);
}

void loop() {
  if (dmpReady && (mpuInterrupt || (fifoCount >= packetSize))) {
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpuPackage.status = FIFO_OVERRRUN;
      DEBUG_PRINTLN(F("Fifo overrun!"));
    } else if (mpuIntStatus & 0x02) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      float pitch = (ypr[1] * 180/M_PI);
      float roll = (ypr[2] * 180/M_PI);

      mpuPackage.status = MPU_OK;
      mpuPackage.pitch = pitch;
      mpuPackage.roll = roll;
      mpuPackage.count++;

      transferPackage();
    }
  }

  #ifdef DEBUG
    long size = sizeof(MpuPackage);
    Serial.print(size, DEC); DEBUG_PRINT(" ");
    DEBUG_PRINT(mpuPackage.count); DEBUG_PRINT(" ");
    DEBUG_PRINT(mpuPackage.status); DEBUG_PRINT(" ");
    DEBUG_PRINT(mpuPackage.roll); DEBUG_PRINT(" ");
    DEBUG_PRINTLN(mpuPackage.pitch);
  #else 
    if ((mpuPackage.status != MPU_OK) && ((packageTransferTime + PACKAGE_SEND_INTERVAL) < millis())) {
      packageTransferTime = millis();
      transferPackage();
    }
  #endif
}

