#include <Arduino.h>
#include "config.h"
#include "device_config.h"
#include <fport.h>
#include <Wire.h>
#include "../../MPU/src/package.h"
#include "SerialTransfer.h"
#include <PID_v1.h>

#include "motors/bts7960.h"
#include "motors/pot_limited_motor.h"
#include "pid_motor.h"

SerialTransfer mpuPackageTransfer;
MpuPackage mpuPackage;

FPort fport;
float batteryVoltage = 12.0;

Bts7960 mainMotor = Bts7960(MAIN_ENABLE, MAIN_L, MAIN_R);
PidMotor mainPid = PidMotor(&mainMotor, MAIN_KP, MAIN_KI, MAIN_KD, PidMotor::Direction::Direct);

Bts7960 s2sUnlimitedMotor = Bts7960(S2S_ENABLE, S2S_L, S2S_R);
ReachedLimitAtMinimum s2sLeftLimit = ReachedLimitAtMinimum(S2S_LEFT_MINIMUM);
ReachedLimitAtMaximum s2sRightLimit = ReachedLimitAtMaximum(S2S_RIGHT_MAXIMUM);
PotLimitedMotor s2sMotor = PotLimitedMotor(&s2sUnlimitedMotor, S2S_POT, &s2sLeftLimit, &s2sRightLimit);
PidMotor s2sPid = PidMotor(&s2sMotor, S2S_KP, S2S_KI, S2S_KD, PidMotor::Direction::Reverse);

unsigned long long lastMpuPackage;

uint32_t getBatteryVoltage(sensorType type) {
  switch(type) {
    case FSSP_DATAID_VFAS1: return batteryVoltage * 100;
    default: return 0;
  }
}

void setup() {
  MPU_SERIAL.begin(MpuSerialSpeed);
  mpuPackageTransfer.begin(MPU_SERIAL);

  pinMode(DOME_POT, INPUT);
  pinMode(DOME_ENABLE, OUTPUT);
  pinMode(DOME_L, OUTPUT);
  pinMode(DOME_R, OUTPUT);

  DEBUG_SERIAL2.begin(115200);
  DEBUG_SERIAL2.println("CB-23 ready");

  fport.begin(&RC_SERIAL);
  fport.addSensor(FSSP_DATAID_VFAS1, getBatteryVoltage);

  mainMotor.begin();
  mainMotor.setMinimumActivationSpeed(20);

  s2sMotor.begin();
  s2sUnlimitedMotor.setMinimumActivationSpeed(20);
}

uint16_t previous[18];

void loop() {
  if (DEBUG_SERIAL2.available()) {
    char in = DEBUG_SERIAL2.read();
    if (in == 'l') {
      s2sPid.setSetpoint(s2sPid.getSetpoint() - 0.5);
    } else if (in == 'r') {
      s2sPid.setSetpoint(s2sPid.getSetpoint() + 0.5);
    }
    DEBUG_SERIAL2.print("setpoint: ");
    DEBUG_SERIAL2.println(s2sPid.getSetpoint());
  }

  uint8_t status = fport.loop();

  if (status & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED | RX_TIMEOUT | RX_NOT_INITIALIZED)) {
      // DEBUG_SERIAL2.print("Sport status ");
      // DEBUG_SERIAL2.println(status); 
      // DEBUG_SERIAL2.print("E");
      // DEBUG_SERIAL2.print(status); 
      // DEBUG_SERIAL2.print("-");
      s2sPid.setSetpoint(0);
      mainPid.setSetpoint(0);
  } else if (status == RX_FRAME_COMPLETE) {
      int changes = 0;
      for(int i = 0; i < 4; i++) {
        if (fport.channels[i] != previous[i]) {
          previous[i] = fport.channels[i];
          // DEBUG_SERIAL2.print(i); DEBUG_SERIAL2.print(": "); DEBUG_SERIAL2.print(fport.channels[i]);
          // DEBUG_SERIAL2.print(" - ");
          // 172, 992, 1811
          //float s2starget = map(channels[1], 900, 1800)
          changes = 1;
        }
      }

      float s2starget = map((float)fport.channels[1], 172, 1811, -25.0, 25.0);
      s2sPid.setSetpoint((double)s2starget);
      
      float maintarget = map((float)fport.channels[2], 172, 1811, -25.0, 25.0);
      mainPid.setSetpoint((double)maintarget);
  }

  if (mpuPackageTransfer.available()) {
    lastMpuPackage = millis();
    uint16_t recSize = 0;
    recSize = mpuPackageTransfer.rxObj(mpuPackage, recSize);
    // DEBUG_SERIAL2.print(mpuPackage.count); DEBUG_SERIAL2.print(",");
    // DEBUG_SERIAL2.print(mpuPackage.status); DEBUG_SERIAL2.print(",");
    // DEBUG_SERIAL2.print(mpuPackage.pitch); DEBUG_SERIAL2.print(",");
    // DEBUG_SERIAL2.println(mpuPackage.roll);
  }

  if ((lastMpuPackage + 500) < millis()) {
    DEBUG_SERIAL2.println("Did not receive MPU package in last 500ms");
    s2sMotor.stop();
    mainMotor.stop();
    return;
  }

  s2sMotor.updateLimit();
  mainPid.loop(mpuPackage.pitch);
  s2sPid.loop(mpuPackage.roll);
}
