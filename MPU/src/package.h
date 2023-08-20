#pragma once

const unsigned long MpuSerialSpeed = 1000000;

#define MPU_NOT_FOUND 1
#define MEMORY_LOAD_FAILED 2
#define DMP_CONFIGURATION_FAILED 3
#define FIFO_OVERRRUN 4
#define MPU_OK 5

struct __attribute__ ((packed)) MpuPackage {
    uint8_t count;
    uint8_t status;
    float pitch;
    float roll;
};