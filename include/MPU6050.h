#ifndef __MPU6050_H__
#define __MPU6050_H__
#include "stdint.h"
#include "esp_err.h"
#include "driver/gptimer.h"

enum{
    ACC_AXIS_X = 0x3B,
    ACC_AXIS_Y = 0x3D,
    ACC_AXIS_Z = 0x3F,
    GYRO_AXIS_X = 0x43,
    GYRO_AXIS_Y = 0x45,
    GYRO_AXIS_Z = 0x47
};
enum{
    AXIS_ANGLE_X,
    AXIS_ANGLE_Y,
    AXIS_ANGLE_Z
};

typedef struct {
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyX;
    int16_t gyY;
    int16_t gyZ;
}rawData_t;

/* Funciones para control de MPU*/
esp_err_t mpu_init(uint8_t _sclGpio,uint8_t _sdaGpio,uint8_t intGpio);
void mpu_readAllAxis(void);
void mpu_deInit(void);
float getAngle(uint8_t eje);
rawData_t getRawData();

#endif
