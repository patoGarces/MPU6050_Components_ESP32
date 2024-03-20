#ifndef __MPU6050_H__
#define __MPU6050_H__
#include "stdint.h"
#include "esp_err.h"
#include "driver/gptimer.h"

enum {
    AXIS_ANGLE_X,
    AXIS_ANGLE_Y,
    AXIS_ANGLE_Z
};
// Accel sensitivity
// ±2g: 16384 LSB/g
// ±4g: 8192 LSB/g
// ±8g: 4096 LSB/g
// ±16g: 2048 LSB/g
typedef enum {
    MPU_ACCEL_SENS_2G,
    MPU_ACCEL_SENS_4G,
    MPU_ACCEL_SENS_8G,
    MPU_ACCEL_SENS_16G,
} accel_sensitivity_t;

// Gyro sensitivity(degrees per second):
// ±250, 
// ±500, 
// ±1000, 
// ±2000
typedef enum {
    MPU_GYRO_SENS_250,
    MPU_GYRO_SENS_500,
    MPU_GYRO_SENS_1000,
    MPU_GYRO_SENS_2000,
} gyro_sensitivity_t;

typedef struct {
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyX;
    int16_t gyY;
    int16_t gyZ;
} mpu_raw_data_t;

typedef struct {
    uint8_t sclGpio;
    uint8_t sdaGpio;
    uint8_t intGpio;
    accel_sensitivity_t accelSensitivity;
    gyro_sensitivity_t gyroSensitivity;
    float sampleTimeInMs;
    uint8_t priorityTask;
} mpu6050_init_t;

/* Funciones para control de MPU*/
esp_err_t mpuInit(mpu6050_init_t initConfig);
void mpuCalibrate(void);
void mpuReadAllAxis(void);
void mpuDeInit(void);
float mpuGetAngle(uint8_t eje);
mpu_raw_data_t mpuGetRawData(void);

#endif
