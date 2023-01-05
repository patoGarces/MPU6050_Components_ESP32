// #ifndef __MPU6050_H__
// #define __MPU6050_H__
#include "stdint.h"
#include "esp_err.h"
#include "driver/timer.h"

#define acc2ms(ms)(float)(ms * (9.81/16384.0))
#define gyro2rads(gyro)(float)(gyro * (250.0/32768.0))
// #define gyro2rads(gyro)(float)(gyro /131)

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


void i2c_init(void);
uint8_t i2c_read(int8_t readAddr);
uint16_t i2c_readReg(int8_t readAddr);
void i2c_write(int8_t writeAddr,uint8_t writeVal, uint16_t len);
esp_err_t mpu_init(void);
int16_t mpu_readAxis(uint8_t axis);
void mpu_readAllAxis(void);
void mpu_deInit(void);
float getAngle(uint8_t eje);

void initTimer(void);




























// #endif
