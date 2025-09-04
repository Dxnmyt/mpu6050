#ifndef mpu6050_H
#define mpu6050_H

#include "hi2c.h"
#include "stdio.h"
#include "math.h"

/* MPU6050 寄存器地址定义 */

#define MPU6050_ADDR    0x68 // MPU6050 I2C地址

// 配置寄存器
#define	MPU6050_SMPLRT_DIV		0x19	// 采样率分频器  0x08
#define	MPU6050_CONFIG			0x1A	// 配置   0x06
#define	MPU6050_GYRO_CONFIG		0x1B	// 陀螺仪配置  0x10
#define	MPU6050_ACCEL_CONFIG	0x1C	// 加速度计配置  0x10

// 加速度计数据寄存器
#define	MPU6050_ACCEL_XOUT_H	0x3B	// 加速度计X轴 高8位  
#define	MPU6050_ACCEL_XOUT_L	0x3C	// 加速度计X轴 低8位
#define	MPU6050_ACCEL_YOUT_H	0x3D	// 加速度计Y轴 高8位
#define	MPU6050_ACCEL_YOUT_L	0x3E	// 加速度计Y轴 低8位
#define	MPU6050_ACCEL_ZOUT_H	0x3F	// 加速度计Z轴 高8位
#define	MPU6050_ACCEL_ZOUT_L	0x40	// 加速度计Z轴 低8位

// 温度数据寄存器
#define	MPU6050_TEMP_OUT_H		0x41	// 温度 高8位
#define	MPU6050_TEMP_OUT_L		0x42	// 温度 低8位

// 陀螺仪数据寄存器
#define	MPU6050_GYRO_XOUT_H		0x43	// 陀螺仪X轴 高8位
#define	MPU6050_GYRO_XOUT_L		0x44	// 陀螺仪X轴 低8位
#define	MPU6050_GYRO_YOUT_H		0x45	// 陀螺仪Y轴 高8位
#define	MPU6050_GYRO_YOUT_L		0x46	// 陀螺仪Y轴 低8位
#define	MPU6050_GYRO_ZOUT_H		0x47	// 陀螺仪Z轴 高8位
#define	MPU6050_GYRO_ZOUT_L		0x48	// 陀螺仪Z轴 低8位

// 电源管理及设备ID寄存器
#define	MPU6050_PWR_MGMT_1		0x6B	// 电源管理 1  0x05
#define	MPU6050_PWR_MGMT_2		0x6C	// 电源管理 2  0x00
#define	MPU6050_WHO_AM_I		0x75	// 设备ID

typedef struct
{
    uint8_t accel_x_h; // 加速度计X轴高8位
    uint8_t accel_x_l; 
    uint8_t accel_y_h;
    uint8_t accel_y_l; // 加速度计Y轴低8位
    uint8_t accel_z_h;
    uint8_t accel_z_l; // 加速度计Z轴低8位
    uint8_t gyro_x_h;
    uint8_t gyro_x_l;
    uint8_t gyro_y_h;
    uint8_t gyro_y_l;
    uint8_t gyro_z_h;
    uint8_t gyro_z_l;
    int16_t accel_x; // 加速度计X轴数据
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
}mpu6050_HXYZ_t;

typedef struct 
{
    float value;          // 当前滤波后的加速度值
    float uncertainty;    // 估计的不确定性
    float process_noise;  // 过程噪声（运动的随机性）
    float measurement_noise; // 测量噪声（传感器误差）
} KalmanFilter_1D;


extern mpu6050_HXYZ_t mpu6050_xyz;
extern int16_t GRAVITY_BASE;
extern float AccFiltered;
void MPU6050_Init(void);
void MPU6050_WriteReg(uint8_t reg_addr, uint8_t data);
void MPU6050_ReadXYZ(void);
void MPU6050_GetData(mpu6050_HXYZ_t *data);
void kalman_filter_init(KalmanFilter_1D *filter);
float kalman_filter_update(KalmanFilter_1D *filter, float measurement);
void InitStepDetection(void);
int step_detect(void);

#endif
