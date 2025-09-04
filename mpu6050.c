#include "mpu6050.h"

mpu6050_HXYZ_t mpu6050_xyz;
float AccFiltered;
int16_t GRAVITY_BASE = 2048;
uint8_t IsCalibrated = 0;

#define PEAK_THRESHOLD 300      // 触发计步的波峰阈值
#define VALLEY_THRESHOLD 150    // 判断步伐结束的波谷阈值
#define MIN_STEP_INTERVAL 300    // 最小步间隔

void MPU6050_WriteReg(uint8_t reg_addr, uint8_t data)
{
    I2C_Write_Reg(MPU6050_ADDR, reg_addr, data);
}

void MPU6050_Init(void)
{
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01); // 0x05: 启动MPU6050
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00); // 0x00: 启动所有传感器
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18); // 设置陀螺仪量程
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18); // 设置加速度计量程
}

void MPU6050_ReadXYZ(void)
{
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, &mpu6050_xyz.accel_x_h);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_ACCEL_XOUT_L, &mpu6050_xyz.accel_x_l);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, &mpu6050_xyz.accel_y_h);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_ACCEL_YOUT_L, &mpu6050_xyz.accel_y_l);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, &mpu6050_xyz.accel_z_h);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_L, &mpu6050_xyz.accel_z_l);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, &mpu6050_xyz.gyro_x_h);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_GYRO_XOUT_L, &mpu6050_xyz.gyro_x_l);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_GYRO_YOUT_H, &mpu6050_xyz.gyro_y_h);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_GYRO_YOUT_L, &mpu6050_xyz.gyro_y_l);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_GYRO_ZOUT_H, &mpu6050_xyz.gyro_z_h);
    I2C_Read_Reg(MPU6050_ADDR, MPU6050_GYRO_ZOUT_L, &mpu6050_xyz.gyro_z_l);
}

void MPU6050_GetData(mpu6050_HXYZ_t *data)
{
    MPU6050_ReadXYZ();
    data->accel_x = (int16_t)((mpu6050_xyz.accel_x_h << 8) | mpu6050_xyz.accel_x_l);
    data->accel_y = (int16_t)((mpu6050_xyz.accel_y_h << 8) | mpu6050_xyz.accel_y_l);
    data->accel_z = (int16_t)((mpu6050_xyz.accel_z_h << 8) | mpu6050_xyz.accel_z_l);
    data->gyro_x = (int16_t)((mpu6050_xyz.gyro_x_h << 8) | mpu6050_xyz.gyro_x_l);
    data->gyro_y = (int16_t)((mpu6050_xyz.gyro_y_h << 8) | mpu6050_xyz.gyro_y_l);
    data->gyro_z = (int16_t)((mpu6050_xyz.gyro_z_h << 8) | mpu6050_xyz.gyro_z_l);
}

void kalman_filter_init(KalmanFilter_1D *filter)
{
    filter->measurement_noise = 1.0f;
    filter->process_noise = 0.1f;
    filter->uncertainty = 100.0f;
    filter->value = 0.0f;
}

float kalman_filter_update(KalmanFilter_1D *filter, float measurement)
{
    float kalman_value = filter->value;  // 预测值
    float kalman_uncertainty = filter->uncertainty + filter->process_noise;  // 预测不确定性

    float kalman_gain = kalman_uncertainty / (kalman_uncertainty + filter->measurement_noise);  // 卡尔曼增益

    filter->value = kalman_value + kalman_gain * (measurement - kalman_value);  // 更新值

    filter->uncertainty = (1 - kalman_gain) * kalman_uncertainty;  // 更新不确定性

    return filter->value; // 返回滤波后的值
}

void InitStepDetection(void)
{
	float AccSum = 0;
	int ValidSamples = 0;
	
	for(int i = 0; i < 200; i++)
	{
		MPU6050_GetData(&mpu6050_xyz);
        float AccMagnitude = sqrt(
            (mpu6050_xyz.accel_x * mpu6050_xyz.accel_x) +
            (mpu6050_xyz.accel_y * mpu6050_xyz.accel_y) +
            (mpu6050_xyz.accel_z * mpu6050_xyz.accel_z)
        );
		
        if(i > 0 && AccMagnitude - AccSum / ValidSamples < 100)  //这里判断是否静止，当加速度小于这个值代表静止累加一次数值
        {
            ValidSamples++;
            AccSum += AccMagnitude;
        }
        else
        {
            ValidSamples = 1;
            AccSum = AccMagnitude;
        }

        if(ValidSamples >= 50)
        {
            GRAVITY_BASE = AccSum / ValidSamples;
            AccFiltered = GRAVITY_BASE;
            IsCalibrated = 1;
        }
        else
        {
            if(i > 0)
            {
                AccFiltered = AccSum / ValidSamples;  //这里确保一定会进行记步程序，并且是为了有时候启动的时候能进行记步
                IsCalibrated = 1;
            }
        }
	}
}

int step_detect(void)
{
    static KalmanFilter_1D kalman_filter;
    static int step_count = 0;
    static uint8_t step_detected = 0;
    static uint32_t last_step_time = 0;
    static uint8_t first_run = 1;
    
    uint32_t current_time = HAL_GetTick();
    
    if(first_run)
    {
        InitStepDetection();
        kalman_filter_init(&kalman_filter);
        first_run = 0;
    }
    
    float AccMagnitude = sqrt
	(
        (mpu6050_xyz.accel_x * mpu6050_xyz.accel_x) +
        (mpu6050_xyz.accel_y * mpu6050_xyz.accel_y) +
        (mpu6050_xyz.accel_z * mpu6050_xyz.accel_z)
    );
    //AccFiltered = AccFiltered * 0.85 + AccMagnitude * 0.15;
    AccFiltered = kalman_filter_update(&kalman_filter, AccMagnitude);  //替换成新的卡尔曼滤波
    
	int32_t diff = AccFiltered - GRAVITY_BASE;
	//条件1：加速度超过600才算 条件2：判断是否准备测下次一波峰 条件3：一定要大于300ms才能计算一次步数(正常情况下200-400ms算是一次走路)
    if(diff > 600 && !step_detected && (current_time - last_step_time) > MIN_STEP_INTERVAL)
    {
        step_count++;
        step_detected = 1;
        last_step_time = current_time;
    }
	//这里判断我的加速度是否小于规定值，确认波谷的到来才会开始下一次波峰的测量
	if (step_detected && diff < -VALLEY_THRESHOLD) //
	{
		step_detected = 0;
	}
	return step_count;
}

