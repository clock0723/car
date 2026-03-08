#include "mpu6050.h"
#include "i2c.h"
#include "math.h"
#include "motor.h"

void mpu6050_init(void){
    uint8_t data[2]={0};
    data[0]=0x6b; //复位寄存器地址
    data[1]=0x80; //复位
   HAL_I2C_Master_Transmit(&hi2c1, mpu6050_add_w, data, 2, HAL_MAX_DELAY);
    data[0]=0x6b; //睡眠寄存器地址
    data[1]=0x00; //唤醒
    HAL_I2C_Master_Transmit(&hi2c1, mpu6050_add_w, data, 2, HAL_MAX_DELAY);
    data[0]=0x1b; //陀螺仪配置寄存器地址
    data[1]=0x00; //设置量程，00为±250°/s，01为±500°/s，10为±1000°/s，11为±2000°/s
    HAL_I2C_Master_Transmit(&hi2c1, mpu6050_add_w, data, 2, HAL_MAX_DELAY);
}

void mpu6050_read( float *yaw){
    uint8_t data[2]={0};
    int16_t gz_raw;
    HAL_I2C_Mem_Read(&hi2c1,mpu6050_add_w,0x47,2,data,2,HAL_MAX_DELAY);
    gz_raw=(int16_t)(data[0]<<8)+data[1];
    *yaw = gz_raw * to_rad * 250.0f / 32768.0f*to_rad;
}

void angle_Cal(position* pos, PID* pid){
    float target_yaw = atan2f(pos->target_y, pos->target_x + 1e-6f); //避免除以零
    pid->target = target_yaw;
    float error = target_yaw - pid->current;
    if (error > pi) {
        error -= 2 * pi;
    } else if (error < -pi) {
        error += 2 * pi;
    }
   pid->error = error;
}

void distance_Cal(position* pos, PID* pid){
    float target_distance = sqrtf(pos->target_x * pos->target_x + pos->target_y * pos->target_y);
    pid->target = target_distance;
     float distance_err = sqrtf(powf(pos->target_x - pos->current_x, 2) + powf(pos->target_y - pos->current_y, 2));
     pid->error = distance_err;
}

void position_Cal(position* pos, float yaw, float speed_left, float speed_right){
    float v = (speed_left + speed_right) / 2.0f;
    float w = (speed_right - speed_left) / L;
    pos->current_x += v * cosf(yaw) * t;
    pos->current_y += v * sinf(yaw) * t;
}
