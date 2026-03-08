#include "motor.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "tim.h"
Motor Motor_l = {
  .pid = {
    .current = 0.0f,
    .target = 0.0f,
    .error = 0.0f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .Kp = 7.5f,
    .Ki = 12.0f,
    .Kd = 0.0f,
    .output = 0.0f
  },
  .motor = {
    .port1 = port1_l,
    .pin1 = pin1_l,
    .port2 = port2_l,
    .pin2 = pin2_l,
    .pwm_tim = &PWM,
    .pwm_channel = PWML,
    .encoder_tim = &ENCODERL
  }
};

Motor Motor_r = {
  .pid = {
    .current = 0.0f,
    .target = 0.0f,
    .error = 0.0f,
    .last_error = 0.0f,
    .integral = 0.0f,
    .Kp = 7.5f,
    .Ki = 12.0f,
    .Kd = 0.0f,
    .output = 0.0f
  },
  .motor = {
    .port1 = port1_r,
    .pin1 = pin1_r,
    .port2 = port2_r,
    .pin2 = pin2_r,
    .pwm_tim = &PWM,
    .pwm_channel = PWMR,
    .encoder_tim = &ENCODERR
  }
};

void motor_control(Motor* p){
 
  if (p->pid.output >= 0) {
    HAL_GPIO_WritePin(p->motor.port1, p->motor.pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p->motor.port1, p->motor.pin2, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(p->motor.port1, p->motor.pin1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(p->motor.port1, p->motor.pin2, GPIO_PIN_RESET);
    p->pid.output = -(p->pid.output);
  }
  __HAL_TIM_SET_COMPARE(p->motor.pwm_tim, p->motor.pwm_channel, (uint16_t)p->pid.output);
}

void send_vofa_data(Motor* p){
  HAL_UART_Transmit(&huart1, (uint8_t*)&p->pid.output, 4, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t*)&p->pid.current, 4, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t*)&p->pid.target, 4, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t*)tail, 4, HAL_MAX_DELAY);
}

float PID_Cal(PID* p){
  float output;
  p->error = p->target - p->current;
  p->integral += p->error;
  if (p->integral > integral_max) 
    p->integral = integral_max;
  else if (p->integral < -integral_max)
    p->integral = -integral_max;
  output = p->Kp * p->error + p->Ki * p->integral + p->Kd * (p->error - p->last_error);
  p->last_error = p->error;
  if (output > pwm_max) output = pwm_max;
  else if (output < -pwm_max) output = -pwm_max;
  p->output = output;
  return p->output;
}

void speed_Get(Motor* p){
  int16_t encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(p->motor.encoder_tim);
  p->pid.current = encoder_count * pi * D / (proportion * t);
  __HAL_TIM_SET_COUNTER(p->motor.encoder_tim, 0);
}

void speed_Cal(float v, float w){
  float speed_left = v - w * L / 2.0f;
  float speed_right = v + w * L / 2.0f;
  Motor_l.pid.target = speed_left;
  Motor_r.pid.target = speed_right;
}
