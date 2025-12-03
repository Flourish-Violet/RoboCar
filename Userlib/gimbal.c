//
// Created by DKM on 2025/11/25.
//
#include "gimbal.h"

#include "cmsis_os.h"
#include "PS2.h"
extern ps2 ps2_Instance;
extern TIM_HandleTypeDef htim3;
int16_t pwm1 = 1500;
int16_t pwm2 = 1500;
void gimbal_init()
{
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    // 初始化PWM占空比为50%
    pwm1 = 1500;
    pwm2 = 1500;

    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm1);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pwm2);

}

void setGimbal(int16_t pitch,int16_t yaw)
{
    pwm1 -= (yaw*k_gimbal_yaw);
    pwm2 -= (pitch*k_gimbal_pitch);
    if(pwm1 > yaw_MAX)
        pwm1 = yaw_MAX;
    else if(pwm1 < yaw_MIN)
        pwm1 = yaw_MIN;
    if(pwm2 > pitch_MAX)
        pwm2 = pitch_MAX;
    else if(pwm2 < pitch_MIN)
        pwm2 = pitch_MIN;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm1);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pwm2);
}
