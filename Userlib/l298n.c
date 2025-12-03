//
// Created by DKM on 2025/11/24.
//
#include "l298n.h"

void tt_motor_Init(TT_MOTOR* motor,
    GPIO_TypeDef* IN1_PORT,uint16_t IN1_PIN,
    GPIO_TypeDef* IN2_PORT,uint16_t IN2_PIN,
    TIM_HandleTypeDef *TIM, uint32_t CH)
{
    motor->IN1_PORT = IN1_PORT;
    motor->IN1_PIN = IN1_PIN;
    motor->IN2_PORT = IN2_PORT;
    motor->IN2_PIN = IN2_PIN;
    motor->TIM = TIM;
    motor->CH = CH;
    //开启pwm调速
    HAL_TIM_PWM_Start(TIM,CH);
    //规定正转方向
    HAL_GPIO_WritePin(motor->IN1_PORT,motor->IN1_PIN,GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->IN2_PORT,motor->IN2_PIN,GPIO_PIN_RESET);
}
void setSpeed(TT_MOTOR* motor,int16_t speed )
{
    //限幅
    if (speed > MAX_SPEED)
    {
        speed = MAX_SPEED;
    }else if (speed < -MAX_SPEED)
    {
        speed = -MAX_SPEED;
    }
    //设置正反转方向
    if (speed>=0)   //正转
    {
        HAL_GPIO_WritePin(motor->IN1_PORT,motor->IN1_PIN,GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_PORT,motor->IN2_PIN,GPIO_PIN_RESET);

        //设置速度
        __HAL_TIM_SetCompare(motor->TIM,motor->CH,speed);
    }else  //反转
    {
        HAL_GPIO_WritePin(motor->IN1_PORT,motor->IN1_PIN,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_PORT,motor->IN2_PIN,GPIO_PIN_SET);

        //设置速度
        __HAL_TIM_SetCompare(motor->TIM,motor->CH,-speed);
    }

    //设置速度

}
