//
// Created by DKM on 2025/11/24.
//

#ifndef ROBOCAR_L298N_H
#define ROBOCAR_L298N_H

#include "tim.h"

#define MAX_SPEED 100

typedef struct {
    GPIO_TypeDef* IN1_PORT;uint16_t IN1_PIN;
    GPIO_TypeDef* IN2_PORT;uint16_t IN2_PIN;
    TIM_HandleTypeDef *TIM; uint32_t CH;
}TT_MOTOR;
void tt_motor_Init(TT_MOTOR* motor,
    GPIO_TypeDef* IN1_PORT,uint16_t IN1_PIN,
    GPIO_TypeDef* IN2_PORT,uint16_t IN2_PIN,
    TIM_HandleTypeDef *TIM, uint32_t CH);
void setSpeed(TT_MOTOR* motor,int8_t speed );
#endif //ROBOCAR_L298N_H