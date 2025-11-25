//
// Created by DKM on 2025/11/21.
//

#ifndef ROBOCAR_PS2_H
#define ROBOCAR_PS2_H
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "math.h"

#define CS_L HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)  //拉低，开始通讯
#define CS_H HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)

typedef struct{
    uint8_t SELECT;
    uint8_t START;

    uint8_t UP;
    uint8_t DOWN;
    uint8_t LEFT;
    uint8_t RIGHT;

    uint8_t L1;
    uint8_t L2;
    uint8_t L3;
    uint8_t R1;
    uint8_t R2;
    uint8_t R3;

    uint8_t triangle;//三角
    uint8_t circle;//园
    uint8_t square;//方形
    uint8_t furcation;//叉

    float LX;
    float LY;
    float RX;
    float RY;

}ps2;
extern ps2 PS2;

void PS2_SetInit(void);
void PS2_Get(void);
void PS2_DataParse(ps2 *PS2_Handler);
void PS2_Vibration(uint8_t motor1, uint8_t motor2);
void PS2_EnableVibration(void);
void PS2_EnterConfig(void);
void PS2_ExitConfig(void);


void PS2_ShortPoll(void);
void PS2_TurnOnAnalogMode(void);
void PS2_VibrationMode(void);
void PS2_ConfigInit(void);
uint8_t PS2_IsConnected(void);
uint8_t PS2_GetMode(void);

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
float map_int_to_float(int int_value, int int_min, int int_max, float float_min, float float_max);

#endif //ROBOCAR_PS2_H