//
// Created by DKM on 2025/12/3.
//
#include "machinGun.h"
#include "cmsis_os.h"
// void machineGun_fire()
// {
//     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
// }
// void machineGun_ThreeRound_fire()
// {
//     for (uint8_t i=0;i<3;i++)
//     {
//
//     }
//     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
// }
void machineGun_init()
{
    //水枪初始化
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
}

void machineGun_task(){
    machineGun_init();
    while (1)
    {
        osDelay(1);
    }
}