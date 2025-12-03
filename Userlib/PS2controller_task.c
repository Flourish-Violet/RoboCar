//
// Created by DKM on 2025/11/22.
//
#include "PS2.h"
#include "chassis.h"
#include "gimbal.h"
#include "tim.h"
ps2 ps2_Instance;
extern float motor_vel[4];
extern uint16_t K;
float vx,vy,omega;
uint8_t is_RoboCar_turn_ON = 0; //开机标识
void controller_task()
{
    //云台初始化
    gimbal_init();
    //水枪初始化
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
    //手柄初始化
    PS2_ConfigInit(); // 设置初始化
    osDelay(2000); //等待手柄连接
    //速度初始化
    vx = 0;
    vy = 0;
    omega = 0;

    while(1)
    {
        PS2_Get();
        PS2_DataParse(&ps2_Instance);

        // 检查手柄连接状态
        if(PS2_IsConnected())
        {
            PS2_DataParse(&ps2_Instance);
            //开机解锁
            if (ps2_Instance.START) is_RoboCar_turn_ON = 1;
            if (is_RoboCar_turn_ON)
            {
                //地盘控制
                if (ps2_Instance.LX>controller_dead_zone)
                {
                    vy = 1;
                }else if (ps2_Instance.LX<-controller_dead_zone)
                {
                    vy = -1;
                }else
                {
                    vy = 0;
                }

                if (ps2_Instance.LY>controller_dead_zone)
                {
                    vx = -1;
                }else if (ps2_Instance.LY<-controller_dead_zone)
                {
                    vx = 1;
                }else
                {
                    vx = 0;
                }

                //旋转
                // 根据按键控制震动
                if(ps2_Instance.R1)
                {
                    PS2_Vibration(0x01, 0xFF);  // 强烈震动
                    omega = -8;
                }
                else if(ps2_Instance.L1)
                {
                    PS2_Vibration(0x01, 0x80);  // 中等震动
                    omega = 8;
                }
                else
                {
                    PS2_Vibration(0x00, 0x00);  // 停止震动
                    omega = 0;
                }
                //底盘速度档
                if (ps2_Instance.triangle)
                {
                    K = 300;
                }
                if (ps2_Instance.furcation)
                {
                    K = 150;
                }

                //控制水枪
                if (ps2_Instance.R2)
                {
                    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
                    PS2_Vibration(0x01, 0xFF);
                }else
                {
                    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
                    PS2_Vibration(0x00, 0x00);
                }
                //设置速度
                setChassisSpeed(vx,vy,omega,motor_vel);
                //设置云台
                setGimbal(ps2_Instance.RY,ps2_Instance.RX);
            }

        }
        else
        {
            // 手柄未连接，尝试重新初始化
            PS2_ConfigInit();
        }

        osDelay(10);
    }

}
