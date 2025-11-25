//
// Created by DKM on 2025/11/17.
//
#include "chassis.h"
#include "PS2.h"
#include "l298n.h"
extern ps2 ps2_Instance;

TT_MOTOR motor1,motor2;
void Movement_Inverse_Kinematics(float vx,float vy,float omega,float* receiver)
{
    receiver[0]=(-vx+vy+omega*(wheel_position_X+wheel_position_Y))*1.0f/wheel_Radius;
    receiver[1]=(-vx-vy+omega*(wheel_position_X+wheel_position_Y))*1.0f/wheel_Radius;
    receiver[2]=(vx-vy+omega*(wheel_position_X+wheel_position_Y))*1.0f/wheel_Radius;
    receiver[3]=(vx+vy+omega*(wheel_position_X+wheel_position_Y))*1.0f/wheel_Radius;
}

float motor_vel[4];
void chassis_task()
{
    int8_t setspeed = 100;
    tt_motor_Init(&motor1,GPIOB,GPIO_PIN_12,GPIOB,GPIO_PIN_13,&htim2,TIM_CHANNEL_1);
    tt_motor_Init(&motor2,GPIOB,GPIO_PIN_14,GPIOB,GPIO_PIN_15,&htim2,TIM_CHANNEL_2);
    while (1)
    {
        if (ps2_Instance.LY>controller_dead_zone)
        {
            setSpeed(&motor1,setspeed);
            setSpeed(&motor2,setspeed);
        }else if (ps2_Instance.LY<controller_dead_zone)
        {
            setSpeed(&motor1,-setspeed);
            setSpeed(&motor2,-setspeed);
        }else
        {
            setSpeed(&motor1,0);
            setSpeed(&motor2,0);
        }
        osDelay(1);
    }

}
