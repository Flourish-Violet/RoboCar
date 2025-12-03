//
// Created by DKM on 2025/11/17.
//
#include "chassis.h"
#include "PS2.h"
#include "l298n.h"
extern ps2 ps2_Instance;
float motor_vel[4]={0}; //存储底盘解算后各个电机的速度
TT_MOTOR motor1,motor2,motor3,motor4;
uint16_t K = 150;  // K为调速比例系数 K越大速度越快 K<=300,
//底盘解算
void Movement_Inverse_Kinematics(float vx,float vy,float omega,float* receiver)
{
    receiver[0]=(-vx+vy+omega*(wheel_position_X+wheel_position_Y))*1.0f/wheel_Radius;
    receiver[1]=(-vx-vy+omega*(wheel_position_X+wheel_position_Y))*1.0f/wheel_Radius;
    receiver[2]=(vx-vy+omega*(wheel_position_X+wheel_position_Y))*1.0f/wheel_Radius;
    receiver[3]=(vx+vy+omega*(wheel_position_X+wheel_position_Y))*1.0f/wheel_Radius;
}
//设置速度
void setChassisSpeed(float vx,float vy,float omega,float* receiver)
{
    Movement_Inverse_Kinematics(vx,vy,omega,motor_vel);
    setSpeed(&motor1,(int16_t)(receiver[0]*K));  //保留两位小数
    setSpeed(&motor2,(int16_t)(receiver[1]*K));
    setSpeed(&motor3,(int16_t)(receiver[2]*K));
    setSpeed(&motor4,(int16_t)(receiver[3]*K));
}

void chassis_task()
{
    //电机初始化
    tt_motor_Init(&motor1,GPIOB,GPIO_PIN_12,GPIOB,GPIO_PIN_13,&htim2,TIM_CHANNEL_1);
    tt_motor_Init(&motor2,GPIOB,GPIO_PIN_14,GPIOB,GPIO_PIN_15,&htim2,TIM_CHANNEL_2);
    tt_motor_Init(&motor3,GPIOA,GPIO_PIN_8,GPIOA,GPIO_PIN_9,&htim2,TIM_CHANNEL_3);
    tt_motor_Init(&motor4,GPIOB,GPIO_PIN_0,GPIOB,GPIO_PIN_1,&htim2,TIM_CHANNEL_4);
    while (1)
    {

        osDelay(1);
    }

}
