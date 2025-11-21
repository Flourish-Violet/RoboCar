//
// Created by DKM on 2025/11/17.
//
#include "chassis.h"

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

    while (1)
    {
        Movement_Inverse_Kinematics(1,0,0,motor_vel);

        osDelay(1);
    }
}

void controller_task()
{

}
void gimbal_task()
{

}