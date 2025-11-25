//
// Created by DKM on 2025/11/17.
//

#ifndef ROBOCAR_CHASSIS_H
#define ROBOCAR_CHASSIS_H

#include "cmsis_os.h"
//单位 米
#define wheel_position_X 0.015f
#define wheel_position_Y 0.01f
#define wheel_Radius 0.03f

#define controller_dead_zone 0

void Movement_Inverse_Kinematics(float vx,float vy,float omega,float* receiver);
#endif //ROBOCAR_CHASSIS_H