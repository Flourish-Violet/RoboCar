//
// Created by DKM on 2025/12/3.
//

#ifndef ROBOCAR_GIMBAL_H
#define ROBOCAR_GIMBAL_H
#include "main.h"
#define k_gimbal_yaw 0.12f
#define k_gimbal_pitch 0.05f
#define pitch_MAX 2000
#define pitch_MIN 980
#define yaw_MAX 2500
#define yaw_MIN 500
#define lift_max 2500
#define lift_min 500
void gimbal_init();
void setGimbal(int16_t pitch,int16_t yaw);
void setLift(int16_t lift_x);
#endif //ROBOCAR_GIMBAL_H