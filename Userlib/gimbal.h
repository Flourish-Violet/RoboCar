//
// Created by DKM on 2025/12/3.
//

#ifndef ROBOCAR_GIMBAL_H
#define ROBOCAR_GIMBAL_H
#include "main.h"
#define k_gimbal_yaw 0.15f
#define k_gimbal_pitch 0.08f
#define pitch_MAX 2000
#define pitch_MIN 980
#define yaw_MAX 2500
#define yaw_MIN 500
void gimbal_init();
void setGimbal(int16_t pitch,int16_t yaw);
#endif //ROBOCAR_GIMBAL_H