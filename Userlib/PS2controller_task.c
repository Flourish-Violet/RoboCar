//
// Created by DKM on 2025/11/22.
//
#include "PS2.h"
ps2 ps2_Instance;
void controller_task()
{
    PS2_ConfigInit(); // 设置初始化
    osDelay(2000); //等待手柄连接
    while(1)
    {
        PS2_Get();
        PS2_DataParse(&ps2_Instance);
        // 检查手柄连接状态
        if(PS2_IsConnected())
        {
            PS2_DataParse(&ps2_Instance);

            // 根据按键控制震动
            if(ps2_Instance.R1)
            {
                PS2_Vibration(0x01, 0xFF);  // 强烈震动
            }
            else if(ps2_Instance.L1)
            {
                PS2_Vibration(0x01, 0x80);  // 中等震动
            }
            else
            {
                PS2_Vibration(0x00, 0x00);  // 停止震动
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
