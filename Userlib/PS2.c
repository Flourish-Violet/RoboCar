#include "PS2.h"

uint8_t cmd[9] = {0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  // 请求接受数据
uint8_t PS2data[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //存储手柄返回数据


extern SPI_HandleTypeDef hspi1;

/******************************************************
函数功能: 完整的配置初始化（官方例程风格）
入口参数: 无
返回  值: 无
******************************************************/
void PS2_ConfigInit(void)
{
	// 1. 多次短轮询建立稳定连接
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();

	// 2. 进入配置模式
	PS2_EnterConfig();

	// 3. 开启模拟模式（摇杆功能）
	PS2_TurnOnAnalogMode();

	// 4. 开启震动模式
	PS2_VibrationMode();

	// 5. 退出配置模式并保存设置
	PS2_ExitConfig();

	delay_ms(100); // 等待配置生效
}
/******************************************************
函数功能: 短轮询命令 - 用于建立稳定通信
入口参数: 无
返回  值: 无
******************************************************/
void PS2_ShortPoll(void)
{
    uint8_t shortpoll_cmd[5] = {0x01, 0x42, 0x00, 0x00, 0x00};
    uint8_t response[5] = {0};

    CS_L;
    delay_us(16);

    // 发送短轮询命令序列
    for(uint8_t i = 0; i < 5; i++)
    {
        HAL_SPI_TransmitReceive(&hspi1, &shortpoll_cmd[i], &response[i], 1, 10);
        delay_us(16);
    }

    CS_H;
    delay_us(16);
}

/******************************************************
函数功能: 开启模拟模式 - 启用摇杆功能
入口参数: 无
返回  值: 无
******************************************************/
void PS2_TurnOnAnalogMode(void)
{
    uint8_t analog_cmd[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    uint8_t response[9] = {0};

    CS_L;
    delay_us(16);

    for(uint8_t i = 0; i < 9; i++)
    {
        HAL_SPI_TransmitReceive(&hspi1, &analog_cmd[i], &response[i], 1, 10);
        delay_us(16);
    }

    CS_H;
    delay_us(16);
}

/******************************************************
函数功能: 震动模式设置
入口参数: 无
返回  值: 无
******************************************************/
void PS2_VibrationMode(void)
{
    uint8_t vib_mode_cmd[5] = {0x01, 0x4D, 0x00, 0x00, 0x01};
    uint8_t response[5] = {0};

    CS_L;
    delay_us(16);

    for(uint8_t i = 0; i < 5; i++)
    {
        HAL_SPI_TransmitReceive(&hspi1, &vib_mode_cmd[i], &response[i], 1, 10);
        delay_us(16);
    }

    CS_H;
    delay_us(16);
}


/******************************************************
函数功能: 检查手柄连接状态
入口参数: 无
返回  值: 1-已连接, 0-未连接
******************************************************/
uint8_t PS2_IsConnected(void)
{
    // 检查数据有效性，通常PS2data[1]应为0x73或0x41
    if(PS2data[1] == 0x41 || PS2data[1] == 0x73) {
        return 1;
    }
    return 0;
}

/******************************************************
函数功能: 获取手柄模式
入口参数: 无
返回  值: 0-红灯模式, 1-绿灯模式
******************************************************/
uint8_t PS2_GetMode(void)
{
    if(PS2data[1] == 0x73) {
        return 0; // 红灯模式
    } else if(PS2data[1] == 0x41) {
        return 1; // 绿灯模式
    }
    return 2; // 未知模式
}

void PS2_Get(void)    //接受ps2数据
{
	uint8_t i = 0;

	CS_L;  //低高，开始通讯

	HAL_SPI_TransmitReceive(&hspi1,&cmd[0],&PS2data[0],1,10); // 发送0x01，请求接受数据
	delay_us(16);

	HAL_SPI_TransmitReceive(&hspi1,&cmd[1],&PS2data[1],1,10); // 发送0x42，接受0x01（PS2表示开始通信）
	delay_us(16);

	HAL_SPI_TransmitReceive(&hspi1,&cmd[2],&PS2data[2],1,10); // 发送0x00，接受ID（红绿灯模式）
	delay_us(16);

	for(i = 3;i <9;i++)
	{
		HAL_SPI_TransmitReceive(&hspi1,&cmd[2],&PS2data[i],1,0xffff); // 接受数据
		delay_us(16);
	}

	CS_H;

}

void PS2_DataParse(ps2 *PS2_Handler)
{
    // 解析按键数据
    PS2_Handler->SELECT = (PS2data[3] & 0x01) ? 0 : 1;
    PS2_Handler->START  = (PS2data[3] & 0x08) ? 0 : 1;

    PS2_Handler->UP    = (PS2data[3] & 0x10) ? 0 : 1;
    PS2_Handler->DOWN  = (PS2data[3] & 0x40) ? 0 : 1;
    PS2_Handler->LEFT  = (PS2data[3] & 0x80) ? 0 : 1;
    PS2_Handler->RIGHT = (PS2data[3] & 0x20) ? 0 : 1;

    PS2_Handler->L1 = (PS2data[4] & 0x04) ? 0 : 1;
    PS2_Handler->L2 = (PS2data[4] & 0x01) ? 0 : 1;
    PS2_Handler->L3 = (PS2data[3] & 0x02) ? 0 : 1;
    PS2_Handler->R1 = (PS2data[4] & 0x08) ? 0 : 1;
    PS2_Handler->R2 = (PS2data[4] & 0x02) ? 0 : 1;
    PS2_Handler->R3 = (PS2data[3] & 0x04) ? 0 : 1;

    PS2_Handler->triangle = (PS2data[4] & 0x10) ? 0 : 1;
    PS2_Handler->circle   = (PS2data[4] & 0x20) ? 0 : 1;
    PS2_Handler->square   = (PS2data[4] & 0x80) ? 0 : 1;
    PS2_Handler->furcation= (PS2data[4] & 0x40) ? 0 : 1;

    // 解析摇杆数据并映射到-128~128范围
    // 注意：PS2摇杆原始数据范围通常是0x00~0xFF
    // 中间位置大约是0x80

    // 右摇杆Y轴
    PS2_Handler->RY = -map_int_to_float((int)PS2data[6], 0, 255, -100.0f, 100.0f);

    // 左摇杆X轴
    PS2_Handler->LX = map_int_to_float((int)PS2data[7], 0, 255, -100.0f, 100.0f);

    // 左摇杆Y轴
    PS2_Handler->LY = -map_int_to_float((int)PS2data[8], 0, 255, -100.0f, 100.0f);

    // 右摇杆X轴
    PS2_Handler->RX = map_int_to_float((int)PS2data[5], 0, 255, -100.0f, 100.0f);

    // 可选：添加死区处理，防止摇杆微小漂移
    // 如果摇杆值在死区范围内，则设为0
    #define DEAD_ZONE 10.0f

    if(fabsf(PS2_Handler->LX) < DEAD_ZONE) PS2_Handler->LX = 0.0f;
    if(fabsf(PS2_Handler->LY) < DEAD_ZONE) PS2_Handler->LY = 0.0f;
    if(fabsf(PS2_Handler->RX) < DEAD_ZONE) PS2_Handler->RX = 0.0f;
    if(fabsf(PS2_Handler->RY) < DEAD_ZONE) PS2_Handler->RY = 0.0f;
}

/******************************************************
函数功能: 手柄震动函数
入口参数: motor1:右侧小震动电机 0x00关，0x01开
		 motor2:左侧大震动电机 0x00~0xFF，值越大震动越强
返回  值: 无
******************************************************/
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	uint8_t vibration_cmd[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t response[9] = {0};

	// 构建震动命令 - 根据PS2协议，震动数据在第4和第5字节
	vibration_cmd[3] = motor1 ? 0x01 : 0x00;  // 小电机开关
	vibration_cmd[4] = motor2;                // 大电机强度

	CS_L;
	delay_us(16);

	// 发送完整的9字节命令
	for(uint8_t i = 0; i < 9; i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, &vibration_cmd[i], &response[i], 1, 10);
		delay_us(16);  // 增加延时确保稳定
	}

	CS_H;
	delay_us(16);
}

/******************************************************
函数功能: 开启震动模式
入口参数: 无
返回  值: 无
******************************************************/
void PS2_EnableVibration(void)
{
	uint8_t vib_cmd[9] = {0x01, 0x4D, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
	uint8_t response[9] = {0};

	CS_L;
	delay_us(16);

	for(uint8_t i = 0; i < 9; i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, &vib_cmd[i], &response[i], 1, 10);
		delay_us(5);
	}

	CS_H;
	delay_us(16);
}

/******************************************************
函数功能: 进入配置模式
入口参数: 无
返回  值: 无
******************************************************/
void PS2_EnterConfig(void)
{
	uint8_t config_cmd[9] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t response[9] = {0};

	CS_L;
	delay_us(16);

	for(uint8_t i = 0; i < 9; i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, &config_cmd[i], &response[i], 1, 10);
		delay_us(5);
	}

	CS_H;
	delay_us(16);
}

/******************************************************
函数功能: 退出配置模式
入口参数: 无
返回  值: 无
******************************************************/
void PS2_ExitConfig(void)
{
	uint8_t exit_cmd[9] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
	uint8_t response[9] = {0};

	CS_L;
	delay_us(16);

	for(uint8_t i = 0; i < 9; i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, &exit_cmd[i], &response[i], 1, 10);
		delay_us(5);
	}

	CS_H;
	delay_us(16);
}

/**************************************************************************
函数功能：手柄配置初始化
入口参数：无
返回  值：无
**************************************************************************/
void PS2_SetInit(void)
{
	PS2_EnterConfig();		//进入配置模式
	PS2_EnableVibration();
	PS2_ExitConfig();
}

// 微秒级延时函数
void delay_us(uint32_t us)
{
    // 根据72MHz时钟频率调整循环次数
    // 通过实际测试校准这个值
    uint32_t delay_cycles = us * 9;  // 需要根据实际情况校准

    for(uint32_t i = 0; i < delay_cycles; i++)
    {
        __NOP();  // 空操作，确保循环不被优化
    }
}

// 毫秒级延时函数
void delay_ms(uint32_t ms)
{
   osDelay(ms);
}

float map_int_to_float(int int_value, int int_min, int int_max, float float_min, float float_max) {
    // 处理边界情况
    if (int_max == int_min) {
        return (float_min + float_max) / 2.0f;
    }

    // 直接计算映射，不进行钳制
    float ratio = (float)(int_value - int_min) / (float)(int_max - int_min);
    return float_min + ratio * (float_max - float_min);
}