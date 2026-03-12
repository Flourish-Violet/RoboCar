//
// ros_cmd.c – ROS ↔ STM32 速度指令处理
//
// 通过 USART1 接收 10 字节二进制帧，收到有效帧后驱动底盘运动。
// 若超时未收到新帧，则切回 PS2 手柄控制模式。
//

#include "ros_cmd.h"
#include "usart.h"
#include "chassis.h"
#include "cmsis_os.h"

/* ---- 共享状态 -------------------------------------------------------- */
RosCmd_t ros_cmd = {0.0f, 0.0f, 0.0f, 0U, 0U};

/**
 * @brief  ROS 控制模式标志，供 controller_task 判断是否让出控制权。
 *         收到新 ROS 帧时置 1，超时后清零。
 */
volatile uint8_t ros_mode_active = 0;

/* ---- UART 接收缓冲区 ------------------------------------------------- */
static uint8_t  rx_byte;
static uint8_t  frame_buf[ROS_CMD_FRAME_LEN];
static uint8_t  frame_idx = 0;

extern float   motor_vel[4];

/* ---- 内部辅助函数 ---------------------------------------------------- */

/**
 * @brief  校验并解析完整的 10 字节帧。
 * @return 解析成功返回 1，校验失败返回 0。
 */
static uint8_t parse_frame(void)
{
    /* 校验和 = 字节 [2..8]（TYPE 到 OM_L）的异或值 */
    uint8_t xor_val = 0;
    for (uint8_t i = 2U; i < (ROS_CMD_FRAME_LEN - 1U); i++)
    {
        xor_val ^= frame_buf[i];
    }
    if (xor_val != frame_buf[ROS_CMD_FRAME_LEN - 1U])
    {
        return 0U;
    }

    /* 解码 int16_t 值（大端序，放大 100 倍） */
    int16_t vx_raw    = (int16_t)(((uint16_t)frame_buf[3] << 8) | frame_buf[4]);
    int16_t vy_raw    = (int16_t)(((uint16_t)frame_buf[5] << 8) | frame_buf[6]);
    int16_t omega_raw = (int16_t)(((uint16_t)frame_buf[7] << 8) | frame_buf[8]);

    ros_cmd.vx           = vx_raw    / 100.0f;
    ros_cmd.vy           = vy_raw    / 100.0f;
    ros_cmd.omega        = omega_raw / 100.0f;
    ros_cmd.timestamp_ms = osKernelSysTick();
    ros_cmd.is_active    = 1U;

    return 1U;
}

/* ---- 公共 API ---------------------------------------------------------- */

/**
 * @brief  初始化状态变量并启动首次逐字节 UART 接收。
 */
void ros_cmd_init(void)
{
    ros_cmd.vx           = 0.0f;
    ros_cmd.vy           = 0.0f;
    ros_cmd.omega        = 0.0f;
    ros_cmd.is_active    = 0U;
    ros_mode_active      = 0U;
    frame_idx            = 0U;

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

/**
 * @brief  由 HAL_UART_RxCpltCallback 调用。
 *         将字节累积到 frame_buf，帧满后调用 parse_frame()。
 */
void ros_uart_rx_callback(void)
{
    switch (frame_idx)
    {
        case 0U:
            if (rx_byte == ROS_CMD_START1) { frame_buf[frame_idx++] = rx_byte; }
            break;
        case 1U:
            if (rx_byte == ROS_CMD_START2) { frame_buf[frame_idx++] = rx_byte; }
            else                           { frame_idx = 0U; }
            break;
        default:
            frame_buf[frame_idx++] = rx_byte;
            if (frame_idx == ROS_CMD_FRAME_LEN)
            {
                parse_frame();
                frame_idx = 0U;
            }
            break;
    }

    /* 重新启动单字节接收 */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

/**
 * @brief  FreeRTOS 任务：将 ROS 速度指令应用到底盘，并处理指令超时。
 *
 * 近期收到有效帧时由 ros_cmd 驱动底盘；
 * 否则清除 ros_mode_active，使 controller_task 恢复 PS2 手柄控制。
 */
void ros_uart_task(void const *argument)
{
    ros_cmd_init();

    for (;;)
    {
        if (ros_cmd.is_active)
        {
            uint32_t elapsed = osKernelSysTick() - ros_cmd.timestamp_ms;
            if (elapsed > ROS_CMD_TIMEOUT_MS)
            {
                /* 超时 – 将控制权归还给 PS2 手柄 */
                ros_cmd.vx        = 0.0f;
                ros_cmd.vy        = 0.0f;
                ros_cmd.omega     = 0.0f;
                ros_cmd.is_active = 0U;
                ros_mode_active   = 0U;
                setChassisSpeed(0.0f, 0.0f, 0.0f, motor_vel);
            }
            else
            {
                ros_mode_active = 1U;
                setChassisSpeed(ros_cmd.vx, ros_cmd.vy, ros_cmd.omega, motor_vel);
            }
        }

        osDelay(20U);
    }
}
