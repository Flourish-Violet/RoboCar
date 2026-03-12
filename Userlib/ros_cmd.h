//
// ros_cmd.h – ROS ↔ STM32 速度指令通信协议
//
// 数据帧格式（共 10 字节）：
//   [0xAA][0x55][TYPE][VX_H][VX_L][VY_H][VY_L][OM_H][OM_L][XOR]
//
// VX / VY / OMEGA 为 int16_t 类型，放大 100 倍传输
//（即 1.0 m/s 或 1.0 rad/s 编码为 100）。
// XOR = 字节 2～8（TYPE 到 OM_L）的异或校验值。
//

#ifndef ROBOCAR_ROS_CMD_H
#define ROBOCAR_ROS_CMD_H

#include <stdint.h>
#include "cmsis_os.h"

/* ---- 协议常量 ------------------------------------------------- */
#define ROS_CMD_START1      0xAAU
#define ROS_CMD_START2      0x55U
#define ROS_CMD_TYPE_VEL    0x01U
#define ROS_CMD_FRAME_LEN   10U     /* 每帧总字节数 */

/* ---- 运行时配置 ---------------------------------------------- */
/** 在此时间窗口内未收到有效帧则停止运动。 */
#define ROS_CMD_TIMEOUT_MS  500U

/* ---- 共享数据 ------------------------------------------------- */
/**
 * @brief  从 ROS 主机接收到的最新速度指令。
 *
 * 所有字段仅在 UART 接收回调（中断上下文）中写入，
 * 并在 ros_uart_task 中读取，单核 MCU 上天然串行化，无需额外互斥。
 */
typedef struct
{
    float    vx;           /**< 前进速度 [m/s]   */
    float    vy;           /**< 横向速度 [m/s]   */
    float    omega;        /**< 旋转角速度 [rad/s] */
    uint32_t timestamp_ms; /**< 最后一帧有效数据的时间戳 osKernelSysTick() */
    uint8_t  is_active;    /**< 1 表示指令新鲜有效，0 表示已超时 */
} RosCmd_t;

extern RosCmd_t ros_cmd;

/* ---- 公共 API ---------------------------------------------------------- */
void ros_cmd_init(void);
void ros_uart_task(void const *argument);

/* 由 HAL_UART_RxCpltCallback 调用，请勿直接调用。 */
void ros_uart_rx_callback(void);

#endif /* ROBOCAR_ROS_CMD_H */
