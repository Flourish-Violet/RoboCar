//
// ros_cmd.h – ROS ↔ STM32 velocity command protocol
//
// Frame layout (10 bytes total):
//   [0xAA][0x55][TYPE][VX_H][VX_L][VY_H][VY_L][OM_H][OM_L][XOR]
//
// VX / VY / OMEGA are int16_t values scaled ×100
// (i.e. 1.0 m/s or 1.0 rad/s is encoded as 100).
// XOR = checksum of bytes 2–8 (TYPE through OM_L).
//

#ifndef ROBOCAR_ROS_CMD_H
#define ROBOCAR_ROS_CMD_H

#include <stdint.h>
#include "cmsis_os.h"

/* ---- Protocol constants ------------------------------------------------- */
#define ROS_CMD_START1      0xAAU
#define ROS_CMD_START2      0x55U
#define ROS_CMD_TYPE_VEL    0x01U
#define ROS_CMD_FRAME_LEN   10U     /* total bytes per frame               */

/* ---- Runtime configuration ---------------------------------------------- */
/** If no valid frame is received within this window the robot stops. */
#define ROS_CMD_TIMEOUT_MS  500U

/* ---- Shared data --------------------------------------------------------- */
/**
 * @brief  Latest velocity command received from the ROS host.
 *
 * All fields are protected by the fact that they are written exclusively from
 * the UART RX callback (ISR context) and read from ros_uart_task, which are
 * serialised naturally on a single-core MCU.
 */
typedef struct
{
    float    vx;           /**< Forward  velocity [m/s]   */
    float    vy;           /**< Lateral  velocity [m/s]   */
    float    omega;        /**< Rotation velocity [rad/s] */
    uint32_t timestamp_ms; /**< osKernelSysTick() at last valid frame */
    uint8_t  is_active;    /**< 1 while commands are fresh, 0 after timeout */
} RosCmd_t;

extern RosCmd_t ros_cmd;

/* ---- Public API ---------------------------------------------------------- */
void ros_cmd_init(void);
void ros_uart_task(void const *argument);

/* Called from HAL_UART_RxCpltCallback – do not call directly. */
void ros_uart_rx_callback(void);

#endif /* ROBOCAR_ROS_CMD_H */
