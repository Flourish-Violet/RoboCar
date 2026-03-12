//
// ros_cmd.c – ROS ↔ STM32 velocity command handler
//
// Receives 10-byte binary frames over USART1 and drives the chassis when
// a valid frame arrives.  Falls back to PS2 control after a timeout.
//

#include "ros_cmd.h"
#include "usart.h"
#include "chassis.h"
#include "cmsis_os.h"

/* ---- Shared state -------------------------------------------------------- */
RosCmd_t ros_cmd = {0.0f, 0.0f, 0.0f, 0U, 0U};

/**
 * @brief  Flag visible to controller_task so it can yield to ROS commands.
 *         Set to 1 when a fresh ROS frame is present; cleared on timeout.
 */
volatile uint8_t ros_mode_active = 0;

/* ---- UART receive buffer ------------------------------------------------- */
static uint8_t  rx_byte;
static uint8_t  frame_buf[ROS_CMD_FRAME_LEN];
static uint8_t  frame_idx = 0;

extern float   motor_vel[4];

/* ---- Internal helpers ---------------------------------------------------- */

/**
 * @brief  Validate and decode a complete 10-byte frame.
 * @return 1 on success, 0 if the checksum fails.
 */
static uint8_t parse_frame(void)
{
    /* Checksum = XOR of bytes [2..8] (TYPE through OM_L) */
    uint8_t xor_val = 0;
    for (uint8_t i = 2U; i < (ROS_CMD_FRAME_LEN - 1U); i++)
    {
        xor_val ^= frame_buf[i];
    }
    if (xor_val != frame_buf[ROS_CMD_FRAME_LEN - 1U])
    {
        return 0U;
    }

    /* Decode int16_t values (big-endian, scaled ×100) */
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

/* ---- Public API ---------------------------------------------------------- */

/**
 * @brief  Initialise state and start the first byte-by-byte UART receive.
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
 * @brief  Called from HAL_UART_RxCpltCallback.
 *         Accumulates bytes into frame_buf and calls parse_frame() when full.
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

    /* Re-arm single-byte receive */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

/**
 * @brief  FreeRTOS task: applies ROS velocity commands to the chassis and
 *         handles command timeout.
 *
 * When a valid frame has been received recently the chassis is driven by
 * ros_cmd; otherwise ros_mode_active is cleared so controller_task can
 * resume PS2 control.
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
                /* Timeout – hand control back to PS2 */
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
