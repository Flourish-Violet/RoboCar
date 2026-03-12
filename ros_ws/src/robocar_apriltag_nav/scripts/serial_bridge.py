#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge.py
~~~~~~~~~~~~~~~~
ROS1 节点：将 /cmd_vel 话题上的 geometry_msgs/Twist 消息转换为
STM32 底盘固件所需的二进制串口协议并发送。

数据帧格式（共 10 字节）
------------------------
  字节 0   : 0xAA  （帧头 1）
  字节 1   : 0x55  （帧头 2）
  字节 2   : 0x01  （指令类型：速度）
  字节 3–4 : vx    （int16_t 大端序，放大 100 倍）
  字节 5–6 : vy    （int16_t 大端序，放大 100 倍）
  字节 7–8 : omega （int16_t 大端序，放大 100 倍）
  字节 9   : 字节 2–8 的 XOR 校验值

缩放比例
--------
   1.0 m/s  编码为  100（int16）
  -0.5 m/s  编码为  -50（int16）
  最大可表示速度：±327.67 m/s（远超物理限制，实际不会到达）
"""

import struct
import rospy
import serial
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# 协议常量
# ---------------------------------------------------------------------------
FRAME_START1   = 0xAA
FRAME_START2   = 0x55
CMD_TYPE_VEL   = 0x01
SCALE          = 100          # float [m/s 或 rad/s] → int16 的缩放系数


def _encode_frame(vx: float, vy: float, omega: float) -> bytes:
    """
    将速度指令打包为 10 字节二进制帧。

    参数
    ----
    vx    : 前进速度 [m/s]
    vy    : 横向速度 [m/s]
    omega : 旋转角速度 [rad/s]

    返回
    ----
    长度为 10 的 bytes 对象
    """
    vx_int    = int(round(vx    * SCALE))
    vy_int    = int(round(vy    * SCALE))
    omega_int = int(round(omega * SCALE))

    # 限幅至 int16 范围
    vx_int    = max(-32768, min(32767, vx_int))
    vy_int    = max(-32768, min(32767, vy_int))
    omega_int = max(-32768, min(32767, omega_int))

    # 打包数据字节（类型 + 三个大端序 int16）
    data = struct.pack(
        ">Bhhh",
        CMD_TYPE_VEL,
        vx_int,
        vy_int,
        omega_int,
    )  # 7 字节

    # 对全部 7 个数据字节求 XOR 校验值
    checksum = 0
    for byte in data:
        checksum ^= byte

    return bytes([FRAME_START1, FRAME_START2]) + data + bytes([checksum])


class SerialBridge:
    """订阅 /cmd_vel 并通过串口将速度指令转发给 STM32。"""

    def __init__(self):
        rospy.init_node("serial_bridge", anonymous=False)

        port     = rospy.get_param("~port",     "/dev/ttyUSB0")
        baudrate = rospy.get_param("~baudrate", 115200)
        timeout  = rospy.get_param("~timeout",  1.0)

        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            rospy.loginfo("串口桥接节点: 已打开 %s，波特率 %d", port, baudrate)
        except serial.SerialException as exc:
            rospy.logerr("串口桥接节点: 无法打开串口 %s – %s", port, exc)
            raise SystemExit(1) from exc

        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_cb, queue_size=1)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("串口桥接节点: 已就绪，正在监听 /cmd_vel")

    # -----------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist):
        """编码并发送 Twist 消息。"""
        frame = _encode_frame(
            vx    = msg.linear.x,
            vy    = msg.linear.y,
            omega = msg.angular.z,
        )
        try:
            self.ser.write(frame)
        except serial.SerialException as exc:
            rospy.logerr("串口桥接节点: 写入错误 – %s", exc)

    def _shutdown(self):
        """节点关闭时发送停止帧并关闭串口。"""
        try:
            stop_frame = _encode_frame(0.0, 0.0, 0.0)
            self.ser.write(stop_frame)
            self.ser.close()
            rospy.loginfo("串口桥接节点: 串口已关闭")
        except serial.SerialException:
            pass

    def spin(self):
        rospy.spin()


# ---------------------------------------------------------------------------
# 程序入口
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        bridge = SerialBridge()
        bridge.spin()
    except rospy.ROSInterruptException:
        pass
