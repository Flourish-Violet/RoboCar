#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_bridge.py
~~~~~~~~~~~~~~~~
ROS1 node that translates geometry_msgs/Twist messages on /cmd_vel into the
binary serial protocol expected by the STM32 chassis firmware.

Frame layout (10 bytes)
-----------------------
  Byte 0   : 0xAA  (start marker 1)
  Byte 1   : 0x55  (start marker 2)
  Byte 2   : 0x01  (command type: velocity)
  Bytes 3–4: vx    (int16_t big-endian, scaled × 100)
  Bytes 5–6: vy    (int16_t big-endian, scaled × 100)
  Bytes 7–8: omega (int16_t big-endian, scaled × 100)
  Byte 9   : XOR checksum of bytes 2–8

Scaling
-------
  A velocity of  1.0 m/s  is encoded as  100 (int16).
  A velocity of -0.5 m/s  is encoded as  -50 (int16).
  Maximum representable speed: ±327.67 m/s (far above any physical limit).
"""

import struct
import rospy
import serial
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
FRAME_START1   = 0xAA
FRAME_START2   = 0x55
CMD_TYPE_VEL   = 0x01
SCALE          = 100          # multiply float [m/s or rad/s] → int16


def _encode_frame(vx: float, vy: float, omega: float) -> bytes:
    """
    Pack a velocity command into the 10-byte binary frame.

    Parameters
    ----------
    vx    : forward  velocity [m/s]
    vy    : lateral  velocity [m/s]
    omega : rotation velocity [rad/s]

    Returns
    -------
    bytes of length 10
    """
    vx_int    = int(round(vx    * SCALE))
    vy_int    = int(round(vy    * SCALE))
    omega_int = int(round(omega * SCALE))

    # Clamp to int16 range
    vx_int    = max(-32768, min(32767, vx_int))
    vy_int    = max(-32768, min(32767, vy_int))
    omega_int = max(-32768, min(32767, omega_int))

    # Pack data bytes (type + three int16 big-endian)
    data = struct.pack(
        ">Bhhh",
        CMD_TYPE_VEL,
        vx_int,
        vy_int,
        omega_int,
    )  # 7 bytes

    # XOR checksum over all 7 data bytes
    checksum = 0
    for byte in data:
        checksum ^= byte

    return bytes([FRAME_START1, FRAME_START2]) + data + bytes([checksum])


class SerialBridge:
    """Subscribes to /cmd_vel and forwards commands to the STM32 over serial."""

    def __init__(self):
        rospy.init_node("serial_bridge", anonymous=False)

        port     = rospy.get_param("~port",     "/dev/ttyUSB0")
        baudrate = rospy.get_param("~baudrate", 115200)
        timeout  = rospy.get_param("~timeout",  1.0)

        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            rospy.loginfo("SerialBridge: opened %s at %d baud", port, baudrate)
        except serial.SerialException as exc:
            rospy.logerr("SerialBridge: cannot open serial port %s – %s", port, exc)
            raise SystemExit(1) from exc

        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_cb, queue_size=1)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo("SerialBridge: ready, listening on /cmd_vel")

    # -----------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist):
        """Encode and transmit a Twist message."""
        frame = _encode_frame(
            vx    = msg.linear.x,
            vy    = msg.linear.y,
            omega = msg.angular.z,
        )
        try:
            self.ser.write(frame)
        except serial.SerialException as exc:
            rospy.logerr("SerialBridge: write error – %s", exc)

    def _shutdown(self):
        """Send a stop command and close the port on node shutdown."""
        try:
            stop_frame = _encode_frame(0.0, 0.0, 0.0)
            self.ser.write(stop_frame)
            self.ser.close()
            rospy.loginfo("SerialBridge: serial port closed")
        except serial.SerialException:
            pass

    def spin(self):
        rospy.spin()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        bridge = SerialBridge()
        bridge.spin()
    except rospy.ROSInterruptException:
        pass
