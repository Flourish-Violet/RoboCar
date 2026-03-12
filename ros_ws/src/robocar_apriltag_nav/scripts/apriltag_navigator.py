#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apriltag_navigator.py
~~~~~~~~~~~~~~~~~~~~~
ROS1 节点：利用 apriltag_ros 的检测结果将 RoboCar 导航至指定区域。

架构
----
  /tag_detections  (apriltag_ros/AprilTagDetectionArray)
        │
        ▼
  [apriltag_navigator]   ── 状态机 ──►  /cmd_vel  (geometry_msgs/Twist)

状态机
------
  SEARCHING  – 未检测到目标标签，机器人原地旋转搜索
  ALIGNING   – 检测到标签但横向偏移过大，原地旋转对齐
  APPROACHING– 对齐完成，向前靠近标签
  ARRIVED    – 距离误差小于到达阈值，停止运动

目标标签为 target_tag_ids 列表中第一个可见的标签。
"""

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray


# ---------------------------------------------------------------------------
# 状态标识符
# ---------------------------------------------------------------------------
STATE_SEARCHING  = "SEARCHING"
STATE_ALIGNING   = "ALIGNING"
STATE_APPROACHING = "APPROACHING"
STATE_ARRIVED    = "ARRIVED"


class AprilTagNavigator:
    """PD 控制器，引导机器人向最近的目标 AprilTag 靠近。"""

    def __init__(self):
        rospy.init_node("apriltag_navigator", anonymous=False)

        # ---- 参数 ---------------------------------------------------
        ns = "~"
        self.kp_angular      = rospy.get_param(ns + "kp_angular",      1.5)
        self.kd_angular      = rospy.get_param(ns + "kd_angular",      0.1)
        self.kp_linear       = rospy.get_param(ns + "kp_linear",       0.6)
        self.kd_linear       = rospy.get_param(ns + "kd_linear",       0.05)
        self.target_distance = rospy.get_param(ns + "target_distance",  0.30)
        self.arrival_thresh  = rospy.get_param(ns + "arrival_threshold",0.05)
        self.max_linear      = rospy.get_param(ns + "max_linear_vel",   0.5)
        self.max_angular     = rospy.get_param(ns + "max_angular_vel",  1.2)
        self.align_threshold = rospy.get_param(ns + "align_threshold",  0.15)
        self.search_omega    = rospy.get_param(ns + "search_angular_vel",0.4)
        self.cmd_rate        = rospy.get_param(ns + "cmd_rate",         20)
        self.target_ids      = rospy.get_param(ns + "target_tag_ids",   [0])

        # ---- 状态 --------------------------------------------------------
        self.state           = STATE_SEARCHING
        self.last_detection  = None   # 最新匹配标签的位姿
        self.prev_err_x      = 0.0    # 角速度微分项
        self.prev_err_z      = 0.0    # 线速度微分项
        self.arrived         = False

        # ---- ROS 话题 ------------------------------------------------------
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(
            "/tag_detections",
            AprilTagDetectionArray,
            self._detection_cb,
            queue_size=1,
        )

        rospy.loginfo(
            "AprilTag导航节点已就绪。目标标签ID: %s  目标距离: %.2f m",
            self.target_ids,
            self.target_distance,
        )

    # -----------------------------------------------------------------------
    # 回调函数
    # -----------------------------------------------------------------------

    def _detection_cb(self, msg):
        """保存最新一帧中第一个匹配标签的位姿。"""
        for detection in msg.detections:
            if any(tid in self.target_ids for tid in detection.id):
                self.last_detection = detection.pose.pose.pose
                return
        # 本帧中未检测到匹配标签
        self.last_detection = None

    # -----------------------------------------------------------------------
    # 控制辅助函数
    # -----------------------------------------------------------------------

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _pd_angular(self, err_x, dt):
        """角速度 PD 控制器（修正标签横向偏移）。"""
        derivative = (err_x - self.prev_err_x) / dt if dt > 0 else 0.0
        self.prev_err_x = err_x
        return -(self.kp_angular * err_x + self.kd_angular * derivative)

    def _pd_linear(self, err_z, dt):
        """线速度 PD 控制器（缩短与标签的距离）。"""
        derivative = (err_z - self.prev_err_z) / dt if dt > 0 else 0.0
        self.prev_err_z = err_z
        return self.kp_linear * err_z + self.kd_linear * derivative

    # -----------------------------------------------------------------------
    # 状态机
    # -----------------------------------------------------------------------

    def _transition(self, new_state):
        if new_state != self.state:
            rospy.loginfo("导航状态: %s → %s", self.state, new_state)
            self.state = new_state

    def _compute_twist(self, dt):
        """
        执行导航状态机的一个控制步，返回 Twist 速度指令。

        apriltag_ros 使用的相机坐标系约定：
          +x  向右
          +y  向下
          +z  远离相机（深度方向）
        """
        twist = Twist()

        if self.arrived:
            self._transition(STATE_ARRIVED)
            return twist  # 零速度

        pose = self.last_detection

        if pose is None:
            # ---- 搜索阶段 ------------------------------------------------
            self._transition(STATE_SEARCHING)
            self.prev_err_x = 0.0
            self.prev_err_z = 0.0
            twist.angular.z = self.search_omega
            return twist

        # 已检测到标签
        err_x = pose.position.x          # 横向偏移 [m]
        err_z = pose.position.z - self.target_distance  # 深度误差 [m]

        if abs(err_z) < self.arrival_thresh:
            # ---- 到达目标 --------------------------------------------------
            self.arrived = True
            self._transition(STATE_ARRIVED)
            rospy.loginfo(
                "已到达目标标签！当前位置: x=%.3f z=%.3f",
                pose.position.x,
                pose.position.z,
            )
            return twist  # 零速度

        angular_cmd = self._pd_angular(err_x, dt)

        if abs(err_x) > self.align_threshold:
            # ---- 对齐阶段 -------------------------------------------------
            self._transition(STATE_ALIGNING)
            twist.angular.z = self._clamp(angular_cmd, self.max_angular)
        else:
            # ---- 靠近阶段 ----------------------------------------------
            self._transition(STATE_APPROACHING)
            linear_cmd = self._pd_linear(err_z, dt)
            twist.linear.x  = self._clamp(linear_cmd,  self.max_linear)
            twist.angular.z = self._clamp(angular_cmd, self.max_angular)

        return twist

    # -----------------------------------------------------------------------
    # 主循环
    # -----------------------------------------------------------------------

    def spin(self):
        rate     = rospy.Rate(self.cmd_rate)
        prev_t   = rospy.Time.now()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt  = (now - prev_t).to_sec()
            prev_t = now

            twist = self._compute_twist(max(dt, 1e-3))
            self.cmd_pub.publish(twist)
            rate.sleep()


# ---------------------------------------------------------------------------
# 程序入口
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        navigator = AprilTagNavigator()
        navigator.spin()
    except rospy.ROSInterruptException:
        pass
