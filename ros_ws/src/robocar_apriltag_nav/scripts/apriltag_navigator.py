#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apriltag_navigator.py
~~~~~~~~~~~~~~~~~~~~~
ROS1 node that uses apriltag_ros detections to navigate the RoboCar to a
designated area.

Architecture
------------
  /tag_detections  (apriltag_ros/AprilTagDetectionArray)
        │
        ▼
  [apriltag_navigator]   ── state machine ──►  /cmd_vel  (geometry_msgs/Twist)

State machine
-------------
  SEARCHING  – no target tag visible; robot rotates to search
  ALIGNING   – tag visible but lateral offset too large; rotate in place
  APPROACHING– tag aligned; drive forward
  ARRIVED    – within arrival_threshold; stop

The target tag is the *first* visible tag whose ID is in `target_tag_ids`.
"""

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray


# ---------------------------------------------------------------------------
# State identifiers
# ---------------------------------------------------------------------------
STATE_SEARCHING  = "SEARCHING"
STATE_ALIGNING   = "ALIGNING"
STATE_APPROACHING = "APPROACHING"
STATE_ARRIVED    = "ARRIVED"


class AprilTagNavigator:
    """PD controller that steers the robot toward the nearest target AprilTag."""

    def __init__(self):
        rospy.init_node("apriltag_navigator", anonymous=False)

        # ---- Parameters ---------------------------------------------------
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

        # ---- State --------------------------------------------------------
        self.state           = STATE_SEARCHING
        self.last_detection  = None   # most recent matching tag pose
        self.prev_err_x      = 0.0    # for derivative term (angular)
        self.prev_err_z      = 0.0    # for derivative term (linear)
        self.arrived         = False

        # ---- ROS I/O ------------------------------------------------------
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(
            "/tag_detections",
            AprilTagDetectionArray,
            self._detection_cb,
            queue_size=1,
        )

        rospy.loginfo(
            "AprilTagNavigator ready. Target IDs: %s  Target distance: %.2f m",
            self.target_ids,
            self.target_distance,
        )

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def _detection_cb(self, msg):
        """Store the most recent pose of the first matching tag."""
        for detection in msg.detections:
            if any(tid in self.target_ids for tid in detection.id):
                self.last_detection = detection.pose.pose.pose
                return
        # No matching tag in this frame
        self.last_detection = None

    # -----------------------------------------------------------------------
    # Control helpers
    # -----------------------------------------------------------------------

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _pd_angular(self, err_x, dt):
        """PD controller for angular velocity (corrects lateral tag offset)."""
        derivative = (err_x - self.prev_err_x) / dt if dt > 0 else 0.0
        self.prev_err_x = err_x
        return -(self.kp_angular * err_x + self.kd_angular * derivative)

    def _pd_linear(self, err_z, dt):
        """PD controller for forward velocity (closes distance to tag)."""
        derivative = (err_z - self.prev_err_z) / dt if dt > 0 else 0.0
        self.prev_err_z = err_z
        return self.kp_linear * err_z + self.kd_linear * derivative

    # -----------------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------------

    def _transition(self, new_state):
        if new_state != self.state:
            rospy.loginfo("Navigator: %s → %s", self.state, new_state)
            self.state = new_state

    def _compute_twist(self, dt):
        """
        Run one step of the navigation state machine and return a Twist.

        The camera frame convention used by apriltag_ros is:
          +x  right
          +y  down
          +z  away from camera (depth)
        """
        twist = Twist()

        if self.arrived:
            self._transition(STATE_ARRIVED)
            return twist  # zero velocity

        pose = self.last_detection

        if pose is None:
            # ---- SEARCHING ------------------------------------------------
            self._transition(STATE_SEARCHING)
            self.prev_err_x = 0.0
            self.prev_err_z = 0.0
            twist.angular.z = self.search_omega
            return twist

        # Tag is visible
        err_x = pose.position.x          # lateral offset  [m]
        err_z = pose.position.z - self.target_distance  # depth error [m]

        if abs(err_z) < self.arrival_thresh:
            # ---- ARRIVED --------------------------------------------------
            self.arrived = True
            self._transition(STATE_ARRIVED)
            rospy.loginfo(
                "Arrived at target tag! Final position: x=%.3f z=%.3f",
                pose.position.x,
                pose.position.z,
            )
            return twist  # zero velocity

        angular_cmd = self._pd_angular(err_x, dt)

        if abs(err_x) > self.align_threshold:
            # ---- ALIGNING -------------------------------------------------
            self._transition(STATE_ALIGNING)
            twist.angular.z = self._clamp(angular_cmd, self.max_angular)
        else:
            # ---- APPROACHING ----------------------------------------------
            self._transition(STATE_APPROACHING)
            linear_cmd = self._pd_linear(err_z, dt)
            twist.linear.x  = self._clamp(linear_cmd,  self.max_linear)
            twist.angular.z = self._clamp(angular_cmd, self.max_angular)

        return twist

    # -----------------------------------------------------------------------
    # Main loop
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
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        navigator = AprilTagNavigator()
        navigator.spin()
    except rospy.ROSInterruptException:
        pass
