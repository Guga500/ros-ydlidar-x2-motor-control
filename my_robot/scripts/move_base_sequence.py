#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalStatus

"""
อ่านพารามิเตอร์จาก ROS param server:
  move_base_seq/p_seq   : ลิสต์จุด [x1,y1,z1, x2,y2,z2, ...] (เฟรม map)
  move_base_seq/yaw_seq : ลิสต์มุม yaw (องศา) ยาวเท่ากับจำนวนจุด
"""

def read_params():
    p = rospy.get_param("move_base_seq/p_seq", [])
    yaws_deg = rospy.get_param("move_base_seq/yaw_seq",
                               rospy.get_param("move_base_seq/yea_seq", []))  # เผื่อใช้ชื่อเดิม
    if not p or not yaws_deg:
        rospy.logerr("Params not found. Need move_base_seq/p_seq and move_base_seq/yaw_seq (or yea_seq).")
        return [], []

    if len(p) % 3 != 0:
        rospy.logwarn("p_seq length is not multiple of 3. Truncating extra values.")
        p = p[:(len(p)//3)*3]

    points = [p[i:i+3] for i in range(0, len(p), 3)]
    if len(points) != len(yaws_deg):
        rospy.logwarn("Number of points (%d) != number of yaws (%d). Using min length.",
                      len(points), len(yaws_deg))
    n = min(len(points), len(yaws_deg))
    points = points[:n]
    yaws_deg = yaws_deg[:n]
    return points, yaws_deg


class MoveBaseSeqNode(object):
    def __init__(self):
        rospy.init_node("move_base_sequence")

        self.points, self.yaws_deg = read_params()
        if not self.points:
            rospy.signal_shutdown("No waypoints.")
            return

        # สร้าง Pose list
        self.poses = []
        for (x, y, z), yaw_deg in zip(self.points, self.yaws_deg):
            q = Quaternion(*quaternion_from_euler(0.0, 0.0, math.radians(yaw_deg)))
            self.poses.append(Pose(Point(x, y, z), q))

        self.goal_idx = 0
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server…")
        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server not available.")
            rospy.signal_shutdown("move_base unavailable")
            return
        rospy.loginfo("Connected to move_base. Sending goals…")

        self.send_current_goal()
        rospy.spin()

    def send_current_goal(self):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = "map"
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose = self.poses[self.goal_idx]

        rospy.loginfo("Sending goal %d/%d: %s",
                      self.goal_idx + 1, len(self.poses), str(self.poses[self.goal_idx]))
        self.client.send_goal(g,
                              done_cb=self.done_cb,
                              active_cb=self.active_cb,
                              feedback_cb=self.feedback_cb)

    # callbacks
    def active_cb(self):
        rospy.loginfo("Goal %d is now active.", self.goal_idx + 1)

    def feedback_cb(self, fb):
        # ถ้าต้องการ log เพิ่มเติม เปิดบรรทัดล่างได้
        # rospy.loginfo("Feedback for goal %d", self.goal_idx + 1)
        pass

    def done_cb(self, status, result):
        name = {
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.REJECTED: "REJECTED",
            GoalStatus.PENDING: "PENDING",
            GoalStatus.ACTIVE: "ACTIVE",
        }.get(status, str(status))
        rospy.loginfo("Goal %d finished with status: %s", self.goal_idx + 1, name)

        if status == GoalStatus.SUCCEEDED:
            self.goal_idx += 1
            if self.goal_idx < len(self.poses):
                self.send_current_goal()
            else:
                rospy.loginfo("All goals reached. Shutting down.")
                rospy.signal_shutdown("sequence complete")
        elif status in (GoalStatus.ABORTED, GoalStatus.REJECTED):
            rospy.logerr("Goal %d failed (%s). Stopping sequence.", self.goal_idx + 1, name)
            rospy.signal_shutdown("sequence aborted")
        else:
            # CANCELLED/PREEMPTED/อื่น ๆ → ไม่ส่งต่ออัตโนมัติ
            rospy.logwarn("Goal %d ended with status %s. Not continuing.", self.goal_idx + 1, name)
            rospy.signal_shutdown("sequence stopped")


if __name__ == "__main__":
    try:
        MoveBaseSeqNode()
    except rospy.ROSInterruptException:
        pass
