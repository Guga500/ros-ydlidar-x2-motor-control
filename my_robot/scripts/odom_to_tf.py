#!/usr/bin/env python3
import rospy
import math                              # ★ เพิ่มบรรทัดนี้
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

BASE_FRAME = rospy.get_param("~base_frame", "base_footprint")
ODOM_FRAME = rospy.get_param("~odom_frame", "odom")

br = None

def cb(msg: Odometry):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()    # ใช้เวลาปัจจุบันกัน lag
    t.header.frame_id = ODOM_FRAME       # parent
    t.child_frame_id  = BASE_FRAME       # child

    # translation
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    # rotation (กัน NaN / ศูนย์หมด)
    q = msg.pose.pose.orientation
    if any(map(math.isnan, [q.x, q.y, q.z, q.w])) or (abs(q.x)+abs(q.y)+abs(q.z)+abs(q.w)) < 1e-9:
        q.x = q.y = q.z = 0.0
        q.w = 1.0
    t.transform.rotation = q

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("odom_to_tf")
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber("/odom", Odometry, cb, queue_size=50)
    rospy.loginfo("odom_to_tf up: publishing TF %s -> %s", ODOM_FRAME, BASE_FRAME)
    rospy.spin()

