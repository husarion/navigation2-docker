#!/usr/bin/python3
import rclpy
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

print ("SLAM_MODE:" + os.environ.get("SLAM_MODE"))

def callback(msg):
    print(f"received amcl_pose")
    exit(0)

rclpy.init(args=[])
latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

node = rclpy.create_node("amcl_pose_listener")
sub = node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", callback, qos_profile=latching_qos)
rclpy.spin_once(node)
print(f"amcl_pose timeout")
exit(1)
