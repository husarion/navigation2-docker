#!/usr/bin/python3
import rclpy
import os
from nav2_msgs.srv import SaveMap

print ("SLAM_MODE:" + os.environ.get("SLAM_MODE"))

rclpy.init(args=[])
node = rclpy.create_node("map_saver")
client = node.create_client(SaveMap, '/map_saver/save_map')

if not client.wait_for_service(timeout_sec=2.0):
    print(f"map timeout")
    exit (1)

request = SaveMap.Request()
request.free_thresh = 0.15
request.map_topic = "/map"
request.map_url = "/maps/map"
request.map_mode = "trinary"
request.image_format = "png"

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
print(f"map saved")
exit(0)