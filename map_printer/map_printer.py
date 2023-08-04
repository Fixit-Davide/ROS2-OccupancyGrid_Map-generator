#!/usr/bin/env python3

# Libraries
import math
import numpy as np
import sys
import csv

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData

global occupancy_1
occupancy_1 = []
global occupancy_2
occupancy_2 = []
global occupancy_3
occupancy_3 = []

class MapPrinter(Node):
    def __init__(self):
        super().__init__("map_printer")

        self.declare_parameter("map_1", "/map_printer/map_1")
        self.map_1_pub_name = self.get_parameter("map_1").value

        self.declare_parameter("map_2", "/map_printer/map_2")
        self.map_2_pub_name = self.get_parameter("map_2").value

        self.declare_parameter("map_3", "/map_printer/map_3")
        self.map_3_pub_name = self.get_parameter("map_3").value

        self.timer_period = 0.5  # seconds
        self.map_pub_1 = self.create_publisher(OccupancyGrid, self.map_1_pub_name, 10)
        self.map_pub_2 = self.create_publisher(OccupancyGrid, self.map_2_pub_name, 10)
        self.map_pub_3 = self.create_publisher(OccupancyGrid, self.map_3_pub_name, 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        map_1_msg_pose = Pose()
        map_1_msg_pose.position.x = 5.0
        map_1_msg_pose.position.y = 0.0
        map_1_msg_pose.position.z = 0.0
        map_1_msg = OccupancyGrid(
            header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map"), 
            info = MapMetaData(width=100, height=80, resolution=0.05, map_load_time= self.get_clock().now().to_msg(), origin=map_1_msg_pose)
        )
        for i in range(0,100):
            for j in range(0,80):
                map_1_msg.data.append(int(occupancy_1[i][j] * 50))
        self.map_pub_1.publish(map_1_msg)

        map_2_msg_pose = Pose()
        map_2_msg_pose.position.x = 0.0
        map_2_msg_pose.position.y = 5.0
        map_2_msg_pose.position.z = 0.0
        map_2_msg = OccupancyGrid(
            header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map"), 
            info = MapMetaData(width=40, height=80, resolution=0.05, map_load_time= self.get_clock().now().to_msg(), origin=map_2_msg_pose)
        )
        for i in range(0,40):
            for j in range(0,80):
                map_2_msg.data.append(int(occupancy_2[i][j]*20))
        self.map_pub_2.publish(map_2_msg)

        map_3_msg_pose = Pose()
        map_3_msg_pose.position.x = -5.0
        map_3_msg_pose.position.y = 7.0
        map_3_msg_pose.position.z = 0.0
        # map_3_msg_pose.orientation.x = 0.0
        # map_3_msg_pose.orientation.y = 0.0
        # map_3_msg_pose.orientation.z = 0.7071
        # map_3_msg_pose.orientation.w = 0.7071
        map_3_msg = OccupancyGrid(
            header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map"), 
            info = MapMetaData(width=80, height=30, resolution=0.05, map_load_time= self.get_clock().now().to_msg(), origin=map_3_msg_pose)
        )
        for i in range(0,80):
            for j in range(0,30):
                map_3_msg.data.append(int(occupancy_3[i][j]*80)) 
        self.map_pub_3.publish(map_3_msg)



def main(args=None):
    rclpy.init(args=args)
    node = MapPrinter()
    try:

        for i in range(0,100):
            occupancy_1.append([])
        for i in range(0,100):
            for j in range(0,80):
                occupancy_1[i].append(0.5)

        for i in range(0,40):
            occupancy_2.append([])
        for i in range(0,40):
            for j in range(0,80):
                occupancy_2[i].append(0.5)

        for i in range(0,80):
            occupancy_3.append([])
        for i in range(0,80):
            for j in range(0,30):
                occupancy_3[i].append(0.5)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException:
        node.get_logger().info('[MapPrinter] Exception:')
        raise
    finally:
        rclpy.shutdown()

# Main
if __name__ == '__main__':
    main()