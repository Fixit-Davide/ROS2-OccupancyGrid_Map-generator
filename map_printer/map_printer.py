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

        self.declare_parameter("map_a", "/map_printer/map_a")
        self.map_a_pub_name = self.get_parameter("map_a").value

        self.declare_parameter("map_b", "/map_printer/map_b")
        self.map_b_pub_name = self.get_parameter("map_b").value

        self.declare_parameter("map_c", "/map_printer/map_c")
        self.map_c_pub_name = self.get_parameter("map_c").value
        
        self.declare_parameter("origin_map_a", "/map_printer/origin_map_a")
        self.origin_map_a_pub_name = self.get_parameter("origin_map_a").value
        
        self.declare_parameter("origin_map_b", "/map_printer/origin_map_b")
        self.origin_map_b_pub_name = self.get_parameter("origin_map_b").value
        
        self.declare_parameter("origin_map_c", "/map_printer/origin_map_c")
        self.origin_map_c_pub_name = self.get_parameter("origin_map_c").value

        self.map_a_pub = self.create_publisher(OccupancyGrid, self.map_a_pub_name, 10)
        self.origin_map_a_pub = self.create_publisher(PoseStamped, self.origin_map_a_pub_name, 10)        
        self.map_b_pub = self.create_publisher(OccupancyGrid, self.map_b_pub_name, 10)
        self.origin_map_b_pub = self.create_publisher(PoseStamped, self.origin_map_b_pub_name, 10)
        self.map_c_pub = self.create_publisher(OccupancyGrid, self.map_c_pub_name, 10)
        self.origin_map_c_pub = self.create_publisher(PoseStamped, self.origin_map_c_pub_name, 10)
        
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        map_a_msg_pose = Pose()
        map_a_msg_pose.position.x = 5.0
        map_a_msg_pose.position.y = 0.0
        map_a_msg_pose.position.z = 0.0
        map_a_msg = OccupancyGrid(
            header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map"), 
            info = MapMetaData(width=100, height=80, resolution=0.05, map_load_time= self.get_clock().now().to_msg(), origin=map_a_msg_pose)
        )        
        for i in range(0,100):
            for j in range(0,80):
                map_a_msg.data.append(int(occupancy_1[i][j] * 50))
        self.map_a_pub.publish(map_a_msg)
        
        origin_map_a_msg_pose = PoseStamped()
        origin_map_a_msg_pose.pose = map_a_msg_pose
        origin_map_a_msg_pose.header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map")
        self.origin_map_a_pub.publish(origin_map_a_msg_pose)

        map_b_msg_pose = Pose()
        map_b_msg_pose.position.x = 0.0
        map_b_msg_pose.position.y = 5.0
        map_b_msg_pose.position.z = 0.0
        map_b_msg = OccupancyGrid(
            header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map"), 
            info = MapMetaData(width=40, height=80, resolution=0.05, map_load_time= self.get_clock().now().to_msg(), origin=map_b_msg_pose)
        )
        for i in range(0,40):
            for j in range(0,80):
                map_b_msg.data.append(int(occupancy_2[i][j]*20))
        self.map_b_pub.publish(map_b_msg)
        
        origin_map_b_msg_pose = PoseStamped()
        origin_map_b_msg_pose.pose = map_b_msg_pose
        origin_map_b_msg_pose.header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map")
        self.origin_map_b_pub.publish(origin_map_b_msg_pose)

        map_c_msg_pose = Pose()
        map_c_msg_pose.position.x = -5.0
        map_c_msg_pose.position.y = 7.0
        map_c_msg_pose.position.z = 0.0
        # map_c_msg_pose.orientation.x = 0.0
        # map_c_msg_pose.orientation.y = 0.0
        # map_c_msg_pose.orientation.z = 0.7071
        # map_c_msg_pose.orientation.w = 0.7071
        map_c_msg = OccupancyGrid(
            header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map"), 
            info = MapMetaData(width=80, height=30, resolution=0.05, map_load_time= self.get_clock().now().to_msg(), origin=map_c_msg_pose)
        )
        for i in range(0,80):
            for j in range(0,30):
                map_c_msg.data.append(int(occupancy_3[i][j]*80)) 
        self.map_c_pub.publish(map_c_msg)
        
        origin_map_c_msg_pose = PoseStamped()
        origin_map_c_msg_pose.pose = map_c_msg_pose
        origin_map_c_msg_pose.header = Header(stamp =  self.get_clock().now().to_msg(), frame_id="map")
        self.origin_map_c_pub.publish(origin_map_c_msg_pose)



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