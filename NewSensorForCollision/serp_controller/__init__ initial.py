#!/usr/bin/env python3
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions

import math

class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")        
        self.pub:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)
        self.create_subscription(Collisions, "/collisions", self.processCollisions, 1)

    def change_robot_speeds(self, publisher, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        publisher.publish(twist_msg)

    def move_model(self, model_name, x, y, theta):
        client = self.create_client(MoveModel, "/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = model_name
        request.pose = Pose2D()
        request.pose.x = x
        request.pose.y = y
        request.pose.theta = theta
        client.call_async(request)
    
    # Handle LiDAR data
    def processLiDAR(self, data):
        num_lasers = len(data.ranges)
        max_distance = 1.0 # maximum allowed distance for an obstacle in one of three sensor readings before the robot starts spining around itself
        middle_laser_range = data.ranges[math.floor(num_lasers / 2 + 0.5)]
        # if the distance reading is invalid or the distance is below the maximum allowed distance
        if( not (middle_laser_range >= data.range_min and middle_laser_range <= data.range_max) or middle_laser_range < max_distance): 
            self.change_robot_speeds(self.pub, 0.0, -2.0)
        else:
            # MOVE FORWARD
            self.change_robot_speeds(self.pub, 1.0, 0.0)

        return
    
    # Process collisions
    def processCollisions(self, data):
        #########################################
        #    
        #if len(data.collisions) > 0:
        #    self.move_model('serp', 0.0, 0.0, -1.57079632679)
        ########################################

        return

def main(args = None):
    rclpy.init()
    
    serp = SerpController()

    rclpy.spin(serp)

if __name__ == "__main__":
    main()
