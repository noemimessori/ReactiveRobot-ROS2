#!/usr/bin/env python3
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class WallFollowerController(Node):
    def __init__(self) -> None:
        super().__init__("wall_following_controller")
        
        
        self.pub = self.create_publisher(Twist, "/robot2/cmd_vel", 1)
        
        self.create_subscription(LaserScan, "/robot2/static_laser", self.process_lidar, 1)
        
        self.Kp = 4.0
        self.Ki = 0.01
        self.Kd = 2.0

        # parameters -
        # Speed 
        self.max_forward_speed = 0.15
        self.min_forward_speed = 0.05
        self.max_angular_speed = 2.0
        self.current_speed = 0.0
        
        # Wall following 
        self.desired_distance = 0.3
        
        # PID memory variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()
        
        # Fallback behavior parameters
        self.fallback_linear_speed = 0.15
        self.fallback_angular_speed = 1.0

    def check_front_left_wall(self, data):
        
        angle_increment = data.angle_increment    
       
        start_angle = math.pi/4    
        end_angle = math.pi/2    
        
        start_idx = math.floor((start_angle - data.angle_min) / angle_increment)
        end_idx = math.floor((end_angle - data.angle_min) / angle_increment)
        
        valid_readings = 0
        total_readings = end_idx - start_idx + 1
        
        for i in range(start_idx, end_idx + 1):
            if i >= 0 and i < len(data.ranges):
                reading = data.ranges[i]
                if (self.is_valid_reading(reading, data) and 
                    reading < data.range_max * 0.95):
                    valid_readings += 1
        
        return (valid_readings / total_readings) > 0.3

    def get_wall_distance(self, data):
        
        angle_increment = data.angle_increment     
        
        start_angle = math.pi/6  
        end_angle = 5*math.pi/6  
        
        start_idx = math.floor((start_angle - data.angle_min) / angle_increment)
        end_idx = math.floor((end_angle - data.angle_min) / angle_increment)
        
        valid_readings = []
        
        for i in range(start_idx, end_idx + 1):
            reading = data.ranges[i]
            angle = data.angle_min + (i * angle_increment)
            
            if self.is_valid_reading(reading, data) and reading < data.range_max * 0.95:
                perp_distance = abs(reading * math.sin(angle))
                valid_readings.append(perp_distance)
        
        if not valid_readings:
            return None
            
        return min(valid_readings)

    def process_lidar(self, data):

        if self.check_robot_in_front(data):
            self.change_robot_speeds(0.0, 0.0)  # Stop if robot detected
            return
            
        if not self.check_front_left_wall(data):
            self.change_robot_speeds(self.fallback_linear_speed, 
                                   self.fallback_angular_speed)
            return
        
        distance = self.get_wall_distance(data)
        
        if distance is None:
            self.change_robot_speeds(self.fallback_linear_speed, 
                                   self.fallback_angular_speed)
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        error = distance - self.desired_distance
        
        self.integral = max(min(self.integral + error * dt, 1.0), -1.0)

        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        # PID terms
        p_term = self.Kp * error
        i_term = self.Ki * self.integral
        d_term = self.Kd * derivative
        
        angular_velocity = p_term + i_term + d_term
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), 
                             -self.max_angular_speed)
        
        # Dynamic speed control
        speed_factor = 1.0

        error_ratio = abs(error) / self.desired_distance
        speed_factor *= max(0.3, 1.0 - error_ratio)

        turn_factor = 1.0 - min(abs(angular_velocity) / self.max_angular_speed, 0.8)
        speed_factor *= turn_factor

        forward_speed = self.max_forward_speed * speed_factor
        forward_speed = max(min(forward_speed, self.max_forward_speed), 
                          self.min_forward_speed)
        
        self.prev_error = error
        self.change_robot_speeds(forward_speed, angular_velocity)

    def check_robot_in_front(self, data):
        angle_increment = data.angle_increment
        center_idx = len(data.ranges) // 2

        angles_to_check = [-10, -5, 0, 5, 10]  # Degrees
        
        for angle_deg in angles_to_check:
            offset = int(math.radians(angle_deg) / angle_increment)
            check_idx = center_idx - offset
            
            if check_idx >= 0 and check_idx < len(data.ranges):
                reading = data.ranges[check_idx]
                if self.is_valid_reading(reading, data):
                    if reading <= 0.5:  # Detection range
                        return True
        
        return False

    def is_valid_reading(self, reading, data):
        return (reading >= data.range_min and 
                reading <= data.range_max and 
                not math.isnan(reading) and 
                not math.isinf(reading))
    
    def change_robot_speeds(self, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = WallFollowerController()
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()