#!/usr/bin/env python3
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import numpy as np
import json
from datetime import datetime

class WallFollowerController(Node):
    def __init__(self) -> None:
        super().__init__("WallFollowerController")
        
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.debug_pub = self.create_publisher(String, "/wall_follower/debug", 1)
        self.create_subscription(LaserScan, "/static_laser", self.process_lidar, 1)
        
        self.Kp = 6.0
        self.Ki = 0.01
        self.Kd = 2.5
        self.max_forward_speed = 0.6
        self.min_forward_speed = 0.05
        self.max_angular_speed = 2.0
        self.current_speed = 0.0
        self.desired_distance = 0.3
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()
        self.fallback_linear_speed = 0.15
        self.fallback_angular_speed = 1.0
        
        self.start_time = datetime.now()
        self.fallback_count = 0
        self.mode = "NORMAL"
        self.current_distance = None
        self.current_error = None
        
        self.bar_width = 40  
        self.max_visual_distance = 1.0  

        self.create_timer(0.1, self.display_visual_feedback)  

    def create_distance_bar(self, distance, target, width=40):
        if distance is None:
            return "[NO WALL DETECTED]" + " " * (width - 15)
            
        normalized_dist = min(distance / self.max_visual_distance, 1.0)
        normalized_target = min(target / self.max_visual_distance, 1.0)

        dist_pos = int(normalized_dist * width)
        target_pos = int(normalized_target * width)
 
        bar = list(" " * width)
        
        if 0 <= target_pos < width:
            bar[target_pos] = "│"
        
        if 0 <= dist_pos < width:
            if dist_pos == target_pos:
                bar[dist_pos] = "⊕"  # Perfect alignment
            else:
                bar[dist_pos] = "●"
        
        # Add wall indicators at edges
        bar[0] = "["
        bar[-1] = "]"
        
        return "".join(bar)

    def create_mode_indicator(self):
      
        if self.mode == "NORMAL":
            return "\033[92m[NORMAL]\033[0m"  # Green
        else:
            return "\033[93m[FALLBACK]\033[0m"  # Yellow

    def create_error_indicator(self):
     
        if self.current_error is None:
            return "[ NO ERROR DATA ]"
            
        error_chars = 20
        max_error = 0.5
        
        # Normalize error
        normalized_error = max(min(self.current_error / max_error, 1.0), -1.0)
        center = error_chars // 2
        error_pos = center + int(normalized_error * center)
        
        # Create the indicator
        indicator = list("-" * error_chars)
        indicator[center] = "|"  # Center mark
        if 0 <= error_pos < error_chars:
            indicator[error_pos] = "●"
            
        return "[" + "".join(indicator) + "]"

    def display_visual_feedback(self):
     
        # Create the visualization
        runtime = (datetime.now() - self.start_time).total_seconds()
        
        visual_output = [
            "\n" + "="*60,
            f"Wall Follower Status - Runtime: {runtime:.1f}s",
            "="*60,
            f"Mode: {self.create_mode_indicator()}",
            "",
            "Wall Distance:",
            self.create_distance_bar(self.current_distance, self.desired_distance),
            f"Current: {self.current_distance:.3f}m" if self.current_distance else "No wall detected",
            "",
            "Error:",
            self.create_error_indicator(),
            f"Value: {self.current_error:.3f}m" if self.current_error else "No error data",
            "",
            f"Fallback Events: {self.fallback_count}",
            "="*60
        ]
        
        # Print the visualization
        self.get_logger().info("\n".join(visual_output))

    def process_lidar(self, data):
        # Wall detection and fallback check
        if not self.check_front_left_wall(data):
            self.change_robot_speeds(self.fallback_linear_speed, self.fallback_angular_speed)
            self.current_distance = None
            self.current_error = None
            return
            
        distance = self.get_wall_distance(data)
        self.current_distance = distance
        
        if distance is None:
            self.change_robot_speeds(self.fallback_linear_speed, self.fallback_angular_speed)
            self.current_error = None
            return
        
        # PID control
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        error = distance - self.desired_distance
        self.current_error = error
        
        self.integral = max(min(self.integral + error * dt, 1.0), -1.0)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        p_term = self.Kp * error
        i_term = self.Ki * self.integral
        d_term = self.Kd * derivative
        
        # Control calculations
        angular_velocity = p_term + i_term + d_term
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)
        
        speed_factor = 1.0
        error_ratio = abs(error) / self.desired_distance
        speed_factor *= max(0.3, 1.0 - error_ratio)
        
        turn_factor = 1.0 - min(abs(angular_velocity) / self.max_angular_speed, 0.8)
        speed_factor *= turn_factor
        
        forward_speed = self.max_forward_speed * speed_factor
        forward_speed = max(min(forward_speed, self.max_forward_speed), self.min_forward_speed)
        
        self.prev_error = error
        self.change_robot_speeds(forward_speed, angular_velocity)

    def check_front_left_wall(self, data):
        # Previous implementation remains the same...
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
        
        wall_detected = (valid_readings / total_readings) > 0.3
        self.mode = "NORMAL" if wall_detected else "FALLBACK"
        if not wall_detected:
            self.fallback_count += 1
        return wall_detected

    def get_wall_distance(self, data):
        # Previous implementation remains the same...
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