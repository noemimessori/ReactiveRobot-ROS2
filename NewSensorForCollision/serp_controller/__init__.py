#!/usr/bin/env python3
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import numpy as np
from datetime import datetime

class WallFollowerController(Node):
    def __init__(self) -> None:
        super().__init__("WallFollowerController")
        
        # Publishers
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.debug_pub = self.create_publisher(String, "/wall_follower/debug", 1)
        
        # Subscribers for both lasers
        self.create_subscription(LaserScan, "/static_laser", self.process_lidar, 1)
        self.create_subscription(LaserScan, "/robot_laser", self.process_robot_lidar, 1)
        
        # PID parameters
        self.Kp = 4.0
        self.Ki = 0.01
        self.Kd = 2.0
        
        # Speed parameters
        self.max_forward_speed = 0.25
        self.min_forward_speed = 0.05
        self.max_angular_speed = 2.0
        self.current_speed = 0.0
        
        # Wall following parameters
        self.desired_distance = 0.3
        
        # PID memory variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()
        
        # Fallback behavior parameters
        self.fallback_linear_speed = 0.15
        self.fallback_angular_speed = 1.0
        
        # Robot detection parameters
        self.robot_speed_factor = 1.0
        self.safe_distance = 0.7     # Start slowing down at this distance
        self.stop_distance = 0.3     # Complete stop at this distance
        self.current_robot_distance = None
        
        # Visual parameters for the progress bar
        self.max_visual_distance = 1.0
        self.current_distance = None
        self.current_error = None
        self.mode = "NORMAL"
        
        # Create timer for visual updates
        self.create_timer(0.1, self.display_visual_feedback)

    def process_robot_lidar(self, data):       
        center_idx = len(data.ranges) // 2
        start_idx = center_idx - 8  
        end_idx = center_idx + 8

        min_robot_distance = float('inf')
        min_angle = 0
        
        # Check for robot in front arc
        for i in range(start_idx, end_idx + 1):
            if i >= 0 and i < len(data.ranges):
                reading = data.ranges[i]
                if self.is_valid_reading(reading, data):
                    if reading < min_robot_distance:
                        min_robot_distance = reading
                        min_angle = (i - center_idx) * data.angle_increment * 180 / math.pi

        self.current_robot_distance = min_robot_distance if min_robot_distance != float('inf') else None
                
        # Calculate speed factor based on distance
        if min_robot_distance <= self.stop_distance:
            self.robot_speed_factor = 0.0
            self.mode = "ROBOT DETECTED"
        elif min_robot_distance <= self.safe_distance:
            # Linear interpolation between 0 and 1 based on distance
            self.robot_speed_factor = (min_robot_distance - self.stop_distance) / (self.safe_distance - self.stop_distance)
            self.mode = "ROBOT FOLLOWING"
        else:
            self.robot_speed_factor = 1.0
            if self.mode in ["ROBOT DETECTED", "ROBOT FOLLOWING"]:
                self.mode = "NORMAL"

    def check_front_left_wall(self, data):        
        angle_increment = data.angle_increment
        
        # 45-90 degrees (front-left sector)
        start_angle = math.pi/4  # 45 degrees
        end_angle = math.pi/2    # 90 degrees
        
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
        
        
        start_angle = math.pi/6  # 30 degrees
        end_angle = 5*math.pi/6  # 150 degrees
        
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
        current_max_speed = self.max_forward_speed * self.robot_speed_factor
        
        if not self.check_front_left_wall(data):
            self.mode = "FALLBACK"
            self.change_robot_speeds(self.fallback_linear_speed * self.robot_speed_factor, 
                                   self.fallback_angular_speed)
            return
        
        # Get wall distance
        distance = self.get_wall_distance(data)
        self.current_distance = distance
        
        if distance is None:
            self.mode = "SEARCHING"
            self.change_robot_speeds(self.fallback_linear_speed * self.robot_speed_factor, 
                                   self.fallback_angular_speed)
            return
        
        # Normal PID control
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        error = distance - self.desired_distance
        self.current_error = error
        
        self.integral = max(min(self.integral + error * dt, 1.0), -1.0)
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        # PID terms
        p_term = self.Kp * error
        i_term = self.Ki * self.integral
        d_term = self.Kd * derivative
        
        angular_velocity = p_term + i_term + d_term
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), 
                             -self.max_angular_speed)
        
        speed_factor = 1.0
        
        # Reduce speed based on error magnitude
        error_ratio = abs(error) / self.desired_distance
        speed_factor *= max(0.3, 1.0 - error_ratio)
        
        turn_factor = 1.0 - min(abs(angular_velocity) / self.max_angular_speed, 0.8)
        speed_factor *= turn_factor
        
        # Calculate forward speed
        forward_speed = current_max_speed * speed_factor
        forward_speed = max(min(forward_speed, current_max_speed), 
                          self.min_forward_speed * self.robot_speed_factor)
        
        self.prev_error = error
        if self.mode not in ["ROBOT DETECTED", "ROBOT FOLLOWING"]:
            self.mode = "NORMAL"
        
        # Apply the calculated speeds
        self.change_robot_speeds(forward_speed, angular_velocity)

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
                bar[dist_pos] = "⊕"
            else:
                bar[dist_pos] = "●"
        
        bar[0] = "["
        bar[-1] = "]"
        
        return "".join(bar)

    def create_mode_indicator(self):
        mode_display = {
            "ROBOT DETECTED": "\033[91m[ROBOT DETECTED]\033[0m",  # Red
            "ROBOT FOLLOWING": "\033[93m[FOLLOWING]\033[0m",      # Yellow
            "FALLBACK": "\033[95m[FALLBACK]\033[0m",             # Purple
            "NORMAL": "\033[92m[NORMAL]\033[0m"                  # Green
        }
        return mode_display.get(self.mode, mode_display["NORMAL"])

    def create_robot_distance_bar(self, width=40):
        if self.current_robot_distance is None:
            return "[NO ROBOT DETECTED]" + " " * (width - 16)
            
        normalized_dist = min(self.current_robot_distance / self.safe_distance, 1.0)
        stop_pos = int((self.stop_distance / self.safe_distance) * width)
        safe_pos = width - 1
        curr_pos = int(normalized_dist * width)
        
        bar = list(" " * width)
        
        # Add markers
        if 0 <= stop_pos < width:
            bar[stop_pos] = "X"  # Stop threshold
        bar[safe_pos] = "│"  # Safe distance
        
        if 0 <= curr_pos < width:
            if curr_pos == stop_pos:
                bar[curr_pos] = "⊗"  # Special marker when at stop threshold
            elif curr_pos == safe_pos:
                bar[curr_pos] = "⊕"  # Special marker when at safe distance
            else:
                bar[curr_pos] = "●"
        
        bar[0] = "["
        bar[-1] = "]"
        
        return "".join(bar)

    def display_visual_feedback(self):
        mode_str = self.create_mode_indicator()
        wall_bar = self.create_distance_bar(self.current_distance, self.desired_distance)
        robot_bar = self.create_robot_distance_bar()
        
        status = [
            "",  
            "="*60,
            f"Wall Follower Status",
            "="*60,
            "",  
            f"Mode: {mode_str}",
            f"Speed Factor: {self.robot_speed_factor:.2f}",
            "",  
            "Wall Distance:",
            wall_bar,
            f"Current: {self.current_distance:.3f}m" if self.current_distance else "No wall detected",
            "",  
            "Robot Distance:",
            robot_bar,
            f"Current: {self.current_robot_distance:.3f}m" if self.current_robot_distance else "No robot detected",
            "",  
            "="*60,
            "",  
        ]
        
        display = "\033[2J\033[H" + "\n".join(status)
        self.get_logger().info(display)

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
