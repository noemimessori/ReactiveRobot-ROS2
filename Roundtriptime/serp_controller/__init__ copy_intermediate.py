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
        super().__init__("WallFollowerController")
        

        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        
        # Subscriber for laser data
        self.create_subscription(LaserScan, "/static_laser", self.process_lidar, 1)
        
        self.Kp = 3.0  
        self.Ki = 0.008 
        self.Kd = 0.8  
        
        self.max_forward_speed = 0.2 
        self.min_forward_speed = 0.05 
        self.max_angular_speed = 1.5   
        self.max_acceleration = 0.1    
        self.current_speed = 0.0       
        
        self.desired_distance = 0.4    
        self.corner_detection_threshold = 0.3  

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_distance = None
        self.last_time = self.get_clock().now()

    def get_wall_distance(self, data):
        num_lasers = len(data.ranges)
        angle_increment = data.angle_increment

        start_angle = math.pi/4  # 45 degrees
        end_angle = 3*math.pi/4  # 135 degrees
        
        start_idx = math.floor((start_angle - data.angle_min) / angle_increment)
        end_idx = math.floor((end_angle - data.angle_min) / angle_increment)
        
        valid_readings = []
        valid_angles = []
        
        for i in range(start_idx, end_idx + 1):
            reading = data.ranges[i]
            angle = data.angle_min + (i * angle_increment)
            
            if (self.is_valid_reading(reading, data) and 
                reading < data.range_max * 0.95):  # Add small margin for max range
                
                perp_distance = abs(reading * math.sin(angle))
                valid_readings.append(perp_distance)
                valid_angles.append(math.degrees(angle))
        
        if not valid_readings:
            return None, None, []
        
        # Calculate statistics
        median_distance = np.median(valid_readings)
        std_dev = np.std(valid_readings) if len(valid_readings) > 1 else 0

        filtered_readings = [d for d in valid_readings 
                           if abs(d - median_distance) < 2 * std_dev]
        
        if not filtered_readings:
            return None, None, []
        
        final_distance = np.median(filtered_readings)
        
        return final_distance, valid_angles, filtered_readings

    def process_lidar(self, data):
        # Get wall distance estimate
        distance, valid_angles, readings = self.get_wall_distance(data)
        
        if distance is None:
            self.get_logger().info("No valid wall distance found! Searching...")
            self.change_robot_speeds(0.0, 0.5)  # Rotate to find wall
            return
            

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        distance_rate = 0.0
        if self.prev_distance is not None:
            distance_rate = (distance - self.prev_distance) / dt
        self.prev_distance = distance

        error = distance - self.desired_distance
   
        self.integral = max(min(self.integral + error * dt, 1.0), -1.0)
        
        # PID terms
        p_term = self.Kp * error
        i_term = self.Ki * self.integral
        d_term = self.Kd * (error - self.prev_error) / dt if dt > 0 else 0
      
        angular_velocity = p_term + i_term + d_term
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), 
                             -self.max_angular_speed)

        # Dynamic speed control based on multiple factors
        speed_factor = 1.0
        
        error_factor = 1.0 - min(abs(error) / self.desired_distance, 0.8)
        speed_factor *= error_factor

        angular_factor = 1.0 - min(abs(angular_velocity) / self.max_angular_speed, 0.8)
        speed_factor *= angular_factor

        rate_factor = 1.0 - min(abs(distance_rate) / self.corner_detection_threshold, 0.8)
        speed_factor *= rate_factor

        # Calculate target forward speed
        target_speed = self.max_forward_speed * speed_factor

        speed_change = target_speed - self.current_speed
        speed_change = max(min(speed_change, self.max_acceleration * dt), 
                          -self.max_acceleration * dt)
        self.current_speed += speed_change
        
        self.current_speed = max(self.current_speed, self.min_forward_speed)

        # Apply speeds
        self.change_robot_speeds(self.current_speed, angular_velocity)
        
        # Store error for next iteration
        self.prev_error = error
        
        # Debug output
        self.get_logger().info(
            f"Distance: {distance:.3f}m, Speed: {self.current_speed:.2f}, "
            f"Angular: {angular_velocity:.2f}, "
            f"Factors: Error={error_factor:.2f}, Angular={angular_factor:.2f}, "
            f"Rate={rate_factor:.2f}"
        )
        
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

def main(args = None):
    rclpy.init(args=args)
    controller = WallFollowerController()
    rclpy.spin(controller)
    
    # Cleanup
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()