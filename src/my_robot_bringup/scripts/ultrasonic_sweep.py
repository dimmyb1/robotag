#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float64MultiArray, Float64 # <-- Added Float64 for servo commands

class SweepingUltrasonicNode(Node):
    def __init__(self):
        super().__init__('sweeping_ultrasonic_node')
        
        # 1. Grab the namespace securely to handle empty namespaces
        robot_name = self.get_namespace().strip('/')
        topic_prefix = f"/{robot_name}" if robot_name else ""
        
        # 2. Build the dynamically named topics
        topic_joint = f"{topic_prefix}/joint_states"
        topic_LIDAR = f"{topic_prefix}/scan"
        topic_object = f"{topic_prefix}/object_data"
        
        # NOTE: Change this topic to match whatever your ros2_control or servo plugin uses!
        topic_servo_cmd = f"{topic_prefix}/servo_cmd" 
        
        # Subscribers
        self.joint_sub = self.create_subscription(JointState, topic_joint, self.joint_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, topic_LIDAR, self.scan_callback, 10)
        
        # Publishers
        self.object_pub = self.create_publisher(Float64MultiArray, topic_object, 10)
        
        # Command publisher for the servo. (If your controller requires a Float64MultiArray instead, change this)
        self.cmd_pub = self.create_publisher(Float64, topic_servo_cmd, 10)
        
        # State variables for detection
        self.current_servo_angle = 0.0
        self.is_detecting = False
        self.entry_angle = float('inf')
        self.min_distance = float('inf')
        
        # State variables for sweeping
        self.target_angle = 0.0
        self.sweep_direction = 1  # 1 for increasing angle, -1 for decreasing
        self.sweep_speed = 1.0   # Radians per second
        self.timer_period = 0.05  # 20 Hz update rate
        
        # Create a timer to constantly publish movement commands
        self.timer = self.create_timer(self.timer_period, self.sweep_timer_callback)

    def sweep_timer_callback(self):
        # Calculate how much to move this cycle
        step = self.sweep_speed * self.timer_period
        
        # Update target angle
        self.target_angle += (self.sweep_direction * step)
        
        # Reverse direction if we hit the -90 or +90 degree limits (approx 1.57 radians)
        limit = math.pi / 2.0
        if self.target_angle >= limit:
            self.target_angle = limit
            self.sweep_direction = -1
        elif self.target_angle <= -limit:
            self.target_angle = -limit
            self.sweep_direction = 1
            
        # Publish the command to the servo
        cmd_msg = Float64()
        cmd_msg.data = self.target_angle
        self.cmd_pub.publish(cmd_msg)

    def joint_callback(self, msg):
        robot_name = self.get_namespace().strip('/')
        joint_name = f'{robot_name}_servo_joint' if robot_name else 'servo_joint'
        try:
            joint_index = msg.name.index(joint_name)
            self.current_servo_angle = msg.position[joint_index]
        except ValueError:
            pass

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            current_distance = min(valid_ranges)
            
            if not self.is_detecting:
                self.is_detecting = True
                self.entry_angle = self.current_servo_angle
                self.min_distance = current_distance
            else:
                if current_distance < self.min_distance:
                    self.min_distance = current_distance
                    
        else:
            if self.is_detecting:
                self.is_detecting = False
                exit_angle = self.current_servo_angle
                
                # Pack the three values into a Float64MultiArray and publish it
                result_msg = Float64MultiArray()
                result_msg.data = [self.entry_angle, exit_angle, self.min_distance]
                self.object_pub.publish(result_msg)
                
                self.get_logger().info(f"Object data published to {self.object_pub.topic_name}!")
                
                self.entry_angle = float('inf')
                self.min_distance = float('inf')


def main(args=None):
    rclpy.init(args=args)
    node = SweepingUltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()