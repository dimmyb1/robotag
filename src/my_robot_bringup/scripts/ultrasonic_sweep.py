#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float64MultiArray, Float64, String # <-- Added Float64 for servo commands
import json

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
        topic_sweep = f"{topic_prefix}/sweep"
        
        # NOTE: Change this topic to match whatever your ros2_control or servo plugin uses!
        topic_servo_cmd = f"{topic_prefix}/servo_cmd" 
        
        # Subscribers
        self.joint_sub = self.create_subscription(JointState, topic_joint, self.joint_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, topic_LIDAR, self.scan_callback, 10)
        self.sweep_sub = self.create_subscription(String, topic_sweep, self.sweep_callback, 10)
        
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

        #functionality
        self.sweep = False
        self.multiple = False
        
        # Create a timer to constantly publish movement commands
        self.timer = self.create_timer(self.timer_period, self.sweep_timer_callback)

    def sweep_callback(self, msg):
        data = json.loads(msg.data)
        self.sweep = data.get("sweep", False)
        self.multiple = data.get("multiple", False)

    def sweep_timer_callback(self):
       
        if self.sweep:
             # Calculate how much to move this cycle
            step = self.sweep_speed * self.timer_period
            limit = math.pi / 2.0

            if self.multiple: #keep sweeping until sweep is false
                # Update target angle
                self.target_angle += (self.sweep_direction * step)
                
                # Reverse direction if we hit the -90 or +90 degree limits (approx 1.57 radians)
                if self.target_angle >= limit:
                    self.target_angle = limit
                    self.sweep_direction = -1
                elif self.target_angle <= -limit:
                    self.target_angle = -limit
                    self.sweep_direction = 1

            else: #do a single sweep
                if self.single_sweep_phase == 0:
                    self.target_angle = -limit  # command it to go to -90
                    # wait until it's actually there before sweeping
                    if abs(self.current_servo_angle - (-limit)) < 0.05:  # 0.05 rad tolerance ~3 degrees
                        self.single_sweep_phase = 1

                elif self.single_sweep_phase == 1:
                    self.target_angle += step
                    if self.target_angle >= limit:
                        self.target_angle = limit
                        self.single_sweep_phase = 0
                        self.sweep = False

        else:
            #set to 90 (head on and dont sweep)
            self.target_angle = 0

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
        if not self.sweep:
            self.entry_angle = float('inf')
            self.min_distance = float('inf')

        else:    
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
                    
                    #self.get_logger().info(f"Object data published to {self.object_pub.topic_name}!")
                    
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