import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import Float64MultiArray # <-- IMPORT THIS

class SweepingUltrasonicNode(Node):
    def __init__(self):
        super().__init__('sweeping_ultrasonic_node')
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        robot_name = self.get_namespace().strip('/')
        topic_LIDAR = f'/{robot_name}/scan'
        
        # Subscribe to the narrow-beam LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            topic_LIDAR,
            self.scan_callback,
            10
        )
        
        # ADD A PUBLISHER
        self.object_pub = self.create_publisher(Float64MultiArray, '/object_data', 10)
        
        self.current_servo_angle = 0.0
        self.is_detecting = False
        self.entry_angle = None
        self.min_distance = float('inf')

    def joint_callback(self, msg):
        try:
            joint_index = msg.name.index('servo_joint')
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
                
                # --- NEW PUBLISHING LOGIC ---
                # Pack the three values into a Float64MultiArray and publish it
                result_msg = Float64MultiArray()
                result_msg.data = [self.entry_angle, exit_angle, self.min_distance]
                self.object_pub.publish(result_msg)
                
                self.get_logger().info("Object data published to /object_data topic!")
                # ----------------------------
                
                self.entry_angle = None
                self.min_distance = float('inf')

def main(args=None):
    rclpy.init(args=args)
    node = SweepingUltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
