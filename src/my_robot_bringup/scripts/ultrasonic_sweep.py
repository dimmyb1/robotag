import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState

class SweepingUltrasonicNode(Node):
    def __init__(self):
        super().__init__('sweeping_ultrasonic_node')
        
        # Subscribe to the simulated servo's joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        topic_LIDAR = f'/{robot_name}/scan'
        
        # Subscribe to the narrow-beam LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            topic_LIDAR,
            self.scan_callback,
            10
        )
        
        
        # State variables to track the scanning process
        self.current_servo_angle = 0.0
        self.is_detecting = False
        self.entry_angle = None
        self.min_distance = float('inf')

    def joint_callback(self, msg):
        # Update our current angle based on the servo's position.
        # Replace 'servo_joint' with the actual name of your revolute joint in your URDF.
        try:
            joint_index = msg.name.index('servo_joint')
            self.current_servo_angle = msg.position[joint_index]
        except ValueError:
            # 'servo_joint' not found in this joint_states message
            pass

    def scan_callback(self, msg):
        # Because the FOV is limited to ~1-2 degrees, taking the minimum 
        # of the few valid samples gives us the distance straight ahead.
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            # We are currently looking at an object
            current_distance = min(valid_ranges)
            
            if not self.is_detecting:
                # STATE CHANGE: Empty Space -> Object
                # We just hit the first edge of the object
                self.is_detecting = True
                self.entry_angle = self.current_servo_angle
                self.min_distance = current_distance
                self.get_logger().info(f"Edge detected! Angle: {self.entry_angle:.3f} rad")
            else:
                # STATE CONTINUES: Object -> Object
                # We are sweeping across the surface of the object. 
                # Keep track of the shortest distance seen so far.
                if current_distance < self.min_distance:
                    self.min_distance = current_distance
                    
        else:
            # We are currently looking at empty space
            if self.is_detecting:
                # STATE CHANGE: Object -> Empty Space
                # We just passed the other edge of the object
                self.is_detecting = False
                exit_angle = self.current_servo_angle
                
                self.get_logger().info(f"Object ended! Angle: {exit_angle:.3f} rad")
                self.get_logger().info(f"--- Object Summary ---")
                self.get_logger().info(f"Edges: {self.entry_angle:.3f} rad to {exit_angle:.3f} rad")
                self.get_logger().info(f"Shortest Distance: {self.min_distance:.3f} m")
                self.get_logger().info(f"----------------------\n")
                
                # Reset tracking variables for the next time we sweep past it
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