#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2


class CameraFollower(Node):
    def __init__(self):
        super().__init__('camera_house_follower')
        self.TARGET_HOUSE = "HOUSE_2"
        # Camera subscription
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Stop distance proxy (image-based)
        self.stop_ratio = 0.25  # Stop when house fills 25% of center

        # HSV colour ranges derived from SDF diffuse colours
        self.house_colours = {
            # brown
            "HOUSE_1":   ((5, 100, 50),   (15, 255, 200)), 
            # yellow
            "HOUSE_2":  ((20, 120, 120), (35, 255, 255)),
            # orange
            "HOUSE_3":  ((10, 120, 120), (20, 255, 255)),
            # pink
            "HOUSE_4":    ((150, 80, 120), (170, 255, 255)),
            # red
            "HOUSE_5":     ((0, 120, 120),  (8, 255, 255)),
            # cyan
            "HOUSE_6":    ((80, 120, 120), (100, 255, 255)),
            # magenta
            "HOUSE_7": ((140, 120, 120),(160, 255, 255)),
            # purple
            "HOUSE_8":  ((120, 120, 120),(140, 255, 255)),
            # blue
            "HOUSE_9":    ((100, 120, 120),(120, 255, 255)),
            # light blue
            "HOUSE_10":  ((90, 50, 180),  (110, 180, 255)),
            # white
            "PO":   ((0, 0, 200),    (179, 40, 255)),
            # green
            "C_green":    ((45, 120, 120), (75, 255, 255)),
        }

        if self.TARGET_HOUSE not in self.house_colours:
            raise RuntimeError("Unknown house selected")

        self.lower, self.upper = self.house_colours[self.TARGET_HOUSE]

        self.get_logger().info("Camera House Follower Started")
        self.get_logger().info(f"Target house: {self.TARGET_HOUSE}")

    def image_callback(self, msg):
        """Process camera image and follow dark objects"""
        # Convert image message to numpy array
        height = msg.height
        width = msg.width

        # Convert bytes to numpy array and reshape
        img = np.array(msg.data, dtype=np.uint8).reshape(height, width, 3)

        # Convert RGB → HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Colour mask for selected house
        mask = cv2.inRange(
            hsv,
            np.array(self.lower),
            np.array(self.upper)
        )

        # Clean mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Regions
        center_col = width // 2
        stripe = 80

        center = mask[:, center_col - stripe:center_col + stripe]
        left = mask[:, :width // 2]
        right = mask[:, width // 2:]

        center_ratio = np.sum(center > 0) / center.size
        left_pixels = np.sum(left > 0)
        right_pixels = np.sum(right > 0)
        total_pixels = np.sum(mask > 0)

        cmd = Twist()

        # ------------------------
        # CONTROL LOGIC
        # ------------------------
        if total_pixels > 500:
            if center_ratio > self.stop_ratio:
                cmd.linear.x = 0.0
                self.get_logger().info(
                    f"{self.TARGET_HOUSE} reached → STOP"
                )
            elif center_ratio > 0.1:
                cmd.linear.x = 0.2
                self.get_logger().info(
                    f"{self.TARGET_HOUSE} ahead → FORWARD"
                )
            else:
                if left_pixels > right_pixels:
                    cmd.angular.z = 0.4
                    self.get_logger().info("Aligning → LEFT")
                else:
                    cmd.angular.z = -0.4
                    self.get_logger().info("Aligning → RIGHT")
        else:
            cmd.angular.z = 0.3
            self.get_logger().info("Searching for target house...")
        
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = CameraFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Stop the robot
    stop_cmd = Twist()
    node.cmd_pub.publish(stop_cmd)
    node.get_logger().info('Shutting down - Robot stopped')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()