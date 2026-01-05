#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2

class CameraFollower(Node):
    # call it with the house its finding
    def __init__(self, target_house="HOUSE_2"):
        super().__init__('camera_house_follower')
        # set a target house
        self.TARGET_HOUSE = target_house

        # Camera subscription
        self.house_colour_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )

        # Publish velocity commands - controls the robot's linear and angular motion
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Stop distance proxy image-based - when house fills 25% of center
        # fine tuned based on experiement with house 2
        self.stop_ratio = 0.95

        # Exact RGB colors from Gazebo diffuse values
        colors = {
            "HOUSE_1": (97, 63, 0),
            "HOUSE_2": (253, 255, 0),
            "HOUSE_3": (255, 161, 0),
            "HOUSE_4": (252, 149, 209),
            "HOUSE_5": (255, 16, 0),
            "HOUSE_6": (0, 251, 255),
            "HOUSE_7": (255, 0, 229),
            "HOUSE_8": (120, 255, 255),
            "HOUSE_9": (0, 44, 255),
            "HOUSE_10": (146, 220, 255),
            "PO": (255, 255, 255),
            "C0": (0, 255, 45),
            "C1": (0, 255, 45),
            "C2": (0, 255, 45),
        }

        # Convert exact RGB to HSV using OpenCV
        self.house_colours = {}
        for name, rgb in colors.items():
            rgb_np = np.uint8([[list(rgb)]])
            hsv = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2HSV)[0][0]

            # widen HSV range to tolerate lighting
            h, s, v = hsv
            # print("Value of h in hsv", h, s, v, "in name", name)
            # widen HSV range to tolerate lighting and distinguish similar hues
            lower = (max(h - 5, 0), 100, 80)
            upper = (min(h + 5, 179), 255, 255)



            self.house_colours[name] = (lower, upper)

        # prevent errors
        if self.TARGET_HOUSE not in self.house_colours:
            raise RuntimeError("Unknown house selected")

        self.lower, self.upper = self.house_colours[self.TARGET_HOUSE]

        self.get_logger().info("Camera House Follower Started")
        self.get_logger().info(f"Target house: {self.TARGET_HOUSE}")

    def image_callback(self, msg):
        """Process camera image and follow target house"""
        height = msg.height
        width = msg.width

        # Convert bytes to numpy array and reshape
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)

        # Convert RGB or BGR to HSV - Hue Saturation Value
        if msg.encoding == "rgb8":
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            # print("Value of hsv found", hsv)
        else:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Colour mask for selected house
        mask = cv2.inRange(
            hsv,
            np.array(self.lower),
            np.array(self.upper)
        )

        # Clean mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Regions for alignment
        center_col = width // 2
        stripe = 80

        center = mask[:, center_col - stripe:center_col + stripe]
        left = mask[:, :width // 2]
        right = mask[:, width // 2:]

        # how much of the center view is house
        center_ratio = np.sum(center > 0) / center.size
        # where the house is
        left_pixels = np.sum(left > 0)
        right_pixels = np.sum(right > 0)
        # is the house visible
        total_pixels = np.sum(mask > 0)
        # print(total_pixels)
        # print(hsv)
        
        # CHECK FOR OTHER OBJECTS
        other_objects = []
        for name, (lower, upper) in self.house_colours.items():
            if name == self.TARGET_HOUSE:
                continue
            obj_mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            if np.sum(obj_mask > 0) > 50:
                other_objects.append(name)
        if other_objects:
            self.get_logger().info(f"Other objects detected: {', '.join(other_objects)}")

        cmd = Twist()

        # CONTROL LOGIC
        if total_pixels > 1200:
            if center_ratio > self.stop_ratio:
                cmd.linear.x = 0.0
                self.get_logger().info(
                    f"{self.TARGET_HOUSE} reached → STOP"
                )
            # elif center_ratio > 0.1:
            else:
                cmd.linear.x = 0.2
                self.get_logger().info(
                    f"{self.TARGET_HOUSE} ahead FORWARD"
            )
            # not needed due to line following
            # else:
            #     if left_pixels > right_pixels:
            #         cmd.angular.z = 0.9
            #         self.get_logger().info("Aligning LEFT")
            #     else:
            #         cmd.angular.z = -0.9
            #         self.get_logger().info("Aligning RIGHT")
        else:
            cmd.angular.z = 0.3
            self.get_logger().info(f"Searching for target house {self.TARGET_HOUSE}")

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
