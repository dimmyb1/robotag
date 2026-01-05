#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from enum import Enum


class Mode(Enum):
    FOLLOW_LINE = 1
    VERIFY_HOUSE = 2
    STOP = 3


class CameraFollower(Node):
    # call it with the house its finding
    def __init__(self, target_house="HOUSE_2"):
        super().__init__('camera_house_follower')
        # set a target house
        self.TARGET_HOUSE = target_house
        # start with follow mode
        self.mode = Mode.FOLLOW_LINE

        # Front - house colour detection
        self.front_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.front_callback,
            10
        )

        # Bottom-middle - main line following
        self.bm_sub = self.create_subscription(
            Image,
            '/bm_camera/image_raw',
            self.bm_callback,
            10
        )

        # Bottom-left - intersection / left line
        self.bl_sub = self.create_subscription(
            Image,
            '/bl_camera/image_raw',
            self.bl_callback,
            10
        )

        # Bottom-right - intersection / right line
        self.br_sub = self.create_subscription(
            Image,
            '/br_camera/image_raw',
            self.br_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.control_loop)

        self.line_found = False
        self.line_error = 0.0
        # track last line seen
        self.last_line_seen = False  
        # track last error 
        self.last_line_error = 0.0     

        self.left_line = False
        self.right_line = False

        self.house_visible = False
        self.house_reached = False
        # Count the number of times the house was seen
        # This is done to switch to house detection mode
        self.house_seen_frames = 0

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

    def detect_black(self, hsv):
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        return cv2.inRange(hsv, lower_black, upper_black)

    def bm_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        roi = hsv[int(h * 0.6):h, :]
        mask = self.detect_black(roi)

        M = cv2.moments(mask)
        if M["m00"] > 800:
            cx = int(M["m10"] / M["m00"])
            self.line_found = True
            self.line_error = cx - (w // 2)
            self.last_line_seen = True        
            self.last_line_error = self.line_error  
        else:
            self.line_found = False

    def bl_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = self.detect_black(hsv)
        self.left_line = np.sum(mask > 0) > 500

    def br_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = self.detect_black(hsv)
        self.right_line = np.sum(mask > 0) > 500

    def front_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        center = mask[:, w//2 - 80:w//2 + 80]
        self.house_visible = np.sum(mask > 0) > 1200
        self.house_reached = (np.sum(center > 0) / center.size) > self.stop_ratio

    def control_loop(self):
        cmd = Twist()

        if self.mode == Mode.FOLLOW_LINE:
            if self.line_found:
                cmd.linear.x = 0.22
                cmd.angular.z = -self.line_error * 0.003
            else:
                # line lost - try to recover
                cmd.linear.x = 0.1  # move forward slowly to help catch line
                # use side cameras if available
                if self.left_line and not self.right_line:
                    cmd.angular.z = 0.8  # turn left faster
                elif self.right_line and not self.left_line:
                    cmd.angular.z = -0.8  # turn right faster
                elif self.last_line_seen:
                    cmd.angular.z = -np.sign(self.last_line_error) * 0.8  # faster turn
                else:
                    cmd.angular.z = 1.0  # spin in place faster

            # Intersection hook 
            if self.left_line and self.right_line:
                pass  # TO ADD INTERSECTION LOGIC

            if self.house_visible:
                self.house_seen_frames += 1
                if self.house_seen_frames > 8:
                    self.mode = Mode.VERIFY_HOUSE
            else:
                self.house_seen_frames = 0

        elif self.mode == Mode.VERIFY_HOUSE:
            if self.line_found:
                cmd.linear.x = 0.15
                cmd.angular.z = -self.line_error * 0.003

            if self.house_reached:
                self.mode = Mode.STOP

        elif self.mode == Mode.STOP:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

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
