#!/usr/bin/env python3

# Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
# control robot velocity through this
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from enum import Enum
from config import directions
import math

# robots states
class Mode(Enum):
    FOLLOW_LINE = 1
    VERIFY_HOUSE = 2
    STOP = 3

class CameraFollower(Node):
    # call it with the house its finding
    def __init__(self, target_house="HOUSE_2", start="PO"):
        super().__init__('camera_house_follower')
        # set a target house
        self.TARGET_HOUSE = target_house
        # The start position
        self.start = start
        self.mode = Mode.FOLLOW_LINE

        # Direction plan
        self.turn_plan = directions[start][self.TARGET_HOUSE]
        self.get_logger().info(f"Turning plan: {self.turn_plan}")
        self.turn_index = 0
        self.doing_turn = False
        self.all_turns_complete = False
        self.mustIncrementIndex = False

        # Odometry
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0

        self.line_found = False
        # offset from center
        self.line_error = 0.0
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

        # Subscriptions
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

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishes velocity commands to the robot
        # linear.x - forward/backward
        # angular.z - left/right
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Loop every 0.05 seconds
        self.create_timer(0.05, self.control_loop)


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
            # self.get_logger().info("Value of h in hsv", h, s, v, "in name", name)
            # widen HSV range to tolerate lighting and distinguish similar hues
            lower = (max(h - 5, 0), 100, 80)
            upper = (min(h + 5, 179), 255, 255)

            self.house_colours[name] = (lower, upper)

        # prevent errors
        if self.TARGET_HOUSE not in self.house_colours:
            raise RuntimeError("Unknown house selected")

        self.lower, self.upper = self.house_colours[self.TARGET_HOUSE]

        # self.get_logger().info("Camera House Follower Started")
        self.get_logger().info(f"Target house: {self.TARGET_HOUSE}")

    # Helper function used throughout this class
    def detect_black(self, hsv):
        # returns a mask of black pixels in the imag
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        return cv2.inRange(hsv, lower_black, upper_black)

    def bm_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        roi = hsv[int(h * 0.6):h, :]
        # Check for black in this camera
        mask = self.detect_black(roi)

        # calculates center of black line in ROI Region of Interest
        M = cv2.moments(mask)
        if M["m00"] > 800:
            # computer error from center to steer
            cx = int(M["m10"] / M["m00"])
            self.line_found = True
            self.line_error = cx - (w // 2)
            self.last_line_error = self.line_error
        else:
            self.line_found = False
    
    # check if enough pixels is found in any side camera
    # help to detect which side to move
    def bl_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Same as right
        mask = self.detect_black(hsv)
        self.left_line = np.sum(mask > 0) > 500

    def br_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = self.detect_black(hsv)
        self.right_line = np.sum(mask > 0) > 500

    def front_callback(self, msg):
        # Only process front camera after all turns are complete
        if not self.all_turns_complete:
            return
            
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        center = mask[:, w//2 - 80:w//2 + 80]
        # Is the house detectable
        self.house_visible = np.sum(mask > 0) > 1200
        self.house_reached = (np.sum(center > 0) / center.size) > self.stop_ratio

    def odom_callback(self, msg):
        # get yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > (2.0 * math.pi):
            angle -= 2.0 * math.pi
        while angle < 0.0:
            angle += 2.0 * math.pi
        return angle

    def start_turn(self, turn_right, half_turn=False):
        self.get_logger().info(f"STARTING TURN {self.turn_index + 1}/{len(self.turn_plan)}: {'RIGHT' if turn_right else 'LEFT'} {'180°' if half_turn else '90°'}")
        self.doing_turn = True
        self.start_yaw = self.current_yaw

        # Calculate target: 90 degrees right is -pi/2, left is +pi/2
        delta = (-math.pi/2 if turn_right else math.pi/2)
        if half_turn:
            delta *= 2  # make 180°

        self.target_yaw = self.normalize_angle(self.start_yaw + delta)
        self.get_logger().info(f"Start yaw: {self.start_yaw:.2f}, Target yaw: {self.target_yaw:.2f}")
        return self.target_yaw


    def control_loop(self):
        cmd = Twist()
        if self.turn_index == 0 and not self.doing_turn:
            while((self.line_found and self.left_line==False) and (self.line_found and self.right_line==False)):
                #reverse until you have 1 or both options for turning.
                cmd.linear.x = -0.22
                cmd.angular.z = 0.0
            half_turn = (self.start in ["HOUSE_2", "HOUSE_7"] and self.turn_plan[0] == "right")
            self.start_turn(self.turn_plan[0], half_turn=half_turn)
            self.cmd_pub.publish(Twist()) 
            return


        if self.mode == Mode.FOLLOW_LINE:
            # Handle active turn
            if self.doing_turn:
                cmd.linear.x = 0.0
                
                # Calculate shortest angular distance to target
                error = self.normalize_angle(self.target_yaw - self.current_yaw)
                
                # Proportional control for turning
                kp_rot = 2.5
                cmd.angular.z = kp_rot * error
                
                # Clamp rotation speed
                max_rot_speed = 0.8
                cmd.angular.z = max(min(cmd.angular.z, max_rot_speed), -max_rot_speed)
                
                # Check if turn is complete (within ~3 degrees)
                if abs(error) < 0.05:
                    self.doing_turn = False
                    self.turn_index += 1
                    self.get_logger().info(f"TURN {self.turn_index}/{len(self.turn_plan)} COMPLETE")
                    
                    # Check if all turns are done
                    if self.turn_index >= len(self.turn_plan):
                        self.all_turns_complete = True
                        self.get_logger().info("ALL TURNS COMPLETE - Now searching for house")
                    
                    cmd.angular.z = 0.0
                
                self.cmd_pub.publish(cmd)
                return
        
            # Detect intersection and start next turn (only if turns remaining)
            intersection_detected = (
                (self.left_line and self.line_found) or
                (self.right_line and self.line_found) or
                (self.left_line and self.right_line and self.line_found)
            )

            if intersection_detected and not self.all_turns_complete and not self.doing_turn:
                if self.turn_index < len(self.turn_plan):
                    #can we just go straight?
                    if((self.left_line and self.line_found and self.right_line==False and self.turn_plan[self.turn_index]==True) or 
                       (self.right_line and self.line_found and self.left_line==False and self.turn_plan[self.turn_index]==False)):
                        #yes - just walk forward
                        cmd.linear.x = 0.22
                        cmd.angular.z = -self.line_error * 0.003
                        self.mustIncrementIndex = True
                        
                    else:
                        # no - must turn
                        self.start_turn(self.turn_plan[self.turn_index])
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0

                        #walk forwards to clear the intersection
                        cmd.linear.x = 0.22
                        cmd.angular.z = -self.line_error * 0.003
                        
                        self.cmd_pub.publish(cmd)
                    return

            # Normal line following
            if self.line_found and not self.doing_turn:
                cmd.linear.x = 0.22
                cmd.angular.z = -self.line_error * 0.003

                if(self.mustIncrementIndex==True):
                    self.turn_index+=1
                    self.mustIncrementIndex=False
            elif not self.doing_turn:
                # Lost line - turn based on last known position
                cmd.linear.x = 0.0
                cmd.angular.z = -np.sign(self.last_line_error) * 0.8

            # House detection
            if self.all_turns_complete and self.house_visible:
                self.house_seen_frames += 1
                if self.house_seen_frames > 2:
                    self.mode = Mode.VERIFY_HOUSE
                    self.get_logger().info("House detected - Switching to VERIFY_HOUSE mode")
            else:
                self.house_seen_frames = 0

        elif self.mode == Mode.VERIFY_HOUSE:
            # Continue following line toward house
            if self.line_found:
                cmd.linear.x = 0.05
                cmd.angular.z = -self.line_error * 0.003

            # Stop when house is close enough
            if self.house_reached:
                self.mode = Mode.STOP
                self.get_logger().info("House reached - STOPPING")

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
