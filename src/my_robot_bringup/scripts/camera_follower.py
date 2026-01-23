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
from std_msgs.msg import String
import json
from rclpy.qos import QoSProfile, DurabilityPolicy

# robots states
class Mode(Enum):
    FOLLOW_LINE = 1
    VERIFY_HOUSE = 2
    STOP = 3

class CameraFollower(Node):
    # call it with the house its finding
    def __init__(self):
        super().__init__('camera_house_follower')
        self.TARGET_HOUSE = None
        self.start = None
        self.mode = Mode.FOLLOW_LINE
        self.navigation_active = False

        self.turn_index = 0.0
        self.doing_turn = False
        self.all_turns_complete = False
        self.needToClearIntersection = False

        # Odometry
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0
        self.odom_ready = False
        self.cardinals_initialized = False # New flag to set cardinals once

        self.kp = 0.8
        self.ki = 0.01
        self.kd = 0.5   

        self.line_found = False
        # offset from center
        self.line_error = 0.0
        self.last_line_error = 0.0
        self.sum_line_error = 0.0
        self.left_line = False
        self.right_line = False
        self.already_failed = False

        self.f_line_found = False
        self.heading_ref = None
        self.heading_kp = 0.6    # start 0.4–0.8

        self.house_visible = False
        self.house_reached = False
        # Count the number of times the house was seen
        # This is done to switch to house detection mode
        self.house_seen_frames = 0

        # Stop distance proxy image-based - when house fills 25% of center
        # fine tuned based on experiement with house 2
        self.stop_ratio = 0.95

        #self.wait_until = self.get_clock().now()

        # Subscriptions
        self.front_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.front_callback,
            1
        )

        #"""

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

        #"""

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
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # Cardinal placeholders (Now properly set in odom_callback)
        self.cardinals = {}
        self.current_cardinal_target = 0.0
        
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        # subscribe to which house they need to navigate to
        self.subscription = self.create_subscription(
            String,
            'navigate_to_house',
            self.nav_callback,
            qos
        )


        # Loop every 0.05 seconds
        self.cmd = Twist()
        self.control_timer = self.create_timer(0.05, self.control_loop)
        #self.publish_timer = self.create_timer(0.05, self.publish_current_cmd)
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
        
        # Send a sign to deliver robot client to see if the robot has arrived to one house 
        # this is done so it can send another house
        self.done_pub = self.create_publisher(String, 'navigation_done', 10)

    def nav_callback(self, msg):
        data = json.loads(msg.data)

        self.start = data['start']
        self.TARGET_HOUSE = data['target']

        self.get_logger().info(
            f"Received navigation command: {self.start} → {self.TARGET_HOUSE}"
        )

        # --- RESET STATE ---
        self.turn_index = 0
        self.doing_turn = False
        self.last_line_error = 0
        self.needToClearIntersection = False
        self.all_turns_complete = False
        self.mode = Mode.FOLLOW_LINE
        self.house_seen_frames = 0
        self.house_visible = False
        self.house_reached = False

        # Reset heading lock to start reference
        if self.cardinals_initialized:
            self.get_logger().info(f"RESETTING CARDINALS TO NORTH")
            self.current_cardinal_target = self.cardinals['NORTH']

        # --- COMPUTE TURN PLAN ---
        self.turn_plan = directions[self.start][self.TARGET_HOUSE]

        # --- SET HOUSE COLOR ---
        if self.TARGET_HOUSE not in self.house_colours:
            self.get_logger().error(f"Unknown house: {self.TARGET_HOUSE}")
            return

        self.lower, self.upper = self.house_colours[self.TARGET_HOUSE]

        self.navigation_active = True

    def publish_done(self):
        msg = String()
        msg.data = json.dumps({'reached': self.TARGET_HOUSE})
        self.done_pub.publish(msg)
        self.get_logger().info(f"Reached {self.TARGET_HOUSE}")

    # Helper function used throughout this class
    def detect_black(self, hsv):
        # returns a mask of black pixels in the image
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
            self.line_found = True
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
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        
        # 1. Scan the bottom area
        scan_row = int(h * 0.9)
        row_data = gray[scan_row, :]
        _, thresh = cv2.threshold(row_data, 50, 255, cv2.THRESH_BINARY_INV)

        # NEW: Detect and reject horizontal/perpendicular lines
        # A horizontal line will have black pixels spanning most of the width
        total_black_pixels = np.sum(thresh == 255)
        horizontal_line_detected = total_black_pixels > (w * 0.6)  # If >60% of width is black
        
        if horizontal_line_detected:
            # This is likely a perpendicular T-junction line, ignore it completely
            # Instead, look higher up in the image where only the forward line exists
            scan_row = int(h * 0.7)  # Look much higher
            row_data = gray[scan_row, :]
            _, thresh = cv2.threshold(row_data, 50, 255, cv2.THRESH_BINARY_INV)

        # 2. Define Custom Segment Boundaries
        m_start = int((w/2) - ((1/10) * w) / 2)
        m_end = int((w/2) + ((1/10) * w) / 2)

        segments = {
            'LEFT':   thresh[0 : m_start],
            'MIDDLE': thresh[m_start : m_end],
            'RIGHT':  thresh[m_end : w]
        }

        # 3. Check which segments have a line
        sides = m_start * 1/4
        middle = (m_end - m_start) * 3/4
        segment_density = {
            'LEFT':   np.sum(segments['LEFT'] == 255) > sides,
            'MIDDLE': np.sum(segments['MIDDLE'] == 255) > middle,
            'RIGHT':  np.sum(segments['RIGHT'] == 255) > sides
        }

        target_cx = None

        # 4. Prioritize MIDDLE strongly when it exists
        if segment_density['MIDDLE']:
            M = cv2.moments(segments['MIDDLE'])
            if M["m00"] > 0:
                target_cx = (M["m10"] / M["m00"]) + m_start
            self.f_line_found = True
        
        # Only check sides if MIDDLE is NOT found
        elif segment_density['LEFT'] and segment_density['RIGHT']:
            # Both sides visible - use weighted average
            M_left = cv2.moments(segments['LEFT'])
            M_right = cv2.moments(segments['RIGHT'])
            
            if M_left["m00"] > 0 and M_right["m00"] > 0:
                cx_left = M_left["m10"] / M_left["m00"]
                cx_right = (M_right["m10"] / M_right["m00"]) + m_end
                target_cx = (cx_left + cx_right) / 2.0
            self.f_line_found = True
            
        elif segment_density['LEFT']:
            M = cv2.moments(segments['LEFT'])
            if M["m00"] > 0:
                target_cx = M["m10"] / M["m00"]
            self.f_line_found = True
        
        elif segment_density['RIGHT']:
            M = cv2.moments(segments['RIGHT'])
            if M["m00"] > 0:
                target_cx = (M["m10"] / M["m00"]) + m_end
            self.f_line_found = True
        else:
            self.f_line_found = False

        # 5. Calculate error
        if target_cx is not None:
            new_error = float(target_cx - (w / 2)) / (w / 2)
            if abs(new_error) < 0.08:
                self.heading_ref = self.current_yaw
            
            # Apply minimum force threshold
            min_force = 0.1
            if new_error > 0:
                self.line_error = max(new_error, min_force)
            else:
                self.line_error = min(new_error, -min_force)
            
            self.f_line_found = True
        else:
            self.f_line_found = False

        # House detection logic (unchanged)
        if self.all_turns_complete:   
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

            center = mask[:, w//2 - 80:w//2 + 80]
            self.house_visible = np.sum(mask > 0) > 1200
            self.house_reached = (np.sum(center > 0) / center.size) > self.stop_ratio




    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        # get yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
        if not self.cardinals_initialized:
            self.get_logger().info("Initialised cardinals")
            self.start_yaw = self.current_yaw 
            # Facing SOUTH (~3.14), adding pi/2 (Left) should result in EAST (~ -1.57)
            self.cardinals = {
                'SOUTH': self.normalize_angle(self.start_yaw),
                'WEST':  self.normalize_angle(self.start_yaw - math.pi/2 ) , # Right -0.2 to maybe correct 0.2 rad diff
                'NORTH': self.normalize_angle(self.start_yaw + math.pi),    # Behind
                'EAST':  self.normalize_angle(self.start_yaw + math.pi/2 )  # Left
            }
            self.current_cardinal_target = self.cardinals['SOUTH']
            self.cardinals_initialized = True

        self.odom_ready = True
    
    def angle_error(self, target, current):
        return math.atan2(
            math.sin(target - current),
            math.cos(target - current)
        )

    
    def calculate_line_following_command(self, base_speed):

        # Update the Integral
        self.sum_line_error += self.line_error
        
        # Anti-Windup
        max_integral = 3.0
        self.sum_line_error = max(min(self.sum_line_error, max_integral), -max_integral)
        
        # PID gains
        kp = self.kp
        ki = self.ki
        kd = self.kd
        
        # Calculate terms
        P = self.line_error * kp
        I = self.sum_line_error * ki
        D = (self.line_error - self.last_line_error) * kd
        
        # Combine
        angular = -(P + I + D)

        # Heading hold correction
        if self.heading_ref is not None:
            heading_error = self.angle_error(self.heading_ref, self.current_yaw)
            angular += self.heading_kp * heading_error

        
        # Reset integral on zero crossing
        if (self.line_error > 0 and self.last_line_error < 0) or \
        (self.line_error < 0 and self.last_line_error > 0):
            self.sum_line_error = 0.0

        self.last_line_error = self.line_error
        
        # Cap the output
        angular = max(min(angular, 0.35), -0.35)
        
        return base_speed, angular
    

    def start_turn(self, turn_right, half_turn=False):
        self.doing_turn = True
        
        if half_turn:
            # Turn around
            #self.current_cardinal_target = self.normalize_angle(self.current_cardinal_target + math.pi)

            if(self.current_cardinal_target == self.cardinals["NORTH"]):
                self.current_cardinal_target = self.cardinals["SOUTH"]
            elif(self.current_cardinal_target == self.cardinals["SOUTH"]):
                self.current_cardinal_target = self.cardinals["NORTH"]
            elif(self.current_cardinal_target == self.cardinals["WEST"]):
                self.current_cardinal_target = self.cardinals["EAST"]
            elif(self.current_cardinal_target == self.cardinals["EAST"]):
                self.current_cardinal_target = self.cardinals["WEST"]

        elif turn_right:
            # RIGHT = Subtract 90 degrees (Clockwise in ROS)
            #self.current_cardinal_target = self.normalize_angle(self.current_cardinal_target - math.pi/2)
            if(self.current_cardinal_target == self.cardinals["NORTH"]):
                self.current_cardinal_target = self.cardinals["EAST"]
            elif(self.current_cardinal_target == self.cardinals["SOUTH"]):
                self.current_cardinal_target = self.cardinals["WEST"]
            elif(self.current_cardinal_target == self.cardinals["WEST"]):
                self.current_cardinal_target = self.cardinals["NORTH"]
            elif(self.current_cardinal_target == self.cardinals["EAST"]):
                self.current_cardinal_target = self.cardinals["SOUTH"]
        else:
            # LEFT = Add 90 degrees (Counter-Clockwise in ROS)
            #self.current_cardinal_target = self.normalize_angle(self.current_cardinal_target + math.pi/2)
            if(self.current_cardinal_target == self.cardinals["NORTH"]):
                self.current_cardinal_target = self.cardinals["WEST"]
            elif(self.current_cardinal_target == self.cardinals["SOUTH"]):
                self.current_cardinal_target = self.cardinals["EAST"]
            elif(self.current_cardinal_target == self.cardinals["WEST"]):
                self.current_cardinal_target = self.cardinals["SOUTH"]
            elif(self.current_cardinal_target == self.cardinals["EAST"]):
                self.current_cardinal_target = self.cardinals["NORTH"]
                
        self.target_yaw = self.current_cardinal_target

    def control_loop(self):
        if not self.navigation_active:
            self.publisher.publish(Twist())
            return

        if not self.odom_ready:
            self.publisher.publish(Twist())
            return
        
        # Initial setup 
        if self.turn_index == 0 and not self.doing_turn:
            self.get_logger().info("Starting navigation - initiating first turn")
            half_turn = (self.start in ["HOUSE_2", "HOUSE_7"] and self.turn_plan[0] == "right")
            self.start_turn(self.turn_plan[0] == "right", half_turn=half_turn)

        if self.mode == Mode.FOLLOW_LINE:
            # Handle active turn
            if self.doing_turn:
                self.cmd.linear.x = 0.0
                
                # Calculate shortest angular distance to target
                error = self.angle_error(self.target_yaw, self.current_yaw)
                
                ANGULAR = self.kp * error
                
                # Minimum rotation speed to overcome friction
                if ANGULAR > 0:
                    self.cmd.angular.z = max(ANGULAR, 0.15)
                else:
                    self.cmd.angular.z = min(ANGULAR, -0.15)

                # Check if turn is complete
                if abs(error) < 0.05:
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.0
                    self.doing_turn = False
                    self.turn_index += 1
                    self.get_logger().info(f"Turn {self.turn_index}/{len(self.turn_plan)} complete")
                    self.needToClearIntersection = True
                    
                    # Check if all turns are done
                    if self.turn_index >= len(self.turn_plan):
                        self.all_turns_complete = True
                        self.get_logger().info("All turns complete - searching for house")
                                
                self.publisher.publish(self.cmd)
                
            # Normal line following mode
            else:
                # CRITICAL: Intersection detection using BOTTOM cameras only
                # T-junction: middle line exists AND (left OR right line appears)
                intersection_detected = (
                    self.line_found and 
                    (self.left_line or self.right_line)
                )

                # Detect intersection and execute turn
                if intersection_detected and not self.all_turns_complete and not self.needToClearIntersection:
                    
                    self.needToClearIntersection = True

                    # Stop completely before turning
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                    self.publisher.publish(self.cmd)
                    
                    # Initiate turn
                    turn_direction = "RIGHT" if self.turn_plan[self.turn_index] == "right" else "LEFT"
                    self.get_logger().info(f"Executing turn {self.turn_index + 1}: {turn_direction}")

                    #Go straight at intersection Logic
                    if(self.turn_plan[self.turn_index] and self.right_line) or (not self.turn_plan[self.turn_index] and self.left_line):
                        self.start_turn(self.turn_plan[self.turn_index])
                        return
                    
                    #otherwise go onto line following vvv
                    

                # Normal line following
                if self.f_line_found:
                    linear, angular = self.calculate_line_following_command(0.15)
                    self.cmd.linear.x = linear
                    self.cmd.angular.z = angular
                    self.already_failed = False

                    # Clear intersection flag when we've passed it
                    if not intersection_detected and self.needToClearIntersection:
                        self.needToClearIntersection = False
                        self.get_logger().info("Intersection cleared")
                    
                else:
                    # Lost line - recovery mode
                    self.sum_line_error = 0.0
                    self.cmd.linear.x = 0.0
                    
                    # Spin in direction of last known error
                    if self.already_failed:
                        spin_speed = 0.7
                    else:
                        spin_speed = 0.4
                        self.already_failed = True
                    
                    if self.last_line_error > 0:
                        self.cmd.angular.z = -spin_speed  # Turn right
                    else:
                        self.cmd.angular.z = spin_speed   # Turn left
                        
                    self.get_logger().debug("Line lost - recovering...")

            # House detection
            if self.all_turns_complete and self.house_visible:
                self.house_seen_frames += 1
                if self.house_seen_frames > 3:
                    self.mode = Mode.VERIFY_HOUSE
                    self.get_logger().info("House confirmed - switching to approach mode")
            else:
                self.house_seen_frames = 0

        elif self.mode == Mode.VERIFY_HOUSE:
            # Approach house slowly
            if self.f_line_found:
                linear, angular = self.calculate_line_following_command(0.08)
                self.cmd.linear.x = linear
                self.cmd.angular.z = angular

            # Stop when close enough
            if self.house_reached:
                self.mode = Mode.STOP
                self.get_logger().info("House reached - STOPPING")

        elif self.mode == Mode.STOP:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)

            self.navigation_active = False
            self.get_logger().info("Navigation complete")
            self.publish_done()

        self.publisher.publish(self.cmd)

def main():
    rclpy.init()
    node = CameraFollower()
    node.get_logger().info("Waiting for simulation to initialize...")

    node.get_logger().info("Starting control loop...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Stop the robot
    stop_cmd = Twist()
    node.publisher.publish(stop_cmd)
    node.get_logger().info('Shutting down - Robot stopped')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
