#!/usr/bin/env python3

# Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
# Control robot velocity through this
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from enum import Enum
from config import directions
import math
from std_msgs.msg import String
import json
from rclpy.qos import QoSProfile, DurabilityPolicy
import subprocess

# Robot's states
class Mode(Enum):
    FOLLOW_LINE = 1
    VERIFY_HOUSE = 2
    STOP = 3

class CameraFollower(Node):
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
        # Flag to set cardinals once
        self.cardinals_initialized = False 

        self.kp = 0.6
        self.ki = 0.0
        self.kd = 0.1

        # Offset from centre
        self.was_line_lost = False
        self.line_error = 0.0
        self.last_line_error = 0.0
        self.sum_line_error = 0.0
        self.left_line = False
        self.right_line = False
        # Front path detection
        self.front_line = False  
        
        # Intersection detection
        self.at_intersection = False
        # For slowing down approach
        self.approaching_intersection = False  
        self.front_magenta_ratio = 0.0
        self.aligning_at_intersection = False
        
        # Forward line alignment for intersections
        self.forward_line_error = 0.0
        self.forward_line_found = False

        self.f_line_found = False
        self.heading_ref = None
        self.heading_kp = 0.6    

        self.house_visible = False
        self.house_visible_front = False
        self.house_visible_left = False
        self.house_visible_right = False
        self.house_reached = False
        # Count the number of times the house was seen
        # This is done to switch to house detection mode
        self.house_seen_frames = 0
        
        # Correction mode when house is on side but not front
        self.correcting_to_house = False

        # Stop distance proxy image-based - when house fills 95% of centre
        # Fine tuned based on experiement with house 2
        self.stop_ratio = 0.95
        self.obstacle_stop_ratio = 0.70
        self.side_stop_ratio = 0.65

        # Cardinal placeholders
        self.cardinals = {}
        self.current_cardinal_target = 0.0

        # Subscriptions
        # Front camera for line-following
        self.front_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.front_callback,
            1
        )

        # Bottom-middle - bottom intersection arrival detection
        self.bm_sub = self.create_subscription(
            Image,
            '/bm_camera/image_raw',
            self.bm_callback,
            10
        )

        # Left - intersection / left line
        self.bl_sub = self.create_subscription(
            Image,
            '/bl_camera/image_raw',
            self.bl_callback,
            10
        )

        # Right - intersection / right line
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
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        
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
        # Exact RGB colors from Gazebo diffuse values
        colors = {
            "HOUSE_1": (97, 63, 0),
            "HOUSE_2": (253, 255, 0),
            "HOUSE_3": (255, 161, 0),
            "HOUSE_4": (252, 149, 209),
            "HOUSE_5": (255, 16, 0),
            "HOUSE_6": (0, 251, 255),
            "HOUSE_7": (255, 0, 229),
            "HOUSE_8": (120, 0, 255),
            "HOUSE_9": (0, 44, 255),
            "HOUSE_10": (146, 220, 255),
            "PO": (255, 255, 255),
        }

        # Convert exact RGB to HSV using OpenCV
        self.house_colours = {}
        for name, rgb in colors.items():
            rgb_np = np.uint8([[list(rgb)]])
            hsv = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2HSV)[0][0]

            # Widen HSV range to tolerate lighting
            h, s, v = hsv
            # self.get_logger().info("Value of h in hsv", h, s, v, "in name", name)
            # Widen HSV range to tolerate lighting and distinguish similar hues for these specific houses
            if name == "HOUSE_4":
                lower = (max(h - 8, 0), 30, 40)
                upper = (min(h + 8, 179), 255, 255)
            elif name == "HOUSE_8":
                lower = (135, 40, 40)
                upper = (165, 255, 255)
            elif name == "HOUSE_9":
                lower = (95, 40, 40)
                upper = (135, 255, 255)
            else:
                lower = (max(h - 6, 0), 60, 40)
                upper = (min(h + 6, 179), 255, 255)


            self.house_colours[name] = (lower, upper)
        
        # Send a sign to deliver robot client to see if the robot has arrived to one house 
        # this is done so it can send another house
        self.done_pub = self.create_publisher(String, 'navigation_done', 10)

        self.box_pub = self.create_publisher(
            String,
            'spawn_box_for_house',
            qos
        )

        self.box_spawned = False
        self.obstacle_detected = False
        self.obstacle_cleared = False
        self.obstacle_stop_start = None
        self.obstacle_stop_duration = 20.0 
        self.box_disappear_duration = 10.0 

    def spawn_box_once(self, house):
        if self.box_spawned:
            return
        
        if self.start == "PO":
            msg = String()
            msg.data = house
            self.box_pub.publish(msg)

            self.get_logger().info(f"Requested box spawn for {house}")
            self.box_spawned = True

    def nav_callback(self, msg):
        data = json.loads(msg.data)

        self.start = data['start']
        self.TARGET_HOUSE = data['target']
        self.spawn_box_once(self.TARGET_HOUSE)

        self.get_logger().info(
            f"Received navigation command: {self.start} → {self.TARGET_HOUSE}"
        )

        # --- RESET STATE ---
        self.turn_index = 0
        self.doing_turn = False
        self.last_line_error = 0
        self.needToClearIntersection = False

        # --- COMPUTE TURN PLAN ---
        # true or false list for right or left turns respectively
        self.turn_plan = directions[self.start][self.TARGET_HOUSE]

        # --- SET HOUSE COLOR ---
        if self.TARGET_HOUSE not in self.house_colours:
            self.get_logger().error(f"Unknown house: {self.TARGET_HOUSE}")
            return

        self.lower, self.upper = self.house_colours[self.TARGET_HOUSE]
        self.colour_low = (40, 60, 40)
        self.colour_up  = (65, 255, 255)

        # Colour camera
        self.colour_sub = self.create_subscription(
            Image,
            '/colour_camera/image_raw',
            self.colour_callback,
            1
        )

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
        upper_black = np.array([26, 26, 26])
        return cv2.inRange(hsv, lower_black, upper_black)
    
    def detect_magenta_ratio(self, img):
        """
        Detect magenta (RGB 255, 0, 255) in an image.
        Returns the ratio of magenta pixels to total pixels.
        """
        # Define magenta color range in RGB
        lower_magenta = np.array([100, 0, 100])
        upper_magenta = np.array([255, 50, 255])
        
        # Create mask for magenta
        mask = cv2.inRange(img, lower_magenta, upper_magenta)
        
        # Calculate ratio
        magenta_pixels = np.sum(mask > 0)
        total_pixels = mask.size
        ratio = magenta_pixels / total_pixels if total_pixels > 0 else 0.0
        
        return ratio

    def get_forward_line_alignment(self, img, h, w):
        """
        Look at TOP portion of front camera to find the black line beyond intersection.
        Returns angular correction needed to align with that line.
        """
        # Look at top 30% of frame - beyond the intersection
        top_section = img[0:int(h*0.3), :]
        hsv = cv2.cvtColor(top_section, cv2.COLOR_RGB2HSV)
        
        # Detect black line
        mask_black = self.detect_black(hsv)
        
        # Find horizontal centre of black pixels
        black_pixels = np.where(mask_black > 0)
        
        # Enough pixels to be reliable
        if len(black_pixels[1]) > 50:  
            # Calculate centroid
            cx = np.mean(black_pixels[1])
            
            # Calculate error from image centre
            error = (cx - w/2) / (w/2)
            
            # error, line_found
            return error, True  
        
        return 0.0, False  # No line found

    
    def bm_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)

        # Check if robot CENTRE is over magenta - intersection confirmation
        magenta_ratio = self.detect_magenta_ratio(img)
        self.front_magenta_ratio = magenta_ratio
        
        # Robot is ON intersection when bottom-middle sees significant magenta
        if magenta_ratio > 0.60:  # More than 60% of image is magenta (robot centre is ON the tile)
            self.at_intersection = True
        else:
            if(self.at_intersection and self.needToClearIntersection):
                self.needToClearIntersection = False
                self.get_logger().info("Intersection cleared")
            self.at_intersection = False

    # check if enough pixels is found in any side camera
    # help to detect which side to move
    def bl_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # House detection logic 
        if self.all_turns_complete:   
            mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

            centre = mask[:, w//2 - 80:w//2 + 80]
            self.house_visible_left = np.sum(mask > 0) > 1200
            self.house_visible = self.house_visible or self.house_visible_left

            if (np.sum(centre > 0) / centre.size) > self.side_stop_ratio and not self.doing_turn:
                self.get_logger().info("Found house on left side, turning...")
                self.start_turn(False)

        else:
            # Check for BLACK line detection - both at intersection and normal
            mask_black = self.detect_black(hsv)
            black_pixels = np.sum(mask_black > 0)
            
            # At intersection: need to see black line to confirm path exists
            self.left_line = black_pixels > 100

    def br_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # House detection logic 
        if self.all_turns_complete:   
            mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

            centre = mask[:, w//2 - 80:w//2 + 80]
            self.house_visible_right = np.sum(mask > 0) > 1200
            self.house_visible = self.house_visible or self.house_visible_right

            if (np.sum(centre > 0) / centre.size) > self.side_stop_ratio and not self.doing_turn:
                self.get_logger().info("Found house on right side, turning...")
                self.start_turn(True)
            
        else:
            # Check for BLACK line detection - both at intersection and normal
            mask_black = self.detect_black(hsv)
            black_pixels = np.sum(mask_black > 0)
            
            # At intersection: need to see black line to confirm path exists
            self.right_line = black_pixels > 100
    
    def front_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        
        # At intersection, check for forward line alignment
        if self.at_intersection:
            self.forward_line_error, self.forward_line_found = self.get_forward_line_alignment(img, h, w)
        
        # Check for magenta in BOTTOM portion of front camera - approaching intersection
        # Only look at bottom 30% of the frame
        roi_start = int(h * 0.85)  # Start from 85% down
        img_roi = img[roi_start:h, :]
        magenta_ratio_roi = self.detect_magenta_ratio(img_roi)
        
        # If we see magenta in bottom of front camera - approaching intersection
        if (magenta_ratio_roi > 0.30) or (not self.needToClearIntersection and not self.at_intersection and self.approaching_intersection):  
            # 80% threshold in the ROI - we don't want it locking too early, because it may be misaligned. 
            # OR:
            # keep perpetuating this value until we either have cleared the intersection - entered go straight at intersection
            # or if we're at the intersection
            self.approaching_intersection = True
            self.get_logger().info(f"Approaching intersection")
            
            self.f_line_found = False

            # 2. Check for black line in TOP MIDDLE to see if path continues forward
            top_limit = int(h * 0.50)
            left_limit = int(w * 0.30)
            right_limit = int(w * 0.70)
            
            # Crop the HSV image to this top-middle box
            hsv_top_middle = img[:, :]
            
            # Detect black in this specific ROI
            mask_black_ahead = self.detect_black(hsv_top_middle)
            black_pixels_ahead = np.sum(mask_black_ahead > 0)
            
            # If enough black pixels are found ahead, mark front_line as valid
            self.front_line = black_pixels_ahead > 100 
            
            return
 
        else:
            self.approaching_intersection = False

        
        # Line following logic 
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        
        # 1. Scan the bottom area
        scan_row = int(h * 0.9)
        row_data = gray[scan_row, :]
        _, thresh = cv2.threshold(row_data, 50, 255, cv2.THRESH_BINARY_INV)

        # Detect and reject horizontal/perpendicular lines
        # A horizontal line will have black pixels spanning most of the width
        total_black_pixels = np.sum(thresh == 255)
        horizontal_line_detected = total_black_pixels > (w * 0.3)  # If >30% of width is black
        
        if horizontal_line_detected:
            # This is likely a perpendicular T-junction line, ignore it completely
            # Instead, look higher up in the image where only the forward line exists
            scan_row = int(h * 0.7)  
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
        
        # Only check sides if MIDDLE is NOT found
        elif segment_density['LEFT'] and segment_density['RIGHT']:
            # Both sides visible - use weighted average
            M_left = cv2.moments(segments['LEFT'])
            M_right = cv2.moments(segments['RIGHT'])
            
            if M_left["m00"] > 0 and M_right["m00"] > 0:
                cx_left = M_left["m10"] / M_left["m00"]
                cx_right = (M_right["m10"] / M_right["m00"]) + m_end
                target_cx = (cx_left + cx_right) / 2.0
            
        elif segment_density['LEFT']:
            M = cv2.moments(segments['LEFT'])
            if M["m00"] > 0:
                target_cx = M["m10"] / M["m00"]
        
        elif segment_density['RIGHT']:
            M = cv2.moments(segments['RIGHT'])
            if M["m00"] > 0:
                target_cx = (M["m10"] / M["m00"]) + m_end

        # 5. Calculate error
        if target_cx is not None:
            new_error = float(target_cx - (w / 2)) / (w / 2)
            if abs(new_error) < 0.08:
                self.heading_ref = self.current_yaw
            else:
                self.heading_ref = None
            
            self.line_error = new_error
            self.f_line_found = True
        else:
            self.f_line_found = False
            

    def colour_callback(self, msg):
        # self.get_logger().info("Colour callback triggered")
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # self.get_logger().info(f"hsv {hsv}")
        # Only check for obstacle if not yet cleared
        if not self.obstacle_cleared and self.start == "PO":
            mask = cv2.inRange(hsv, np.array(self.colour_low), np.array(self.colour_up))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            # self.get_logger().info(f"Obstacle pixels detected: {np.sum(mask > 0)}")

            centre = mask[:, w//2 - 80:w//2 + 80]
            # self.get_logger().info(f"Obstacle centre ratio: {np.sum(centre > 0) / centre.size:.3f}")
            self.obstacle_detected = (np.sum(centre > 0) / centre.size) > self.obstacle_stop_ratio
            if self.obstacle_detected and self.obstacle_stop_start is None:
                self.get_logger().info("Obstacle detected - stopping for 20s")
                self.obstacle_stop_start = self.get_clock().now()

        # House detection logic 
        if self.all_turns_complete:   
            mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

            centre = mask[:, w//2 - 80:w//2 + 80]
            # self.get_logger().info(f"House centre ratio: {np.sum(centre > 0) / centre.size:.3f}")
            self.house_visible_front = np.sum(mask > 0) > 1200
            
            # Overall visibility is if any camera sees it
            self.house_visible = self.house_visible_front or self.house_visible_left or self.house_visible_right
            
            self.house_reached = (np.sum(centre > 0) / centre.size) > self.stop_ratio     
            # self.get_logger().info(f"House visible: {self.house_visible}, House reached ratio: {np.sum(centre > 0) / centre.size:.3f}")


    def remove_box(self):
        if not self.box_spawned:
            self.get_logger().warn("No box to remove")
            return
        self.get_logger().info("Removing box from simulation...")
        box_name = "random_green_box"
        try:
            cmd = [
                'gz', 'service',
                '-s', '/world/empty/remove',
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'name: "{box_name}" type: 2'  # type: 2 means MODEL
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info(f'Box {box_name} removed successfully!')
                self.obstacle_cleared = True
            else:
                self.get_logger().error(f'Failed to remove box: {result.stderr}')
        except Exception as e:
            self.get_logger().error(f'Error removing box: {str(e)}')

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        # get yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
        if not self.cardinals_initialized:
            self.start_yaw = self.current_yaw 
            self.get_logger().info(f"Initialised cardinals where start_yaw is {self.start_yaw}")
            # Facing SOUTH (~3.14), adding pi/2 (Left) should result in EAST (~ -1.57)
            self.cardinals = {
                'NORTH': self.normalize_angle(self.start_yaw),
                'WEST':  self.normalize_angle(self.start_yaw - math.pi/2 ) , # Right 
                'SOUTH': self.normalize_angle(self.start_yaw + math.pi),    # Behind
                'EAST':  self.normalize_angle(self.start_yaw + math.pi/2 )  # Left
            }
            self.current_cardinal_target = self.cardinals['SOUTH']
            self.cardinals_initialized = True
            self.get_logger().info(f"SOUTH: {self.cardinals["SOUTH"]}")
            self.get_logger().info(f"WEST: {self.cardinals["WEST"]}")
            self.get_logger().info(f"NORTH: {self.cardinals["NORTH"]}")
            self.get_logger().info(f"EAST: {self.cardinals["EAST"]}")
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
        
        # Calculate terms
        P = self.line_error * self.kp
        I = self.sum_line_error * self.ki
        D = (self.line_error - self.last_line_error) * self.kd
        
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
        self.get_logger().info(f"Starting turn {'RIGHT' if turn_right else 'LEFT'}{' (180)' if half_turn else ''}")
        if half_turn:
            # Turn around
            #self.current_cardinal_target = self.normalize_angle(self.current_cardinal_target + math.pi)
            if(self.is_same_angle(self.current_cardinal_target, self.cardinals["NORTH"])):
                self.current_cardinal_target = self.cardinals["SOUTH"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["SOUTH"])):
                self.current_cardinal_target = self.cardinals["NORTH"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["WEST"])):
                self.current_cardinal_target = self.cardinals["EAST"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["EAST"])):
                self.current_cardinal_target = self.cardinals["WEST"]
            else:
                self.get_logger().info("FAILED TO FIND AN ANGLE TO TURN TO")
            
        elif turn_right:
            # RIGHT = Subtract 90 degrees (Clockwise in ROS)
            #self.current_cardinal_target = self.normalize_angle(self.current_cardinal_target - math.pi/2)
            if(self.is_same_angle(self.current_cardinal_target, self.cardinals["NORTH"])):
                self.current_cardinal_target = self.cardinals["EAST"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["SOUTH"])):
                self.current_cardinal_target = self.cardinals["WEST"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["WEST"])):
                self.current_cardinal_target = self.cardinals["NORTH"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["EAST"])):
                self.current_cardinal_target = self.cardinals["SOUTH"]
            else:
                self.get_logger().info("FAILED TO FIND AN ANGLE TO TURN TO")
            

        else:
            # LEFT = Add 90 degrees (Counter-Clockwise in ROS)
            #self.current_cardinal_target = self.normalize_angle(self.current_cardinal_target + math.pi/2)
            if(self.is_same_angle(self.current_cardinal_target, self.cardinals["NORTH"])):
                self.current_cardinal_target = self.cardinals["WEST"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["SOUTH"])):
                self.current_cardinal_target = self.cardinals["EAST"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["WEST"])):
                self.current_cardinal_target = self.cardinals["SOUTH"]
            elif(self.is_same_angle(self.current_cardinal_target, self.cardinals["EAST"])):
                self.current_cardinal_target = self.cardinals["NORTH"]
            else:
                self.get_logger().info("FAILED TO FIND AN ANGLE TO TURN TO")  

        self.target_yaw = self.current_cardinal_target

    def control_loop(self):
        # Handle obstacle stopping
        if self.obstacle_detected and self.obstacle_stop_start is not None and self.start == "PO":
            elapsed = (self.get_clock().now() - self.obstacle_stop_start).nanoseconds / 1e9
            if elapsed >= self.box_disappear_duration and not self.obstacle_cleared:
                self.remove_box()
                self.get_logger().info("Obstacle cleared - resuming navigation")
            if elapsed < self.obstacle_stop_duration:
                # Stop robot
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher.publish(self.cmd)
                return
        if not self.navigation_active:
            self.publisher.publish(Twist())
            return

        if not self.odom_ready:
            self.publisher.publish(Twist())
            return
        
        # Initial setup 
        if self.turn_index == 0 and not self.doing_turn:
            self.get_logger().info("Starting navigation - initiating first turn")
            half_turn = (self.start in ["HOUSE_2", "HOUSE_7"] and self.turn_plan[0])
            self.start_turn(self.turn_plan[0], half_turn=half_turn)

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

                    if self.at_intersection:
                        self.needToClearIntersection = True
                    
                    # Check if all turns are done
                    if self.turn_index >= len(self.turn_plan):
                        self.all_turns_complete = True
                        self.get_logger().info("All turns complete - searching for house")
            # not doing turn
            else:

                # Slow down when approaching intersection
                if self.approaching_intersection and not self.at_intersection and not self.all_turns_complete:
                    self.cmd.linear.x = 0.1
                    self.cmd.angular.z = 0.0
                    self.publisher.publish(self.cmd)
                    self.get_logger().info("Approaching intersection - moving slowly")
                    return
                
                # need to do turn?
                # NEW: Intersection detection using MAGENTA from middle camera
                # Detect intersection and execute turn
                if self.at_intersection and not self.all_turns_complete and not self.needToClearIntersection:
                    # Step 1: Stop and start alignment
                    if not self.aligning_at_intersection:
                        self.approaching_intersection = False
                        self.get_logger().info(f"Intersection detected! Magenta ratio: {self.front_magenta_ratio:.2f}")
                        
                        # Stop completely
                        self.cmd.linear.x = 0.0
                        self.cmd.angular.z = 0.0
                        self.publisher.publish(self.cmd)
                        
                        # Set flag to start alignment
                        self.aligning_at_intersection = True
                        return
                    
                    # Step 2: Align using forward line visual feedback
                    elif self.aligning_at_intersection:
                        # Use forward line to align if visible
                        if self.forward_line_found:
                            # PID control to centre on forward line
                            angular = self.kp * self.forward_line_error
                            
                            if abs(self.forward_line_error) < 0.05:  # Aligned!
                                self.cmd.angular.z = 0.0
                                self.cmd.linear.x = 0.0
                                self.aligning_at_intersection = False
                                
                                self.get_logger().info(f"Aligned using forward line! Error was: {self.forward_line_error:.3f}")
                                
                                # NOW take the camera readings after alignment
                                self.get_logger().info(f"Path availability - Left: {self.left_line}, Right: {self.right_line}, Front: {self.front_line}")
                                
                                self.needToClearIntersection = True
                                
                                # Initiate turn based on turn plan
                                turn_direction = "RIGHT" if self.turn_plan[self.turn_index] else "LEFT"
                                self.get_logger().info(f"Executing turn {self.turn_index + 1}: {turn_direction}")

                                # Logic: only execute turn if the path exists
                                if (self.turn_plan[self.turn_index] and self.right_line) or (not self.turn_plan[self.turn_index] and self.left_line):
                                    self.start_turn(self.turn_plan[self.turn_index])
                                    self.publisher.publish(self.cmd)
                                    return
                                elif self.front_line:
                                    self.get_logger().info("Intended turn path blocked, continuing straight")
                                else:
                                    self.get_logger().warn("No valid path detected at intersection!")
                            else:
                                # Continue aligning
                                self.cmd.linear.x = 0.0
                                self.cmd.angular.z = angular
                                self.get_logger().debug(f"Aligning to forward line... error: {self.forward_line_error:.3f}")
                        else:
                            # No forward line visible - skip alignment and just execute turn
                            self.get_logger().warn("No forward line for alignment - executing turn without alignment")
                            self.aligning_at_intersection = False
                            
                            # Read paths and execute turn
                            self.get_logger().info(f"Path availability - Left: {self.left_line}, Right: {self.right_line}, Front: {self.front_line}")
                            self.needToClearIntersection = True
                            
                            turn_direction = "RIGHT" if self.turn_plan[self.turn_index] else "LEFT"
                            self.get_logger().info(f"Executing turn {self.turn_index + 1}: {turn_direction}")
                            
                            if (self.turn_plan[self.turn_index] and self.right_line) or (not self.turn_plan[self.turn_index] and self.left_line):
                                self.start_turn(self.turn_plan[self.turn_index])
                            elif self.front_line:
                                self.get_logger().info("Intended turn path blocked, continuing straight")
                            else:
                                self.get_logger().warn("No valid path detected at intersection!")
                        
                        self.publisher.publish(self.cmd)
                        return

                # Normal line following
                if self.f_line_found:
                    #reset PID stuff after correcting back to line.
                    if self.was_line_lost:
                        self.sum_line_error = 0.0
                        self.last_line_error = 0.0
                        self.heading_ref = None
                        self.was_line_lost = False

                    linear, angular = self.calculate_line_following_command(0.15)  # Normal speed
                    
                    self.cmd.linear.x = linear
                    self.cmd.angular.z = angular

                else:
                
                    # Lost line - recovery mode
                    self.was_line_lost = True
                    self.sum_line_error = 0.0
                    self.cmd.linear.x = 0.0
                    
                    # Spin in direction of last known error
                    spin_speed = 0.3
                    if self.last_line_error > 0:
                        self.cmd.angular.z = -spin_speed  # Turn right
                    else:
                        self.cmd.angular.z = spin_speed   # Turn left
                        
                    self.get_logger().debug("Line lost - recovering...")
                    

            # House detection
            if self.all_turns_complete and self.house_visible:
                self.get_logger().info(f"House visible: {self.house_visible} turns complete: {self.all_turns_complete}")
                self.house_seen_frames += 1
                self.get_logger().info(f"House seen frames: {self.house_seen_frames}")
                if self.house_seen_frames > 3:
                    self.mode = Mode.VERIFY_HOUSE
                    self.get_logger().info("House confirmed - switching to approach mode")
            elif self.all_turns_complete:
                self.house_seen_frames = 0

        elif self.mode == Mode.VERIFY_HOUSE:
            
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
                    self.get_logger().info(f"Turn {self.turn_index}/{len(self.turn_plan)} complete")
                    self.house_reached = True
                                
                self.publisher.publish(self.cmd)

            # Stop when close enough
            elif self.house_reached:
                self.mode = Mode.STOP
                self.house_reached = False
                self.house_visible = False
                self.house_visible_left = False
                self.house_visible_right = False
                self.get_logger().info("House reached - STOPPING")
            
            # Check if we need to correct alignment - house on side but not front
            elif self.house_visible_front:
                self.cmd.linear.x = 1.0
                self.cmd.angular.z = 0.0
            else:
            
                if (self.house_visible_right or self.house_visible_left) and not self.house_visible_front and not self.house_reached:
                    # Only correct if we're not super close
                    # This prevents correction when very close
                    
                    if not self.correcting_to_house:
                        self.get_logger().info(f"House on side but not front - correcting alignment. Right: {self.house_visible_right}, Left: {self.house_visible_left}")
                        self.correcting_to_house = True
                    
                    # Slow rotation toward the house
                    self.cmd.linear.x = 0.0
                    
                    if self.house_visible_right:
                        self.cmd.angular.z = -0.2  # Turn right slowly
                    else:  # house_visible_left
                        self.cmd.angular.z = 0.2   # Turn left slowly
                    
            
                # Normal approach - house is in front or correction complete
                self.correcting_to_house = False
                
                # Approach house slowly
                if self.f_line_found:
                    linear, angular = self.calculate_line_following_command(0.08)
                    self.cmd.linear.x = linear
                    self.cmd.angular.z = angular
                else:
                    #Lost line - recovery mode
                    self.sum_line_error = 0.0
                    self.cmd.linear.x = 0.0
                        
                    # Spin in direction of last known error
                    spin_speed = 0.3
                        
                    if self.last_line_error > 0:
                        self.cmd.angular.z = -spin_speed  # Turn right
                    else:
                        self.cmd.angular.z = spin_speed   # Turn left
                        
                    self.get_logger().debug("Line lost - recovering...")


        elif self.mode == Mode.STOP:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)

            self.navigation_active = False
            self.mode = Mode.FOLLOW_LINE
            self.all_turns_complete = False
            self.turn_index = 0
            self.get_logger().info("Navigation complete")
            self.publish_done()

        self.publisher.publish(self.cmd)

    def is_same_angle(self,a, b, tol=0.15):
        return abs(self.angle_error(a, b)) < tol

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