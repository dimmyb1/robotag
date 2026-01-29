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
        self.kd = 0.0471

        # offset from center
        self.line_error = 0.0
        self.last_line_error = 0.0
        self.sum_line_error = 0.0
        self.left_line = False
        self.right_line = False
        self.front_line = False  # NEW: front path detection
        self.already_failed = False
        
        # NEW: Intersection detection
        self.at_intersection = False
        self.approaching_intersection = False  # NEW: for slowing down approach
        self.front_magenta_ratio = 0.0

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

        #front camera for line-following
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

        # left - intersection / left line
        self.bl_sub = self.create_subscription(
            Image,
            '/bl_camera/image_raw',
            self.bl_callback,
            10
        )

        # right - intersection / right line
        self.br_sub = self.create_subscription(
            Image,
            '/br_camera/image_raw',
            self.br_callback,
            10
        )

        # Colour camera, top camera for house detection
        self.colour_sub = self.create_subscription(
            Image,
            'colour_camera/image_raw',
            self.colour_callback,
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
            "obstacle": (128, 0, 128)
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

        self.box_pub = self.create_publisher(
            String,
            'spawn_box_for_house',
            qos
        )

        self.box_spawned = False
        self.obstacle_detected = False
        self.obstacle_cleared = False
        self.obstacle_stop_start = None
        self.obstacle_stop_duration = 30.0  # seconds
        self.box_disappear_duration = 20.0  # seconds
        self.box_removed = False

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
        self.colour_low, self.colour_up = self.house_colours["obstacle"]

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
    
    def detect_magenta_ratio(self, img):
        """
        Detect magenta (RGB 255, 0, 255) in an image.
        Returns the ratio of magenta pixels to total pixels.
        """
        # Define magenta color range in RGB
        lower_magenta = np.array([100, 0, 100])
        upper_magenta = np.array([255, 0, 255])
        
        # Create mask for magenta
        mask = cv2.inRange(img, lower_magenta, upper_magenta)
        
        # Calculate ratio
        magenta_pixels = np.sum(mask > 0)
        total_pixels = mask.size
        ratio = magenta_pixels / total_pixels if total_pixels > 0 else 0.0
        
        return ratio

    
    def bm_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)

        # NEW: Check if robot CENTER is over magenta (intersection confirmation)
        magenta_ratio = self.detect_magenta_ratio(img)
        self.front_magenta_ratio = magenta_ratio
        
        # Robot is ON intersection when bottom-middle sees significant magenta
        if magenta_ratio > 0.90:  # More than 90% of image is magenta (robot center is ON the tile)
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

        # Check for BLACK line detection (both at intersection and normal)
        mask = self.detect_black(hsv)
        black_pixels = np.sum(mask > 0)
        
        # At intersection: need to see black line to confirm path exists
        self.left_line = black_pixels > 100

    def br_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Check for BLACK line detection (both at intersection and normal)
        mask = self.detect_black(hsv)
        black_pixels = np.sum(mask > 0)
        
        # At intersection: need to see black line to confirm path exists
        self.right_line = black_pixels > 100
    
    def front_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        
        # NEW: Check for magenta in BOTTOM portion of front camera (approaching intersection)
        # Only look at bottom 30% of the frame
        roi_start = int(h * 0.85)  # Start from 85% down
        img_roi = img[roi_start:h, :]
        magenta_ratio_roi = self.detect_magenta_ratio(img_roi)
        
        # If we see magenta in bottom of front camera (approaching intersection)
        if (magenta_ratio_roi > 0.80) or (not self.needToClearIntersection and not self.at_intersection and self.approaching_intersection):  
            # 80% threshold in the ROI - we don't want it locking too early, because it may be misaligned. 
            # OR:
            #keep perpetuating this value until we either have cleared the intersection (entered go straight at intersection)
            #  or if we're at the intersection (and we're gonna handle that)
            self.approaching_intersection = True
            self.get_logger().info(f"Approaching intersection")
            
            self.f_line_found = False

            # 2. Check for black line in TOP MIDDLE (to see if path continues forward)
            # Define ROI: Top 30% of height, middle 20% of width
            top_limit = int(h * 0.50)
            left_limit = int(w * 0.40)
            right_limit = int(w * 0.60)
            
            # Crop the HSV image to this top-middle box
            hsv_top_middle = img[0:top_limit, left_limit:right_limit]
            
            # Detect black in this specific ROI
            mask_black_ahead = self.detect_black(hsv_top_middle)
            black_pixels_ahead = np.sum(mask_black_ahead > 0)
            
            # If enough black pixels are found ahead, mark front_line as valid
            self.front_line = black_pixels_ahead > 100 
            
            return
 
        else:
            self.approaching_intersection = False

        
        # ORIGINAL: Line following logic (unchanged)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        
        # 1. Scan the bottom area
        scan_row = int(h * 0.9)
        row_data = gray[scan_row, :]
        _, thresh = cv2.threshold(row_data, 50, 255, cv2.THRESH_BINARY_INV)

        # NEW: Detect and reject horizontal/perpendicular lines
        # A horizontal line will have black pixels spanning most of the width
        total_black_pixels = np.sum(thresh == 255)
        horizontal_line_detected = total_black_pixels > (w * 0.3)  # If >30% of width is black
        
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
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # House detection logic 
        if self.all_turns_complete:   
            mask = cv2.inRange(hsv, np.array(self.lower), np.array(self.upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

            center = mask[:, w//2 - 80:w//2 + 80]
            self.house_visible = np.sum(mask > 0) > 1200
            self.house_reached = (np.sum(center > 0) / center.size) > self.stop_ratio
        # Only check for obstacle if not yet cleared
        elif not self.obstacle_cleared:
            mask = cv2.inRange(hsv, np.array(self.colour_low), np.array(self.colour_up))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

            center = mask[:, w//2 - 80:w//2 + 80]
            self.obstacle_detected = (np.sum(center > 0) / center.size) > self.stop_ratio
            if self.obstacle_detected:
                self.get_logger().info("Obstacle detected - stopping for 30s")
                self.obstacle_stop_start = self.get_clock().now()

                # Schedule box removal after 20 seconds
                self.get_logger().info("Box will disappear in 20s")
                self.create_timer(self.box_disappear_duration, self.remove_box)

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
                'SOUTH': self.normalize_angle(self.start_yaw),
                'WEST':  self.normalize_angle(self.start_yaw - math.pi/2 ) , # Right -0.2 to maybe correct 0.2 rad diff
                'NORTH': self.normalize_angle(self.start_yaw + math.pi),    # Behind
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
        # I think we need a check here for whether or not self.current_cardinal_target is actually correct
        # since the map has some corners, at which the robot would need to turn, but these turns 
        # aren't in dir dir, and not intersections

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
        # Handle obstacle stopping
        if self.obstacle_detected and self.obstacle_stop_start is not None:
            elapsed = (self.get_clock().now() - self.obstacle_stop_start).nanoseconds / 1e9
            if elapsed < self.obstacle_stop_duration:
                # Stop robot
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher.publish(self.cmd)
                return
            else:
                # Obstacle cleared - resume
                self.obstacle_detected = False
                self.obstacle_cleared = True
                self.obstacle_stop_start = None
                self.get_logger().info("Obstacle cleared - resuming navigation")
                
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

                """
                if(abs(error) >= 1.396263): #80 degrees, the robot must have turned at some point
                    closestCardinal = self.current_cardinal_target
                    for c in ["NORTH", "SOUTH", "EAST", "WEST"]:
                        if self.angle_error(self.cardinals[c], self.current_yaw) < error :
                            closestCardinal = self.cardinals[c]

                    self.current_cardinal_target = closestCardinal
                    self.target_yaw = closestCardinal
                """
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
                                
                self.publisher.publish(self.cmd)
                
            #NOT DOING TURN
            else:

                # Slow down when approaching intersection
                if self.approaching_intersection and not self.at_intersection:
                    self.cmd.linear.x = 0.1
                    self.cmd.angular.z = 0.0
                    self.publisher.publish(self.cmd)
                    self.get_logger().info("Approaching intersection - moving slowly")
                    return
                
                #need to do turn?
                # NEW: Intersection detection using MAGENTA from middle camera
                # Detect intersection and execute turn
                if self.at_intersection and not self.all_turns_complete and not self.needToClearIntersection:
                    #also had and not self.front_line,  removing it for now
                    self.get_logger().info(f"Intersection detected! Magenta ratio: {self.front_magenta_ratio:.2f}")
                    self.get_logger().info(f"Path availability - Left: {self.left_line}, Right: {self.right_line}, Front: {self.front_line}")
                    
                    self.needToClearIntersection = True

                    # Stop completely before turning
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                    self.publisher.publish(self.cmd)
                    
                    # Initiate turn based on turn plan
                    turn_direction = "RIGHT" if self.turn_plan[self.turn_index] == True else "LEFT"
                    self.get_logger().info(f"Executing turn {self.turn_index + 1}: {turn_direction}")

                    # Logic: only execute turn if the path exists
                    # If turning right and right path exists, OR turning left and left path exists
                    if (self.turn_plan[self.turn_index] and self.right_line) or (not self.turn_plan[self.turn_index] and self.left_line):
                        self.start_turn(self.turn_plan[self.turn_index])
                        return
                    elif self.front_line:
                        # If the intended turn path doesn't exist but front does, go straight
                        self.get_logger().info("Intended turn path blocked, continuing straight")
                        # Don't start a turn, just continue following the line
                    else:
                        self.get_logger().warn("No valid path detected at intersection!")

                # Normal line following
                if self.f_line_found:
                    
                    linear, angular = self.calculate_line_following_command(0.15)  # Normal speed
                    
                    self.cmd.linear.x = linear
                    self.cmd.angular.z = angular
                    self.publisher.publish(self.cmd)
                    self.already_failed = False

                else:
                    if self.at_intersection and self.needToClearIntersection and not self.f_line_found:
                        self.cmd.linear.x = 0.1
                        self.cmd.angular.z = 0.0
                        self.publisher.publish(self.cmd)
                        self.get_logger().info("Going straight at intersection - slowly.")
                        return
                
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
                        
                    self.get_logger().info("Line lost - recovering...")
                    

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