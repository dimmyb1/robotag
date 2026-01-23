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
        self.f_left_line = False
        self.f_right_line = False

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

        """

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

        """

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
            # computer error from center to steer
            cx = int(M["m10"] / M["m00"])
            self.line_found = True
            # Now normalized to [-1, 1] range
            #self.line_error = float(cx - (w / 2)) / (w / 2)
            #self.last_line_error = self.line_error
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

        # Update intersection detection flags FIRST
        self.f_left_line = segment_density['LEFT']
        self.f_right_line = segment_density['RIGHT']

        target_cx = None

        # 4. FIXED: Prioritize MIDDLE strongly when it exists
        # Only look at sides if middle is completely gone
        if segment_density['MIDDLE']:
            M = cv2.moments(segments['MIDDLE'])
            if M["m00"] > 0:
                target_cx = (M["m10"] / M["m00"]) + m_start
            self.f_line_found = True
        
        # Only check sides if MIDDLE is NOT found
        elif segment_density['LEFT'] and segment_density['RIGHT']:
            # Both sides visible - likely at intersection, use weighted average
            M_left = cv2.moments(segments['LEFT'])
            M_right = cv2.moments(segments['RIGHT'])
            
            if M_left["m00"] > 0 and M_right["m00"] > 0:
                cx_left = M_left["m10"] / M_left["m00"]
                cx_right = (M_right["m10"] / M_right["m00"]) + m_end
                # Average the two to stay centered
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
            
            # Apply minimum force threshold
            min_force = 0.3
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
        # 1. Update the Integral (Sum of errors)
        self.sum_line_error += self.line_error
        
        # 2. Anti-Windup: Limit the memory so it doesn't go crazy
        # This prevents the "I" term from overpowering everything else
        max_integral = 5.0
        self.sum_line_error = max(min(self.sum_line_error, max_integral), -max_integral)
        
        if(self.f_left_line or self.f_right_line):
            kp = self.kp *0.7
            kd = self.kd *0.7
        else:
            kp = self.kp
            kd = self.kd

        # 3. Calculate terms
        P = self.line_error * kp
        I = self.sum_line_error * self.ki
        D = (self.line_error - self.last_line_error) * kd
        
        # 4. Combine (Negative sign for direction correction)
        angular = -(P + I + D)
        
        # 5. Reset memory if we cross zero (Optional but helpful)
        # If we crossed the center line, we don't need to 'remember' the old bias anymore
        if (self.line_error > 0 and self.last_line_error < 0) or \
        (self.line_error < 0 and self.last_line_error > 0):
            self.sum_line_error = 0.0

        self.last_line_error = self.line_error
        
        # Cap the output
        angular = max(min(angular, 0.6), -0.6)

        
        
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
            self.get_logger().info("Waiting for Navigation...")
            self.publisher.publish(Twist())
            return

        if not self.odom_ready:
            self.get_logger().info("Waiting for Odometry...")
            self.publisher.publish(Twist())
            return
        
        """
        
        if self.get_clock().now() < self.wait_until:
            # Keep the robot still while waiting
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)
            return # Skip the rest of the logic

        """
        
        # Initial setup 
        if self.turn_index == 0 and not self.doing_turn:
            self.get_logger().info("DEBUG: initial setup before first turn")
            half_turn = (self.start in ["HOUSE_2", "HOUSE_7"] and self.turn_plan[0] == "right")
            self.start_turn(self.turn_plan[0] == "right", half_turn=half_turn)

        if self.mode == Mode.FOLLOW_LINE:
            #self.get_logger().info(f"Doing turn {self.doing_turn} -> ANGULAR {self.cmd.angular.z} LINEAR {self.cmd.linear.x} left or right {self.turn_plan[self.turn_index]}")
            # Handle active turn
            if self.doing_turn:
                self.cmd.linear.x = 0.0
                
                # Calculate shortest angular distance to target
                error = self.angle_error(self.target_yaw, self.current_yaw)
                
                ANGULAR = self.kp * error
                #max_rot_speed = 0.3 #was 0.1 -> 0.5 -> 0.3 
                #ANGULAR = max(min(ANGULAR, max_rot_speed), -max_rot_speed)
                
                #dont let it  be too small otherwise it's not gonna manage to have enough power to turn
                #self.get_logger().info(f"ANGULAR: {ANGULAR}")
                if ANGULAR > 0:
                    self.cmd.angular.z = max(ANGULAR, 0.1)
                else:
                    self.cmd.angular.z = min(ANGULAR, -0.1)


                # Check if turn is complete
                if abs(error) < 0.01:
                    # IMPO - Set to 0 to try and avoid circular moving
                    self.cmd.angular.z = 0.0
                    self.cmd.linear.x = 0.0
                    self.get_logger().info("DEBUG: STOPPED turning")
                    self.doing_turn = False

                    self.turn_index+=1
                    self.get_logger().info(f"TURN {self.turn_index}/{len(self.turn_plan)} COMPLETE")
                    self.needToClearIntersection = True
                    
                    
                    # Check if all turns are done
                    if self.turn_index >= len(self.turn_plan):
                        self.all_turns_complete = True
                        self.get_logger().info("ALL TURNS COMPLETE - Now searching for house")
                                    
                self.publisher.publish(self.cmd)
            # Normal moving forward
            else:

                # Detect intersection and start next turn
                intersection_detected = (
                    (self.left_line and self.line_found) or
                    (self.right_line and self.line_found) or
                    (self.left_line and self.right_line and self.line_found)
                )

                if intersection_detected==False and self.needToClearIntersection==True:
                    self.needToClearIntersection = False
                    self.get_logger().info("Cleared intersection")
                

                #this section isn't right, nehhejtuli l logic kollu rip- note to self redo the go straight where necessary logic 
                elif intersection_detected and not self.all_turns_complete and not self.doing_turn and not self.needToClearIntersection:
                    self.get_logger().info("DEBUG: INTERSECTION DETECTED")
                    self.needToClearIntersection = True
                    if self.turn_index < len(self.turn_plan):
                        # Start the next turn based on direction plan
                        self.start_turn(self.turn_plan[self.turn_index]=="right")
                        self.publisher.publish(self.cmd)

                elif not self.doing_turn:
                    if self.line_found:
                        linear, angular = self.calculate_line_following_command(0.1)
                        self.cmd.linear.x = linear
                        self.cmd.angular.z = angular
                        self.already_failed = False
                    else:
                        self.sum_line_error = 0.0
                        self.cmd.linear.x = 0.0
                        # Spin in the direction of the last known error to find it again!
                        # If error was positive (Line on Right), spin Right (Negative Z)
                        if(self.already_failed):
                            if self.last_line_error > 0:
                                self.cmd.angular.z = -0.6 # Turn Right
                            else:
                                self.cmd.angular.z = 0.6  # Turn Left
                        else:
                            if self.last_line_error > 0:
                                self.cmd.angular.z = -0.3 # Turn Right
                            else:
                                self.cmd.angular.z = 0.3  # Turn Left

                            self.already_failed = True
                        self.get_logger().info("Lost line... Recovering")
                

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
            linear, angular = self.calculate_line_following_command(0.08)
            if self.line_found:
                self.cmd.linear.x = linear
                self.cmd.angular.z = angular

            # Stop when house is close enough
            if self.house_reached:
                self.mode = Mode.STOP
                self.get_logger().info("House reached - STOPPING")

        elif self.mode == Mode.STOP:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)

            self.navigation_active = False
            self.get_logger().info("Navigation complete. Waiting for next command.")
            self.publish_done()

        #self.get_logger().info(f"CMD: v={self.cmd.linear.x:.2f}, w={self.cmd.angular.z:.2f}")
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
