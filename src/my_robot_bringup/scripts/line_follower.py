#!/usr/bin/env python3
# Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan
from nav_msgs.msg import Odometry
# Control robot velocity through this
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from enum import Enum
from config import directions
import math
from std_msgs.msg import String, Float64, Float64MultiArray, Bool,  Float32MultiArray
import json
from rclpy.qos import QoSProfile, DurabilityPolicy
import subprocess
import time
import random

import heapq #for dijkstra

#list of constants and hyperparameters
#BOXMEAS - measurable
#PERSISTENCE - tunable
#CONTAMINATION - tunable
#CERTAINTY - tunable
#CONSIDER_NODES - tunable, partly measurable
#GRAY_COOLDOWN - measurable
#SENSE_COOLDOWN - measurable, partly tunable
#TURN_TIME - measurable
#TIME_VARIANCE - measurable, partly tunable
#PAUSE_TIME - measurable, partly tunable
#MIN_DIST - measurable
#TAG_COOLDOWN - measurable, partly tunable

class Noden():
    def __init__(self, nc, ec, sc, wc, n, t):
        #dir_c -> where dir can be N, E, S, W and c just means 'char' so you would have 'A', 'B' .. etc 'A' - 'H' representing the node name
        self.Nc = nc
        self.Ec = ec
        self.Sc = sc
        self.Wc = wc

        #.name -> the name of this node as a char : 'A' - 'H'
        self.name = n

        #.Times -> [1.0, 23.0, 14.5, 16.8]  where Times[0] -> North transition time cost = 1.0
        self.Times = t

     
class Cell():
    def __init__(self, xx, yy):
        self.x = xx
        self.y = yy

    def __eq__(self, other):
        if isinstance(other, Cell):
            return self.x == other.x and self.y == other.y
        return False

class line_follower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        #for time related operations
        self.now = time.time()
        
        #for motion, turns, and line following
        self.speed = 60
        self.dur = 0
        self.elapsed = 0
        self.searchLeft = False
        self.searchRight = False
        self.searching = False
        self.thirty = 2933 # Amount to move for angles
        self.realDelay = 150
        self.motion_active = False
        self.motion_end_time = 0
        self.current_motion = None
        self.searchStep = 0
        self.yaw_deg = 0 #IMU
        self.STARTUP_TIME = -1
        self.ANGLE_TOLERANCE = 4

        #junction turning vars
        self.stateFollow = True
        #self.completeTurn = False
        self.wasLeft = False
        self.imu_turning = False
        self.imu_target = -1
        self.grayEntryTime = -1
        self.GRAY_COOLDOWN = 10.1
        #   tried 8, but 8 seems too high for the lengths of the shortest edges
        #   but 5 is too high when turning, so maybe we can make this variable with turning time
        #   instead, i blocked intersection detection completely when imu_turning at an intersection
        #   5 is too low when traversing, im setting it to 7.
        #   7 seems really good.
        #   it is now clear that the time to handle an intersection has no relation to the length of the shortest edges
        #   and that the gray_cooldown needs to be how long it takes from entering to exiting an intersection, inclusive of turning time.
        #   7 too short, sometimes double-triggering setting to 9.
        #   9 is a lot better, but one double-trigger still occured. raising to 10
        #   10 was leaving a 0.0 residual, so i've raised it 10.1
        self.senseEntryTime = -1
        self.SENSE_COOLDOWN = 6 #tried 5 but seemed low
        self.dontSense = False
        self.dontUpdate = False # for when an intersection misfire is detected
        self.firstNode = True
        #self.haventMovedYet = True
        self.stationaryStartTime = -1
        self.allowCrawl = True #make sure we start at an intersection
        self.crawlingForwardBeforeIMUturn = False
        self.crawlBackBeforeIMUturn = False
        self.crawlingBackwards = False
        self.aligning = False

        #beh modes 3 and 5
        self.lookAround = False
        self.goAhead = False
        self.stepping = False
        self.completeSequence = False

        #tag vars + esp comms
        #look for the paper dated 8 may for a discussion on the tuning of capture_max
        self.CAPTURE_MAX = 0.115 #was (10cm, 0.1m), but was changed to 11.5cm to try to prevent any damage as the robots were bumping into each other
        self.PAUSE_TIME = 14 #was 6, but seemed low so changed to 7. 7 seems low, raising to 14 to avoid immediate tag-backs
        self.startPauseTime = -1
        self.paused = False
        self.time_of_last_tag = -1
        self.TAG_COOLDOWN = 6
        self.tag = False
        self.ack = False
        self.other_tag = False
        self.other_ack = False
        self.initiated_tag = False
        self.doTag = False

        #ultrasonic sensor and servo vars
        self.entry_angle = float('inf')
        self.exit_angle = float('inf')
        self.ultrasonic_distance = 100.0
        self.sweep = False
        self.multiple = False
        self.locateTarget = False
        self.initial_reading_taken = False
        self.waitingForUltrasonic = False
        self.triggerSweep = False

        #IR sensor vars
        self.colours = [0,0,0]
        self.isGray = [0,0,0]
        self.minPixels = 20

        # Graph
        self.A = Noden('B', 'B', 'F', 'E', 'A', [78, 11, 61, 140])
        self.B = Noden('A', 'C', 'D', 'A', 'B', [106, 14, 29, 11])
        self.C = Noden('H', 'D', 'D', 'B', 'C', [155, 49, 10, 15])
        self.D = Noden('C', 'C', 'F', 'B', 'D',  [13,  59, 40, 46])
        self.E = Noden('A', 'F', 'G', 'G', 'E',  [128, 12, 49, 90])
        self.F = Noden('A', 'D', 'G', 'E', 'F',  [111, 41, 10, 34])
        self.G = Noden('F', 'H', 'E', 'E', 'G',   [9,  12, 109,34])
        self.H = Noden('C', 'H', 'H', 'G', 'H',  [130, 85, 89, 14])

        self.myNodes = [self.A, self.B, self.C, self.D, self.E, self.F, self.G, self.H]

        #relating to state and location
        self.current_node = self.A
        self.facing = 2 #start facing south?
        self.E_facing = 2
        self.last_node = 'Z' #for localisation, this is NODE, 'Z' is a placeholder for (re)starting
        self.current_destination = 'F'
        self.destination_id = -1 #this is used when current_destination is a string or list of strings as an identifier for paths which may have the same start and end points but are different
        self.skipZero = False
        self.retryPlan = 0
        self.postRetry = False
        self.retryAttempts = 0
        

        #localisation
        self.departureTime = -1
        self.TIME_VARIANCE = 20 #time variance of 2 was misfiring after about 3-6 intersections. now raising time variance to 3. 3 is very strict.
        #on average, the S.D. of timing tends to be close to 20, so im changing 3 to 20. 
        self.loc_hyp = []
        self.toDepart = False
        self.h1 = None
        self.h2 = None
        self.h3 = None

        #pursuit-evasion behaviour
        self.behaviourMode = 0
        self.patrolPath = ['A', 'F', 'G', 'E', 'F', 'D', 'C', 'H', 'H', 'G', 'E', 'A', 'B', 'C', 'D', 'B']
        self.i_patrol = 0
        self.opp_old_loc = -1
        self.resetBehaviour = False
        self.evading = False 
        
        # behaviourMode settings:
        # 0 - not set (no behaviour)
        # 1 - Simple Line Follower
        # 2 - Random
        # 3 - Greedy
        # 4 - Avoidant
        # 5 - Interceptive

        #start off at 1/16
        #(we have 16 edges)
        self.Po = {
            "Pab1": 0.0625,
            "Pab2": 0.0625,
            "Paf1": 0.0625,
            "Pae1": 0.0625,
            "Pef1": 0.0625,
            "Peg1": 0.0625,
            "Peg2": 0.0625,
            "Pgh1": 0.0625,
            "Phh1": 0.0625,
            "Pfg1": 0.0625,
            "Pdf1": 0.0625,
            "Pbd1": 0.0625,
            "Pbc1": 0.0625,
            "Pcd1": 0.0625,
            "Pcd2": 0.0625,
            "Pch1": 0.0625
        }
        
        self.P = {
            "Pab1": 0.0,
            "Pab2": 0.0,
            "Paf1": 0.0,
            "Pae1": 0.0,
            "Pef1": 0.0,
            "Peg1": 0.0,
            "Peg2": 0.0,
            "Pgh1": 0.0,
            "Phh1": 0.0,
            "Pfg1": 0.0,
            "Pdf1": 0.0,
            "Pbd1": 0.0,
            "Pbc1": 0.0,
            "Pcd1": 0.0,
            "Pcd2": 0.0,
            "Pch1": 0.0
        }

        
        # Subscriptions

        robot_name = self.get_namespace().strip('/')

        if not robot_name:
            robot_name = 'generic'

        topic_L = f'/{robot_name}_ir_L/image_raw'
        topic_M = f'/{robot_name}_ir_M/image_raw'
        topic_R = f'/{robot_name}_ir_R/image_raw'
        topic_IMU = f'/{robot_name}/imu'
        topic_object = f"/{robot_name}/object_data"

        # behaviourMode settings:
        # 0 - not set (no behaviour)
        # 1 - Simple Line Follower
        # 2 - Random
        # 3 - Greedy
        # 4 - Avoidant
        # 5 - Interceptive
        if robot_name == 'twix':
            self.current_node = self.H
            self.get_logger().info("Detected robot: twix. Starting at Node H.")
            other_robot_name = 'twirl'
            self.behaviourMode = 3
            self.evading = False
        elif robot_name == 'twirl':
            self.current_node = self.A
            self.get_logger().info("Detected robot: twirl. Starting at Node A.")
            other_robot_name = 'twix'
            self.behaviourMode = 1
            self.evading = True
            self.i_patrol = 0
            #self.behaviourMode = 2
            #self.evading = True
        else:
            # Fallback in case you run it without a namespace
            self.current_node = self.A 
            self.get_logger().info(f"Unknown robot name '{robot_name}', defaulting to Node A.")


        self.ir_L_sub = self.create_subscription(
            Image,
            topic_L,
            self.ir_L_callback,
            1
        )
        self.ir_M_sub = self.create_subscription(
            Image,
            topic_M,
            self.ir_M_callback,
            1
        )
        self.ir_R_sub = self.create_subscription(
            Image,
            topic_R,
            self.ir_R_callback,
            1
        )

        self.imu_sub = self.create_subscription(
            Imu,
            topic_IMU,
            self.imu_callback,
            10)
        

        
        self.data_sub = self.create_subscription(
            Float64MultiArray,
            topic_object,
            self.ultrasonic_callback,
            10
        )

        self.sweep_pub = self.create_publisher(
            String,
            f'/{robot_name}/sweep',
            10
        )


        
        self.tag_pub = self.create_publisher(
            String, 
            f'/{robot_name}/esp', 
            10
        )

        self.tag_sub = self.create_subscription(
            String, 
            f'/{other_robot_name}/esp', 
            self.tag_callback, 
            10
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()

        self.timer = self.create_timer(0.05, self.loop)

    #ESP callback
    def tag_callback(self, msg):
        #Handshaking
        data = json.loads(msg.data)
        self.other_tag = data.get("tag", False)
        self.other_ack = data.get("ack", False)
            
        
        if self.initiated_tag:
            if not self.ack and not self.other_ack:
                #deadlock: both trying to initiate. I will back down
                #deterministic resolution, if i am twix, then im the one who needs to back down.
                if (self.now > self.time_of_last_tag + self.TAG_COOLDOWN) and self.get_namespace().strip('/') == 'twix':
                    self.initiated_tag = False
                    self.tag = True
                    self.ack = True

            if self.ack and not self.other_ack:
                self.initiated_tag = False
                self.ack = False
                self.doTag = True
            elif self.other_ack:
                self.tag = False
                self.ack = True
        else:
            if self.other_tag and not self.tag and (self.now > self.time_of_last_tag + self.TAG_COOLDOWN):
                self.tag = True
                self.ack = True
            elif self.other_ack:
                self.ack = False
                self.tag = False
                self.doTag = True

        # Optional: Print to the terminal so you can see it working
        # self.get_logger().info(f"{self.other_robot} has the tag: {self.opponent_has_tag}")

    def publish_tag_status(self):
        payload = {
            "tag": self.tag or self.initiated_tag,
            "ack": self.ack
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.tag_pub.publish(msg)

    #MPU5060 / IMU callback
    def imu_callback(self, msg: Imu):
        # The IMU gives us a quaternion (x, y, z, w)
        q = msg.orientation
        
        # Convert the quaternion to Yaw (Rotation around the Z axis) in radians
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = math.degrees(yaw_rad)

        #normalise to [0, 360)
        self.yaw_deg = yaw_deg % 360.0

        #for physical:
        #given that we start facing south
        # if 0.0 <= self.yaw_deg < 45.0 or 315.0 <= self.yaw_deg < 360.0:
        #     self.facing = 2
        # elif 45.0 <= self.yaw_deg < 135.0 :
        #     self.facing = 3
        # elif 135.0 <= self.yaw_deg < 225.0 :
        #     self.facing = 0
        # elif 225.0 <= self.yaw_deg < 315.0 :
        #     self.facing = 1

        #NORTH IS 0/360, WEST IS 90, SOUTH IS 180, EAST IS 270
        #specifically because of the sim, this needs to be written as:
        if 0.0 <= self.yaw_deg < 45.0 or 315.0 <= self.yaw_deg < 360.0:
            self.facing = 0
        elif 45.0 <= self.yaw_deg < 135.0 :
            self.facing = 3
        elif 135.0 <= self.yaw_deg < 225.0 :
            self.facing = 2
        elif 225.0 <= self.yaw_deg < 315.0 :
            self.facing = 1
        
        #self.get_logger().info(f'IMU:: Current Z Rotation (Yaw): {self.yaw_deg:.2f}°, Facing: {self.facing}')

    #Ultrasonic functions
    def ultrasonic_callback(self, msg):
        # Unpack the array based on the order you published it
        #ANGLES ARE IN RADIANS (but i can do math.degrees(v) to convert to normal degrees if i need to)
        self.entry_angle = msg.data[0]
        self.exit_angle = msg.data[1]
        self.ultrasonic_distance = msg.data[2]

        if  self.waitingForUltrasonic:
            self.waitingForUltrasonic = False
            #self.locateTarget = False
        
        self.get_logger().info(f"Received Object Data -> Entry: {self.entry_angle:.2f}, Exit: {self.exit_angle:.2f}, Dist: {self.ultrasonic_distance:.2f}")

    def publish_sweep_command(self):
        payload = {
            "sweep": self.sweep,
            "multiple": self.multiple
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.sweep_pub.publish(msg)
        self.get_logger().info(f"SWEEP PUBLISHED: sweep={self.sweep}, multiple={self.multiple}")

    #Line Following Functions
    def detect_black(self, img):
        # Convert BGR (OpenCV default) to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # returns a mask of black pixels in the image
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([0, 0, 12])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        return cv2.countNonZero(mask) #(int) num of black pixels in img
    
    def detect_gray(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # returns a mask of black pixels in the image
        #these are tuned for ultramarine blue
        lower_gray = np.array([100, 200, 30])
        upper_gray = np.array([130, 250, 130])
        mask = cv2.inRange(hsv, lower_gray, upper_gray)

        #for debugging
        #gray_pixels = hsv[mask > 0]
        # sort by V channel (brightness)
        #gray_pixels = gray_pixels[np.argsort(gray_pixels[:, 2])]
        #self.get_logger().info(f"GRAY: {gray_pixels[:20]}")   # darkest 20 detected pixels
        #self.get_logger().info(f"HSV: {hsv.reshape(-1,3)}")


        return cv2.countNonZero(mask) #(int) num of gray pixels in img
    
    def ir_L_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        self.colours[0] = self.detect_black(img)
        self.isGray[0] = self.detect_gray(img)

    def ir_M_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        self.colours[1] = self.detect_black(img)
        self.isGray[1] = self.detect_gray(img)

    def ir_R_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        self.colours[2] = self.detect_black(img)
        self.isGray[2] = self.detect_gray(img)

    # --------------------------
    # Motor Control Functions
    # --------------------------
    def start_motion(self, linear=0.0, angular=0.0, duration_ms=0):
        self.cmd.linear.x = linear
        self.cmd.angular.z = angular
        self.publisher.publish(self.cmd)

        if duration_ms > 0:
            self.motion_active = True
            self.motion_end_time = self.now + (duration_ms / 1000.0)
        else:
            self.motion_active = False #it will run continuously

    def update_motion(self):
        STARTED_FACING = 0
        
        #self.get_logger().info(f"Turning... yaw={self.yaw_deg:.1f}, target={self.imu_target}")
        if self.motion_active and self.now >= self.motion_end_time:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)
            self.motion_active = False

            if self.crawlingBackwards:
                self.crawlingBackwards = False
            if self.crawlBackBeforeIMUturn:
                self.crawlBackBeforeIMUturn = False #consume
                self.get_logger().info("finished going a tiny bit back, now starting IMU turn.")
                self.startTurnBasedOnIMU()
                

            #self.get_logger().info(f"stopped moving because time expired. imu_turning: {self.imu_turning}, complete_turn: {self.completeTurn}, motion_active: {self.motion_active}")
        elif self.aligning:
            
            target_yaw = {0: 0, 1: 270, 2: 180, 3: 90}
            target = target_yaw[self.facing]

            if target < 0:
                target+=360

            if target < self.ANGLE_TOLERANCE:
                if (360+target) - self.ANGLE_TOLERANCE <= self.yaw_deg or self.yaw_deg <= target + self.ANGLE_TOLERANCE:
                    #aligned
                    self.aligning = False
                    self.stopMov()
                        
                    self.crawlForward()
                    self.crawlingForwardBeforeIMUturn = True
            else:
                if target - self.ANGLE_TOLERANCE <= self.yaw_deg <= target + self.ANGLE_TOLERANCE:
                    #aligned
                    self.aligning = False
                    self.stopMov()

                    self.crawlForward()
                    self.crawlingForwardBeforeIMUturn = True

        elif self.crawlingForwardBeforeIMUturn and (self.isGray[0] <= self.minPixels) and (self.isGray[1] <= self.minPixels) and (self.isGray[2] <= self.minPixels):
            #keep crawling forwards until we aren't seeing any gray anymore.
            self.stopMov()
            self.crawlingForwardBeforeIMUturn = False
            self.get_logger().info("Finished crawling forwards. Now going a tiny bit back.")
            self.crawlBackBeforeIMUturn = True
            
        elif(self.imu_turning):

            target_yaw = {0: 0, 1: 270, 2: 180, 3: 90}
            target = target_yaw[self.imu_target]

            if target < 0:
                target+=360

            if target < self.ANGLE_TOLERANCE:
                if (360+target) - self.ANGLE_TOLERANCE <= self.yaw_deg or self.yaw_deg <= target + self.ANGLE_TOLERANCE:
                    self.get_logger().info(f"in update_motion: target < 4 deg, toDep: {self.toDepart}")
                    #we have completed our turn.
                    self.imu_turning = False
                    self.imu_target = -1
                    self.stopMov()
                    self.retryPlan = 0

                    if self.toDepart:
                        self.toDepart = False #consume
                        self.departureTime = self.now
                        self.stateFollow = True
                        self.get_logger().info("stateFollow is true, depart is false")
            else:
                if target - self.ANGLE_TOLERANCE <= self.yaw_deg <= target + self.ANGLE_TOLERANCE:
                    #we have completed our turn.
                    self.imu_turning = False
                    self.imu_target = -1
                    self.stopMov()
                    self.retryPlan = 0

                    if self.toDepart:
                        self.toDepart = False #consume
                        self.departureTime = self.now
                        self.stateFollow = True
 
        if self.waitingForUltrasonic:
            self.stopMov()

    def stopMov(self) :
        self.start_motion()
        
    # --------------------------
    # Basic Turning Functions
    # --------------------------

    def turnRight(self, duration=0):
        self.start_motion(angular=-0.75, duration_ms=duration)

    def turnLeft(self, duration=0) :
        self.start_motion(angular=0.75, duration_ms=duration)

    #--------------------
    # Searching for Gray
    #--------------------

    def crawlBack(self):
        self.start_motion(linear=-0.35, duration_ms=300) #3000ms was too much, 750ms was too much, 300ms was too little, tried 500ms, went down to 300 again and raised pwr

    def crawlForward(self):
        self.stopMov()
        self.get_logger().info("STARTED CRAWLING FORWARD")
        self.start_motion(linear=+0.15, duration_ms=0) #linear: .35 was too aggressive, would overshoot, so trying .25, trying .15 cos .25 was still too much

    def clearGray(self):
        self.get_logger().info("Called clearGray - aligning before crawling forward")
        self.stopMov()
        self.stateFollow = False
        target_yaw = {0: 0, 1: 270, 2: 180, 3: 90}
        
        target = target_yaw[self.facing]

        if target < 0:
            target+=360

        if target < self.ANGLE_TOLERANCE:
            if (360+target) - self.ANGLE_TOLERANCE <= self.yaw_deg or self.yaw_deg <= target + self.ANGLE_TOLERANCE:
                self.get_logger().info("already aligned")
                #already aligned
                self.aligning = False
                self.crawlForward()
                self.crawlingForwardBeforeIMUturn = True
            else:
                self.aligning = True
                error = (target - self.yaw_deg) % 360.0
                if error <= 180:
                    self.turnLeft(0)   # increase yaw
                else:
                    self.turnRight(0)  # decrease yaw
        else:
            if target - self.ANGLE_TOLERANCE <= self.yaw_deg <= target + self.ANGLE_TOLERANCE:
                #already aligned
                self.get_logger().info("already aligned")
                self.aligning = False
                self.crawlForward()
                self.crawlingForwardBeforeIMUturn = True
            else:
                self.get_logger().info(f"yaw_deg: {self.yaw_deg}")
                self.aligning = True
                error = (target - self.yaw_deg) % 360.0
                if error <= 180:
                    self.turnLeft(0)   # increase yaw
                else:
                    self.turnRight(0)  # decrease yaw
 
        
    #---------------------
    # Searching for Line
    #---------------------

    def smartTurnRight(self, totalDuration,stepDelay = 50) :
        # While the angle is less then the expected turn
        if (self.elapsed < totalDuration):
            # Check if there is a black line in one of the sensors if so return to continue executing
            if (self.minPixels < self.colours[1]) or  (self.minPixels < self.colours[0]) or (self.minPixels < self.colours[2]) :
                
                self.get_logger().info("Line found during right turn!")
                self.stopMov()
                self.searchRight = False
                self.stateFollow = True
                return True
            
            # Continue turning right
            self.turnRight(0)
            
            # Increment the steps
            self.elapsed += stepDelay
            return False # exit and wait for next tick
            
        self.get_logger().info("No line found during smart right turn")
        self.stopMov()
        self.searchRight = False
        return False # finished full arc


    def smartTurnLeft(self, totalDuration,stepDelay = 50) :
        # While the angle is less then the expected turn
        if (self.elapsed < totalDuration):
            
            # Check if there is a black line in one of the sensors if so return to continue executing
            if (self.minPixels < self.colours[1]) or (self.minPixels < self.colours[0]) or (self.minPixels < self.colours[2]):
                self.get_logger().info("Line found during left turn!")
                self.stopMov()
                self.searchLeft = False
                self.stateFollow = True

                return True
            
            # otherwise continue turning left
            self.turnLeft(0)

            # Increment the steps
            self.elapsed += stepDelay
            return False
        
        self.get_logger().info("No line found during smart left turn")
        self.stopMov()
        self.searchLeft = False
        return False
        

    def search(self):
        #start condition
        if not self.searching:
            self.elapsed = 0
            self.found = False
            self.searchLeft = False
            self.searchRight = False

            if self.searchStep == 0:
                self.searchLeft = True
                self.dur = self.thirty

            elif self.searchStep == 1:
                self.searchRight = True
                if self.skipZero:
                    self.dur = self.thirty *2
                else:
                    self.dur = self.thirty *3

            elif self.searchStep == 2:
                self.searchLeft = True
                self.dur = self.thirty * 4

            elif self.searchStep == 3:
                self.searchRight = True
                self.dur = self.thirty *6

            elif self.searchStep == 4:
                self.searchLeft = True
                self.dur = self.thirty * 7

            elif self.searchStep == 5:
                self.searchLeft = True
                self.dur = self.thirty * 3

            elif self.searchStep ==6:
                self.searchLeft = True
                self.dur = self.thirty * 6

            else:
                #reverse and restart search
                self.start_motion(linear=-0.25, duration_ms=self.realDelay)
                self.searchStep = 0
                self.searching = False
                return
            
            self.searching = True
            #self.get_logger().info(f"STEP {self.searchStep}")

        self.handleSearchLoop()

        

    def handleSearchLoop(self):
        #carry out turn
        if self.searchLeft:
            self.found = self.smartTurnLeft(self.dur)
        
        elif self.searchRight:
            self.found = self.smartTurnRight(self.dur)

        #end conditions
        if(self.found):
            self.searchStep = 0
            self.searching = False
            self.elapsed = 0
        elif(self.elapsed>=self.dur):
            self.stopMov()
            self.searchStep+=1
            self.searching = False
            self.elapsed = 0

    #------------------------
    # Normal Line Following
    #------------------------

    def followLine(self):

        L = self.colours[0]
        M = self.colours[1]
        R = self.colours[2]

        if self.minPixels < M :
            self.start_motion(linear=0.15)

            self.searchStep = 0
            self.searching = False
            self.elapsed = 0

            

        elif self.minPixels < L:
            self.turnLeft(self.realDelay)

            self.searchStep = 0
            self.searching = False
            self.elapsed = 0

        elif self.minPixels < R:
            self.turnRight(self.realDelay)

            self.searchStep = 0
            self.searching = False
            self.elapsed = 0

        else:
            self.search()

    def returnNode(self, c):
        if c == 'A':
            return self.A
        elif c == 'B':
            return self.B
        elif c == 'C':
            return self.C
        elif c == 'D':
            return self.D
        elif c == 'E':
            return self.E
        elif c == 'F':
            return self.F
        elif c == 'G':
            return self.G
        elif c == 'H':
            return self.H
        else:
            self.get_logger().info(f"Can't return nonexistent char-node value for {c}. returning None")
            return None
        
    #-----------------------
    # Node Helper Functions
    #-----------------------
    def getEdgesFromNode(self, fromNCHAR):
        #ONLY ACCEPTS CHARS
        if fromNCHAR == 'A': return ["Pab1", "Pab2", "Paf1", "Pae1"]
        if fromNCHAR == 'B': return ["Pab1", "Pab2", "Pbd1", "Pbc1"]
        if fromNCHAR == 'C': return ["Pcd1", "Pcd2", "Pch1", "Pbc1"]
        if fromNCHAR == 'D': return ["Pcd1", "Pcd2", "Pdf1", "Pbd1"]
        if fromNCHAR == 'E': return ["Peg1", "Peg2", "Pae1", "Pef1"]
        if fromNCHAR == 'F': return ["Pef1", "Paf1", "Pfg1", "Pdf1"]
        if fromNCHAR == 'G': return ["Peg1", "Peg2", "Pfg1", "Pgh1"]
        if fromNCHAR == 'H': return ["Pch1", "Phh1", "Pgh1"]

        else: 
            self.get_logger().info(f"From getEdgesFromNode ERR: Can't return nonexistent node's edges")
            return []


    def getNodesFromEdge(self, fromE):
        #returns the parent node/s of an edge
        A = ["Pab1", "Pab2", "Paf1", "Pae1"]
        B = ["Pab1", "Pab2", "Pbd1", "Pbc1"]
        C = ["Pcd1", "Pcd2", "Pch1", "Pbc1"]
        D = ["Pcd1", "Pcd2", "Pdf1", "Pbd1"]
        E = ["Peg1", "Peg2", "Pae1", "Pef1"]
        F = ["Pef1", "Paf1", "Pfg1", "Pdf1"]
        G = ["Peg1", "Peg2", "Pfg1", "Pgh1"]
        H = ["Pch1", "Phh1", "Pgh1"]
        #this is not a mistake. H only has 3 unique edge transitions.
            
        toReturn = []

        if fromE in A:
            if(toReturn):
                toReturn.append('A')
                return toReturn
            else:
                toReturn.append('A')

        if fromE in B:
            if(toReturn):
                toReturn.append('B')
                return toReturn
            else:
                toReturn.append('B')
        if fromE in C:
            if(toReturn):
                toReturn.append('C')
                return toReturn
            else:
                toReturn.append('C')
        if fromE in D:
            if(toReturn):
                toReturn.append('D')
                return toReturn
            else:
                toReturn.append('D')
        if fromE in E:
            if(toReturn):
                toReturn.append('E')
                return toReturn
            else:
                toReturn.append('E')
        if fromE in F:
            if(toReturn):
                toReturn.append('F')
                return toReturn
            else:
                toReturn.append('F')
        if fromE in G:
            if(toReturn):
                toReturn.append('G')
                return toReturn
            else:
                toReturn.append('G')
        if fromE in H:
            if(toReturn):
                toReturn.append('H')
                return toReturn
            else:
                toReturn.append('H')

        return toReturn



        
    def getNeighbourEdgesOf(self, fromE):
        A = ["Pab1", "Pab2", "Paf1", "Pae1"]
        B = ["Pab1", "Pab2", "Pbd1", "Pbc1"]
        C = ["Pcd1", "Pcd2", "Pch1", "Pbc1"]
        D = ["Pcd1", "Pcd2", "Pdf1", "Pbd1"]
        E = ["Peg1", "Peg2", "Pae1", "Pef1"]
        F = ["Pef1", "Paf1", "Pfg1", "Pdf1"]
        G = ["Peg1", "Peg2", "Pfg1", "Pgh1"]
        H = ["Pch1", "Phh1", "Pgh1"]
        #this is not a mistake. H only has 3 unique edge transitions.
            
        toReturn = []

        if fromE in A:
            A.remove(fromE)
            if(toReturn):
                toReturn.extend(A)
                return toReturn
            else:
                toReturn = A

        if fromE in B:
            B.remove(fromE)
            if(toReturn):
                toReturn.extend(B)
                return toReturn
            else:
                toReturn = B
        if fromE in C:
            C.remove(fromE)
            if(toReturn):
                toReturn.extend(C)
                return toReturn
            else:
                toReturn = C
        if fromE in D:
            D.remove(fromE)
            if(toReturn):
                toReturn.extend(D)
                return toReturn
            else:
                toReturn = D
        if fromE in E:
            E.remove(fromE)
            if(toReturn):
                toReturn.extend(E)
                return toReturn
            else:
                toReturn = E
        if fromE in F:
            F.remove(fromE)
            if(toReturn):
                toReturn.extend(F)
                return toReturn
            else:
                toReturn = F
        if fromE in G:
            G.remove(fromE)
            if(toReturn):
                toReturn.extend(G)
                return toReturn
            else:
                toReturn = G
        if fromE in H:
            H.remove(fromE)
            if(toReturn):
                toReturn.extend(H)
                return toReturn
            else:
                toReturn = H

        return toReturn
    

    #--------------------
    # Self-Localisation
    #--------------------
    def adjustDestBasedOnBeh(self):
        # behaviourMode settings:
        # 0 - not set (no behaviour)
        # 1 - Simple Line Follower
        # 2 - Random
        # 3 - Greedy
        # 4 - Avoidant
        # 5 - Interceptive

        if self.behaviourMode == 1:
            #patrol
            self.destination_id = -1
            if self.facing == 0:
                self.current_destination = self.current_node.Nc
            elif self.facing == 1:
                self.current_destination = self.current_node.Ec
            elif self.facing == 2:
                self.current_destination = self.current_node.Sc
            elif self.facing == 3:
                self.current_destination = self.current_node.Wc

            #i_patrol points to the NEXT index (destination index)
            i1 = self.patrolPath.index(self.current_node.name)
            i2 = self.patrolPath.index(self.current_node.name, i1 +1)
            try:
                if self.patrolPath[i1+1] == self.current_destination:
                    self.i_patrol = i1+1
                elif self.patrolPath[i2+1] == self.current_destination:
                    self.i_patrol = i2+1
            except IndexError:
                if self.patrolPath[i1+1] == self.current_destination:
                    self.i_patrol = i1+1
                elif self.patrolPath[0] == self.current_destination:
                    self.i_patrol = 0

        elif self.behaviourMode == 2:
            pass
        elif self.behaviourMode ==3:
            pass
        elif self.behaviourMode ==4:
            pass
        elif self.behaviourMode ==5:
            pass
        else:
            self.get_logger().warning(f"adjusting destination based on non-existent behaviour value failed.")
        

    def self_localise(self, edgeTime):
        #IMPORTANT NOTE FOR FUTURE SELF:
        #self_localise uses STALE VALUES
        #we were at curr_node, we wanted to go to curr_dest, we should be at curr_dest, we took 'elapsed' amount of time to get to the next intersection
        #based on 'elapsed', is it likely that we ended up at curr_dest?
        #if not, look at the neighbouring edges for what may have been the real curr_dest
        #when deploying a hypothesis, we say, well, if we were at bcln (curr_node), then we ended up at bc (curr_dest) based on the closest time bct
        NEGLIGABLE = 0.15

        if(self.firstNode):
            self.get_logger().info("SKIPPING COS FIRST NODE IS TRUE")
            return
        #where edge time is the average timing of the edge we took
        elapsed = self.now - self.departureTime
        maxTime = edgeTime + self.TIME_VARIANCE
        minTime = edgeTime - self.TIME_VARIANCE

        if elapsed < NEGLIGABLE * edgeTime:
            self.get_logger().info("This intersection looks to be a misfire. Skipping UpdatePos.")
            self.dontUpdate = True
            return

        #1. merit hk
        #for all non-None, non-empty hypotheses hk,
        #if hk.dir is possible with our elapsed timings, give merit to that hypothesis and move it a time step forward
        #if hk is empty, then it has lost all of its credibility.
        #keep track of bc and bcln, the best candidate and their last node from h1
        bcln = None 
        bc = None
        bct = 999
        upd = 0
        for hk in [self.h1, self.h2, self.h3]:
            upd +=1
            if hk != None and hk != []:
                new_hk = []
                for hh in hk:
                    hn = self.returnNode(hh)
                    for t in range(4):
                        if minTime < hn.Times[t] < maxTime:
                            d = {0: hn.Nc, 1: hn.Ec, 2: hn.Sc, 3: hn.Wc}
                            if upd == 1:
                                #if we're in h1
                                if abs(hn.Times[t] - elapsed) < bct:
                                    bct = hn.Times[t]
                                    bcln = hn.name
                                    bc = d[t]
                            
                            if type(self.current_destination) == str:
                                if d[t] != self.current_destination and d[t] not in new_hk:
                                    new_hk.append(d[t])
                            elif self.current_destination:
                                #must be a list of integers
                                if t != self.current_destination[0] and t not in new_hk:
                                    new_hk.append(d[t])
                                

                hk.clear()
                hk.extend(new_hk)

        #2. create hx
        #create a new hypothesis, either [] or [...] where we either expected this timing or we didnt
        hx = []
        if (elapsed > maxTime) or (elapsed < minTime):
            if elapsed > maxTime:
                self.get_logger().info(f"traversal took too long. {elapsed}s elapsed but maxTime was {maxTime}s")
            else:
                self.get_logger().info(f"traversal was too fast. {elapsed}s elapsed but minTime was {minTime}s")
                #if it was too fast, we may have incorrectly triggered an intersection detection, re-push our previous stats
                hx.append(self.current_node.name)

            #if it has taken longer than or less than the expected time
            #check if any of the old node's timings match better
            if minTime < self.current_node.Times[0] < maxTime:
                hx.append(self.current_node.Nc)
            elif minTime < self.current_node.Times[1] < maxTime:
                hx.append(self.current_node.Ec)
            elif minTime < self.current_node.Times[2] < maxTime:
                hx.append(self.current_node.Sc)
            elif minTime < self.current_node.Times[3] < maxTime:
                hx.append(self.current_node.Wc)

            #if not, we either got turned around, or we just struggled / found it easy to get here
            #in case we got turned around copy the node we left from:

            #special cases are those edges where you arrive at either node facing the same direction no matter which one you left from, so 
            #if you got turned around, we cant check it just by seeing expected facing
            special_cases = [('A', 'B', 2), ('B', 'A', 2), ('C', 'H', 2), ('H', 'C', 2), ('C', 'D',3), ('D', 'C', 3)]

            if (self.facing != self.E_facing or (self.facing == self.E_facing and (self.current_node.name, self.current_destination, self.facing) in special_cases)) and self.current_node.name not in hx:
                hx.append(self.current_node.name)
            for x in hx:
                if x == self.current_destination:
                    hx.remove(x)
            #otherwise if we just deviated slightly, the rest of the map should match up, so either way let's check the next edge we take:
            self.get_logger().info(f"SELF_LOC: Timing was off, created hypothesis {hx}")

        else:
            self.get_logger().info(f"traversal had expected time of {elapsed}s")

        #3. enroll hx:
        #where hx is either [] or [...]
        #save it in our records of 3 placements h1-3
        #h1 is our oldest possible hyp, h3 is our newest possible hyp
        if self.h1 == None:
            self.h1 = hx.copy()
        elif self.h2 == None:
            self.h2 = hx.copy()
        elif self.h3 == None:
            self.h3 = hx.copy()
        else:
            self.get_logger().info("cannot enroll hx, my h1-3 are full.")

        self.get_logger().info(f"h1: {self.h1}, h2: {self.h2}, h3: {self.h3}")

        #4. graduate
        if self.h3 != None:
            #if h1 is empty, it was discarded
            #if h2 and h3 are both empty, we haven't run into any issues lately, or have no good hypotheses, so we can safely discard h1.
            if self.h1 and (self.h2 or self.h3):
                #there's quite a bit of commotion and noise, h2 and h3 are either both [...], or one of them is [...]
                #this makes it hard to tell how trustworthy h1 is, so lets let h1 graduate but keep note of h2 and h3

                if bc != None:
                    #update using the best candidate from h1
                    self.current_node = self.returnNode(bcln)
                    self.current_destination = bc
                    self.destination_id = -1
                    self.get_logger().info(f"SELF_LOC: Confirmed hypothesis and updating curr. loc. to {self.current_destination}")
                    self.adjustDestBasedOnBeh()
                else:
                    self.get_logger().info(f"SELF_LOC: tried to graduate hypothesis but best candidate was Nonetype.")

            #age up
            self.h1 = self.h2.copy()
            self.h2 = self.h3.copy()
            self.h3 = None


    #-------------------------------------
    # Target Tracking and Route Planning
    #-------------------------------------
        
    #Graph Functions
    def calculateProbabilities(self):
        self.get_logger().info("entered calculateProbabilities function")
        #let's say radar stores the closest ultrasonic ping in euclidean metric in self.ultrasonic_distance
        #we get two readings: first ping entering reading (self.entry_angle)
        # second ping exiting reading (self.exit_angle)
        # the order doesnt make a difference, main thing is that we have the angle, wwe'll just take min or max of the two values.
        #and servo was the angle at which we go tthe reading, +- the known margin of error
        #and that BOXMEAS is the l / w of the boxes in the grid in euclidean metric
        BOXMEAS = 0.4
        PERSISTENCE = 0.7
        CONTAMINATION = 0.3
        
        #and let's say we have estimated our current coordinates to be x and y
        #if we know our current node, we know our current coordinates.
        if self.current_node.name == 'A':
            x = 1
            y = 4
        elif self.current_node.name == 'B':
            x = 2
            y = 4
        elif self.current_node.name == 'C':
            x = 3
            y = 4
        elif self.current_node.name == 'D': 
            x = 3
            y = 3
        elif self.current_node.name == 'E':
            x = 1
            y = 2
        elif self.current_node.name == 'F':
            x = 2
            y = 2
        elif self.current_node.name == 'G':
            x = 2
            y = 1
        elif self.current_node.name == 'H':
            x = 3
            y = 1

        # so if x=2 and y =3 minimum (2,3), maximum (3,4) starts
        #these need to be estimated by timing how long we're following a black line for against our pretimed table
        
        #get current cardinal direction we're facing 
        #check if not detected at all! or if you only have 1 reading i.e. 1 reading before the line of detection
        #so we only see start and not end i.e. somewhere along our 90 degree sides.

        #then we can calculate our minimum and maximums for manhattan distance
        #MAKE A CIRCLE
        #we first find minimum
        minL1 = math.ceil(self.ultrasonic_distance / BOXMEAS)
        maxL1 = (math.ceil(self.ultrasonic_distance / math.sqrt(2 * math.pow(BOXMEAS,2))) *2) +1
        cells = []

        for ix in range(5):
            for iy in range(6):
                if ( (abs(x-ix) + abs(y-iy)) in range(minL1, maxL1 +1)   ):
                    cells.append(Cell(ix,iy))

        #MAKE A CONE
        #trim selection by using servo and mpu angle 

        #HAVE WE ACTUALLY DETECTED THE OPPONENT?? IF NOT, TURN TO FIND THE OPPONENT.
        #let's check if the entry_angle and exit_angle s are float('inf') or not
        #how entry_angle and exit_angle are returned:
        #no detection: float('inf') (this is correct)
        # straight ahead / middle - 0 (this is correct)
        # otherwise returned in RADIANS (this is correct)
        # +1.57 is left 90 (this is correct)
        # -1.57 is right 90 (this is correct)

        
        lesserServo = min(self.entry_angle, self.exit_angle)
        biggerServo = max(self.entry_angle, self.exit_angle)

        #first we will check if the mark is right in front of us
        if(lesserServo <= 0.0) and (biggerServo >= 0.0):
            #cells is a circle  of r=1.18
            if self.facing == 0: #north and south
                cells = [c for c in cells if ((c.x == x) and (c.y >= y))]
            elif self.facing == 2:
                cells = [c for c in cells if ((c.x == x) and (c.y <= y))]
            elif self.facing == 1: # east and west
                cells = [c for c in cells if ((c.y == y) and (c.x >= x))]
            elif self.facing == 3:
                cells = [c for c in cells if ((c.y == y) and (c.x <= x))]

        else:
            #it isnt directly in front of us.
            #is it on the left or on the right?
            servoCells = []
            ceilCells = []
            servoCells.append(Cell(x,y))

            #on the right side:
            if(biggerServo <= 0.0):
                #if facing North
                if(self.facing==0):
                    #take the maximum area
                    diffY = 6 - y
                    diffX = - math.ceil(diffY * math.tan(lesserServo))

                    iy = y+1
                    while(iy < 6):
                        if iy >= 6 - math.ceil(diffY / 2):
                            for ix in range(diffX):
                                if x + ix < 5:
                                    servoCells.append(Cell(x + ix, iy))
                        else:
                            for ix in range(math.ceil(diffX / 2)):
                                if x + ix < 5:
                                    servoCells.append(Cell(x + ix, iy))

                        iy+=1
                            
                    #now make the smaller triangle
                    diffX = - math.ceil(diffY * math.tan(biggerServo))

                    iy = y+1
                    while(iy < 6):
                        if iy >= 6 - math.ceil(diffY / 2):
                            for ix in range(diffX):
                                if x + ix < 5:
                                    ceilCells.append(Cell(x + ix, iy))
                        else:
                            for ix in range(math.ceil(diffX / 2)):
                                if x + ix < 5:
                                    ceilCells.append(Cell(x + ix, iy))

                        iy+=1


                #elif south RIGHT
                elif(self.facing==2):
                    #take the maximum area
                    diffX = x
                    diffY = -1 * math.ceil(diffX * math.tan(lesserServo))

                    iy = y -1
                    while(iy>-1):
                        if iy >= math.ceil(diffY / 2):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if x- ix > -1:
                                    servoCells.append(Cell(- ix + x, iy))
                        else:
                            for iy in range(diffX):
                                if x- ix > -1:
                                    servoCells.append(Cell(- ix + x, iy))

                        iy-=1
                            
                    #now make the smaller triangle
                    diffY = -1 * math.ceil(diffX * math.tan(biggerServo))

                    iy = y -1
                    while(iy>-1):
                        if iy >= math.ceil(diffY / 2):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if x- ix > -1:
                                    ceilCells.append(Cell(-ix + x, iy))
                        else:
                            for iy in range(diffX):
                                if x- ix > -1:
                                    ceilCells.append(Cell(-ix + x, iy))

                        iy-=1

                #east right
                elif self.facing==1:
                    #take the maximum area
                    diffX = 5 - x
                    diffY = -1 * math.ceil(diffX * math.tan(lesserServo))

                    ix = x+1
                    while(ix<5):
                        if ix < 5 -  math.ceil(diffX/2):
                            for iy in range(math.ceil(diffY/2)):
                                if y - iy > -1:
                                    servoCells.append(Cell(ix, y - iy))
                        else:
                            for ix in range(diffX):
                                if y - iy > -1:
                                    servoCells.append(Cell(ix, y - iy))

                        ix+=1
                            
                    #now make the smaller triangle
                    diffY = -1 * math.ceil(diffX * math.tan(biggerServo))

                    ix = x+1
                    while(ix<5):
                        if ix < 5 -  math.ceil(diffX/2):
                            for iy in range(math.ceil(diffY/2)):
                                if y - iy > -1:
                                    ceilCells.append(Cell(ix, y - iy))
                        else:
                            for ix in range(diffX):
                                if y - iy > -1:
                                    ceilCells.append(Cell(ix, y - iy))

                        ix+=1

                #west RIGHT
                elif self.facing==3:
                    diffX = x
                    diffY = -1 * math.ceil(diffX * math.tan(lesserServo))

                    ix = x - 1
                    while ix > -1:
                        if ix < math.ceil(diffX/2):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(diffY):
                                if y + iy < 6:
                                    servoCells.append(Cell(ix, y + iy))
                        else:
                            for iy in range(math.ceil(diffY/2)):
                                if y + iy < 6:
                                    servoCells.append(Cell(ix, y + iy))

                        ix-=1
                            
                    #now make the smaller triangle
                    diffY = -1 * math.ceil(diffX * math.tan(biggerServo))

                    ix = x - 1
                    while ix > -1:
                        if ix < math.ceil(diffX/2):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(diffY):
                                if y + iy < 6:
                                    ceilCells.append(Cell(ix, y + iy))
                        else:
                            for iy in range(math.ceil(diffY/2)):
                                if y + iy < 6:
                                    ceilCells.append(Cell(ix, y + iy))

                        ix-=1

                    

            else:
                #on the LHS
                #if facing North
                if self.facing==0:
                    #take the maximum area
                    diffY = 6 - y
                    diffX = math.ceil(diffY * math.tan(lesserServo))

                    iy = y+1
                    while(iy < 6):
                        if iy >= 6 - math.ceil(diffY / 2):
                            for ix in range(diffX):
                                if x - ix > -1:
                                    servoCells.append(Cell(x - ix, iy))
                        else:
                            for ix in range(math.ceil(diffX / 2)):
                                if x - ix > -1:
                                    servoCells.append(Cell(x - ix, iy))

                        iy+=1
                            
                    #now make the smaller triangle
                    diffX = math.ceil(diffY * math.tan(biggerServo))

                    iy = y+1
                    while(iy < 6):
                        if iy >= 6 - math.ceil(diffY / 2):
                            for ix in range(diffX):
                                if x - ix > -1:
                                    ceilCells.append(Cell(x - ix, iy))
                        else:
                            for ix in range(math.ceil(diffX / 2)):
                                if x - ix > -1:
                                    ceilCells.append(Cell(x - ix, iy))

                        iy+=1

                #if facing south LEFT
                elif(self.facing==2):
                    #take the maximum area
                    diffX = x
                    diffY = math.ceil(diffX * math.tan(biggerServo))

                    iy = y -1
                    while(iy>-1):
                        if iy >= math.ceil(diffY / 2):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if x+ ix < 5:
                                    servoCells.append(Cell(ix + x, iy))
                        else:
                            for iy in range(diffX):
                                if x+ ix < 5:
                                    servoCells.append(Cell(ix + x, iy))

                        iy-=1
                            
                    #now make the smaller triangle
                    diffY = math.ceil(diffX * math.tan(lesserServo))

                    iy = y -1
                    while(iy>-1):
                        if iy >= math.ceil(diffY / 2):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if x+ ix < 5:
                                    ceilCells.append(Cell(ix + x, iy))
                        else:
                            for iy in range(diffX):
                                if x+ ix < 5:
                                    ceilCells.append(Cell(ix + x, iy))

                        iy-=1

                #EAST LEFT
                elif(self.facing==1):
                    #take the maximum area
                    diffX = 5 - x
                    diffY = math.ceil(diffX * math.tan(biggerServo))

                    ix = x+1
                    while(ix<5):
                        if ix < 5 -  math.ceil(diffX/2):
                            for iy in range(math.ceil(diffY/2)):
                                if y + iy < 6:
                                    servoCells.append(Cell(ix, y + iy))
                        else:
                            for ix in range(diffX):
                                if y + iy < 6:
                                    servoCells.append(Cell(ix, y + iy))

                        ix+=1
                            
                    #now make the smaller triangle
                    diffY = math.ceil(diffX * math.tan(lesserServo))

                    ix = x+1
                    while(ix<5):
                        if ix < 5 -  math.ceil(diffX/2):
                            for iy in range(math.ceil(diffY/2)):
                                if y + iy < 6:
                                    ceilCells.append(Cell(ix, y + iy))
                        else:
                            for ix in range(diffX):
                                if y + iy < 6:
                                    ceilCells.append(Cell(ix, y + iy))

                        ix+=1

                elif(self.facing==3):
                    #west left
                    diffX = x
                    diffY = math.ceil(diffX * math.tan(biggerServo))

                    ix = x - 1
                    while ix > -1:
                        if ix < math.ceil(diffX/2):
                            for iy in range(diffY):
                                if y - iy > -1:
                                    servoCells.append(Cell(ix, y - iy))
                        else:
                            for iy in range(math.ceil(diffY/2)):
                                if y - iy > -1:
                                    servoCells.append(Cell(ix, y - iy))

                        ix-=1
                            
                    #now make the smaller triangle
                    diffY = math.ceil(diffX * math.tan(lesserServo))

                    ix = x - 1
                    while ix > -1:
                        if ix < math.ceil(diffX/2):
                            for iy in range(diffY):
                                if y - iy > -1:
                                    ceilCells.append(Cell(ix, y - iy))
                        else:
                            for iy in range(math.ceil(diffY/2)):
                                if y - iy > -1:
                                    ceilCells.append(Cell(ix, y - iy))

                        ix-=1

                    

                
            
            #try to get the smallest cone while finding the INTERSECTION with radius cells (circle)
            ceCel = [c for c in cells if ((c in servoCells) and (c not in ceilCells))] 
            cells.clear()
            cells.extend(ceCel)

        #now we have a pretty small set of cells which the opponent can be in
        #reset probabilities
        self.P = {
            "Pab1": 0.0,
            "Pab2": 0.0,
            "Paf1": 0.0,
            "Pae1": 0.0,
            "Pef1": 0.0,
            "Peg1": 0.0,
            "Peg2": 0.0,
            "Pgh1": 0.0,
            "Phh1": 0.0,
            "Pfg1": 0.0,
            "Pdf1": 0.0,
            "Pbd1": 0.0,
            "Pbc1": 0.0,
            "Pcd1": 0.0,
            "Pcd2": 0.0,
            "Pch1": 0.0
        }

        
        for c in cells:
            if c.x == 0:
                if c.y == 0:
                    self.P["Peg2"] += 0.2106
                elif c.y == 1:
                    self.P["Peg2"] += 0.5466
                    self.P["Peg1"] += 0.0108
                elif c.y == 2:
                    self.P["Pae1"] += 0.2005
                    self.P["Peg2"] += 0.3825
                    self.P["Peg1"] += 0.0031
                elif c.y == 3:
                    self.P["Pae1"] += 0.6361
                    self.P["Paf1"] += 0.0053
                elif c.y == 4:
                    self.P["Pae1"] += 0.5522
                    self.P["Pab2"] += 0.0433
                    self.P["Paf1"] += 0.0099
                elif c.y == 5:
                    self.P["Pab2"] += 0.0293
            elif c.x == 1:
                if c.y == 0:
                    self.P["Peg2"] += 0.5323
                elif c.y == 1:
                    self.P["Peg2"] +=0.0268
                    self.P["Peg1"] +=0.6065
                elif c.y == 2:
                    self.P["Paf1"]+=0.0249
                    self.P["Peg1"]+= 0.0917
                    self.P["Peg2"]+= 0.0322
                    self.P["Pef1"]+= 0.3172
                    self.P["Pae1"]+= 0.2603
                elif c.y == 3:
                    self.P["Pae1"] += 0.0841
                    self.P["Paf1"] += 0.5752
                elif c.y == 4:
                    self.P["Pab1"]+= 0.3221
                    self.P["Pab2"]+= 0.1905
                    self.P["Paf1"]+= 0.1632
                    self.P["Pae1"]+= 0.0297
                elif c.y == 5:
                    self.P["Pab2"] += 0.5855
            elif c.x == 2:
                if c.y == 0:
                    self.P["Peg2"] +=0.1621
                elif c.y == 1:
                    self.P["Pgh1"]+= 0.3086
                    self.P["Pfg1"]+= 0.2170
                    self.P["Peg1"]+= 0.0647
                    self.P["Peg2"]+= 0.1755
                elif c.y == 2:
                    self.P["Pef1"]+= 0.0791
                    self.P["Pfg1"]+= 0.0963
                    self.P["Paf1"]+= 0.2668
                    self.P["Pdf1"]+= 0.3226
                elif c.y == 3:
                    self.P["Pbd1"] += 0.3311
                    self.P["Paf1"] += 0.1247
                    self.P["Pdf1"] += 0.0016
                elif c.y == 4:
                    self.P["Pab1"]+= 0.0753
                    self.P["Pab2"]+= 0.1886
                    self.P["Pbc1"]+= 0.2815
                    self.P["Pbd1"]+= 0.1985
                elif c.y == 5:
                    self.P["Pab2"] += 0.2288
            elif c.x == 3:
                if c.y == 0:
                    self.P["Phh1"] += 0.3463
                elif c.y == 1:
                    self.P["Pgh1"]+= 0.1191
                    self.P["Pch1"]+= 0.2218
                    self.P["Phh1"]+= 0.4263
                elif c.y == 2:
                    self.P["Pdf1"] += 0.2206
                    self.P["Pch1"] += 0.4203
                elif c.y == 3:
                    self.P["Pdf1"]+= 0.3213
                    self.P["Pcd1"]+= 0.0416
                    self.P["Pcd2"]+= 0.2097
                    self.P["Pbd1"]+= 0.1095
                elif c.y == 4:
                    self.P["Pbd1"] += 0.0039
                    self.P["Pcd1"]+= 0.1546
                    self.P["Pcd2"]+= 0.3530
                    self.P["Pbc1"]+= 0.1164
                    self.P["Pch1"]+= 0.1825
                elif c.y == 5:
                    self.P["Pch1"] += 0.4471
            elif c.x == 4:
                if c.y == 0:
                    self.P["Phh1"] += 0.2595
                elif c.y == 1:
                    self.P["Phh1"] += 0.3974
                elif c.y == 2:
                    self.P["Pch1"] += 0.5882
                elif c.y == 3:
                    self.P["Pch1"] += 0.4595
                    self.P["Pcd2"] += 0.1255
                elif c.y == 4:
                    self.P["Pch1"] +=0.4561
                    self.P["Pcd2"] += 0.2891
                elif c.y == 5:
                    self.P["Pch1"] += 0.5771
        


        #BAYES FILTER
        #let's hop on over to Po dict for a moment
        #Po is a dictionary which is storing our historic info
        #we have a temp dict called Px:
        Px = {}
        #we will apply some blurring effect and forgetfulness to Po
        
        for k,v in self.Po.items():
            #reduce the weight of the old info by 1-PERSISTENCE
            Px[k] = PERSISTENCE * v

            #allow for contamination from neighbouring edges at a rate of CONTAMINATION
            #where PERSISTENCE + CONTAMINATION = 1
            neighbours = self.getNeighbourEdgesOf(k)

            #Neighbouring Edge Key -> nek
            for nek in neighbours:
                nekNLen = len(self.getNeighbourEdgesOf(nek))

                Px[k] += CONTAMINATION * ( self.Po[nek] / nekNLen)

        #now we replace Po with Px
        self.Po = Px.copy()

        totalProb = 0.0
        #ok! Po has been blurred!
        #now let us update our P values (Bayesian inference)
        for k in self.P:
            self.P[k] *= self.Po[k]
            self.P[k] = max(self.P[k], 0.001) #clamp to prevent collapse
            totalProb += self.P[k]

        #now we normalise
        for k in self.P:
            self.P[k] /= totalProb

        #self.P gets cleared every time this function runs anyway
        #so all we have to do is save self.Po as self.P for our next run.
        self.Po = self.P.copy()

        

    #dijkstra function written by ChatGPT-5.3 on 9/04/2026
    #in this conversation: https://chatgpt.com/share/69d7f982-5b6c-8330-bd3b-779f35e3c7ed 
    #modified in using Claude Sonnet 4.3 on 13/05/2026
    #in this conversation: https://claude.ai/share/c6df9c61-823e-40fc-8203-13f9eb785a6f 
    #this is a scattered conversation so the message pertaining to this particular fix may be hard to find.
    def dijkstra(self, start_node, goal_char):
        pq = []
        heapq.heappush(pq, (0, start_node))

        distances = {start_node.name: 0}

        # Now stores (previous_node_name, direction_id_taken_to_get_here)
        previous = {start_node.name: (None, None)}

        visited = set()

        while pq:
            current_cost, current_node = heapq.heappop(pq)

            if current_node.name in visited:
                continue
            visited.add(current_node.name)

            if current_node.name == goal_char:
                break

            neighbors = [
                (current_node.Nc, current_node.Times[0], 0),
                (current_node.Ec, current_node.Times[1], 1),
                (current_node.Sc, current_node.Times[2], 2),
                (current_node.Wc, current_node.Times[3], 3),
            ]

            for neighbor_char, cost, direction_id in neighbors:
                if neighbor_char is None:
                    continue

                neighbor_node = self.returnNode(neighbor_char)
                if neighbor_node is None:
                    continue

                new_cost = current_cost + cost

                if (neighbor_char not in distances or
                        new_cost < distances[neighbor_char]):

                    distances[neighbor_char] = new_cost
                    previous[neighbor_char] = (current_node.name, direction_id)
                    heapq.heappush(pq, (new_cost, neighbor_node))

        # Reconstruct path as a list of direction IDs
        if goal_char not in previous and goal_char != start_node.name:
            return None, float('inf')

        if goal_char == start_node.name:
            return [], 0

        directions = []
        current = goal_char

        while previous[current][0] is not None:
            parent, direction_id = previous[current]
            directions.append(direction_id)
            current = parent

        directions.reverse()
        return directions, distances.get(goal_char, float('inf'))

    def generatePathFromNToN(self, n):
        #generate the shortest path from current_node to n
        path, dist = self.dijkstra(self.current_node, n)
        return path

    def generatePathFromNToE(self, e):
        #self.current_node to edge maxK

        #generate the shortest path from N to one of the nodes of the edge.
        parentsList = self.getNodesFromEdge(e)

        pathDistanceTuples = []
        for p in parentsList:
            path, dist = self.dijkstra(self.current_node, p)
            pathDistanceTuples.append((path,dist))

        min = 10000
        minPath = []
        for p,d in pathDistanceTuples:
            if d < min:
                min = d
                minPath = p

        return minPath

    
    def generateShortestPathFromNToListOption(self, listOfSingleParents):
        #generate the shortest path from N to one of the nodes of the edge.
        #listOfSingleParents is a list of char node names

        pathDistanceTuples = []
        for p in listOfSingleParents:
            path, dist = self.dijkstra(self.current_node, p)
            pathDistanceTuples.append((path,dist))

        min = 10000
        minPath = []
        for p,d in pathDistanceTuples:
            if d < min:
                min = d
                minPath = p

        return minPath

    
    def generateSafePathFromEnemyEdge(self, enemyE):
        #safe means not reachable by 1 node
        #so a safe node is 2+ nodes away
        #and there needs to be a valid path between me and the safe node 
        #that i can pass through so that i dont navigate through the enemy
        #so i cannot pass through the enemy's edge
        #and i need to avoid going to neighbouring edges of the enemy's edge

        unsafeNodes = []
        #list of chars

        p = self.getNodesFromEdge(enemyE)
        #p is a list of chars: 'A', or 'B', etc - 'H'
        for n in p:
            if n not in unsafeNodes:
                unsafeNodes.append(n)
            thisNode = self.returnNode(n)
            #check all of its neighbour nodes and add them to the list
            if thisNode.Nc not in unsafeNodes:
                unsafeNodes.append(thisNode.Nc)
            if thisNode.Ec not in unsafeNodes:
                unsafeNodes.append(thisNode.Ec)
            if thisNode.Sc not in unsafeNodes:
                unsafeNodes.append(thisNode.Sc)
            if thisNode.Wc not in unsafeNodes:
                unsafeNodes.append(thisNode.Wc)

        allNodes = ['A', 'B', 'C', 'D', 'E','F', 'G', 'H']
        #safeNodes = allNodes - unsafeNodes
        safeNodes = [s for s in allNodes if s not in unsafeNodes]

        self.get_logger().info(f"unsafeNodes: {unsafeNodes}")
        self.get_logger().info(f"safeNodes: {safeNodes}")

        #are we safe? then just wait here until the situation changes.
        if self.current_node.name in safeNodes:
            return []
            # a wait before next check on the receiving side of this function
            #to check for an empty path
        
        #otherwise, we know for a fact that we are not in a safe node

        #NOW WE NEED TO FIND THE NODES WHICH ARE THE SAFEST TO GET TO
        
        
        if self.current_node.Nc in safeNodes:
            return [0]
        elif self.current_node.Ec in safeNodes:
            return [1]
        elif self.current_node.Sc in safeNodes:
            return [2]
        elif self.current_node.Wc in safeNodes:
            return [3]
        
        #at this point, if you have a safeNode neighbour, you've left for it.

        #otherwise, you're in a tight spot, but for sure there is 1 path at least which is not touching enemy parent nodes.
        #so lets find that 1 / one of those paths

        #as long as we are not going to an enemy parent node, we are probably going in a good direction.
        #remember p are the parent nodes of the enemy
        #we should make the above random from any of the choices - adding some stochasticity
        #instead of making this true random, you could also, like, take from the prob distribution we have
        #and see which one you should take
        #but i think in this scenario they should be an equal distribution kind of, or so similar that in reality it wouldnt really make much of a diff
        #well i guess unless you're going into a cornered node.
        #then i guess smarter evasion would be taking from the prob distribution... hm... (implemen/t)
        #they probably have an equal distribution, not doing that.

        options = []
        if self.current_node.Nc not in p:
            options.append(0)
        if self.current_node.Ec not in p:
            options.append(1)
        if self.current_node.Sc not in p:
            options.append(2)
        if self.current_node.Wc not in p:
            options.append(3)

        if options:
            #choose random option and return that
            ranNum = random.randint(0,len(options) -1)
            return [options[ranNum]]
            
        self.get_logger().info(f"ERR: GenerateSafePathFromEnemyEdge has failed. No path generated. Are all paths blocked?")
        return []
    
    def generateSafePathFromListOfEnemyNodes(self, enemyNCHARList):
        
        #safe means not reachable by 1 node
        #so a safe node is 2+ nodes away
        #and there needs to be a valid path between me and the safe node 
        #that i can pass through so that i dont navigate through the enemy
        #so i cannot pass through the enemy's edge
        #and i need to avoid going to neighbouring edges of the enemy's edge

        unsafeNodes = []
        #list of chars

        for enemyNCHAR in enemyNCHARList:
            thisNode = self.returnNode(enemyNCHAR)
            #check all of its neighbour nodes and add them to the list
            if enemyNCHAR not in unsafeNodes:
                unsafeNodes.append(enemyNCHAR)
            if thisNode.Nc not in unsafeNodes:
                unsafeNodes.append(thisNode.Nc)
            if thisNode.Ec not in unsafeNodes:
                unsafeNodes.append(thisNode.Ec)
            if thisNode.Sc not in unsafeNodes:
                unsafeNodes.append(thisNode.Sc)
            if thisNode.Wc not in unsafeNodes:
                unsafeNodes.append(thisNode.Wc)

        allNodes = ['A', 'B', 'C', 'D', 'E','F', 'G', 'H']
        #safeNodes = allNodes - unsafeNodes
        safeNodes = [s for s in allNodes if s not in unsafeNodes]

        self.get_logger().info(f"unsafeNodes: {unsafeNodes}")
        self.get_logger().info(f"safeNodes: {safeNodes}")
        #are we safe? then just wait here until the situation changes.
        if self.current_node.name in safeNodes:
            return []
            #a wait before next check on the receiving side of this function
            #to check for an empty path
        
        #otherwise, we know for a fact that we are not in a safe node

        #NOW WE NEED TO FIND THE NODES WHICH ARE THE SAFEST TO GET TO
        
        options = []
        if self.current_node.Nc in safeNodes:
            options.append(0)
        elif self.current_node.Ec in safeNodes:
            options.append(1)
        elif self.current_node.Sc in safeNodes:
            options.append(2)
        elif self.current_node.Wc in safeNodes:
            options.append(3)
        

        if options:
            #choose random option and return that
            ranNum = random.randint(0,len(options) -1)
            return [options[ranNum]]
        
        #otherwise, options is still empty
        
        #at this point, if you have a safeNode neighbour, you've left for it.

        #otherwise, you're in a tight spot, but for sure there is 1 path at least which is not touching enemy parent nodes.
        #so lets find that 1 / one of those paths

        #as long as we are not going to an enemy node (WHICH WE THINK CONTAINS THE ENEMY DIRECTLY), we might have a chance to survive.
        if self.current_node.Nc not in enemyNCHARList:
            options.append(0)
        elif self.current_node.Ec not in enemyNCHARList:
            options.append(1)
        elif self.current_node.Sc not in enemyNCHARList:
            options.append(2)
        elif self.current_node.Wc not in enemyNCHARList:
            options.append(3)
            
        if options:
            #choose random option and return that
            ranNum = random.randint(0,len(options) -1)
            return [options[ranNum]]

        #just return an empty list and say stay there and accept your doom, cus it prob means that we did not find a safe path out
        self.get_logger().info(f"ERR / FAILURE : GenerateSafePathFromListOfEnemyNodes has failed. No path generated. Are all paths blocked?")
        return []
    
        
    def generateSafePathFromEnemyNode(self, enemyNCHAR):
        
        #safe means not reachable by 1 node
        #so a safe node is 2+ nodes away
        #and there needs to be a valid path between me and the safe node 
        #that i can pass through so that i dont navigate through the enemy
        #so i cannot pass through the enemy's edge
        #and i need to avoid going to neighbouring edges of the enemy's edge

        unsafeNodes = []
        #list of chars

        
        thisNode = self.returnNode(enemyNCHAR)
        #check all of its neighbour nodes and add them to the list
        unsafeNodes.append(enemyNCHAR)
        if thisNode.Nc not in unsafeNodes:
            unsafeNodes.append(thisNode.Nc)
        if thisNode.Ec not in unsafeNodes:
            unsafeNodes.append(thisNode.Ec)
        if thisNode.Sc not in unsafeNodes:
            unsafeNodes.append(thisNode.Sc)
        if thisNode.Wc not in unsafeNodes:
            unsafeNodes.append(thisNode.Wc)

        self.get_logger().info(f"unsafe nodes: {unsafeNodes}")
        allNodes = ['A', 'B', 'C', 'D', 'E','F', 'G', 'H']
        #safeNodes = allNodes - unsafeNodes
        safeNodes = [s for s in allNodes if s not in unsafeNodes]
        self.get_logger().info(f"safe nodes: {safeNodes}")
        #are we safe? then just wait here until the situation changes.
        if self.current_node.name in safeNodes:
            self.get_logger().info("current node in safe nodes - returning []")
            return []
            #a wait before next check on the receiving side of this function
            #to check for an empty path
        
        #otherwise, we know for a fact that we are not in a safe node

        #NOW WE NEED TO FIND THE NODES WHICH ARE THE SAFEST TO GET TO
        
        
        if self.current_node.Nc in safeNodes:
            self.get_logger().info(f"going north because it's safe")
            return [0]
        elif self.current_node.Ec in safeNodes:
            self.get_logger().info(f"going east because it's safe")
            return [1]
        elif self.current_node.Sc in safeNodes:
            self.get_logger().info(f"going south because it's safe")
            return [2]
        elif self.current_node.Wc in safeNodes:
            self.get_logger().info(f"going west because it's safe")
            return [3]
        
        #at this point, if you have a safeNode neighbour, you've left for it.

        #otherwise, you're in a tight spot, but for sure there is 1 path at least which is not touching enemy parent nodes.
        #so lets find that 1 / one of those paths

        #as long as we are not going to THE enemy node, we might have a chance to survive.
        if self.current_node.Nc != enemyNCHAR:
            self.get_logger().info(f"going north because it's not the worst option imaginable")
            return [0]
        elif self.current_node.Ec != enemyNCHAR:
            self.get_logger().info(f"going east because it's not the worst option imaginable")
            
            return [1]
        elif self.current_node.Sc != enemyNCHAR:
            self.get_logger().info(f"going south because it's not the worst option imaginable")
            return [2]
        elif self.current_node.Wc != enemyNCHAR:
            self.get_logger().info(f"going west because it's not the worst option imaginable")
            return [3]
            

        #A SAFE EDGE IS ANY EDGE TOUCHING A SAFE NODE, EVEN IF ITS COMING FROM AN UNSAFE NODE.
        #if [] is returned, everything has failed somehow, must be an error
        
        self.get_logger().info(f"ERR: GenerateSafePathFromEnemyNode has failed. No path generated. Are all paths blocked?")
        return [] 


    def planDestination(self):
        self.get_logger().info(f"entered planDestination function with goAhead: {self.goAhead} and stepping: {self.stepping}")
        CERTAINTY = 0.6 #threshold for us to definitely assume taht the opponent is at a particular location
        CONSIDER_NODES = 3 #if CERTAINTY threshold is not met, how many of the top probability nodes should we consider?
        choice = -1
        
        #define some preference algorithm.
        #use distances if you want (Nd, Ed, Sd, Wd)
        # behaviourMode settings:
        if self.behaviourMode == 1:
            # 1 - Simple Line Follower (using pink path)

            #self.patrolPath = ['A', 'F', 'G', 'E', 'F', 'D', 'C', 'H', 'H', 'G', 'E', 'A', 'B', 'C', 'D', 'B']
            self.destination_id = -1
            self.i_patrol+=1
            if self.i_patrol >= 16:
                self.i_patrol = 0
            self.current_destination = self.patrolPath[self.i_patrol]

            self.stateFollow = True
            
        elif self.behaviourMode == 2:
            # 2 - Random

            #randomly choose a direction (0-3)
            choice = random.randint(0,3)
            if choice == 0:
                #north
                self.current_destination = self.current_node.Nc
            elif choice == 1:
                #east
                self.current_destination = self.current_node.Ec
            elif choice == 2:
                #south
                self.current_destination = self.current_node.Sc
            elif choice == 3:
                #west
                self.current_destination = self.current_node.Wc
            else:
                self.get_logger().info(f"Bad random number generated, no such choice value as {choice}")
                
            self.get_logger().info(f"Generated direction {choice}")
            self.imu_target = choice
            self.destination_id = choice

        elif self.behaviourMode == 3 or (self.behaviourMode == 5 and self.opp_old_loc==-1):
            # 3 - Greedy
            self.calculateProbabilities()

            #take max likely edge
            maxV = -1
            maxK = ""
            for k,v in self.P.items():
                if v > maxV:
                    maxV = v
                    maxK = k

            

            if(maxV >= CERTAINTY):
                self.get_logger().info(f"Target location: {maxK}")
                #then we want to generate a path from our current node to that edge (maxK)
                self.current_destination = self.generatePathFromNToE(maxK)
            else:
                #find top CONSIDER_NODES (int) max valued edge-probabilities
                topProb = sorted(self.P.items(), key=lambda x: x[1], reverse=True)[:CONSIDER_NODES]
                #returns smth like [('Pbd1', 0.56), ('Pbc1', 0.33), ('Pcd2', 0.10)]

                
                parentsDict = {
                    'A' : 0,
                    'B' : 0,
                    'C' : 0,
                    'D' : 0,
                    'E' : 0,
                    'F' : 0,
                    'G' : 0,
                    'H' : 0
                }

                for k,v in topProb:
                    parentsList = self.getNodesFromEdge(k)

                    for p in parentsList:
                        #p is a char: 'A', or 'B', etc - 'H'
                        parentsDict[p] +=1

                #dont allow yourself to pathfind to where you're currently standing
                parentsDict[self.current_node.name] = 0

                #now we have the most common parent
                #find central node
                nfound = False
                
                #go backwards from CONSIDER_NODES-1 (don't count yourself) to 2
                for cert in range(CONSIDER_NODES-1, 1, -1): 
                    #until you find the max valued parent node
                    #if you found, do nfound yes

                    for toK,v in parentsDict.items():
                    #is there a node?
                        if v == cert:
                            #yes -> generatepathfromNtoN
                            self.get_logger().info(f"Target Location: {toK}")
                            self.current_destination = self.generatePathFromNToN(toK)
                            nfound=True
                            break #stop iterating
                
                #otherwise:
                #no -> try the closest one (if greedy search, then we try closest one and try again)
                #                          (if avoidant search, then we assume the worst-case (closest) but avoid the top few)
                if not nfound:
                    #so parentsDict only continas values of 0 or 1, so
                    #just traverse parentsDict for the non-0, ==1 parents
                    singleParents = [k for k,v in parentsDict.items() if v==1]
                    #singleParents contains a list of chars e.g. 'A' - 'H'

                    #and then just find the closest next node and generate path towards it
                    self.current_destination = self.generateShortestPathFromNToListOption(singleParents)
                    self.get_logger().info(f"Target Location In: {singleParents}")

        elif self.behaviourMode == 4:
            #FOR DEBUGGING:
            # self.get_logger().info(f"DEBUG: SETTING CUR_DEST TO WEST, CUR_LOC IS {self.current_node.name}")
            # self.current_destination = self.current_node.Wc
            # self.destination_id = 3
            # return
            # 4 - Avoidant
            self.destination_id = -1 # all generated paths are int lists
            self.calculateProbabilities()

            #take max likely edge
            maxV = -1
            maxK = ""
            for k,v in self.P.items():
                if v > maxV:
                    maxV = v
                    maxK = k

            

            if(maxV >= CERTAINTY):
                #then we want to generate a path from our current node to that edge (maxK)
                self.current_destination = self.generateSafePathFromEnemyEdge(maxK)
                self.get_logger().info(f"Target Location: {maxK}")
            else:
                #find top CONSIDER_NODES (int) max valued edge-probabilities
                topProb = sorted(self.P.items(), key=lambda x: x[1], reverse=True)[:CONSIDER_NODES]
                #returns smth like [('Pbd1', 0.56), ('Pbc1', 0.33), ('Pcd2', 0.10)]

                
                parentsDict = {
                    'A' : 0,
                    'B' : 0,
                    'C' : 0,
                    'D' : 0,
                    'E' : 0,
                    'F' : 0,
                    'G' : 0,
                    'H' : 0
                }

                for k,v in topProb:
                    parentsList = self.getNodesFromEdge(k)

                    for p in parentsList:
                        #p is a char: 'A', or 'B', etc - 'H'
                        parentsDict[p] +=1

                #dont allow yourself to pathfind to where you're currently standing
                parentsDict[self.current_node.name] = 0

                #now we have the most common parent
                #find central node
                nfound = False
                
                #go backwards from CONSIDER_NODES to 2
                for cert in range(CONSIDER_NODES-1, 1, -1):
                    #until you find the max valued parent node
                    #if you found, do nfound yes

                    for toK,v in parentsDict.items():
                    #is there a node?
                        if v == cert:
                            #yes -> generatepathfromNtoN
                            self.get_logger().info(f"Target Location: {toK}")
                            self.current_destination = self.generateSafePathFromEnemyNode(toK)
                            nfound=True
                            break #stop iterating
                    
                    if nfound:
                        break
                
                #otherwise:
                #no -> try the closest one (if greedy search, then we try closest one and try again)
                #                          (if avoidant search, then we assume the worst-case (closest) but avoid the top few)
                if not nfound:
                    #so parentsDict only continas values of 0 or 1, so
                    #just traverse parentsDict for the non-0, ==1 parents
                    singleParents = [k for k,v in parentsDict.items() if v==1]
                    #singleParents contains a list of chars e.g. 'A' - 'H'

                    #and then just find the closest next node and generate path towards it
                    self.current_destination = self.generateSafePathFromListOfEnemyNodes(singleParents)
                    self.get_logger().info(f"Target Location in: {singleParents}")
            
        elif self.behaviourMode == 5:
            # 5 - Interceptive
            be_greedy = False
            self.calculateProbabilities()
            

            #interceptive takes the straight path
            #taking targets past loc, Curr loc, and assumes straight to future loc

            #1. assuming both last and current location are nodes
            #find edge connecting these nodes if one exists.
            #if one does not exist, then assume the direction that faces a 
            
            #take max likely edge
            maxV = -1
            maxK = ""
            for k,v in self.P.items():
                if v > maxV:
                    maxV = v
                    maxK = k

            

            if(maxV >= CERTAINTY):
                
                #opponent is at an edge
                #so C=E
                #check what O is
                #maxK is the edge we are using to represent the opponent's current edge

                if type(self.opp_old_loc) == str:
                    #then it is an edge
                    par_o = self.getNodesFromEdge(self.opp_old_loc)
                    par_c = self.getNodesFromEdge(maxK)

                    be_greedy = True

                    for no in par_o:
                        for n in par_c:
                            if no == n:
                                #we found a common parent
                                par_c.remove(n)
                                self.get_logger().info(f"Target Location: {par_c[0]}")
                                self.current_destination = self.generatePathFromNToE(par_c[0])
                                be_greedy = False
                                break

                    if(be_greedy):
                        #then we want to generate a path from our current node to that edge (max K)
                        self.current_destination = self.generatePathFromNToE(maxK)
                        self.get_logger().info(f"Target Location: {maxK}")

                    
                else:
                    #then it is a node
                    #O=N, C=E

                    #is C connected to O?
                    if maxK in self.getEdgesFromNode(self.opp_old_loc):
                        d = self.getNodesFromEdge(maxK)
                        d.remove(self.opp_old_loc)
                        self.current_destination = self.generatePathFromNtoN(d[0])
                        self.get_logger().info(f"Target Location: {d[0]}")
                        
                    else:
                        #greedy:
                        #then we want to generate a path from our current node to that edge (max K)
                        self.current_destination = self.generatePathFromNToE(maxK)
                        self.get_logger().info(f"Target Location: {maxK}")

            else:
                #find top CONSIDER_NODES (int) max valued edge-probabilities
                topProb = sorted(self.P.items(), key=lambda x: x[1], reverse=True)[:CONSIDER_NODES]
                #returns smth like [('Pbd1', 0.56), ('Pbc1', 0.33), ('Pcd2', 0.10)]

                
                parentsDict = {
                    'A' : 0,
                    'B' : 0,
                    'C' : 0,
                    'D' : 0,
                    'E' : 0,
                    'F' : 0,
                    'G' : 0,
                    'H' : 0
                }

                for k,v in topProb:
                    parentsList = self.getNodesFromEdge(k)

                    for p in parentsList:
                        #p is a char: 'A', or 'B', etc - 'H'
                        parentsDict[p] +=1

                #dont allow yourself to pathfind to where you're currently standing
                parentsDict[self.current_node.name] = 0

                #now we have the most common parent
                #find central node
                nfound = False
                for toK,v in parentsDict.items():
                    #is there a node?
                    if v == CONSIDER_NODES:
                        #yes -> generatepathfromNtoN
                        #try interceptive:
                        #is O =N? or =E?
                        if type(self.opp_old_loc) == str:
                            #O=E
                            po = self.getNodesFromEdge(self.opp_old_loc)
                            if toK in po:

                                #see which of the node's edges this is e.g. North
                                #and then flip it to get our new direction e.g. South
                                #then current_destination is the node.Sc node!

                                po.remove(toK)
                                #po[0] is other parent
                                #so we want all the edges which connect toK and po[0]
                                #but technically we will just take the first one in the list <PARAMETER> / <OPTIMISATION> / <ASSUMPTION>

                                pc = self.returnNode(toK)
                                if pc.Nc == po[0]:
                                    self.current_destination = self.generatePathFromNToN(pc.Sc)
                                elif pc.Ec == po[0]:
                                    self.current_destination = self.generatePathFromNToN(pc.Wc)
                                elif pc.Sc == po[0]:
                                    self.current_destination = self.generatePathFromNToN(pc.Nc)
                                elif pc.Wc == po[0]:
                                    self.current_destination = self.generatePathFromNToN(pc.Ec)

                                self.get_logger().info(f"Target Location: {toK}")
                                nfound = True
                                break
                            else:

                                #greedy
                                self.current_destination = self.generatePathFromNToN(toK)
                                self.get_logger().info(f"Target Location: {toK}")
                                break
                                
                        else:
                            #O=N, C=N
                            eo = self.getEdgesFromNode(self.opp_old_loc)
                            ec = self.getEdgesFromNode(toK)

                            be_greedy = True
                            for e in eo:
                                for ec1 in ec:
                                    if e == ec1 and not nfound:
                                        be_greedy = False
                                        #take the first edge we find <BIAS> / <OPTIMISATION>
                                        #and follow it to the next node

                                        self.get_logger().info(f"Target Location: {toK}")
                                        po = self.returnNode(self.opp_old_loc)
                                        if po.Nc == toK:
                                            self.current_destination = self.generatePathFromNToN(self.returnNode(toK).Nc)
                                        elif po.Ec == toK:
                                            self.current_destination = self.generatePathFromNToN(self.returnNode(toK).Ec)
                                        elif po.Sc == toK:
                                            self.current_destination = self.generatePathFromNToN(self.returnNode(toK).Sc)
                                        elif po.Wc == toK:
                                            self.current_destination = self.generatePathFromNToN(self.returnNode(toK).Wc)

                                        nfound=True
                                        break #stop iterating
                                        

                            if be_greedy:
                                #greedy
                                self.current_destination = self.generatePathFromNToN(toK)
                                self.get_logger().info(f"Target Location: {toK}")
                                nfound=True
                                break #stop iterating


                        #greedy
                        self.current_destination = self.generatePathFromNToN(toK)
                        self.get_logger().info(f"Target Location: {toK}")
                        nfound=True
                        break #stop iterating
                
                #if not nfound:
                if not nfound:
                    #go backwards from CONSIDER_NODES to 2
                    for cert in range(CONSIDER_NODES-1, 1, -1):
                        #until you find the max valued parent node
                        #if you found, do nfound yes

                        for toK,v in parentsDict.items():
                        #is there a node?
                            if v == cert:
                                #yes -> generatepathfromNtoN
                                #try interceptive:
                                #is O =N? or =E?
                                if type(self.opp_old_loc) == str:
                                    #O=E
                                    po = self.getNodesFromEdge(self.opp_old_loc)
                                    if toK in po:

                                        #see which of the node's edges this is e.g. North
                                        #and then flip it to get our new direction e.g. South
                                        #then current_destination is the node.Sc node!

                                        po.remove(toK)
                                        #po[0] is other parent
                                        #so we want all the edges which connect toK and po[0]
                                        #but technically we will just take the first one in the list <PARAMETER> / <OPTIMISATION> / <ASSUMPTION>
                                        self.get_logger().info(f"Target Location: {toK}")
                                        pc = self.returnNode(toK)
                                        if pc.Nc == po[0]:
                                            self.current_destination = self.generatePathFromNToN(pc.Sc)
                                        elif pc.Ec == po[0]:
                                            self.current_destination = self.generatePathFromNToN(pc.Wc)
                                        elif pc.Sc == po[0]:
                                            self.current_destination = self.generatePathFromNToN(pc.Nc)
                                        elif pc.Wc == po[0]:
                                            self.current_destination = self.generatePathFromNToN(pc.Ec)

                                        nfound = True
                                        break
                                    else:

                                        #greedy
                                        self.current_destination = self.generatePathFromNToN(toK)
                                        self.get_logger().info(f"Target Location: {toK}")
                                        break
                                        
                                else:
                                    #O=N, C=N
                                    eo = self.getEdgesFromNode(self.opp_old_loc)
                                    ec = self.getEdgesFromNode(toK)
                                    self.get_logger().info(f"Target Location: {toK}")

                                    be_greedy = True
                                    for e in eo:
                                        for ec1 in ec:
                                            if e == ec1 and not nfound:
                                                be_greedy = False
                                                #take the first edge we find <BIAS> / <OPTIMISATION>
                                                #and follow it to the next node


                                                po = self.returnNode(self.opp_old_loc)
                                                if po.Nc == toK:
                                                    self.current_destination = self.generatePathFromNToN(self.returnNode(toK).Nc)
                                                elif po.Ec == toK:
                                                    self.current_destination = self.generatePathFromNToN(self.returnNode(toK).Ec)
                                                elif po.Sc == toK:
                                                    self.current_destination = self.generatePathFromNToN(self.returnNode(toK).Sc)
                                                elif po.Wc == toK:
                                                    self.current_destination = self.generatePathFromNToN(self.returnNode(toK).Wc)

                                                nfound=True
                                                break #stop iterating
                                                

                                    if be_greedy:
                                        #greedy
                                        self.current_destination = self.generatePathFromNToN(toK)
                                        self.get_logger().info(f"Target Location: {toK}")
                                        nfound=True
                                        break #stop iterating


                                #greedy
                                self.current_destination = self.generatePathFromNToN(toK)
                                self.get_logger().info(f"Target Location: {toK}")
                                nfound=True
                                break #stop iterating
                
                #otherwise:
                #no -> try the closest one (if greedy search, then we try closest one and try again)
                #                          (if avoidant search, then we assume the worst-case (closest) but avoid the top few)
                if not nfound:
                    #so parentsDict only continas values of 0 or 1, so
                    #just traverse parentsDict for the non-0, ==1 parents
                    singleParents = [k for k,v in parentsDict.items() if v==1]
                    #singleParents contains a list of chars e.g. 'A' - 'H'

                    #and then just find the closest next node and generate path towards it
                    self.current_destination = self.generateShortestPathFromNToListOption(singleParents)
                    self.get_logger().info(f"Target Location in: {singleParents}")
        #else:
            # 0 - not set (no behaviour)
            
        
    
    def startTurnBasedOnIMU(self):
        #self.completeTurn = False
        self.get_logger().info(f"starting turn based on imu. imu_target: {self.imu_target}, facing: {self.facing}")

        if self.imu_target == -1 or self.imu_target == self.facing:
            #no turn
            self.imu_turning = False
            self.stopMov()

            if self.toDepart:
                self.toDepart = False #consume
                self.departureTime = self.now
                self.stateFollow = True

        else:
            self.imu_turning = True
            self.get_logger().info(f"Turning to face {self.imu_target}")

            if (self.facing == 0 and self.imu_target == 3) or (self.facing == 1 and self.imu_target == 0) or (self.facing == 2 and self.imu_target == 1) or (self.facing == 3 and self.imu_target == 2):
                self.turnLeft(0)
            else:
                self.turnRight(0)

            
    def updatePos(self):
        #update current variables
         #do some check to ensure we aren't being triggered by the last gray section we saw
        #blocked if we are turning while currently at an intersection.
        if (not self.toDepart and not self.imu_turning and self.grayEntryTime < self.now - self.GRAY_COOLDOWN) or (self.goAhead and not self.imu_turning):
            #if gray detected:
            # or self.haventMovedYet
            if (self.isGray[0] > self.minPixels) or (self.isGray[1] > self.minPixels)  or (self.isGray[2] > self.minPixels):
                self.get_logger().info(f"Intersection detected! Gray values: {self.isGray}; reminder minPixels is {self.minPixels}")
                self.stopMov()
                self.grayEntryTime = self.now
                self.stateFollow = False
                self.allowCrawl = False
                self.stationaryStartTime = -1  # no longer stationary — gray found

                # Stop crawling the moment gray comes back (checked on next tick via the
                # if-branch above, but we also guard here in case the scan arrives mid-crawl).
                # The motion_active flag means crawlBack is still running its short burst;
                # if gray is now visible, cancel it.
                if self.motion_active:
                    self.get_logger().info("Gray re-detected while crawling back — stopping.")

                
                if self.resetBehaviour:
                    self.firstNode = True
                    #self.haventMovedYet = True
                    self.get_logger().info(f"Resetting Behaviour Variables")
                    #lower flag
                    self.resetBehaviour = False

                    #reset localisation
                    self.last_node = 'Z'

                    # a tag took place, and the behaviour variables need to be reset
                    
                    if self.behaviourMode == 1:
                        #patrol:
                        #we need to correct the direction we are facing to resume flow and update i_patrol
                        #find the first instance of our current node in the patrol list
                        i = self.patrolPath.index(self.current_node.name)
                        nextDest = self.patrolPath[i+1]
                        self.i_patrol = i

                        if self.current_node.Nc == nextDest:
                            self.imu_target = 0
                        elif self.current_node.Ec == nextDest:
                            self.imu_target = 1
                        elif self.current_node.Sc == nextDest:
                            self.imu_target = 2
                        elif self.current_node.Wc == nextDest:
                            self.imu_target = 3
                        
                        #align ourselves properly
                        self.startTurnBasedOnIMU()
                        
                    elif self.behaviourMode == 5:
                        self.opp_old_loc = -1


                    
                #1. self-localise
                if type(self.current_destination) == list:
                    
                    #Update last node + current node
                    #self.current_node = self.current_destination
                    if(self.current_destination) and not self.goAhead:

                        #current node
                        if type(self.current_destination[0]) == int:

                            #localise using old values
                            self.self_localise(self.current_node.Times[self.current_destination[0]])
                            if self.dontUpdate:
                                self.dontUpdate = False #consume
                                return
                            
                            #last node
                            self.last_node = self.current_node

                            top = self.current_destination.pop()
                            if top == 0:
                                self.current_node = self.returnNode(self.current_node.Nc)
                            elif top == 1:
                                self.current_node = self.returnNode(self.current_node.Ec)
                            elif top == 2:
                                self.current_node = self.returnNode(self.current_node.Sc)
                            elif top == 3:
                                self.current_node = self.returnNode(self.current_node.Wc)

                            if not self.current_destination:
                                self.current_destination = 'Z'

                        elif type(self.current_destination[0]) == str:
                            #localise using old values

                            if self.current_node.Nc == self.current_destination[0] and self.destination_id in [-1, 0]:
                                self.self_localise(self.current_node.Times[0])
                            elif self.current_node.Ec == self.current_destination[0] and self.destination_id in [-1, 1]:
                                self.self_localise(self.current_node.Times[1])
                            elif self.current_node.Sc == self.current_destination[0] and self.destination_id in [-1, 2]:
                                self.self_localise(self.current_node.Times[2])
                            elif self.current_node.Wc == self.current_destination[0] and self.destination_id in [-1, 3]:
                                self.self_localise(self.current_node.Times[3])

                            if self.dontUpdate:
                                self.dontUpdate = False #consume
                                return
                            
                            #last node
                            self.last_node = self.current_node

                            self.current_node = self.returnNode(self.current_destination.pop())
                            if not self.current_destination:
                                self.current_destination = 'Z'
                    
                    elif not self.goAhead: #empty list
                    
                        #otherwise, do not update current_node as we are still where we were.
                        #populate our planned path if we don't already have a plan
                        #is it empty? => stay here and wait until our sensing cooldown has gone out.
                        self.dontSense = True
                        
                else : #not list

                    #localise using old values
                    if self.current_node.Nc == self.current_destination and self.destination_id in [-1, 0]:
                        self.self_localise(self.current_node.Times[0])
                    elif self.current_node.Ec == self.current_destination and self.destination_id in [-1, 1]:
                        self.self_localise(self.current_node.Times[1])
                    elif self.current_node.Sc == self.current_destination and self.destination_id in [-1, 2]:
                        self.self_localise(self.current_node.Times[2])
                    elif self.current_node.Wc == self.current_destination and self.destination_id in [-1, 3]:
                        self.self_localise(self.current_node.Times[3])

                    if self.dontUpdate:
                        self.dontUpdate = False #consume
                        return
                    #update current_node and last_node
                    if not self.firstNode:
                        self.last_node = self.current_node
                        self.current_node = self.returnNode(self.current_destination)   

                #COMMON

                if self.current_destination == [] and self.behaviourMode == 4:
                    #when sensing cooldown expires, look again.
                    if self.senseEntryTime < self.now - self.SENSE_COOLDOWN:
                        self.stateFollow = False
                        self.planDestination()
                    else:
                        #cooldown has not expired yet. stay stopped and wait
                        self.stateFollow = False
                        self.dontSense = True
                        return
                    
                    self.dontSense = False

                elif self.behaviourMode in [3,5] and not self.lookAround and not self.goAhead:
                    self.lookAround = True
                    return
                    #if we got this far, we have directions to go somewhere
                    #self.current_destination has been set to either 1 directional number OR a LIST of directional numbers. 
                    #Check which one, if it is a list, we must iterate over it as it is a long path.

                elif not self.stepping:
                    #Update destination
                    self.planDestination()
                    self.goAhead = False #consume

                self.get_logger().info("exited planDestination function")
                #COMMON
                if self.current_destination != []:
                    if self.firstNode:
                        self.firstNode = False

                    if type(self.current_destination) == list:

                        if type(self.current_destination[0]) == int:
                            #if it is a single number from 0 to 3, then it is an immediate neighbour 
                            #e.g. path = [2] i.e. go south
                            self.imu_target = self.current_destination[0]
                            tx = {0: self.current_node.Nc, 1: self.current_node.Ec, 2: self.current_node.Sc, 3: self.current_node.Wc}
                            t = tx[self.imu_target]

                            #update sweeping setting
                            if (self.current_node.name == 'A' and self.imu_target == 0) or (self.current_node.name == 'B' and self.imu_target == 0) or (self.current_node.name == 'C' and self.imu_target == 1) or (self.current_node.name == 'D' and self.imu_target == 3) or (self.current_node.name == 'E' and self.imu_target == 0) or (self.current_node.name == 'F' and self.imu_target == 0) or (self.current_node.name == 'F' and self.imu_target == 3) or (self.current_node.name == 'G' and self.imu_target == 1) or (self.current_node.name == 'G' and self.imu_target == 2) or (self.current_node.name == 'G' and self.imu_target == 3) :
                                #when left first is better than right first
                                self.skipZero = True
                            else:
                                self.skipZero = False

                            if self.stepping:
                                self.goAhead = False
                                self.stepping = False


                        elif type(self.current_destination[0]) == str:
                            #update sweeping setting
                            if (self.current_node.name == 'A' and self.current_destination[0] == self.A.Nc) or (self.current_node.name == 'B' and self.current_destination[0] == self.B.Nc) or (self.current_node.name == 'C' and self.current_destination[0] == self.C.Ec) or (self.current_node.name == 'D' and self.current_destination[0] == self.D.Wc) or (self.current_node.name == 'E' and self.current_destination[0] == self.E.Nc) or (self.current_node.name == 'F' and self.current_destination[0] == self.F.Nc) or (self.current_node.name == 'F' and self.current_destination[0] == self.F.Wc) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Ec) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Sc) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Wc) :
                                #when left first is better than right first
                                self.skipZero = True
                            else:
                                self.skipZero = False

                            #if it is a char from A to H, then it is an immediate neighbour 
                            #e.g. path = ['A', 'B'] 
                            neigh_name = self.current_destination[0]
                            if self.current_node.Nc == neigh_name and self.destination_id in [-1, 0]:
                                self.imu_target = 0
                            elif self.current_node.Ec == neigh_name and self.destination_id in [-1, 1]:
                                self.imu_target = 1
                            elif self.current_node.Sc == neigh_name and self.destination_id in [-1, 2]:
                                self.imu_target = 2
                            elif self.current_node.Wc == neigh_name and self.destination_id in [-1, 3]:
                                self.imu_target = 3
                            
                            t = neigh_name

                    else: #not a list, just a char

                        if self.behaviourMode != 2: #if not random
                            #then it is a char from A to H, and it is an immediate neighbour 
                            #e.g. path = 'A'
                            if self.current_node.Nc == self.current_destination and self.destination_id in [-1, 0]:
                                self.imu_target = 0
                            elif self.current_node.Ec == self.current_destination and self.destination_id in [-1, 1]:
                                self.imu_target = 1
                            elif self.current_node.Sc == self.current_destination and self.destination_id in [-1, 2]:
                                self.imu_target = 2
                            elif self.current_node.Wc == self.current_destination and self.destination_id in [-1, 3]:
                                self.imu_target = 3
                        #else:
                            #BEHAVIOURMODE == 2 (RANDOM)
                            #imu target was already set inside of planDestination
                            #therefore we just need to set t


                        t = self.current_destination

                        #update sweep setting
                        if (self.current_node.name == 'A' and self.current_destination == self.A.Nc) or (self.current_node.name == 'B' and self.current_destination == self.B.Nc) or (self.current_node.name == 'C' and self.current_destination == self.C.Ec) or (self.current_node.name == 'D' and self.current_destination == self.D.Wc) or (self.current_node.name == 'E' and self.current_destination == self.E.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Wc) or (self.current_node.name == 'G' and self.current_destination == self.G.Ec) or (self.current_node.name == 'G' and self.current_destination == self.G.Sc) or (self.current_node.name == 'G' and self.current_destination == self.G.Wc) :
                            #when left first is better than right first
                            self.skipZero = True
                        else:
                            self.skipZero = False




                    #COMMON to all except [] - set Expected facing
                    if (self.current_node.name, t) not in [('A', 'B'), ('B', 'A'), ('C', 'D'), ('D', 'C'), ('E', 'G'), ('G', 'E')]:
                        if self.returnNode(t).Nc == self.current_node.name and self.destination_id in [-1, 0]:
                            self.E_facing = 2
                        elif self.returnNode(t).Ec == self.current_node.name and self.destination_id in [-1, 1]:
                            self.E_facing = 3
                        elif self.returnNode(t).Sc == self.current_node.name and self.destination_id in [-1, 2]:
                            self.E_facing = 0
                        elif self.returnNode(t).Wc == self.current_node.name and self.destination_id in [-1, 3]:
                            self.E_facing = 1
                        
                    else:
                        if self.current_node.name == 'A':
                            if self.imu_target == 0:
                                self.E_facing = 2
                            else:
                                self.E_facing = 1
                        elif self.current_node.name == 'B':
                            if self.imu_target == 0:
                                self.E_facing = 2
                            else:
                                self.E_facing = 3
                        elif self.current_node.name == 'C':
                            if self.imu_target == 1:
                                self.E_facing = 3
                            else:
                                self.E_facing = 2
                        elif self.current_node.name == 'D':
                            if self.imu_target == 1:
                                self.E_facing = 3
                            else:
                                self.E_facing = 0
                        elif self.current_node.name == 'E':
                            if self.imu_target == 3:
                                self.E_facing = 0
                            else:
                                self.E_facing = 1
                        elif self.current_node.name == 'G':
                            if self.imu_target == 3:
                                self.E_facing = 0
                            else:
                                self.E_facing = 3

                    


                    #COMMON EXCEPT FOR []
                    self.toDepart = True

                    if self.behaviourMode!=1:
                        if abs(self.facing - self.imu_target) in [1,3]:
                            #90 degree turn (riskiest for losing line)
                            self.clearGray()
                        else:
                            self.startTurnBasedOnIMU()
                    else:
                        self.toDepart = False #consume
                        self.departureTime = self.now


                else:
                   #empty destination
                    self.senseEntryTime = self.now
                    self.dontSense = True
                    self.stateFollow = False



                self.get_logger().info(f"Current Location:{self.current_node.name}; Current Destination: {self.current_destination}")

            elif(self.allowCrawl): 
                #no IR detection
                #self.get_logger().warning(f"Not detecting gray, cannot enter intersection.")
                # --- Crawl-back recovery ---
                # If the robot has been sitting here without detecting gray, it may have
                # overshot the intersection spot after turning.  After 5 s of being
                # stationary (and not waiting on ultrasonic) nudge backwards once so the
                # IR sensors can re-detect the gray spot.
                if self.stationaryStartTime == -1:
                    self.stationaryStartTime = self.now
                

                STATIONARY_TIMEOUT = 5.0  # seconds before we try crawling back
                if self.now > self.stationaryStartTime + STATIONARY_TIMEOUT:
                    self.get_logger().info("Stationary >5s without gray — crawling back to re-detect.")
                    self.stationaryStartTime = self.now

                    #we've been stationary for too long.
                    #crawl back
                    self.crawlBack()
                    self.stateFollow = False
                    self.crawlingBackwards = True


            

            

    def surveillCapture(self):
        #ultrasonic_sweep.py is constantly turning and checking.
        #ultrasonic is probably measuring in metres (m)
        #~10cm is the maximum distance for capture in tight spaces of the map
        #check tag dry-run dated 8 may for full discussion
        
        #TAG
        if self.ultrasonic_distance < self.CAPTURE_MAX and (self.now > self.time_of_last_tag + self.TAG_COOLDOWN) and not self.doTag and not self.initiated_tag and not self.tag:
            self.initiated_tag = True
            self.get_logger().info("Initiating tag...")
            self.stopMov()
        

        if self.doTag:
            self.doTag = False #consume command
            self.time_of_last_tag = self.now

            #switch mode
            if self.behaviourMode == 1:
                self.behaviourMode = 4
                
            else:
                self.behaviourMode = 1
            
            
            if self.evading:
                #pause new pursuer to give the evader some time to put some distance between them and avoid collisions.
                self.stateFollow = False
                self.startPauseTime = self.now
                self.paused = True

                #unblock line following
                self.retryPlan = 0
                self.postRetry = False
                self.dontSense = False
                self.waitingForUltrasonic = False
                self.allowCrawl = True
            else:
                #resolve destination

                #is path blocked? - it probably is, so let's assume as much.
                #turn 180 and go back to where you were
                if self.facing == 0:
                    self.imu_target = 2
                elif self.facing == 2:
                    self.imu_target = 0
                elif self.facing == 1:
                    self.imu_target = 3
                elif self.facing == 3:
                    self.imu_target = 1
                    
                self.startTurnBasedOnIMU()
                self.current_destination = self.current_node

            #otherwise just keep following the line to your intended destination to resolve your location, then restart process from there.
            if type(self.current_destination) == list:
                if self.current_destination:
                    self.current_destination = self.current_destination[0]

            self.resetBehaviour = True
            self.initial_reading_taken = False

            #flip status
            self.evading = not self.evading

            self.get_logger().info(f"TAG! New Mode: {self.behaviourMode}; Ev?: {self.evading}")

            #clear old values before going in to not propogate stale values.
            self.entry_angle = float('inf')
            self.exit_angle = float('inf')
            self.ultrasonic_distance = float('inf')



    def checkUltra(self):
        self.retryPlan = 0

        #retryPlan values:
        # 0 means we're OK
        #-1 means turn 180
        #-2 means turn left 90
        #-3 means turn right 90

        #no detection
        if self.ultrasonic_distance == float('inf'):
            self.retryPlan = -1

            #only 1 detected
        elif(self.entry_angle == float('inf')):
            #self.locateTarget = True
            if self.exit_angle == float('inf'):
                #no detection
                self.retryPlan =  -1
            else:
                if self.exit_angle < 0:
                    self.retryPlan = -2
                elif self.exit_angle > 0:
                    self.retryPlan = -3

        elif self.exit_angle == float('inf'):
            #self.locateTarget = True
            #only 1 detected
            if self.entry_angle < 0:
                self.retryPlan = -2
            elif self.entry_angle > 0:
                self.retryPlan = -3
            
        #else, it's safe to continue
        if self.behaviourMode in [3,5] and self.retryPlan == 0 and self.completeSequence:
            self.goAhead = True
            self.completeSequence = False

    def takeStep(self):
        c = self.current_node.name
        if type(self.last_node) == str:
            l = 'Z'
            self.get_logger().info(f"LAST NODE IS: {self.last_node}")
        else:
            l = self.last_node.name


        if c == 'A':
            self.current_destination = [1]

        elif c == 'B':
            if l in ['A', 'C', 'D']:
                choice = random.randint(1,2)
                if l == 'A':
                    ch = {1: 1, 2: 2}

                elif l == 'C':
                    ch = {1: 3, 2: 2}

                elif l == 'D':
                    ch = {1: 1, 2: 3}

                self.current_destination = [ch[choice]]
            else:
                choice = random.randint(1,3) #both inclusive
                self.current_destination = [choice]

        elif c == 'C':
            if l in ['B', 'D']:
                if l == 'B':
                    ch = 2

                elif l == 'D':
                    ch = 3

                self.current_destination = [ch]
            else:
                choice = random.randint(2,3) #both inclusive
                self.current_destination = [choice]
            
        elif c == 'D':
            if l in ['B', 'C', 'D']:
                choice = random.randint(1,2)
                if l == 'B':
                    ch = {1: 0, 2: 2}

                elif l == 'C':
                    ch = {1: 3, 2: 2}

                elif l == 'D':
                    ch = {1: 0, 2: 3}

                self.current_destination = [ch[choice]]
            else:
                choice = random.randint(1,3) #both inclusive
                if choice == 1:
                    choice = 0
                
                self.current_destination = [choice]
            
        elif c == 'E':
            self.current_destination = [1]

        elif c == 'F':
            if l in ['G', 'E', 'D']:
                choice = random.randint(1,2)
                if l == 'G':
                    ch = {1: 3, 2: 1}

                elif l == 'E':
                    ch = {1: 1, 2: 2}

                elif l == 'D':
                    ch = {1: 2, 2: 3}

                self.current_destination = [ch[choice]]
            else:
                choice = random.randint(1,3) #both inclusive
                
                self.current_destination = [choice]
        
        elif c == 'G':
            if l in ['F', 'H']:
                if l == 'F':
                    ch = 1

                elif l == 'H':
                    ch = 0

                self.current_destination = [ch]
            else:
                choice = random.randint(0,1) #both inclusive
                self.current_destination = [choice]

        elif c == 'H':
            self.current_destination = [3]
            

            
    
    def loop(self):
        STARTUP_WAIT = 7
        
        #updates
        self.now = time.time() #so that all time variables are up-to-date

        #Wait for everything to settle
        if self.STARTUP_TIME == -1:
            self.STARTUP_TIME = self.now
        
        if self.now < self.STARTUP_TIME + STARTUP_WAIT:
            return

        self.update_motion() #check for end of turn

        #check cooldowns
        if (self.now > self.startPauseTime + self.PAUSE_TIME) and self.paused:
            self.paused = False #consume

        if (self.now > self.senseEntryTime + self.SENSE_COOLDOWN) and self.dontSense and self.current_destination == [] and not self.crawlingForwardBeforeIMUturn and not self.aligning and not self.crawlBackBeforeIMUturn:
            self.dontSense = False #consume
            self.triggerSweep = True

        if self.behaviourMode in [3,5] and self.lookAround:
            self.triggerSweep = True
            self.lookAround = False #consume
            self.completeSequence = True
        

        #Ultrasonic Sweep Modes
        if (self.triggerSweep or self.retryPlan != 0 or (self.behaviourMode in [3,4,5] and not self.initial_reading_taken)) and not self.waitingForUltrasonic and not self.imu_turning and not self.crawlingForwardBeforeIMUturn  and not self.aligning and not self.crawlBackBeforeIMUturn:
            #trigger single sweep
            self.sweep = True
            self.multiple = False
            self.waitingForUltrasonic = True
            #clear old values before going in to not propogate stale values.
            self.entry_angle = float('inf')
            self.exit_angle = float('inf')
            self.ultrasonic_distance = float('inf')
            self.publish_sweep_command()

            if self.retryPlan != 0 or self.triggerSweep:
                self.postRetry = True

            #consume
            self.retryPlan = 0
            self.triggerSweep = False
            self.initial_reading_taken = True
        
        #specifically when retrying
        if self.postRetry and not self.imu_turning and not self.waitingForUltrasonic and not self.crawlingForwardBeforeIMUturn and not self.aligning and not self.crawlBackBeforeIMUturn:
            self.postRetry = False
            self.checkUltra()
            self.stepping = False

            if self.retryPlan != 0:
                # Still can't plan — set up another retry turn
                self.stopMov()

                self.retryAttempts +=1
                if self.retryAttempts >=3 and self.retryPlan not in [-2,-3]:
                    if self.behaviourMode in [3,5]:

                        if self.current_destination == [] or type(self.current_destination) == str:
                            #if we've exhausted our plan or if we are still at our first node
                            #proactive search
                            self.takeStep()
                            self.get_logger().info(f"stepping to {self.current_destination[0]}")
                        elif self.current_destination:
                            #if we have a plan, just keep following it, dont waste time searching here
                            #will still set stepping to true, not actually stepping but it's a good check to use in updatePlan
                            self.get_logger().info("progressing with saved plan.")

                        self.stepping = True
                        self.goAhead = True
                        self.destination_id = -1
                        self.retryPlan = 0
                        self.retryAttempts = 0
                        self.get_logger().info("Departing via 3,5 beh.")
                    else:
                        #passive
                        #don't keep turning 180s, do a finer grain search
                        targets = {0: 1, 2: 3, 1: 2, 3: 0}
                else:
                    if self.retryPlan == -1:
                        targets = {0: 2, 2: 0, 1: 3, 3: 1}
                    elif self.retryPlan == -2:
                        targets = {0: 1, 2: 3, 1: 2, 3: 0}
                    elif self.retryPlan == -3:
                        targets = {0: 3, 2: 1, 1: 0, 3: 2}
                    else:
                        self.retryAttempts = 0
                        targets = {}

                if not self.stepping:
                    self.imu_target = targets.get(self.facing, -1)
                
                    self.startTurnBasedOnIMU()
            
            else:
                #checkUltra returned clear
                self.retryAttempts = 0
                self.allowCrawl = True

                if self.behaviourMode in [3,5]:
                    self.goAhead = True
                    self.get_logger().info("going ahead and generating a new path based on latest findings.")

        #check for tags and publish status
        self.surveillCapture()
        self.publish_tag_status()

        if self.retryPlan != 0 or self.paused or self.dontSense or self.imu_turning or (self.behaviourMode in [3,4,5] and not self.initial_reading_taken) or self.crawlingForwardBeforeIMUturn  or self.aligning or self.crawlBackBeforeIMUturn:
            self.get_logger().info(f"retryPlan: {self.retryPlan}, paus: {self.paused}, dontS: {self.dontSense}, turn: {self.imu_turning}, initial_taken: {self.initial_reading_taken}, crawlF: {self.crawlingForwardBeforeIMUturn}, align: {self.aligning}, crawlBIMU: {self.crawlBackBeforeIMUturn}")
            pass
        elif not self.waitingForUltrasonic:
            #check for intersection, reset behaviour from tag, update location and destination and target tracking
            if self.behaviourMode in [3,4,5]:
                self.checkUltra()

            if self.retryPlan == 0:
                self.get_logger().info(f"entering updatepos with goAhead: {self.goAhead}, step: {self.stepping}, completeSeq: {self.completeSequence}, lookAround: {self.lookAround}")
                self.updatePos() #gated by self.imu_turning and by GRAY_COOLDOWN

        
        if self.retryPlan != 0 or self.postRetry or self.lookAround or self.paused or self.imu_turning or self.dontSense or self.waitingForUltrasonic or self.crawlingBackwards or self.crawlingForwardBeforeIMUturn or self.aligning or self.crawlBackBeforeIMUturn:
            if self.stateFollow: 
                self.get_logger().info(f"Set stateFollow to False in loop(). retryPlan: {self.retryPlan}, postRetry: {self.postRetry}, paused: {self.paused}, imu_turn: {self.imu_turning}, dontSense: {self.dontSense}, wait: {self.waitingForUltrasonic}, crawlBack: {self.crawlingBackwards}, crawlForward: {self.crawlingForwardBeforeIMUturn}, aligning: {self.aligning}")
            self.stateFollow = False

            #Stop sweep.
            if (self.sweep or self.multiple) and not self.waitingForUltrasonic and not self.lookAround:
                self.sweep = False
                self.multiple = False
                self.publish_sweep_command()

        elif self.current_destination == [] and self.behaviourMode == 4:
            self.stateFollow = False

        elif not self.firstNode:
            self.stateFollow = True
            self.allowCrawl = False
        

        if self.stateFollow:
            #simple line following
            self.followLine()

            #Tag Sweep
            if not self.sweep or not self.multiple:
                self.sweep = True
                self.multiple = True
                self.publish_sweep_command()

        #self.get_logger().info(f"dontSense: {self.dontSense}, imu_turning: {self.imu_turning}, wait: {self.waitingForUltrasonic}, yaw_deg: {self.yaw_deg}")

def main():
    rclpy.init()
    node = line_follower()
        
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
