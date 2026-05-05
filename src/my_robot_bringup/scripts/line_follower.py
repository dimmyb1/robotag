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
from std_msgs.msg import String, Float64, Float64MultiArray, Bool
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

        #junction turning vars
        self.stateFollow = True
        #self.completeTurn = False
        self.wasLeft = False
        self.imu_turning = False
        self.imu_target = -1
        self.grayEntryTime = -1
        self.GRAY_COOLDOWN = 7 
        #   tried 8, 8 was too high, 
        #   but 5 is too high when turning, so maybe we can make this variable with turning time
        #   instead, i blocked intersection detection completely when imu_turning at an intersection
        #   5 is too low when traversing, im setting it to 7.
        #   7 seems really good.
        self.senseEntryTime = -1
        self.SENSE_COOLDOWN = 6 #tried 5 but seemed low
        self.dontSense = False
        self.firstNode = True

        #tag vars + esp comms
        self.CAPTURE_MAX = 0.1
        #self.CAPTURE_MIN = 0.05
        self.PAUSE_TIME = 7 #was 6, but seemed low
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
        self.initiated_sweep = False
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
        self.last_node = 'Z' #for localisation, this is NODE, 'Z' is a placeholder for (re)starting
        self.current_destination = 'F'
        self.skipZero = False
        self.retryPlan = 0
        self.postRetry = False

        #localisation
        self.departureTime = -1
        self.TIME_VARIANCE = 2
        self.loc_hyp = []
        self.toDepart = False

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
            "Pfd1": 0.0625,
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
            "Pfd1": 0.0,
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
            self.behaviourMode = 4
            self.evading = True
        elif robot_name == 'twirl':
            self.current_node = self.A
            self.get_logger().info("Detected robot: twirl. Starting at Node A.")
            other_robot_name = 'twix'
            self.behaviourMode = 1
            self.evading = False
            self.i_patrol = 0
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

        #given that we start facing south
        # if 0.0 <= self.yaw_deg < 45.0 or 315.0 <= self.yaw_deg < 360.0:
        #     self.facing = 2
        # elif 45.0 <= self.yaw_deg < 135.0 :
        #     self.facing = 3
        # elif 135.0 <= self.yaw_deg < 225.0 :
        #     self.facing = 0
        # elif 225.0 <= self.yaw_deg < 315.0 :
        #     self.facing = 1

        #specifically because of the sim, this needs to be written as:
        if 0.0 <= self.yaw_deg < 45.0 or 315.0 <= self.yaw_deg < 360.0:
            self.facing = 0
        elif 45.0 <= self.yaw_deg < 135.0 :
            self.facing = 1
        elif 135.0 <= self.yaw_deg < 225.0 :
            self.facing = 2
        elif 225.0 <= self.yaw_deg < 315.0 :
            self.facing = 3
        
        #self.get_logger().info(f'IMU:: Current Z Rotation (Yaw): {self.yaw_deg:.2f}°, Facing: {self.facing}')

    #Ultrasonic functions
    def ultrasonic_callback(self, msg):
        # Unpack the array based on the order you published it
        #ANGLES ARE IN RADIANS (but i can do math.degrees(v) to convert to normal degrees if i need to)
        self.entry_angle = msg.data[0]
        self.exit_angle = msg.data[1]
        self.ultrasonic_distance = msg.data[2]

        if not self.initial_reading_taken:
            self.initial_reading_taken = True #consume
        if  self.waitingForUltrasonic:
            self.waitingForUltrasonic = False
            self.locateTarget = False
        
        #self.get_logger().info(f"Received Object Data -> Entry: {entry_angle:.2f}, Exit: {exit_angle:.2f}, Dist: {distance:.2f}")

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
        # returns a mask of black pixels in the image
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([26, 26, 26])
        mask = cv2.inRange(img, lower_black, upper_black)
        return cv2.countNonZero(mask) #(int) num of black pixels in img
    
    def detect_gray(self, img):
        # returns a mask of black pixels in the image
        lower_gray = np.array([20, 20, 20])
        upper_gray = np.array([85, 85, 85])
        mask = cv2.inRange(img, lower_gray, upper_gray)
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
        ANGLE_TOLERANCE = 4
        #self.get_logger().info(f"Turning... yaw={self.yaw_deg:.1f}, target={self.imu_target}")
        if self.motion_active and self.now >= self.motion_end_time:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)
            self.motion_active = False
            #self.get_logger().info(f"stopped moving because time expired. imu_turning: {self.imu_turning}, complete_turn: {self.completeTurn}, motion_active: {self.motion_active}")

        elif(self.imu_turning):
            #target = ((self.imu_target - STARTED_FACING) * 90)
            target = self.imu_target * 90
            if target < 0:
                target+=360

            if target < ANGLE_TOLERANCE:
                if (360+target) - ANGLE_TOLERANCE <= self.yaw_deg or self.yaw_deg <= target + ANGLE_TOLERANCE:
                    #we have completed our turn.
                    self.imu_turning = False
                    self.imu_target = -1
                    self.stopMov()
                    self.retryPlan = 0

                    if self.toDepart:
                        self.toDepart = False #consume
                        self.departureTime = self.now
                        self.stateFollow = True
            else:
                if target - ANGLE_TOLERANCE <= self.yaw_deg <= target + ANGLE_TOLERANCE:
                    #we have completed our turn.
                    self.imu_turning = False
                    self.imu_target = -1
                    self.stopMov()
                    self.retryPlan = 0

                    if self.toDepart:
                        self.toDepart = False #consume
                        self.departureTime = self.now
                        self.stateFollow = True
 

    def stopMov(self) :
        self.start_motion()
        
    # --------------------------
    # Basic Turning Functions
    # --------------------------

    def turnRight(self, duration=0):
        self.start_motion(angular=-0.75, duration_ms=duration)

    def turnLeft(self, duration=0) :
        self.start_motion(angular=0.75, duration_ms=duration)


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
        if fromNCHAR == 'D': return ["Pcd1", "Pcd2", "Pfd1", "Pbd1"]
        if fromNCHAR == 'E': return ["Peg1", "Peg2", "Pae1", "Pef1"]
        if fromNCHAR == 'F': return ["Pef1", "Paf1", "Pfg1", "Pfd1"]
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
        D = ["Pcd1", "Pcd2", "Pfd1", "Pbd1"]
        E = ["Peg1", "Peg2", "Pae1", "Pef1"]
        F = ["Pef1", "Paf1", "Pfg1", "Pfd1"]
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
        D = ["Pcd1", "Pcd2", "Pfd1", "Pbd1"]
        E = ["Peg1", "Peg2", "Pae1", "Pef1"]
        F = ["Pef1", "Paf1", "Pfg1", "Pfd1"]
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

    def self_localise(self, edgeTime):
        if(self.last_node == 'Z'):
            return
        #where edge time is the average timing of the edge we took
        #otherwise create a variable stating which direction we took i.e. N = 0
        #so that we can say (last_node.Times[0])
        hyp = []

        maxTime = self.departureTime + self.TIME_VARIANCE + edgeTime
        minTime = self.departureTime - self.TIME_VARIANCE + edgeTime
        if (self.now > maxTime) or (self.now < minTime):
            #if it has taken longer than or less than the expected time
            #check if any of the old node's timings match better
            if minTime < self.last_node.Times[0] < maxTime:
                hyp.append(self.last_node.Nc)
            elif minTime < self.last_node.Times[1] < maxTime:
                hyp.append(self.last_node.Ec)
            elif minTime < self.last_node.Times[2] < maxTime:
                hyp.append(self.last_node.Sc)
            elif minTime < self.last_node.Times[3] < maxTime:
                hyp.append(self.last_node.Wc)

            #if not, we either got turned around, or we just struggled / found it easy to get here
            #in case we got turned around copy the node we left from:
            hyp.append(self.last_node.name)
            #otherwise if we just deviated slightly, the rest of the map should match up, so either way let's check the next edge we take:

            if self.loc_hyp:
                #if we formed a hypothesis on where we may be at our last node, then we have previously cast doubt on where we are
                #and if we've gotten to this if statement, that means we have been at two dubious nodes which we are setting to be our limit <BIAS> / <HYPERPARAM?>
                
                #then let's see if one of the old hypothesis nodes could work, and if one of them can,  take the first of which which makes sense, and 
                #change last and current nodes to reflect that (yes this is a bit fickle, but you can technically keep building this up until you're certain)
                #but given we only have 8 nodes, about 2-3 rounds of checks should be enough to localise, but 2 is simpler
                #BIAS
                for h in self.loc_hyp:
                    hn = self.returnNode(h)
                    if minTime < hn.Times[0] < maxTime:
                        self.last_node = h
                        self.current_node = self.returnNode(hn.Nc)
                        self.loc_hyp = []
                        break
                    elif minTime < hn.Times[1] < maxTime:
                        self.last_node = h
                        self.current_node = self.returnNode(hn.Ec)
                        self.loc_hyp = []
                        break
                    elif minTime < hn.Times[2] < maxTime:
                        self.last_node = h
                        self.current_node = self.returnNode(hn.Sc)
                        self.loc_hyp = []
                        break
                    elif minTime < hn.Times[3] < maxTime:
                        self.last_node = h
                        self.current_node = self.returnNode(hn.Wc)
                        self.loc_hyp = []
                        break

            #if we found a possible hypothesis (i.e. there was some issue), record it and store it for the next edge check.
            if hyp:
                self.loc_hyp = hyp.copy()
                return
            
        #otherwise, it has taken the expected amount of time
        #we have no concerns.
        #can exit
        self.loc_hyp = []
        return


    #-------------------------------------
    # Target Tracking and Route Planning
    #-------------------------------------
        
    #Graph Functions
    def calculateProbabilities(self):
        #let's say radar stores the closest ultrasonic ping in euclidean metric in self.ultrasonic_distance
        #we get two readings: first ping entering reading (self.entry_angle)
        # second ping exiting reading (self.exit_angle)
        # the order doesnt make a difference, main thing is that we have the angle, wwe'll just take min or max of the two values.
        #and servo was the angle at which we go tthe reading, +- the known margin of error
        #and that BOXMEAS is the l / w of the boxes in the grid in euclidean metric
        BOXMEAS = 1
        PERSISTENCE = 0.7
        CONTAMINATION = 0.3
        
        #and let's say we have estimated our current coordinates to be x and y
        # x = 2
        # y = 3

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

        # so minimum (2,3), maximum (3,4) starts
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
        #no detection: float('inf')
        # straight ahead / middle - 0
        # otherwise returned in RADIANS
        # +1.57 is left 90
        # -1.57 is right 90

        

        #first we will check if the mark is right in front of us
        lesserServo = min(self.entry_angle, self.exit_angle)
        biggerServo = max(self.entry_angle, self.exit_angle)
        #for now i assume these to be in euclidean 

        



        if(lesserServo < 90 and biggerServo > 90):
            #if current cardinal direction we're facing is North or South
            # for c in cells:
            #     if(c.x != x):
            #         cells.remove(c)

            cells = [c for c in cells if c.x == x]
            cells = [c for c in cells if c.y == y]

            #if current cardinal is East or West:
            # for c in cells:
            #     if(c.y != y):
            #         cells.remove(c)

        else:
            #it isnt directly in front of us.
            #is it on the left or on the right?
            servoCells = []


            #on the right side:
            if(biggerServo <= 90):
                #if facing North
                if(self.facing==0):
                    #take the maximum area
                    diffX = 4-x
                    diffY = math.ceil(diffX * math.tan(biggerServo))

                    ix = x + 1
                    while(ix<5):
                        if  (ix <= (x + math.ceil(diffX / 2))):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(math.ceil(diffY/2)):
                                if(iy + y < 6) and (iy + y > -1):
                                    servoCells.append(Cell(ix, iy+y))
                        else:
                            for iy in range(diffY):
                                if(iy + y < 6) and (iy + y > -1):
                                    servoCells.append(Cell(ix, iy+y))

                        ix+=1
                            
                    #now make the smaller triangle
                    diffY = math.ceil(diffX * math.tan(lesserServo))

                    ix = x + 1
                    while(ix<5):
                        if(ix <= (x + math.ceil(diffX / 2))):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(math.ceil(diffY/2)):
                                if(iy + y < 6) and (iy + y > -1):
                                    servoCells.remove(Cell(ix, iy+y))
                        else:
                            for iy in range(diffY):
                                if(iy + y < 6) and (iy + y > -1):
                                    servoCells.remove(Cell(ix, iy+y))

                        ix+=1


                #elif south
                elif(self.facing==2):
                    #take the maximum area
                    diffX = x
                    diffY = math.ceil(diffX * math.tan(biggerServo))

                    ix = x - 1
                    while(ix>-1):
                        if(ix > (x - math.ceil(diffX / 2))):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(math.ceil(diffY/2)):
                                if(y - iy > 0) and (y - iy < 6):
                                    servoCells.append(Cell(ix, y-iy))
                        else:
                            for iy in range(diffY):
                                if(y - iy > 0) and (y - iy < 6):
                                    servoCells.append(Cell(ix, y-iy))

                        ix-=1
                            
                    #now make the smaller triangle
                    diffY = math.ceil(diffX * math.tan(lesserServo))

                    ix = x - 1
                    while(ix>-1):
                        if(ix > (x - math.ceil(diffX / 2))):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(math.ceil(diffY/2)):
                                if(y - iy > 0) and (y - iy < 6):
                                    servoCells.remove(Cell(ix, y-iy))
                        else:
                            for iy in range(diffY):
                                if(y - iy > 0) and (y - iy < 6):
                                    servoCells.remove(Cell(ix, y-iy))

                        ix-=1

                #east?
                elif self.facing==1:
                    #take the maximum area
                    diffY = y
                    diffX = math.ceil(diffY * math.tan(biggerServo))

                    iy = y - 1
                    while(iy>-1):
                        if((iy >= (y - math.ceil(diffY / 2))) and y%2==0) or ((iy > (y - math.ceil(diffY / 2))) and y%2!=0):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if(x + ix < 5) and (x + ix > -1):
                                    servoCells.append(Cell(x + ix, iy))
                        else:
                            for ix in range(diffX):
                                if(x + ix < 5) and (x + ix > -1):
                                    servoCells.append(Cell(x + ix, iy))

                        iy-=1
                            
                    #now make the smaller triangle
                    diffX = math.ceil(diffY * math.tan(lesserServo))

                    iy = y - 1
                    while(iy>-1):
                        if((iy >= (y - math.ceil(diffY / 2))) and y%2==0) or ((iy > (y - math.ceil(diffY / 2))) and y%2!=0):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if(x + ix < 5) and (x + ix > -1):
                                    servoCells.remove(Cell(x + ix, iy))
                        else:
                            for ix in range(diffX):
                                if(x + ix < 5) and (x + ix > -1):
                                    servoCells.remove(Cell(x + ix, iy))

                        iy-=1

                #west?
                elif self.facing==3:
                    diffY = 5-y
                    diffX = math.ceil(diffY * math.tan(biggerServo))

                    iy = y + 1
                    while(iy<6):
                        if  ((iy < (y + math.ceil(diffY / 2))) and y%2!=0) or ((iy <= (y + math.ceil(diffY / 2))) and y%2==0)  :
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if(x - ix < 5):
                                    servoCells.append(Cell(x - ix, iy))
                        else:
                            for ix in range(diffX):
                                if(x - ix < 5) and (x- ix > -1):
                                    servoCells.append(Cell(x - ix, iy))

                        iy+=1
                            
                    #now make the smaller triangle
                    diffX = math.ceil(diffY * math.tan(lesserServo))

                    iy = y + 1
                    while(iy<6):
                        if ((iy < (y + math.ceil(diffY / 2))) and y%2!=0) or ((iy <= (y + math.ceil(diffY / 2))) and y%2==0):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if(x - ix > -1):
                                    servoCells.remove(Cell(x - ix, iy))
                        else:
                            for ix in range(diffX):
                                if(x - ix > -1):
                                    servoCells.remove(Cell(x - ix, iy))

                        iy+=1

            else:
                #on the LHS
                #if facing North
                if self.facing==0:
                    #take the maximum area
                    diffX = x
                    diffY = math.ceil(diffX * math.tan(180 - lesserServo))

                    ix = x - 1
                    while(ix>-1):
                        if((ix >= (x - math.ceil(diffX / 2))) and y%2==0) or ((ix > (x - math.ceil(diffX / 2))) and y%2!=0):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(math.ceil(diffY/2)):
                                if(iy + y < 6) and (iy + y > -1):
                                    servoCells.append(Cell(ix, iy+y))
                        else:
                            for iy in range(diffY):
                                if(iy + y < 6) and (iy + y > -1):
                                    servoCells.append(Cell(ix, iy+y))

                        ix-=1
                            
                    #now make the smaller triangle
                    diffY = math.ceil(diffX * math.tan(180 - biggerServo))

                    ix = x - 1
                    while(ix>-1):
                        if((ix >= (x - math.ceil(diffX / 2))) and y%2==0) or ((ix > (x - math.ceil(diffX / 2))) and y%2!=0):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(math.ceil(diffY/2)):
                                if(iy + y < 6) and (iy + y > -1):
                                    servoCells.remove(Cell(ix, iy+y))
                        else:
                            for iy in range(diffY):
                                if(iy + y < 6) and (iy + y > -1):
                                    servoCells.remove(Cell(ix, iy+y))

                        ix-=1

                #if facing south
                elif(self.facing==2):
                    #take the maximum area
                    diffX = 4 - x
                    diffY = math.ceil(diffX * math.tan(180 - lesserServo))

                    ix = x + 1
                    while(ix<5):
                        if((ix < (x + math.ceil(diffX / 2))) and (x%2!=0)) or ((ix <= (x + math.ceil(diffX / 2))) and (x%2==0)):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(math.ceil(diffY/2)):
                                if(y - iy > -1) and (y - iy < 6):
                                    servoCells.append(Cell(ix, y - iy))
                        else:
                            for iy in range(diffY):
                                if(y - iy > -1) and (y - iy < 6):
                                    servoCells.append(Cell(ix, y - iy))

                        ix+=1
                            
                    #now make the smaller triangle
                    diffY = math.ceil(diffX * math.tan(180 - biggerServo))

                    ix = x + 1
                    while(ix<5):
                        if((ix < (x + math.ceil(diffX / 2))) and (x%2!=0)) or ((ix <= (x + math.ceil(diffX / 2))) and (x%2==0)):
                            #we need to do the diffY/2 ones at this ix value
                            for iy in range(math.ceil(diffY/2)):
                                if(y - iy > -1) and (y - iy < 6):
                                    servoCells.remove(Cell(ix, y - iy))
                        else:
                            for iy in range(diffY):
                                if(y - iy > -1) and (y - iy < 6):
                                    servoCells.remove(Cell(ix, y - iy))

                        ix+=1
                #EAST
                elif(self.facing==1):
                    #take the maximum area
                    diffY = 5-y
                    diffX = math.ceil(diffY * math.tan(180 - lesserServo))

                    iy = y + 1
                    while(iy<6):
                        if(iy < (y + math.ceil(diffY / 2))) and (y%2==0) or (iy < (y + math.ceil(diffY / 2))) and (y%2!=0):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if(x + ix < 5) and (x+ix>-1):
                                    servoCells.append(Cell(x + ix, iy))
                        else:
                            for ix in range(diffX):
                                if(x + ix < 5) and (x+ix>-1):
                                    servoCells.append(Cell(x + ix, iy))

                        iy+=1
                            
                    #now make the smaller triangle
                    diffX = math.ceil(diffY * math.tan(180 - biggerServo))

                    iy = y + 1
                    while(iy<6):
                        if(iy < (y + math.ceil(diffY / 2))) and (y%2==0) or (iy < (y + math.ceil(diffY / 2))) and (y%2!=0):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if(x + ix < 5) and (x+ix>-1):
                                    servoCells.remove(Cell(x + ix, iy))
                        else:
                            for ix in range(diffX):
                                if(x + ix < 5) and (x+ix>-1):
                                    servoCells.remove(Cell(x + ix, iy))

                        iy+=1

                elif(self.facing==3):
                    #west left
                    diffY = y
                    diffX = math.ceil(diffY * math.tan(180 - lesserServo))

                    iy = y - 1
                    while(iy> -1):
                        if(iy > (y - math.ceil(diffY / 2))) and (y%2!=0) and (iy > (y - math.ceil(diffY / 2))) and (y%2==0):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if(x - ix > -1) and (x-ix<5):
                                    servoCells.append(Cell(x - ix, iy))
                        else:
                            for ix in range(diffX):
                                if(x - ix > -1)and (x-ix<5):
                                    servoCells.append(Cell(x - ix, iy))

                        iy-=1
                            
                    #now make the smaller triangle
                    diffX = math.ceil(diffY * math.tan(180 - biggerServo))

                    iy = y - 1
                    while(iy> -1):
                        if(iy > (y - math.ceil(diffY / 2))) and (y%2!=0) and (iy > (y - math.ceil(diffY / 2))) and (y%2==0):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if(x - ix > -1)and (x-ix<5):
                                    servoCells.remove(Cell(x - ix, iy))
                        else:
                            for ix in range(diffX):
                                if(x - ix > -1)and (x-ix<5):
                                    servoCells.remove(Cell(x - ix, iy))

                        iy-=1
                
            #now find INTERSECTION with radius cells
            # for c in cells:
            #     if c not in servoCells:
            #         cells.remove(c)
                        
            cells = [c for c in cells if c in servoCells]
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
            "Pfd1": 0.0,
            "Pbd1": 0.0,
            "Pbc1": 0.0,
            "Pcd1": 0.0,
            "Pcd2": 0.0,
            "Pch1": 0.0
        }

        


        for c in cells:
            if c.x == 0:
                if c.y == 0:
                    self.P["Peg2"] +=1
                elif c.y == 1:
                    self.P["Peg2"] += 1
                elif c.y == 2:
                    self.P["Pae1"] += 0.25
                    self.P["Peg2"] += 0.75
                elif c.y == 3:
                    self.P["Pae1"] +=1
                elif c.y == 4:
                    self.P["Pae1"] +=1
                # elif c.y == 5:
                #     return
            elif c.x == 1:
                if c.y == 0:
                    self.P["Peg2"] +=1
                elif c.y == 1:
                    self.P["Peg2"] +=0.156
                    self.P["Peg1"] +=0.844
                elif c.y == 2:
                    self.P["Peg1"]+= 0.25
                    self.P["Peg2"]+= 0.25
                    self.P["Pef1"]+= 0.25
                    self.P["Pae1"]+= 0.25
                elif c.y == 3:
                    self.P["Pae1"] += 0.293
                    self.P["Paf1"] += 0.707
                elif c.y == 4:
                    self.P["Pab1"]+= 0.25
                    self.P["Pab2"]+= 0.25
                    self.P["Paf1"]+= 0.25
                    self.P["Pae1"]+= 0.25
                elif c.y == 5:
                    self.P["Pab1"] += 1
            elif c.x == 2:
                if c.y == 0:
                    self.P["Peg2"] +=1
                elif c.y == 1:
                    self.P["Pgh1"]+= 0.25
                    self.P["Pfg1"]+= 0.25
                    self.P["Peg1"]+= 0.25
                    self.P["Peg2"]+= 0.25
                elif c.y == 2:
                    self.P["Pef1"]+= 0.25
                    self.P["Pfg1"]+= 0.25
                    self.P["Paf1"]+= 0.25
                    self.P["Pfd1"]+= 0.25
                elif c.y == 3:
                    self.P["Pbd1"] += 0.498
                    self.P["Paf1"] += 0.498
                    self.P["Pfd1"] += 0.004
                elif c.y == 4:
                    self.P["Pab1"]+= 0.25
                    self.P["Pab2"]+= 0.25
                    self.P["Pbc1"]+= 0.25
                    self.P["Pbd1"]+= 0.25
                elif c.y == 5:
                    self.P["Pab1"] += 1
            elif c.x == 3:
                if c.y == 0:
                    self.P["Phh1"] +=1
                elif c.y == 1:
                    self.P["Pgh1"]+= 0.25
                    self.P["Pch1"]+= 0.25
                    self.P["Phh1"]+= 0.25
                elif c.y == 2:
                    self.P["Pfd1"] += 0.477
                    self.P["Pch1"] += 0.523
                elif c.y == 3:
                    self.P["Pfd1"]+= 0.25
                    self.P["Pcd1"]+= 0.25
                    self.P["Pcd2"]+= 0.25
                    self.P["Pbd1"]+= 0.25
                elif c.y == 4:
                    self.P["Pcd1"]+= 0.25
                    self.P["Pcd2"]+= 0.25
                    self.P["Pbc1"]+= 0.25
                    self.P["Pch1"]+= 0.25
                elif c.y == 5:
                    self.P["Pch1"] +=1
            elif c.x == 4:
                if c.y == 0:
                    self.P["Phh1"] +=1
                elif c.y == 1:
                    self.P["Phh1"] +=1
                elif c.y == 2:
                    self.P["Pch1"] +=1
                elif c.y == 3:
                    self.P["Pch1"] += 0.376
                    self.P["Pcd2"] += 0.301
                elif c.y == 4:
                    self.P["Pch1"] +=0.35
                    self.P["Pcd2"] += 0.65
                elif c.y == 5:
                    self.P["Pch1"] +=1
        


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
    def dijkstra(self, start_node, goal_char):
        """
        start_node: actual node object (e.g., self.A)
        goal_char: target node name (e.g., 'H')
        """

        # Priority queue: (cost, node)
        pq = []
        heapq.heappush(pq, (0, start_node))

        # Distance dictionary
        distances = {start_node.name: 0}

        # Previous node tracker (for path reconstruction)
        previous = {start_node.name: None}

        visited = set()

        while pq:
            current_cost, current_node = heapq.heappop(pq)

            if current_node.name in visited:
                continue
            visited.add(current_node.name)

            # Goal check
            if current_node.name == goal_char:
                break

            # Explore neighbors (N, E, S, W)
            neighbors = [
                (current_node.Nc, current_node.Times[0]),
                (current_node.Ec, current_node.Times[1]),
                (current_node.Sc, current_node.Times[2]),
                (current_node.Wc, current_node.Times[3]),
            ]

            for neighbor_char, cost in neighbors:
                if neighbor_char is None:
                    continue

                neighbor_node = self.returnNode(neighbor_char)
                if neighbor_node is None:
                    continue

                new_cost = current_cost + cost

                if (neighbor_char not in distances or 
                    new_cost < distances[neighbor_char]):

                    distances[neighbor_char] = new_cost
                    previous[neighbor_char] = current_node.name
                    heapq.heappush(pq, (new_cost, neighbor_node))

        #Reconstruct path
        path = []
        current = goal_char

        if current not in previous and current != start_node.name:
            return None, float('inf')  # No path

        while current is not None:
            path.append(current)
            current = previous.get(current)

        path.reverse()

        #where path is a list of node names (chars)
        #, cost - total cost of the path
        #if no path exists, will return None, float('inf')
        return path, distances.get(goal_char, float('inf'))

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
        safeNodes = allNodes.copy()
        for s in safeNodes:
            if s in unsafeNodes:
                safeNodes.remove(s)

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
            
        


        #A SAFE EDGE IS ANY EDGE TOUCHING A SAFE NODE, EVEN IF ITS COMING FROM AN UNSAFE NODE.
        # allEdges = ["Pab1", "Pab2", "Paf1", "Pae1", "Pbd1", "Pbc1", "Pcd1", "Pcd2", "Pch1", "Pfd1", "Peg1", "Peg2", "Pef1", "Pfg1","Pgh1", "Phh1"]
        # unsafeEdges = self.getNeighbourEdgesOf(enemyE)
        # unsafeEdges.append(enemyE)

        # for n in safeNodes:
        #     thisNode = self.returnNode(n)
        #     #check all of its neighbour nodes and add them to the list
        #     for safeE in self.getEdgesFromNode(thisNode):
        #         if safeE in unsafeEdges:
        #             unsafeEdges.remove(safeE)

        
        # safeEdges = allEdges - unsafeEdges

        #myEdges = self.getEdgesFromNode(self.current_node)


        
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
        safeNodes = allNodes.copy()
        for s in safeNodes:
            if s in unsafeNodes:
                safeNodes.remove(s)

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
        if self.current_node.Nc != enemyNCHAR:
            options.append(0)
        elif self.current_node.Ec != enemyNCHAR:
            options.append(1)
        elif self.current_node.Sc != enemyNCHAR:
            options.append(2)
        elif self.current_node.Wc != enemyNCHAR:
            options.append(3)
            
        if options:
            #choose random option and return that
            ranNum = random.randint(0,len(options) -1)
            return [options[ranNum]]

        #A SAFE EDGE IS ANY EDGE TOUCHING A SAFE NODE, EVEN IF ITS COMING FROM AN UNSAFE NODE.
        # allEdges = ["Pab1", "Pab2", "Paf1", "Pae1", "Pbd1", "Pbc1", "Pcd1", "Pcd2", "Pch1", "Pfd1", "Peg1", "Peg2", "Pef1", "Pfg1","Pgh1", "Phh1"]
        # unsafeEdges = self.getNeighbourEdgesOf(enemyE)
        # unsafeEdges.append(enemyE)

        # for n in safeNodes:
        #     thisNode = self.returnNode(n)
        #     #check all of its neighbour nodes and add them to the list
        #     for safeE in self.getEdgesFromNode(thisNode):
        #         if safeE in unsafeEdges:
        #             unsafeEdges.remove(safeE)

        
        # safeEdges = allEdges - unsafeEdges

        #myEdges = self.getEdgesFromNode(self.current_node)


        #we could also just return an empty list and say stay there and accept your doom, cus it prob means that we did not find a safe path out
        
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
        safeNodes = allNodes.copy()
        for s in safeNodes:
            if s in unsafeNodes:
                safeNodes.remove(s)

        #are we safe? then just wait here until the situation changes.
        if self.current_node.name in safeNodes:
            return []
            #a wait before next check on the receiving side of this function
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

        #as long as we are not going to THE enemy node, we might have a chance to survive.
        if self.current_node.Nc != enemyNCHAR:
            return [0]
        elif self.current_node.Ec != enemyNCHAR:
            return [1]
        elif self.current_node.Sc != enemyNCHAR:
            return [2]
        elif self.current_node.Wc != enemyNCHAR:
            return [3]
            

        #A SAFE EDGE IS ANY EDGE TOUCHING A SAFE NODE, EVEN IF ITS COMING FROM AN UNSAFE NODE.
        # allEdges = ["Pab1", "Pab2", "Paf1", "Pae1", "Pbd1", "Pbc1", "Pcd1", "Pcd2", "Pch1", "Pfd1", "Peg1", "Peg2", "Pef1", "Pfg1","Pgh1", "Phh1"]
        # unsafeEdges = self.getNeighbourEdgesOf(enemyE)
        # unsafeEdges.append(enemyE)

        # for n in safeNodes:
        #     thisNode = self.returnNode(n)
        #     #check all of its neighbour nodes and add them to the list
        #     for safeE in self.getEdgesFromNode(thisNode):
        #         if safeE in unsafeEdges:
        #             unsafeEdges.remove(safeE)

        
        # safeEdges = allEdges - unsafeEdges

        #myEdges = self.getEdgesFromNode(self.current_node)


        #if [] is returned, everything has failed somehow, must be an error
        
        self.get_logger().info(f"ERR: GenerateSafePathFromEnemyNode has failed. No path generated. Are all paths blocked?")
        return [] 


    def planDestination(self):
        CERTAINTY = 0.6 #threshold for us to definitely assume taht the opponent is at a particular location
        CONSIDER_NODES = 3 #if CERTAINTY threshold is not met, how many of the top probability nodes should we consider?
        choice = -1
        
        #define some preference algorithm.
        #use distances if you want (Nd, Ed, Sd, Wd)
        # behaviourMode settings:
        if self.behaviourMode == 1:
            # 1 - Simple Line Follower (using pink path)

            #self.patrolPath = ['A', 'F', 'G', 'E', 'F', 'D', 'C', 'H', 'H', 'G', 'E', 'A', 'B', 'C', 'D', 'B']
            
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
            # 4 - Avoidant
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
        if (not self.imu_turning and self.grayEntryTime < self.now - self.GRAY_COOLDOWN) :
            #if gray detected:

            if (self.isGray[0] > self.minPixels) or (self.isGray[1] > self.minPixels)  or (self.isGray[2] > self.minPixels):
                self.grayEntryTime = self.now
                self.get_logger().info("Intersection detected!")
                self.stopMov()
                self.stateFollow = False
                
                if self.resetBehaviour:
                    self.firstNode = True
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

                if type(self.current_destination) == list:
                    
                    #Update last node + current node
                    #self.current_node = self.current_destination
                    if(self.current_destination):

                        #last node
                        self.last_node = self.current_node

                        #current node
                        if type(self.current_destination[0]) == int:
                            if self.current_destination[0] == 0:
                                self.current_node = self.returnNode(self.current_node.Nc)
                            elif self.current_destination[0] == 1:
                                self.current_node = self.returnNode(self.current_node.Ec)
                            elif self.current_destination[0] == 2:
                                self.current_node = self.returnNode(self.current_node.Sc)
                            elif self.current_destination[0] == 3:
                                self.current_node = self.returnNode(self.current_node.Wc)

                        if type(self.current_destination[0]) == str:
                            self.current_node = self.returnNode(self.current_destination[0])
                    
                    #otherwise, do not update current_node as we are still where we were.
                    
                    #Update destionation
                    #self.current_destination = self. Function to Calculate Next Destination.

                    #populate our planned path if we don't already have a plan
                    #is it empty? => stay here and wait until our sensing cooldown has gone out.
                    if not self.current_destination:
                        self.dontSense = True
                        #when sensing cooldown expires, look again.
                        if self.senseEntryTime < self.now - self.SENSE_COOLDOWN:
                            self.stateFollow = False
                            self.planDestination()
                            
                            if not self.imu_turning:
                                self.stateFollow = True
                        else:
                            #cooldown has not expired yet. stay stopped and wait
                            self.stateFollow = False
                            self.dontSense = True
                            return
                    
                        self.dontSense = False

                        #if we got this far, we have directions to go somewhere
                        #self.current_destination has been set to either 1 directional number OR a LIST of directional numbers. 
                        #Check which one, if it is a list, we must iterate over it as it is a long path.
                        if type(self.current_destination) == list:
                            if(self.current_destination):
                                if type(self.current_destination[0]) == int:
                                    
                                    #if it is a single number from 0 to 3, then it is an immediate neighbour 
                                    #e.g. path = [2] i.e. go south
                                    self.imu_target = self.current_destination.pop(0)
                                    self.self_localise(self.current_node.Times[self.imu_target])
                                    
                                    self.toDepart = True
                                    self.startTurnBasedOnIMU()

                                    #update sweeping setting
                                    if (self.current_node.name == 'A' and self.current_destination[0] == 0) or (self.current_node.name == 'B' and self.current_destination[0] == 0) or (self.current_node.name == 'C' and self.current_destination[0] == 1) or (self.current_node.name == 'D' and self.current_destination[0] == 3) or (self.current_node.name == 'E' and self.current_destination[0] == 0) or (self.current_node.name == 'F' and self.current_destination[0] == 0) or (self.current_node.name == 'F' and self.current_destination[0] == 3) or (self.current_node.name == 'G' and self.current_destination[0] == 1) or (self.current_node.name == 'G' and self.current_destination[0] == 2) or (self.current_node.name == 'G' and self.current_destination[0] == 3) :
                                        #when left first is better than right first
                                        self.skipZero = True
                                    else:
                                        self.skipZero = False

                                elif type(self.current_destination[0]) == str:
                                    #if it is a char from A to H, then it is an immediate neighbour 
                                    #e.g. path = ['A', 'B'] 
                                    neigh_name = self.current_destination.pop(0)
                                    if self.current_node.Nc == neigh_name:
                                        self.imu_target = 0
                                        self.self_localise(self.current_node.Times[0])
                                    elif self.current_node.Ec == neigh_name:
                                        self.imu_target = 1
                                        self.self_localise(self.current_node.Times[1])
                                    elif self.current_node.Sc == neigh_name:
                                        self.imu_target = 2
                                        self.self_localise(self.current_node.Times[2])
                                    elif self.current_node.Wc == neigh_name:
                                        self.imu_target = 3
                                        self.self_localise(self.current_node.Times[3])

                                    #self.departureTime = self.now
                                    self.toDepart = True
                                    #self.startTurnBasedOnFacing()
                                    self.startTurnBasedOnIMU()

                                    #update sweeping setting
                                    if (self.current_node.name == 'A' and self.current_destination[0] == self.A.Nc) or (self.current_node.name == 'B' and self.current_destination[0] == self.B.Nc) or (self.current_node.name == 'C' and self.current_destination[0] == self.C.Ec) or (self.current_node.name == 'D' and self.current_destination[0] == self.D.Wc) or (self.current_node.name == 'E' and self.current_destination[0] == self.E.Nc) or (self.current_node.name == 'F' and self.current_destination[0] == self.F.Nc) or (self.current_node.name == 'F' and self.current_destination[0] == self.F.Wc) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Ec) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Sc) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Wc) :
                                        #when left first is better than right first
                                        self.skipZero = True
                                    else:
                                        self.skipZero = False

                            else: #empty destination
                                self.senseEntryTime = self.now
                                self.dontSense = True
                                self.stateFollow = False

                        else : #not list
                            
                            #then it is a char from A to H, and it is an immediate neighbour 
                            #e.g. path = 'A'
                            if self.current_node.Nc == self.current_destination:
                                self.imu_target = 0
                                self.self_localise(self.current_node.Times[0])
                            elif self.current_node.Ec == self.current_destination:
                                self.imu_target = 1
                                self.self_localise(self.current_node.Times[1])
                            elif self.current_node.Sc == self.current_destination:
                                self.imu_target = 2
                                self.self_localise(self.current_node.Times[2])
                            elif self.current_node.Wc == self.current_destination:
                                self.imu_target = 3
                                self.self_localise(self.current_node.Times[3])

                            self.toDepart = True
                            self.startTurnBasedOnIMU()

                            #update sweeping setting
                            if (self.current_node.name == 'A' and self.current_destination == self.A.Nc) or (self.current_node.name == 'B' and self.current_destination == self.B.Nc) or (self.current_node.name == 'C' and self.current_destination == self.C.Ec) or (self.current_node.name == 'D' and self.current_destination == self.D.Wc) or (self.current_node.name == 'E' and self.current_destination == self.E.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Wc) or (self.current_node.name == 'G' and self.current_destination == self.G.Ec) or (self.current_node.name == 'G' and self.current_destination == self.G.Sc) or (self.current_node.name == 'G' and self.current_destination == self.G.Wc) :
                                #when left first is better than right first
                                self.skipZero = True
                            else:
                                self.skipZero = False

                else : #not list
                    #update current_node and last_node
                    if not self.firstNode:
                        self.last_node = self.current_node
                        self.current_node = self.returnNode(self.current_destination)
                    else:
                        self.firstNode = False
                    
                    self.planDestination()
                    
                    if not self.imu_turning:
                        self.stateFollow = True
                    
                    #then it is a char from A to H, and it is an immediate neighbour 
                    #e.g. path = 'A'
                    if self.current_node.Nc == self.current_destination:
                        self.imu_target = 0
                        self.self_localise(self.current_node.Times[0])
                    elif self.current_node.Ec == self.current_destination:
                        self.imu_target = 1
                        self.self_localise(self.current_node.Times[1])
                    elif self.current_node.Sc == self.current_destination:
                        self.imu_target = 2
                        self.self_localise(self.current_node.Times[2])
                    elif self.current_node.Wc == self.current_destination:
                        self.imu_target = 3
                        self.self_localise(self.current_node.Times[3])

                    #update departure time
                    #self.departureTime = self.now 
                    self.toDepart = True
                    #a value will  be added to departure time to represent turning time needed in startTurnBasedOnFacing

                    if self.behaviourMode != 1:
                        #skip this when patrolling as we are most probably already aligned
                        self.startTurnBasedOnIMU()

                    #update sweeping setting
                    if (self.current_node.name == 'A' and self.current_destination == self.A.Nc) or (self.current_node.name == 'B' and self.current_destination == self.B.Nc) or (self.current_node.name == 'C' and self.current_destination == self.C.Ec) or (self.current_node.name == 'D' and self.current_destination == self.D.Wc) or (self.current_node.name == 'E' and self.current_destination == self.E.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Wc) or (self.current_node.name == 'G' and self.current_destination == self.G.Ec) or (self.current_node.name == 'G' and self.current_destination == self.G.Sc) or (self.current_node.name == 'G' and self.current_destination == self.G.Wc) :
                        #when left first is better than right first
                        self.skipZero = True
                    else:
                        self.skipZero = False
        
                self.get_logger().info(f"Current Location:{self.current_node.name}; Current Destination: {self.current_destination}")

    def surveillCapture(self):
        #ultrasonic_sweep.py is constantly turning and checking.
        #self.CAPTURE_MAX =0.1
        #ultrasonic is probably measuring in metres (m)
        #~10cm is the maximum distance for capture in tight spaces of the map
        
        #TAG
        if self.ultrasonic_distance < self.CAPTURE_MAX and (self.now > self.time_of_last_tag + self.TAG_COOLDOWN):
            self.initiated_tag = True
            self.get_logger().info("Initiating tag...")
        else:
            self.initiated_tag = False #don't let it keep going 

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
            else:
                #resolve destination

                #is path blocked?
                if min(self.entry_angle, self.exit_angle) < 90 < max(self.entry_angle, self.exit_angle):
                    #turn 180 and go back to where you were
                    if self.facing == 0:
                        self.imu_target = 2
                    elif self.facing == 2:
                        self.imu_target = 0
                    elif self.facing == 1:
                        self.imu_target = 3
                    elif self.facing == 3:
                        self.imu_target = 1
                    
                    
                    #self.startTurnBasedOnFacing()
                    self.startTurnBasedOnIMU()
                    self.current_destination = self.current_node

            #otherwise just keep following the line to your intended destination to resolve your location, then restart process from there.
            if type(self.current_destination) == list:
                self.current_destination = self.current_destination[0]

            self.resetBehaviour = True
            self.initial_reading_taken = False
            self.initiated_sweep = False

            #flip status
            self.evading = not self.evading

            self.get_logger().info(f"TAG! New Mode: {self.behaviourMode}; Ev?: {self.evading}")



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


    #This segment was generated using Claude Sonnet 4.6 Adaptive on 05/05/2026
    # https://claude.ai/share/b3ef1b16-b390-47b7-9d8a-b0121ba56ef1 
    # though it mostly has stripped my own code and regurgitated it to me in a different placement
    # Post-retry replanning: once the retry turn AND sweep are both done, replan
    # without advancing current_node (robot never moved — it turned in place)
    def retry(self):
        self.postRetry = False

        self.checkUltra()

        if self.retryPlan != 0:
            # Still can't plan — set up another retry turn
            self.stopMov()
            if self.retryPlan == -1:
                targets = {0: 2, 2: 0, 1: 3, 3: 1}
            elif self.retryPlan == -2:
                targets = {0: 1, 2: 3, 1: 2, 3: 0}
            elif self.retryPlan == -3:
                targets = {0: 3, 2: 1, 1: 0, 3: 2}
            else:
                targets = {}
            self.imu_target = targets.get(self.facing, -1)
            self.startTurnBasedOnIMU()
            # retryPlan != 0 → next loop tick will trigger another sweep
        # else:
        #     # planDestination succeeded — apply the new destination
        #     dest = self.current_destination
        #     if isinstance(dest, str):
        #         dirs = [self.current_node.Nc, self.current_node.Ec,
        #                 self.current_node.Sc, self.current_node.Wc]
        #         self.imu_target = dirs.index(dest) if dest in dirs else -1
        #     elif isinstance(dest, list) and dest:
        #         if isinstance(dest[0], int):
        #             self.imu_target = dest[0]
        #         elif isinstance(dest[0], str):
        #             dirs = [self.current_node.Nc, self.current_node.Ec,
        #                     self.current_node.Sc, self.current_node.Wc]
        #             self.imu_target = dirs.index(dest[0]) if dest[0] in dirs else -1
        #     self.toDepart = True
        #     if self.behaviourMode != 1:
        #         self.startTurnBasedOnIMU()



    
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

        if (self.now > self.senseEntryTime + self.SENSE_COOLDOWN) and self.dontSense and self.current_destination == []:
            self.dontSense = False #consume
            self.triggerSweep = True

        #Ultrasonic Sweep Modes
        if (self.triggerSweep or self.retryPlan != 0 or (self.behaviourMode in [3,4,5] and not self.initial_reading_taken)) and not self.waitingForUltrasonic:
            #trigger single sweep
            self.sweep = True
            self.multiple = False
            self.waitingForUltrasonic = True
            self.publish_sweep_command()

            if self.retryPlan != 0:
                self.postRetry = True

            #consume
            self.retryPlan = 0
            self.triggerSweep = False
            self.initial_reading_taken = True
        

        #specifically when retrying
        if self.postRetry and not self.imu_turning and not self.waitingForUltrasonic:
            self.retry()


        #check for tags and publish status
        self.surveillCapture()
        self.publish_tag_status()

        if self.retryPlan != 0 or self.paused or self.dontSense or self.imu_turning or (self.behaviourMode in [3,4,5] and not self.initial_reading_taken):
            pass
        elif not self.waitingForUltrasonic:
            #check for intersection, reset behaviour from tag, update location and destination and target tracking
            if self.behaviourMode in [3,4,5]:
                self.checkUltra()

            if self.retryPlan == 0:
                self.updatePos() #gated by self.imu_turning and by GRAY_COOLDOWN

        
        if self.retryPlan != 0 or self.postRetry or self.paused or self.current_destination == [] or self.imu_turning or self.dontSense or self.waitingForUltrasonic:
            self.stateFollow = False

            #Stop sweep.
            if (self.sweep or self.multiple) and not self.waitingForUltrasonic:
                self.sweep = False
                self.multiple = False
                self.publish_sweep_command()

        else:
            self.stateFollow = True
        

        if self.stateFollow:
            #simple line following
            self.followLine()

            #Tag Sweep
            if not self.sweep or not self.multiple:
                self.sweep = True
                self.multiple = True
                self.publish_sweep_command()

        

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
