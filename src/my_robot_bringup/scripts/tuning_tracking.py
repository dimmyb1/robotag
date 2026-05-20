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
    def __init__(self, nc, ec, sc, wc, n, t, s):
        #dir_c -> where dir can be N, E, S, W and c just means 'char' so you would have 'A', 'B' .. etc 'A' - 'H' representing the node name
        self.Nc = nc
        self.Ec = ec
        self.Sc = sc
        self.Wc = wc

        #.name -> the name of this node as a char : 'A' - 'H'
        self.name = n

        #.Times -> [1.0, 23.0, 14.5, 16.8]  where Times[0] -> North transition time cost = 1.0
        self.Times = t
        self.SDs = s

     
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
        self.safetyStop = False

        #junction turning vars
        self.stateFollow = True
        #self.completeTurn = False
        self.wasLeft = False
        self.imu_turning = False
        self.imu_target = -1
        self.grayEntryTime = -1
        self.GRAY_COOLDOWN = 10.1 #DOC V
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
        #look for the paper dated 8 may for a discussion on the tuning of capture_max DOC V
        self.CAPTURE_MAX = 0.12 #was (10cm, 0.1m), but was changed to 11.5cm to try to prevent any damage as the robots were bumping into each other, #had to raise to 12 because in Pch1 when they face each other, that's the distance.
        self.PAUSE_TIME = 95 #was 6, but seemed low so changed to 7. 7 seems low, raising to 14 to avoid immediate tag-backs #14 is too low because we've extended the number of things that happens on tag. raising to 35. 
        #   35 was also quite low, it was enough for the opponent to finish turning 180, but that's it, therefore i'll add another 20 seconds to allow the opp to manouvre.
        #   55 still wasn't enough, adding another 20 seconds.
        #   75 is perfectly timed for the other robot finish moving, but i think it would be safer if there's a few more seconds gap, adding 20 more
        self.startPauseTime = -1
        self.paused = False
        self.time_of_last_tag = -1
        self.TAG_COOLDOWN = 100 #6 seconds way too short. upping to 60 seconds.letting it be a bit more than pause_time
        self.tag = False
        self.ack = False
        self.other_tag = False
        self.other_ack = False
        self.initiated_tag = False
        self.doTag = False
        self.movBackCosTag = False

        #ultrasonic sensor and servo vars
        self.entry_angle = float('inf')
        self.exit_angle = float('inf')
        self.ultrasonic_distance = float('inf')
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
        #   id = Noden( Nc,  Ec,  Sc,  Wc, name,       Times        ,        SDs      )
        self.A = Noden('B', 'B', 'F', 'E', 'A', [112,  22,  83, 249], [29,  7, 31, 66])
        self.B = Noden('A', 'C', 'D', 'A', 'B', [170,  20,  49,  24], [59,  7, 12, 16])
        self.C = Noden('H', 'D', 'D', 'B', 'C', [219,  91,  18,  20], [53, 22,  8,  7])
        self.D = Noden('C', 'C', 'F', 'B', 'D', [ 14,  77,  48,  58], [ 6, 29,  5, 11])
        self.E = Noden('A', 'F', 'G', 'G', 'E', [230,  20, 115, 202], [74,  7, 87, 73])
        self.F = Noden('A', 'D', 'G', 'E', 'F', [ 96,  64,  17,  33], [30, 34,  8, 10])
        self.G = Noden('F', 'H', 'E', 'E', 'G', [ 16,  17, 182,  78], [ 6,  5, 54, 34])
        self.H = Noden('C', 'H', 'H', 'G', 'H', [206, 122, 158,  20], [58, 34, 87,  7])

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
        self.behaviourMode = 0 #on start, what are you behaving as
        self.otherMode = 0 #on tag, what do you change to
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
            self.otherMode = 1
            self.evading = False
        elif robot_name == 'twirl':
            self.current_node = self.A
            self.get_logger().info("Detected robot: twirl. Starting at Node A.")
            other_robot_name = 'twix'
            self.behaviourMode = 1
            self.otherMode = 3
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
        
        self.get_logger().info(f"SUB: Entry: {self.entry_angle:.2f}, Exit: {self.exit_angle:.2f}, Dist: {self.ultrasonic_distance:.2f}")

    def publish_sweep_command(self):
        payload = {
            "sweep": self.sweep,
            "multiple": self.multiple
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.sweep_pub.publish(msg)
        self.get_logger().info(f"SWEEP PUBLISHED: sweep={self.sweep}, multiple={self.multiple}")

   

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

            #Intersection-related
            if self.crawlingBackwards:
                self.crawlingBackwards = False

            if self.crawlBackBeforeIMUturn:
                self.crawlBackBeforeIMUturn = False #consume
                self.get_logger().info("finished going a tiny bit back, now starting IMU turn.")
                self.startTurnBasedOnIMU()

            #Tag-related
            if self.movBackCosTag:
                self.movBackCosTag = False
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
            self.joltBack()
            self.crawlBackBeforeIMUturn = True
            
        elif(self.imu_turning):

            target_yaw = {0: 0, 1: 270, 2: 180, 3: 90}
            target = target_yaw[self.imu_target]

            if target < 0:
                target+=360

            if target < self.ANGLE_TOLERANCE:
                if (360+target) - self.ANGLE_TOLERANCE <= self.yaw_deg or self.yaw_deg <= target + self.ANGLE_TOLERANCE:
                    #self.get_logger().info(f"in update_motion: target < 4 deg, toDep: {self.toDepart}")
                    #we have completed our turn.
                    self.imu_turning = False
                    self.imu_target = -1
                    self.stopMov()
                    self.retryPlan = 0

                    if self.toDepart:
                        self.toDepart = False #consume
                        self.departureTime = self.now
                        self.grayEntryTime = self.now #safety precaution
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
                        self.grayEntryTime = self.now #safety precaution
                        self.stateFollow = True
 
        if self.waitingForUltrasonic:
            self.stopMov()

    def stopMov(self) :
        self.start_motion()
        
    

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
        if fromNCHAR == 'A': return ["Pab2", "Pab1", "Paf1", "Pae1"]
        if fromNCHAR == 'B': return ["Pab2", "Pbc1", "Pbd1", "Pab1"]
        if fromNCHAR == 'C': return ["Pch1", "Pcd2", "Pcd1", "Pbc1"]
        if fromNCHAR == 'D': return ["Pcd1", "Pcd2", "Pdf1", "Pbd1"]
        if fromNCHAR == 'E': return ["Pae1", "Pef1", "Peg1", "Peg2"]
        if fromNCHAR == 'F': return ["Paf1", "Pdf1", "Pfg1", "Pef1"]
        if fromNCHAR == 'G': return ["Pfg1", "Pgh1", "Peg2", "Peg1"]
        if fromNCHAR == 'H': return ["Pch1", "Phh1", "Phh1", "Pgh1"]

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
                                if x - ix > -1:
                                    servoCells.append(Cell(x - ix, iy))
                        else:
                            for ix in range(diffX):
                                if x - ix > -1:
                                    servoCells.append(Cell(x - ix, iy))

                        iy-=1
                            
                    #now make the smaller triangle
                    diffY = -1 * math.ceil(diffX * math.tan(biggerServo))

                    iy = y -1
                    while(iy>-1):
                        if iy >= math.ceil(diffY / 2):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if x - ix > -1:
                                    ceilCells.append(Cell(x - ix, iy))
                        else:
                            for ix in range(diffX):
                                if x - ix > -1:
                                    ceilCells.append(Cell(x - ix, iy))

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
                            for iy in range(diffY):
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
                            for iy in range(diffY):
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
                                if x + ix < 5:
                                    servoCells.append(Cell(ix + x, iy))
                        else:
                            for ix in range(diffX):
                                if x + ix < 5:
                                    servoCells.append(Cell(ix + x, iy))

                        iy-=1
                            
                    #now make the smaller triangle
                    diffY = math.ceil(diffX * math.tan(lesserServo))

                    iy = y -1
                    while(iy>-1):
                        if iy >= math.ceil(diffY / 2):
                            #we need to do the diffY/2 ones at this ix value
                            for ix in range(math.ceil(diffX/2)):
                                if x + ix < 5:
                                    ceilCells.append(Cell(ix + x, iy))
                        else:
                            for ix in range(diffX):
                                if x + ix < 5:
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
                            for iy in range(diffY):
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
                            for iy in range(diffY):
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



    def planDestination(self):
        CERTAINTY = 0.6 #threshold for us to definitely assume taht the opponent is at a particular location
        CONSIDER_NODES = 3 #if CERTAINTY threshold is not met, how many of the top probability nodes should we consider?
        
        #FOR DEBUGGING:
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

        self.get_logger().info(f"maxV edge: {maxK} with {maxV}. CERTAINTY is {CERTAINTY}")
            

        if(maxV >= CERTAINTY):
            self.get_logger().info(f"Target Location: {maxK}")
        else:
            #find top CONSIDER_NODES (int) max valued edge-probabilities
            topProb = sorted(self.P.items(), key=lambda x: x[1], reverse=True)[:8]
            #returns smth like [('Pbd1', 0.56), ('Pbc1', 0.33), ('Pcd2', 0.10)]

            self.get_logger().info(f"CONSIDER is {CONSIDER_NODES}")
            self.get_logger().info(f"sortedProb is: {topProb}")
            
        
        


            
    def updatePos(self):
        #when sensing cooldown expires, look again.
        self.planDestination()


    def loop(self):
        self.now = time.time()
        if self.ultrasonic_distance != float('inf') and self.entry_angle != float('inf'):
            self.updatePos()
        if self.senseEntryTime + self.SENSE_COOLDOWN > self.now:
            self.senseEntryTime = self.now
            self.sweep = True
            self.multiple = False
            self.publish_sweep_command()
            self.ultrasonic_distance = float('inf')


        
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
