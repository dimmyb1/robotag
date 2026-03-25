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
import time

class Noden():
    def __init__(self):
        self.Nd = 0
        self.Nc = 'Z'
        self.Ed = 0
        self.Ec = 'Z'
        self.Sd = 0
        self.Sc = 'Z'
        self.Wd = 0
        self.Wc = 'Z'
        self.name = 'Z'
        


class line_follower(Node):
    def __init__(self):
        super().__init__('line_follower')

        robot_name = self.get_namespace().strip('/')

        if not robot_name:
            robot_name = 'generic'

        topic_L = f'/{robot_name}_ir_L/image_raw'
        topic_M = f'/{robot_name}_ir_M/image_raw'
        topic_R = f'/{robot_name}_ir_R/image_raw'

        # Array to store the 3 read sensors
        self.colours = [0,0,0]
        self.isGray = [0,0,0]
        # Speed for every movement except right turning 
        self.speed = 60
        # Counter to hold which angles we have searched so that the robot does not move random angles used in the switch case
        self.searchStep = 0
        self.dur = 0
        self.elapsed = 0
        self.searchLeft = False
        self.searchRight = False
        self.searching = False
        self.minPixels = 20

        # ints that will be used throughout
        self.realDelay = 150

        # Amount to move for angles
        self.thirty = 2933

        self.motion_active = False
        self.motion_end_time = 0
        self.current_motion = None

        # Graph
        self.A = Noden(78, 11, 61, 140, 'B', 'B', 'F', 'E', 'A')
        self.B = Noden(106, 14, 29, 11, 'A', 'C', 'D', 'A', 'B')
        self.C = Noden(155, 49, 10, 15, 'H', 'D', 'D', 'B', 'C')
        self.D = Noden(13, 59, 40, 46, 'C', 'C', 'F', 'B', 'D')
        self.E =Noden(128, 12, 49, 90, 'A', 'F', 'G', 'G', 'E')
        self.F =Noden(111, 70, 11, 34, 'A', 'D', 'G', 'E', 'F')
        self.G =Noden(9, 12, 109, 34, 'F', 'H', 'E','E', 'G')
        self.H =Noden(130, 85, 89, 14, 'C', 'H', 'H', 'G', 'H')

        self.current_node = 'A'
        self.current_destination = 'A'
        self.skipZero = False
        self.dummy = 1
        self.behaviourMode = 0
        # behaviourMode settings:
        # 0 - not set (no behaviour)
        # 1 - Simple Line Follower
        # 2 - Random
        # 3 - Greedy
        # 4 - Avoidant
        # 5 - Interceptive
        # 6 - Trap Layer

        # Subscriptions
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

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()

        self.timer = self.create_timer(0.05, self.loop)


    #Graph Functions
    def traverseGraph(self):
        #define some preference algorithm.
        #use distances if you want (Nd, Ed, Sd, Wd)
        self.dummy = 1

    def updatePos(self):
        #update current variables
        #if gray detected (implement):
        if (self.isGray[0] > self.minPixels) or (self.isGray[1] > self.minPixels)  or (self.isGray[2] > self.minPixels):
            self.current_node = self.current_destination
        #   self.current_destination = self. Function to Calculate Next Destination.

        #update sweeping setting
        if (self.current_node == 'A' and self.current_destination == self.A.Nc) or (self.current_node == 'B' and self.current_destination == self.B.Nc) or (self.current_node == 'C' and self.current_destination == self.C.Ec) or (self.current_node == 'E' and self.current_destination == self.E.Nc):
            #special case
            self.skipZero = True
        elif(self.current_node in ['F', 'G']):
            self.skipZero = True
        else:
            self.skipZero = False
        

    #Line Following Functions

    def detect_black(self, img):
        # returns a mask of black pixels in the image
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([26, 26, 26])
        mask = cv2.inRange(img, lower_black, upper_black)
        return cv2.countNonZero(mask) #(int) num of black pixels in img
    
    def detect_gray(self, img):
        # returns a mask of black pixels in the image
        lower_gray = np.array([85, 85, 85])
        upper_gray = np.array([128, 128, 128])
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
            self.motion_end_time = time.time() + (duration_ms / 1000.0)
        else:
            self.motion_active = False #it will run continuously

    def update_motion(self):
        if self.motion_active and time.time() >= self.motion_end_time:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)
            self.motion_active = False

    def stopMov(self) :
        self.start_motion()
        
    # --------------------------
    # Basic Turning Functions
    # --------------------------

    def turnRight(self, duration):
        self.start_motion(angular=-0.75, duration_ms=duration)

    def turnLeft(self, duration) :
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
                
                return True
            
            # Continue turning right
            self.turnRight(0)
            
            # Increment the steps
            self.elapsed += stepDelay
            return False # exit and wait for next tick
            
        return False # finished full arc


    def smartTurnLeft(self, totalDuration,stepDelay = 50) :
        # While the angle is less then the expected turn
        if (self.elapsed < totalDuration):
            
            # Check if there is a black line in one of the sensors if so return to continue executing
            if (self.minPixels < self.colours[1]) or (self.minPixels < self.colours[0]) or (self.minPixels < self.colours[2]):
                self.get_logger().info("Line found during left turn!")
                self.stopMov()
                self.searchLeft = False
                return True
            
            # otherwise continue turning left
            self.turnLeft(0)

            # Increment the steps
            self.elapsed += stepDelay
            return False
        
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
            self.get_logger().info(f"STEP {self.searchStep}")

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

      
    def loop(self):
        self.update_motion()

        if not self.motion_active:
            self.followLine() 

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