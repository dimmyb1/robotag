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
        self.thirty = 1066
        
        # rightOneEighty = 3000 # Not tested (not used)
        # fullRot = realDelay*39 # Not tested (not used)

        self.motion_active = False
        self.motion_end_time = 0
        self.current_motion = None

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

    def detect_black(self, hsv):
        # returns a mask of black pixels in the image
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([26, 26, 26])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        return cv2.countNonZero(mask) #(int) num of black pixels in img
    
    def ir_L_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        self.colours[0] = self.detect_black(img)

    def ir_M_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        self.colours[1] = self.detect_black(img)

    def ir_R_callback(self, msg):
        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        self.colours[2] = self.detect_black(img)

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
        self.start_motion(angular=-0.5, duration_ms=duration)

    def turnLeft(self, duration) :
        self.start_motion(angular=0.5, duration_ms=duration)


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
            
        return False


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
        

    def search(self):
        if self.searchStep == 0:
            self.searchRight = True
            self.dur = self.thirty

        elif self.searchStep == 1:
            self.searchLeft = True
            self.dur = self.thirty * 3

        elif self.searchStep == 2:
            self.searchRight = True
            self.dur = self.thirty *5

        elif self.searchStep == 3:
            self.searchLeft = True
            self.dur = self.thirty * 6

        elif self.searchStep == 4:
            self.searchLeft = True
            self.dur = self.thirty * 3

        elif self.searchStep ==5:
            self.searchLeft = True
            self.dur = self.thirty * 6

        else:
            #reverse and restart search
            self.start_motion(linear=-0.25, duration_ms=self.realDelay)
            self.searchStep = 0
            self.searching = False
            return

        self.handleSearchLoop()

        

    def handleSearchLoop(self):
        #start condition
        if not self.searching:
            self.elapsed = 0
            self.searching = True

        #carry out turn
        if self.searchLeft:
            self.found = self.smartTurnLeft(self.dur)
            self.searchLeft = False
        
        elif self.searchRight:
            self.found = self.smartTurnRight(self.dur)
            self.searchRight = False

        #end conditions
        if(self.found):
            self.searchStep = 0
            self.searching = False
            self.elapsed = 0
        elif(self.elapsed>=self.dur):
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
            self.start_motion(linear=0.25)

        elif self.minPixels < L:
            self.turnLeft(self.realDelay)

        elif self.minPixels < R:
            self.turnRight(self.realDelay)

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