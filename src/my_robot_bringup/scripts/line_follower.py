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
        # Array to store the 3 read sensors
        self.colours = [0,0,0]
        # Speed for every movement except right turning 
        self.speed = 60
        # Counter to hold which angles we have searched so that the robot does not move random angles used in the switch case
        self.searchStep = 0

        self.minPixels = 20

        # ints that will be used throughout
        self.realDelay = 150
        # Amount to move for angles
        self.rightThirty = 500
        self.leftThirty = 500
        self.leftSixty = 1200
        self.rightSixty = 1000
        self.leftNinety = 1600
        self.rightNinety = 1400
        self.leftOneEighty = 3200
        # rightOneEighty = 3000 # Not tested (not used)
        # fullRot = realDelay*39 # Not tested (not used)
        self.motion_active = False
        self.motion_end_time = 0
        self.current_motion = None

        # Subscriptions
        self.ir_L_sub = self.create_subscription(
            Image,
            'ir/image_raw',
            self.ir_L_callback,
            1
        )
        self.ir_M_sub = self.create_subscription(
            Image,
            'ir/image_raw',
            self.ir_M_callback,
            1
        )
        self.ir_R_sub = self.create_subscription(
            Image,
            'ir/image_raw',
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

        self.motion_active = True
        self.motion_end_time = time.time() + (duration_ms / 1000.0)

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
        self.start_motion(angular=-1.0, duration_ms=duration)

    def turnLeft(self, duration) :
        self.start_motion(angular=1.0, duration_ms=duration)


    def smartTurnRight(self, totalDuration,stepDelay = 100) :
        # Let the counter be equal to 0
        elapsed = 0
        # While the angle is less then the expected turn
        while (elapsed < totalDuration):
            # Continue turning right
            self.turnRight(stepDelay)

            # Check if there is a black line in one of the sensors if so return to continue executing
            if (self.blackMin < self.colours[1] < self.blackMax) or  (self.blackMin < self.colours[0] < self.blackMax) or (self.blackMin < self.colours[2] < self.blackMax) :
                self.get_logger().info("Line found during right turn!")
                return True
            
            # Increment the steps
            elapsed += stepDelay
            
        return False


    def smartTurnLeft(self, totalDuration,stepDelay = 100) :
        # Let the counter be equal to 0
        elapsed = 0
        # While the angle is less then the expected turn
        while (elapsed < totalDuration):
            # Continue turning left
            self.turnLeft(stepDelay)

            # Check if there is a black line in one of the sensors if so return to continue executing
            if (self.blackMin < self.colours[1] < self.blackMax) or (self.blackMin < self.colours[0] < self.blackMax) or (self.blackMin < self.colours[2] < self.blackMax):
                self.get_logger().info("Line found during left turn!")
                return True
            
            # Increment the steps
            elapsed += stepDelay
        
        return False
        

    def loop(self):
        self.update_motion()

        if not self.motion_active:
            self.followLine()    
    
    def moveForwardWhileOnTrack(self):
        self.start_motion(linear=1.0)
        

    def search(self):
        if self.searchStep == 0:
            self.found = self.smartTurnRight(self.rightThirty)

        elif self.searchStep == 1:
            self.turnLeft(self.leftThirty)
            self.found = self.smartTurnRight(self.leftSixty)

        elif self.searchStep == 2:
            self.turnRight(self.rightSixty)
            self.found = self.smartTurnRight(self.rightNinety)

        elif self.searchStep == 3:
            self.turnLeft(self.leftNinety)
            self.found = self.smartTurnLeft(self.leftNinety)

        elif self.searchStep == 4:
            self.found = self.smartTurnLeft(self.leftNinety)

        elif self.searchStep ==5:
            self.found = self.smartTurnLeft(self.leftOneEighty)

        else:
            #reverse and restart search
            self.start_motion(linear=-1.0, duration_ms=self.realDelay)
            self.searchStep = 0
            return

        if(self.found):
            self.searchStep = 0
        else:
            self.searchStep+=1

    def followLine(self):

        L = self.colours[0]
        M = self.colours[1]
        R = self.colours[2]

        if self.minPixels < M :
            self.moveForwardWhileOnTrack()

        elif self.minPixels < L:
            self.turnLeft(self.realDelay)

        elif self.minPixels < R:
            self.turnRight(self.realDelay)

        else:
            self.search()

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