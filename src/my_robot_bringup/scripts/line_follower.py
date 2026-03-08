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

class line_follower():
    def __init__(self):
        # Array to store the 3 read sensors
        colours = [0,0,0]
        # Speed for every movement except right turning 
        speed = 60
        # Counter to hold which angles we have searched so that the robot does not move random angles used in the switch case
        searchStep = 0

        # Calibrated black colour minimum and maximum
        # These might be changed for the actual demo during preperation time
        blackMax = 900
        blackMin = 500

        # ants that will be used throughout
        realDelay = 150
        # Ammount to move for angles
        rightThirty = 500
        leftThirty = 500
        leftSixty = 1200
        rightSixty = 1000
        leftNinety = 1600
        rightNinety = 1400
        leftOneEighty = 3200
        # rightOneEighty = 3000 # Not tested (not used)
        # fullRot = realDelay*39 # Not tested (not used)

        # Subscriptions
        # Front camera for line-following
        self.cam_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.front_callback,
            1
        )

        # Bottom-middle - bottom intersection arrival detection
        self.ir_L_sub = self.create_subscription(
            Image,
            '/ir/image_raw',
            self.bm_callback,
            10
        )
        self.ir_M_sub = self.create_subscription(
            Image,
            '/ir/image_raw',
            self.bm_callback,
            10
        )
        self.ir_R_sub = self.create_subscription(
            Image,
            '/ir/image_raw',
            self.bm_callback,
            10
        )

        # Left - intersection / left line
        self.lidar_sub = self.create_subscription(
            Image,
            '/sensor_msgs/LaserScan',
            self.bl_callback,
            10
        )

        # Right - intersection / right line
        self.esp_sub = self.create_subscription(
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


    # --------------------------
    # Motor Control Functions
    # --------------------------
    def moveFwd(self, speed) :
        self.cmd.linear.x = speed
        self.publisher.publish(self.cmd)

    def moveRev(self, speed) :
        self.cmd.linear.x = - speed
        self.publisher.publish(self.cmd)

    def stopMov(self) :
        self.cmd.angular.z = 0.0
        self.cmd.linear.x = 0.0
        self.publisher.publish(self.cmd)

    def delay(self, dummyVar) :
        #create a timer
        dummyVar+=0

    # --------------------------
    # Basic Turning Functions
    # --------------------------

    def turnRight(self, speed, realDelay) :
        speed = 70
        self.cmd.angular.z = speed
        self.delay(realDelay)
        self.stopMov()
        speed = 60

    def turnLeft(self, speed, realDelay) :
        self.cmd.angular.z = speed
        self.delay(realDelay)
        self.stopMov()


    def smartTurnRight(self, totalDuration,stepDelay = 100) :
        # Let the counter be equal to 0
        elapsed = 0
        # While the angle is less then the expected turn
        while (elapsed < totalDuration):
            # Continue turning right
            turnRight(stepDelay)

            # Read sensor again
            colours[0] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L()
            colours[1] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M()
            colours[2] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R()

            # Check if there is a black line in one of the sensors if so return to continue executing
            if (blackMin < colours[1] < blackMax) or  (blackMin < colours[0] < blackMax) or (blackMin < colours[2] < blackMax) :
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
            turnLeft(stepDelay)

            # Read sensor again
            colours[0] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L()
            colours[1] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M()
            colours[2] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R()

            # Check if there is a black line in one of the sensors if so return to continue executing
            if (blackMin < colours[1] < blackMax) or (blackMin < colours[0] < blackMax) or (blackMin < colours[2] < blackMax):
                self.get_logger().info("Line found during left turn!")
                return True
            
            # Increment the steps
            elapsed += stepDelay
        
        return False
        


    def setup(self) :
        Serial.begin(9600)
        #AppITR20001.DeviceDriverSet_ITR20001_Init()


    def loop(self) :
        self.get_logger().info("ITR20001_getAnaloguexxx_L=")
        colours[0] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L()
        self.get_logger().info(colours[0])
        self.get_logger().info("ITR20001_getAnaloguexxx_M=")
        colours[1] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M()
        self.get_logger().info(colours[1])
        self.get_logger().info("ITR20001_getAnaloguexxx_R=")
        colours[2] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R()
        self.get_logger().info(colours[2])

        # Call the follow line function
        followLine()


    def moveForwardWhileOnTrack(self) :
        # Move foward
        rightFwd()
        leftFwd()
        # Scan
        while (True):
            colours[0] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L()
            colours[1] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M()
            colours[2] = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R()

            # If the middle is not black
            if (not (blackMin < colours[1] < blackMax)) :
                # Stop moving and return
                stopMov()
                break
            
            delay(10) # micro-loop
        


    def followLine(self):
        #colours[] -> 0:L, 1:M, 2:R
        if(blackMin < colours[1] < blackMax): #if M==black, move forward
            moveForwardWhileOnTrack()
        elif (blackMin < colours[0] < blackMax) :#if L==black, turn a bit to the left to correct
            turnLeft(realDelay)
        elif (blackMin < colours[2] < blackMax):#if R==black, turn a bit to the right to correct
            turnRight(realDelay)
        #if nowhere is black, try to find the path again.
        else:
            found = False
            if searchStep == 0:
                self.get_logger().info("Search Step 0: Turn right 30")
                # Turn 30 right whilst searching 
                found = smartTurnRight(rightThirty)
            elif searchStep == 1: 
                self.get_logger().info("Search Step 1: Turn left 60")
                # Turn 30 left to reset
                turnLeft(leftThirty)
                # Turn 60 left whilst searching 
                found = smartTurnLeft(leftSixty)
                    
            elif searchStep == 2:
                self.get_logger().info("Search Step 2: Turn right 90")
                # Turn 60 right to reset
                turnRight(rightSixty)
                # Turn 90 right whilst searching 
                found = smartTurnRight(rightNinety)
                    
            elif searchStep == 3:
                self.get_logger().info("Search Step 3: Turn left 90")
                # Turn 90 left to reset
                turnLeft(leftNinety)
                # Turn 90 left whilst searching 
                found = smartTurnLeft(leftNinety)
                    
            elif searchStep == 4:
                self.get_logger().info("Search Step 4: Turn left 180")
                # Turn 180 left whilst searching - only 90 left to turn 180
                found = smartTurnLeft(leftNinety)
                    
            elif searchStep == 5:
                self.get_logger().info("Search Step 5: Turn left 360")
                # Turn a whole turn whilst searching 
                found = smartTurnLeft(leftOneEighty)
                    
            else:
                self.get_logger().info("Search failed. Reversing...")
                # Reverse and restart the loop
                rightRev()
                leftRev()
                delay(realDelay)
                stopMov()
                searchStep = 0
                    
                
            if (found) :
                # reset search if line found
                searchStep = 0
            else:
                # continue search if not found
                searchStep+=1