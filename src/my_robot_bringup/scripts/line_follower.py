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
import random

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
        self.Nt = 0.0
        self.Et = 0.0
        self.St = 0.0
        self.Wt = 0.0
     
class Cell():
    def __init__(self):
        self.x = 0
        self.y = 0

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

        self.myNodes = [self.A, self.B, self.C, self.D, self.E, self.F, self.G, self.H]

        self.current_node = self.A
        self.current_destination = 'F'
        self.skipZero = False
        self.behaviourMode = 0
        self.patrolPath = ['F', 'G', 'E', 'F', 'D', 'C', 'H', 'H', 'G', 'E', 'A', 'B', 'C', 'D', 'B', 'A']
        self.i_patrol = 0
        # behaviourMode settings:
        # 0 - not set (no behaviour)
        # 1 - Simple Line Follower
        # 2 - Random
        # 3 - Greedy
        # 4 - Avoidant
        # 5 - Interceptive
        # 6 - Trap Layer

        self.Pab1 = 0.0
        self.Pab2 = 0.0
        self.Paf1 = 0.0
        self.Pae1 = 0.0
        self.Pef1 = 0.0
        self.Peg1 = 0.0
        self.Peg2 = 0.0
        self.Pgh1 = 0.0
        self.Phh1 = 0.0
        self.Pfg1 = 0.0
        self.Pfd1 = 0.0
        self.Pbd1 = 0.0
        self.Pbc1 = 0.0
        self.Pcd1 = 0.0
        self.Pcd2 = 0.0
        self.Pch1 = 0.0

        self.PA = 0.0
        self.PB = 0.0
        self.PC = 0.0
        self.PD = 0.0
        self.PE = 0.0
        self.PF = 0.0
        self.PG = 0.0
        self.PH = 0.0

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

    def findPossiblePaths(self, listOfNodes, seenLastList, seenNowList)

    def upBound(self, x, manhattan, bound):
        upx = x + manhattan 
        if upx > bound:
            upx = bound + 1
        return upx 
    
    def lowBound(self, x, manhattan):
        lowx = x - manhattan 
        if lowx < 0:
            lowx = 0
        return lowx
    
    #Graph Functions
    def calculateProbabilities(self):
        #let's say radar stores the closest ultrasonic ping in euclidean metric
        radar = 8.2
        servo = 10
        #and servo was the angle at which we go tthe reading, +- the known margin of error
        #and that boxmeas is the l / w of the boxes in the grid in euclidean metric
        boxmeas = 2

        #and let's say we have estimated our current coordinates to be x and y
        x = 2
        y = 3
        # so minimum (2,3), maximum (3,4) starts
        #these need to be estimated by timing how long we're following a black line for against our pretimed table

        #then we can calculate our minimum and maximums for manhattan distance
        manhattan = math.ceil(radar / boxmeas) 

        
        #keeping within the bounds of a 5x6 matrix (0-4 , 0-5)
        #find all of the cells are within a manhattan distance of "manhattan" from x and y for our minimum band
        #and from x+1 and y+1 for our maximum band
        #then, we have a hard maximum set for x coord at matrixBound(x+- manhattan) and matrixBound(y coord at y+-manhattan)
        #add padding that allows a manhattan+1 distance from x+1 and y+1 staying within the two bounds
        upx = self.upBound(x, manhattan, 4)
        upy = self.upBound(y, manhattan, 5)
        lowx = self.lowBound(x, manhattan)
        lowy = self.lowBound(y, manhattan)
        padx = self.upBound(x+1, manhattan+1, upx)
        pady = self.upBound(y+1, manhattan+1, upy)

        cells = []
        for ix in range(lowx, padx):
            for iy in range(lowy, pady):
                if (Cell(ix,iy) not in cells) and ((abs(x - ix) + abs(y - ix) == manhattan) or (abs(x - ix) + abs(y - ix) == manhattan+1)):
                    cells.append(Cell(ix,iy))

        #trim selection by using servo and mpu angle (implement)
        
        #now we have a set of cells which the opponent can be in


        #reset probabilities
        self.Pab1 = 0.0
        self.Pab2 = 0.0
        self.Paf1 = 0.0
        self.Pae1 = 0.0
        self.Pef1 = 0.0
        self.Peg1 = 0.0
        self.Peg2 = 0.0
        self.Pgh1 = 0.0
        self.Phh1 = 0.0
        self.Pfg1 = 0.0
        self.Pfd1 = 0.0
        self.Pbd1 = 0.0
        self.Pbc1 = 0.0
        self.Pcd1 = 0.0
        self.Pcd2 = 0.0
        self.Pch1 = 0.0

        self.PA = 0.0
        self.PB = 0.0
        self.PC = 0.0
        self.PD = 0.0
        self.PE = 0.0
        self.PF = 0.0
        self.PG = 0.0
        self.PH = 0.0


        for c in cells:
            if c.x == 0:
                if c.y == 0:
                    self.Peg2 +=1
                elif c.y == 1:
                    self.Peg2 += 1
                elif c.y == 2:
                    self.Pae1 += 0.25
                    self.Peg2 += 0.75
                elif c.y == 3:
                    self.Pae1 +=1
                elif c.y == 4:
                    self.Pae1 +=1
                # elif c.y == 5:
                #     return
            elif c.x == 1:
                if c.y == 0:
                    self.Peg2 +=1
                elif c.y == 1:
                    self.Peg2 +=0.156
                    self.Peg1 +=0.844
                elif c.y == 2:
                    self.PE = 1
                    self.Peg1+= 0.25
                    self.Peg2+= 0.25
                    self.Pef1+= 0.25
                    self.Pae1+= 0.25
                elif c.y == 3:
                    self.Pae1 += 0.293
                    self.Paf1 += 0.707
                elif c.y == 4:
                    self.PA = 1
                    self.Pab1+= 0.25
                    self.Pab2+= 0.25
                    self.Paf1+= 0.25
                    self.Pae1+= 0.25
                elif c.y == 5:
                    self.Pab1 += 1
            elif c.x == 2:
                if c.y == 0:
                    self.Peg2 +=1
                elif c.y == 1:
                    self.PG = 1
                    self.Pgh1+= 0.25
                    self.Pfg1+= 0.25
                    self.Peg1+= 0.25
                    self.Peg2+= 0.25
                elif c.y == 2:
                    self.PF = 1
                    self.Pef1+= 0.25
                    self.Pfg1+= 0.25
                    self.Paf1+= 0.25
                    self.Pfd1+= 0.25
                elif c.y == 3:
                    self.Pbd1 += 0.498
                    self.Paf1 += 0.498
                    self.Pfd1 += 0.004
                elif c.y == 4:
                    self.PB = 1
                    self.Pab1+= 0.25
                    self.Pab2+= 0.25
                    self.Pbc1+= 0.25
                    self.Pbd1+= 0.25
                elif c.y == 5:
                    self.Pab1 += 1
            elif c.x == 3:
                if c.y == 0:
                    self.Phh1 +=1
                elif c.y == 1:
                    self.PH = 1
                    self.Pgh1+= 0.25
                    self.Pch1+= 0.25
                    self.Phh1+= 0.25
                elif c.y == 2:
                    self.Pfd1 += 0.477
                    self.Pch1 += 0.523
                elif c.y == 3:
                    self.PD = 1
                    self.Pfd1+= 0.25
                    self.Pcd1+= 0.25
                    self.Pcd2+= 0.25
                    self.Pbd1+= 0.25
                elif c.y == 4:
                    self.PC = 1
                    self.Pcd1+= 0.25
                    self.Pcd2+= 0.25
                    self.Pbc1+= 0.25
                    self.Pch1+= 0.25
                elif c.y == 5:
                    self.Pch1 +=1
            elif c.x == 4:
                if c.y == 0:
                    self.Phh1 +=1
                elif c.y == 1:
                    self.Phh1 +=1
                elif c.y == 2:
                    self.Pch1 +=1
                elif c.y == 3:
                    self.Pch1 += 0.376
                    self.Pcd2 += 0.301
                elif c.y == 4:
                    self.Pch1 +=0.35
                    self.Pcd2 += 0.65
                elif c.y == 5:
                    self.Pch1 +=1
        
        #finally, we have aggregated all the probabilities - so we take the maximum value
        max = -1
        consider = [self.Pab1,self.Pab2,self.Paf1,self.Pae1,self.Pef1,self.Peg1,self.Peg2,self.Pgh1,self.Phh1,self.Pfg1,self.Pfd1,self.Pbd1,self.Pbc1,self.Pcd1,self.Pcd2,self.Pch1,self.PA,self.PB,self.PC,self.PD,self.PE,self.PF,self.PG,self.PH]
        for a in consider:
            if a>=max:
                max = a
            else:
                consider.remove(a)

        for a in consider:
            if a < max:
                consider.remove(a)

        #now consider only contains the max probability options for where the opponent can be
        #check if the top option is a node
        b = False
        for a in consider:
            if a in [self.PA, self.PB, self.PC, self.PD, self.PE, self.PF, self.PG, self.PH]:
                self.opponentCurrentProb = a
                b = True

        if not b:
            #if no nodes, then they are along an edge
            #we should check where the opponent was last seen to decide which one is 
            lastDest = self.opponentCurrent
            #self.opponentCurrent = consider

            considerNeighbour = []

            for a in consider:
                #abc
                dummy = 1



    def planDestination(self):
        choice = -1
        #define some preference algorithm.
        #use distances if you want (Nd, Ed, Sd, Wd)
        # behaviourMode settings:
        if self.behaviourMode == 1:
            # 1 - Simple Line Follower (using pink path)
            #self.patrolPath = ['F', 'G', 'E', 'F', 'D', 'C', 'H', 'H', 'G', 'E', 'A', 'B', 'C', 'D', 'B', 'A']
            self.i_patrol+=1
            if self.i_patrol >= 16:
                self.i_patrol = 0
            self.current_destination = self.patrolPath[self.i_patrol]
            
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
                
            
        elif self.behaviourMode == 3:
            # 3 - Greedy

            # access radar info (implement)
            
            return
        elif self.behaviourMode == 4:
            # 4 - Avoidant
            return
        elif self.behaviourMode == 5:
            # 5 - Interceptive
            return
        elif self.behaviourMode == 6:
            # 6 - Trap Layer
            return
        else:
            # 0 - not set (no behaviour)
            return

    def updatePos(self):
        #update current variables
        #if gray detected:
        if (self.isGray[0] > self.minPixels) or (self.isGray[1] > self.minPixels)  or (self.isGray[2] > self.minPixels):
            #do some check to ensure we aren't being triggered by the last gray section we saw (IMPLEMENT)

            self.get_logger().info("Intersection detected!")
            #self.current_node = self.current_destination
            for n  in [self.A, self.B, self.C, self.D, self.E, self.F, self.G, self.H]:
                if n.name == self.current_destination:
                    self.current_node = n
        #   self.current_destination = self. Function to Calculate Next Destination.
        self.planDestination()

        #update sweeping setting
        if (self.current_node == 'A' and self.current_destination == self.A.Nc) or (self.current_node == 'B' and self.current_destination == self.B.Nc) or (self.current_node == 'C' and self.current_destination == self.C.Ec) or (self.current_node == 'E' and self.current_destination == self.E.Nc):
            #special case
            self.skipZero = True
        elif(self.current_node in ['F', 'G']):
            self.skipZero = True
        else:
            self.skipZero = False

        #call function to turn towards intended direction by using MPU readings for current facing direction and the intended cardinal direction.(IMPLEMENT)
        
    #Ultrasonic Functions
    #callback for ultrasonic - radar (implement) - keep some vars up to date

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