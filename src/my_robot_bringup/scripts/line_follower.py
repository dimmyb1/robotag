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
from std_msgs.msg import String, Float64, Float64MultiArray
import json
from rclpy.qos import QoSProfile, DurabilityPolicy
import subprocess
import time
import random

class Noden():
    def __init__(self, nd, nc, ed, ec, sd,sc, wd, wc, n, t):
        self.Nd = nd
        self.Nc = nc
        self.Ed = ed
        self.Ec = ec
        self.Sd = sd
        self.Sc = sc
        self.Wd = wd
        self.Wc = wc
        self.name = n
        self.Times = t
     
class Cell():
    def __init__(self):
        self.x = 0
        self.y = 0

class line_follower(Node):
    def __init__(self):
        super().__init__('line_follower')

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

        #ultrasonic sensor related variables
        self.entry_angle = 0.0
        self.exit_angle = 0.0
        self.ultrasonic_distance = 0.0

        # Graph
        self.A = Noden(78, 11, 61, 140, 'B', 'B', 'F', 'E', 'A', [78, 11, 60, 140])
        self.B = Noden(106, 14, 29, 11, 'A', 'C', 'D', 'A', 'B', [106, 13, 29, 11])
        self.C = Noden(155, 49, 10, 15, 'H', 'D', 'D', 'B', 'C', [155, 49, 10, 15])
        self.D = Noden(13, 59, 40, 46, 'C', 'C', 'F', 'B', 'D',  [13,  59, 40, 46])
        self.E =Noden(128, 12, 49, 90, 'A', 'F', 'G', 'G', 'E',  [128, 12, 49, 90])
        self.F =Noden(111, 70, 11, 34, 'A', 'D', 'G', 'E', 'F',  [111, 41, 10, 34])
        self.G =Noden(9, 12, 109, 34, 'F', 'H', 'E','E', 'G',    [9,  12, 109, 34])
        self.H =Noden(130, 85, 89, 14, 'C', 'H', 'H', 'G', 'H',  [130, 85, 89, 14])

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

        robot_name = self.get_namespace().strip('/')

        if not robot_name:
            robot_name = 'generic'

        topic_L = f'/{robot_name}_ir_L/image_raw'
        topic_M = f'/{robot_name}_ir_M/image_raw'
        topic_R = f'/{robot_name}_ir_R/image_raw'
        topic_IMU = f'/{robot_name}/imu'
        topic_object = f"{robot_name}/object_data"



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
            
        

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()

        self.timer = self.create_timer(0.05, self.loop)

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
            self.get_logger().info(f"Can't return nonexistent char-node value")
            return None
        
    #UNUSED :(
    def allReachable(self, nFrom, nTo, tE, F):
        paths = []
        explorable = [(nFrom, 0.0, [])]
        #explored = []
        #keep finding the directly reachable nodes until we've exhausted all possible paths
        while(explorable):
            t1=-1.0
            t2=-1.0

            #where at means aggregated Time per path
            n, at, pSoFar = explorable.pop()
            #explored.append(n)
            neighbours = [n.Nc, n.Ec, n.Sc, n.Wc]
            #is the destination directly reachable from the current node (nFrom)?
            if nTo.name in neighbours :
                c = nTo.name
                i = neighbours.index(c)
                t1 = n.Times[i] +at

                if(t1 < tE + F):
                    pSoFar1 = pSoFar + [i]
                    paths.append([pSoFar1])

                #and (self.returnNode(ch) not in explored) and (self.returnNode(ch) not in explorable)
                #prepare next paths
                for ch in neighbours:
                    if (ch != c)  and (n.Times[neighbours.index(ch)] + at < tE + F) and (n.Times[neighbours.index(ch)] + at > tE - F):
                        at2 = n.Times[neighbours.index(ch)] + at
                        pSoFar2 = pSoFar + [neighbours.index(ch)]
                        explorable.append((self.returnNode(ch), at2, pSoFar2))

                neighbours.remove(c)

                

                if c in neighbours:
                    #if there's another path to the same node
                    i = neighbours.index(c) +1
                    t2 = n.Times[i] +at

                    if(t2 < tE + F):
                        pSoFar1 = pSoFar + [i]
                        paths.append([pSoFar1])

                    neighbours.remove(c)

        #paths: e.g. [ [0], [3,2,1]  ] -> path 1 is: go north (0); path 2 is: go west (3) go south, (2), go east (1)
        return paths
            
    
    #UNUSED :(
    def findPossiblePaths(self, seenLastList, seenNowList, timeElapsed):
        FORGIVENESS_IN_TIME_S = 5.0

        #create path 
        #is there a path from last pos to a in consider?
        #which has a similar estimated time from the last time we checked?

        possiblePaths = []
        for l in seenLastList:
            for n in seenNowList:
                #is it directly reachable? then add them to the possible paths as a list of directions. therefore possiblePaths is a list of lists of directions
                pdir = self.allReachable(l,n, timeElapsed, FORGIVENESS_IN_TIME_S)
                possiblePaths.append((self.returnNode(l), pdir))

        return possiblePaths

                



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
        #let's say radar stores the closest ultrasonic ping in euclidean metric in self.ultrasonic_distance
        #we get two readings: first ping entering reading (self.entry_angle)
        # second ping exiting reading (self.exit_angle)
        # the order doesnt make a difference, main thing is that we have the angle, wwe'll just take min or max of the two values.
        #and servo was the angle at which we go tthe reading, +- the known margin of error
        #and that BOXMEAS is the l / w of the boxes in the grid in euclidean metric
        BOXMEAS = 2

        #and let's say we have estimated our current coordinates to be x and y
        x = 2
        y = 3
        # so minimum (2,3), maximum (3,4) starts
        #these need to be estimated by timing how long we're following a black line for against our pretimed table

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
        #trim selection by using servo and mpu angle (implement)

        #first we will check if the mark is right in front of us
        lesserServo = min(self.entry_angle, self.exit_angle)
        biggerServo = max(self.entry_angle, self.exit_angle)
        #for now i assume these to be in euclidean 

        #get current cardinal direction we're facing (implement)
        #check if not detected at all! or if you only have 1 reading! (implement)
        if(lesserServo < 90 and biggerServo > 90):
            #if current cardinal direction we're facing is North or South
            for c in cells:
                if(c.x != x):
                    cells.remove(c)

            #if current cardinal is East or West:
            for c in cells:
                if(c.y != y):
                    cells.remove(c)

        else:
            #it isnt directly in front of us.
            #is it on the left or on the right?
            servoCells = []


            #on the right side:
            if(biggerServo <= 90):
                #if facing North
                if(dummy==1):
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
                elif(dummy==2):
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


                #elif east
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

                #elif west
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
                if(dummy==1):
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
                elif(dummy==1):
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
                elif(dummy==2):
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

                elif(dummy==3):
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
            for c in cells:
                if c not in servoCells:
                    cells.remove(c)
                        

                        


        
        #now we have a pretty small set of cells which the opponent can be in


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
        
        #first, let us normalise all the probabilities so that they are comparable
        #since currently there would be a bias based on the length of the edge, allowing probabilities to go above 1.0, and others to never get to 1.0
        #check the short edges first:
        for a in [self.Pfg1, self.Pgh1, self.Pab2, self.Pbc1, self.Pcd1, self.Pef1]:
            a = a / 0.5

        self.Pfd1 /= 0.981
        self.Pbd1 /= 0.998
        self.Peg1 /= 1.344
        self.Pcd2 /= 1.451
        self.Paf1 /= 1.705
        self.Pab1 /= 2.5
        self.Pae1 /= 3.043
        self.Phh1 /= 3.5
        self.Pch1 /= 4.749
        self.Peg2 /= 5.406

        #now everything is written as P E [0,1]


        #finally, we have aggregated all the probabilities - so we take the maximum value
        #ACTUALLY WE'RE NO LONGER TAKING MAXIMUM VALUE.

        #what we are doing now is: anything less than 0.5 probability (unless it is max value) is cut
        max = -1
        consider = [self.Pab1,self.Pab2,self.Paf1,self.Pae1,self.Pef1,self.Peg1,self.Peg2,self.Pgh1,self.Phh1,self.Pfg1,self.Pfd1,self.Pbd1,self.Pbc1,self.Pcd1,self.Pcd2,self.Pch1]
        for a in consider:
            if(a == 0):
                consider.remove(a)
                #remove any 0 probability paths

            elif a>=max:
                max = a
                #keep track of the maximum

        #TRIM UNLIKELY OPTIONS

        #0.5 is the minimum probability for the shortest paths in the map
        if(max>= 0.5):
            for a in consider:
                if (a < 0.5):
                    consider.remove(a)

        #0.25 was generally found to be the lowest but most common low probability value. (e.g. half a short path)
        elif(max> 0.25):
            for a in consider:
                if (a <= 0.25):
                    consider.remove(a)

        

        #check if the likeliest option is a node
        
        #so, is it worth considering the node? or is there a likely edge?
        b = False
        
        count = 0
        for c in cells:
            count+=1

        if(max < 1 / count):
            nConsider = [self.PA,self.PB,self.PC,self.PD,self.PE,self.PF,self.PG,self.PH]
            #the robot is more likely to be exactly at an intersection than on any particular path.
            for a in nConsider:
                if a != 0:
                    #node probabilities are set to 1.0 by default if the cell is active
                    #so let's normalise.
                    a /= count
                    # node probability /= number of cells selected
                    # so PX = 1 / numOfCells

                    
                    b = True
                    #b means we found a selected node
                else:
                    nConsider.remove(a)
                    #remove any 0 probability options

        #otherwise, we just wont consider any of the nodes.
        

        #consider now only contains the likeliest options
        #if we are using nodes, then we are dealing with (probably only 1) node in nConsider
        #but are they *possible*?

        if not b:
            #if no nodes, then they are along an edge
            #we should check where the opponent was last seen to decide which one is 
            self.opponentLast = self.opponentCurrent
            self.opponentCurrent = consider

            #let's say we have  atime elapsed variable saved somewhere (implement)
            timeElapsed = 1.111111

            #we need to check exactly which of current possible edges would be feasible to arrive at in the time that has passed since our last check
            #so we go to our other function

            #possibePaths = self.findPossiblePaths(self.opponentLast, self.opponentCurrent, timeElapsed)

            #this gives us all the legal paths the opponent could have done in the time.
            #this only spells out *complete* node to node paths
            #therefore, on top of these paths, we must also consider that a new edge may have been started and the robot is somewhere along that path.
            #so what do we do?
            #well, we can first parse possiblePaths to see if arriving at a particular edge is possible.
            #but we won't be doing this with the exact timings, because that would be too brittle of a system.
            #possiblePaths looks something like {[], [1, 3, 2], [3, 0, 0, 0]} where 0-3 is north - west respectively
            #therefore we can reconstruct a path that would eventually lead to that edge.
            #hm.
            #ok so after calling possiblePaths, we know all the nodes the robot could have arrived to and left from in this time. 
            #so we have 4^NC (possible node count) possible edges the opponent could be going down.
            #i think we need another overlap, which will just check if each of these edges are 0 or not.
            #we do this per node.
            #and we just go down their cardinal nodes.

            #ok recap:
            #possible paths is currently useless.

            #mela, we pass possiblepaths our consider or a matrix of cell to probabilities etc.
            #at this point, cells is no longer useful and we will never use it again for now.

            #i think the probabilities should be a dictionary i wont lie
            #IDK HOW IM GONNA DO THIS.


            #ok.
            #so we just finished calculating all edge and node probabilities, right? and we normalised 'em.
            #so now, we take all of the top 0.5 from the edge probabilities
            #we also definitely store the most probable edge
            #and we also try to find an intersecting node between all probable edges, and overlap that with our nConsider
            #and we save the most probable node
            #and we have a bool which will just say: do i pay more attention to edge or to node location?
            #and bam, we have some kinda targetting.

            

        else:
            #if we are almost certainly at a node
            #ez
            #just pop a node and assume that to be the location
            dummy = nConsider.pop()



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
        
    #MPU5060 / IMU callback
    def imu_callback(self, msg: Imu):
        # The IMU gives us a quaternion (x, y, z, w)
        q = msg.orientation
        
        # Convert the quaternion to Yaw (Rotation around the Z axis) in radians
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        
        # Optional: Convert to degrees for easier human reading
        yaw_deg = math.degrees(yaw_rad)
        
        self.get_logger().info(f'Current Z Rotation (Yaw): {yaw_deg:.2f}°')



    #Ultrasonic functions
    def ultrasonic_callback(self, msg):
        # Unpack the array based on the order you published it
        #ANGLES ARE IN RADIANS (but i can do math.degrees(v) to convert to normal degrees if i need to)
        self.entry_angle = msg.data[0]
        self.exit_angle = msg.data[1]
        self.ultrasonic_distance = msg.data[2]
        
        # Now you have exactly what you care about to use in this script
        #self.get_logger().info(f"Received Object Data -> Entry: {entry_angle:.2f}, Exit: {exit_angle:.2f}, Dist: {distance:.2f}")



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