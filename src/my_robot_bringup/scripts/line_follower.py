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

import heapq #for dijkstra

#list of constants and hyperparameters
#BOXMEAS - measurable
#PERSISTENCE - tunable
#CONTAMINATION - tunable
#CERTAINTY - tunable
#CONSIDER_NODES - tunable, partly measurable
#GRAY_COOLDOWN - measurable
#SENSE_COOLDOWN - measurable, partly tunable

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

        #junction turning vars
        self.completeTurn = False
        self.imu_turning = False
        self.imu_target = -1
        self.grayEntryTime = -1
        self.GRAY_COOLDOWN = 8
        self.senseEntryTime = -1
        self.SENSE_COOLDOWN = 5

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
        self.A = Noden('B', 'B', 'F', 'E', 'A', [78, 11, 61, 140])
        self.B = Noden('A', 'C', 'D', 'A', 'B', [106, 14, 29, 11])
        self.C = Noden('H', 'D', 'D', 'B', 'C', [155, 49, 10, 15])
        self.D = Noden('C', 'C', 'F', 'B', 'D',  [13,  59, 40, 46])
        self.E = Noden('A', 'F', 'G', 'G', 'E',  [128, 12, 49, 90])
        self.F = Noden('A', 'D', 'G', 'E', 'F',  [111, 41, 10, 34])
        self.G = Noden('F', 'H', 'E', 'E', 'G',   [9,  12, 109,34])
        self.H = Noden('C', 'H', 'H', 'G', 'H',  [130, 85, 89, 14])

        self.myNodes = [self.A, self.B, self.C, self.D, self.E, self.F, self.G, self.H]

        self.current_node = self.A
        self.last_node = self.A #for localisation
        self.current_destination = 'F'
        self.skipZero = False
        self.facing = 2 #start facing south?

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

        # def getEdgesFromNode(self, fromNCHAR):
        #     #ONLY ACCEPTS CHARS
        #     if fromNCHAR == 'A': return ["Pab1", "Pab2", "Paf1", "Pae1"]
        #     if fromNCHAR == 'B': return ["Pab1", "Pab2", "Pbd1", "Pbc1"]
        #     if fromNCHAR == 'C': return ["Pcd1", "Pcd2", "Pch1", "Pbc1"]
        #     if fromNCHAR == 'D': return ["Pcd1", "Pcd2", "Pfd1", "Pbd1"]
        #     if fromNCHAR == 'E': return ["Peg1", "Peg2", "Pae1", "Pef1"]
        #     if fromNCHAR == 'F': return ["Pef1", "Paf1", "Pfg1", "Pfd1"]
        #     if fromNCHAR == 'G': return ["Peg1", "Peg2", "Pfg1", "Pgh1"]
        #     if fromNCHAR == 'H': return ["Pch1", "Phh1", "Pgh1"]

        #     else: 
        #         self.get_logger().info(f"From getEdgesFromNode ERR: Can't return nonexistent node's edges")
        #         return []


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
        
    """
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

    """ 
    
    #Graph Functions
    def calculateProbabilities(self):
        #let's say radar stores the closest ultrasonic ping in euclidean metric in self.ultrasonic_distance
        #we get two readings: first ping entering reading (self.entry_angle)
        # second ping exiting reading (self.exit_angle)
        # the order doesnt make a difference, main thing is that we have the angle, wwe'll just take min or max of the two values.
        #and servo was the angle at which we go tthe reading, +- the known margin of error
        #and that BOXMEAS is the l / w of the boxes in the grid in euclidean metric
        BOXMEAS = 2
        PERSISTENCE = 0.7
        CONTAMINATION = 0.3
        
        #and let's say we have estimated our current coordinates to be x and y
        x = 2
        y = 3
        # so minimum (2,3), maximum (3,4) starts
        #these need to be estimated by timing how long we're following a black line for against our pretimed table
        
        #get current cardinal direction we're facing 
        #check if not detected at all! or if you only have 1 reading i.e. 1 reading before the line of detection (implement)
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

        #first we will check if the mark is right in front of us
        lesserServo = min(self.entry_angle, self.exit_angle)
        biggerServo = max(self.entry_angle, self.exit_angle)
        #for now i assume these to be in euclidean 

        

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
            for c in cells:
                if c not in servoCells:
                    cells.remove(c)
                        

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

        #these will likely go unused, for now they remain here
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
                    self.PE = 1
                    self.P["Peg1"]+= 0.25
                    self.P["Peg2"]+= 0.25
                    self.P["Pef1"]+= 0.25
                    self.P["Pae1"]+= 0.25
                elif c.y == 3:
                    self.P["Pae1"] += 0.293
                    self.P["Paf1"] += 0.707
                elif c.y == 4:
                    self.PA = 1
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
                    self.PG = 1
                    self.P["Pgh1"]+= 0.25
                    self.P["Pfg1"]+= 0.25
                    self.P["Peg1"]+= 0.25
                    self.P["Peg2"]+= 0.25
                elif c.y == 2:
                    self.PF = 1
                    self.P["Pef1"]+= 0.25
                    self.P["Pfg1"]+= 0.25
                    self.P["Paf1"]+= 0.25
                    self.P["Pfd1"]+= 0.25
                elif c.y == 3:
                    self.P["Pbd1"] += 0.498
                    self.P["Paf1"] += 0.498
                    self.P["Pfd1"] += 0.004
                elif c.y == 4:
                    self.PB = 1
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
                    self.PH = 1
                    self.P["Pgh1"]+= 0.25
                    self.P["Pch1"]+= 0.25
                    self.P["Phh1"]+= 0.25
                elif c.y == 2:
                    self.P["Pfd1"] += 0.477
                    self.P["Pch1"] += 0.523
                elif c.y == 3:
                    self.PD = 1
                    self.P["Pfd1"]+= 0.25
                    self.P["Pcd1"]+= 0.25
                    self.P["Pcd2"]+= 0.25
                    self.P["Pbd1"]+= 0.25
                elif c.y == 4:
                    self.PC = 1
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


        #now we take top X probable options and return them
        #e.g. top 4 or all above 50%  
        #tbf, i think we're done here, then actual selection is done by the individual strategy.






        """
        #first, let us normalise all the probabilities so that they are comparable
        #since currently there would be a bias based on the length of the edge, allowing probabilities to go above 1.0, and others to never get to 1.0
        #check the short edges first:
        # for a in [self.P["Pfg1"], self.P["Pgh1"], self.P["Pab2"], self.P["Pbc1"], self.P["Pcd1"], self.P["Pef1"]]:
        #     a = a / 0.5

        # self.P["Pfd1"] /= 0.981
        # self.P["Pbd1"] /= 0.998
        # self.P["Peg1"] /= 1.344
        # self.P["Pcd2"] /= 1.451
        # self.P["Paf1"] /= 1.705
        # self.P["Pab1"] /= 2.5
        # self.P["Pae1"] /= 3.043
        # self.P["Phh1"] /= 3.5
        # self.P["Pch1"] /= 4.749
        # self.P["Peg2"] /= 5.406

        #these are actually flawed because now shorter / more compact edges are made likelier than edges 
        #which are more spread out

        #now everything is written as P E [0,1]


        #finally, we have aggregated all the probabilities - so we take the maximum value
        #ACTUALLY WE'RE NO LONGER TAKING MAXIMUM VALUE.

        #what we are doing now is: anything less than 0.5 probability (unless it is max value) is cut
        # max = -1
        # consider = [self.P["Pab1"],self.P["Pab2"],self.P["Paf1"],self.P["Pae1"],self.P["Pef1"],self.P["Peg1"],self.P["Peg2"],self.P["Pgh1"],self.P["Phh1"],self.P["Pfg1"],self.P["Pfd1"],self.P["Pbd1"],self.P["Pbc1"],self.P["Pcd1"],self.P["Pcd2"],self.P["Pch1"]]
        # for a in consider:
        #     if(a == 0):
        #         consider.remove(a)
        #         #remove any 0 probability paths

        #     elif a>=max:
        #         max = a
        #         #keep track of the maximum

        # #TRIM UNLIKELY OPTIONS

        # #0.5 is the minimum probability for the shortest paths in the map
        # if(max>= 0.5):
        #     for a in consider:
        #         if (a < 0.5):
        #             consider.remove(a)

        # #0.25 was generally found to be the lowest but most common low probability value. (e.g. half a short path)
        # elif(max> 0.25):
        #     for a in consider:
        #         if (a <= 0.25):
        #             consider.remove(a)
        # else:
        #     for a in consider:
        #         if (a == 0.0):
        #             consider.remove(a)

        

        #check if the likeliest option is a node
        
        #so, is it worth considering the node? or is there a likely edge?
        # aNodeFound = False
        # manyNodesFound = False
        # count = 0
        # for c in cells:
        #     count+=1

        # #if(max < 1 / count):
        # nConsider = [self.PA,self.PB,self.PC,self.PD,self.PE,self.PF,self.PG,self.PH]
        # #the robot is more likely to be exactly at an intersection than on any particular path.
        # for nodeProb in nConsider:
        #     if nodeProb != 0:
        #         #node probabilities are set to 1.0 by default if the cell is active
        #         #so let's normalise.
        #         nodeProb /= count
        #         # node probability /= number of cells selected
        #         # so PX = 1 / numOfCells

        #         if(aNodeFound):
        #             manyNodesFound = True

        #         aNodeFound = True

                
        #     else:
        #         nConsider.remove(nodeProb)
        #         #remove any 0 probability options


        #lets say we have 2 variables for target location: max edge and node
        # oppEdge = 1
        # oppNode = 2

        # if(consider):
        #     #make sure consider actually has something in it first.

        #     #we need to take the max valued edge (implemen/t)
        #     dummy = 3

        # if(aNodeFound):
        #     if(manyNodesFound):
        #         dummy = 3
        #         #ghandna problema 
        #         #lets find the most common one.
        #         #(implemen/t)
        #     else:
        #         #ok, set node to the 1 option available
        #         oppNode = nConsider.pop()

        # else:
        #     dummy = 2
            #we need to make up the closest node
            #but we need to set the lookAtNode variable (implemen/t) to be false so that we only look at edge, but keep this as reference

        #consider now only contains the likeliest options
        #if we are using nodes, then we are dealing with (probably only 1) node in nConsider
        #but are they *possible*?

        #if not b:
            #if no nodes, then they are along an edge
            #we should check where the opponent was last seen to decide which one is 
            #self.opponentLast = self.opponentCurrent
            #self.opponentCurrent = consider

            #ok. (implemen/t)
            #so we just finished calculating all edge and node probabilities, right? and we normalised 'em.
            #so now, we take all of the top 0.5 from the edge probabilities
            #we also definitely store the most probable edge
            #and we also try to find an intersecting node between all probable edges, and overlap that with our nConsider
            #and we save the most probable node
            #and we have a bool which will just say: do i pay more attention to edge or to node location?
            #and bam, we have some kinda targetting.

            

        #else:
            #if we are almost certainly at a node
            #ez
            #just pop a node and assume that to be the location
            #dummy = nConsider.pop()
        """

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
        safeNodes = allNodes - unsafeNodes

        #are we safe? then just wait here until the situation changes.
        if self.current_node in safeNodes:
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
        safeNodes = allNodes - unsafeNodes

        #are we safe? then just wait here until the situation changes.
        if self.current_node in safeNodes:
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
        safeNodes = allNodes - unsafeNodes

        #are we safe? then just wait here until the situation changes.
        if self.current_node in safeNodes:
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


        #if None is returned, everything has failed somehow, must be an error
        
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
                parentsDict[self.current_node] = 0

                #now we have the most common parent
                #find central node
                nfound = False
                for toK,v in parentsDict.items():
                    #is there a node?
                    if v == CERTAINTY:
                        #yes -> generatepathfromNtoN
                        self.current_destination = self.generatePathFromNToN(toK)
                        nfound=True
                        break #stop iterating
                
                #if not nfound:
                if not nfound:
                    #go backwards from CERTAINTY to 2
                    for cert in range(CERTAINTY-1, 1, -1):
                        #until you find the max valued parent node
                        #if you found, do nfound yes

                        for toK,v in parentsDict.items():
                        #is there a node?
                            if v == cert:
                                #yes -> generatepathfromNtoN
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
                parentsDict[self.current_node] = 0

                #now we have the most common parent
                #find central node
                nfound = False
                for toK,v in parentsDict.items():
                    #is there a node?
                    if v == CERTAINTY:
                        #yes -> generatepathfromNtoN
                        self.current_destination = self.generateSafePathFromEnemyNode(toK)
                        nfound=True
                        break #stop iterating
                
                #if not nfound:
                if not nfound:
                    #go backwards from CERTAINTY to 2
                    for cert in range(CERTAINTY-1, 1, -1):
                        #until you find the max valued parent node
                        #if you found, do nfound yes

                        for toK,v in parentsDict.items():
                        #is there a node?
                            if v == cert:
                                #yes -> generatepathfromNtoN
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
            
        elif self.behaviourMode == 5:
            # 5 - Interceptive
            last_pos = "?" #implement
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
                parentsDict[self.current_node] = 0

                #now we have the most common parent
                #find central node
                nfound = False
                for toK,v in parentsDict.items():
                    #is there a node?
                    if v == CERTAINTY:
                        #yes -> generatepathfromNtoN
                        self.current_destination = self.generatePathFromNToN(toK)
                        nfound=True
                        break #stop iterating
                
                #if not nfound:
                if not nfound:
                    #go backwards from CERTAINTY to 2
                    for cert in range(CERTAINTY-1, 1, -1):
                        #until you find the max valued parent node
                        #if you found, do nfound yes

                        for toK,v in parentsDict.items():
                        #is there a node?
                            if v == cert:
                                #yes -> generatepathfromNtoN
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
        else:
            # 0 - not set (no behaviour)
            return



    def startTurnBasedOnFacing(self):
        if self.facing == 0:
            if self.imu_target == 0:
                #no turn
                self.imu_turning = False
                self.completeTurn = True

            elif self.imu_target == 1:
                self.turnRight()
            elif self.imu_target == 2:
                self.turnRight()
            elif self.imu_target == 3:
                self.turnLeft()
            
        elif self.facing == 1:
            if self.imu_target == 1:
                #no turn
                self.imu_turning = False
                self.completeTurn = True

            elif self.imu_target == 2:
                self.turnRight()
            elif self.imu_target == 3:
                self.turnRight()
            elif self.imu_target == 0:
                self.turnLeft()

        elif self.facing == 2:
            if self.imu_target == 2:
                #no turn
                self.imu_turning = False
                self.completeTurn = True

            elif self.imu_target == 3:
                self.turnRight()
            elif self.imu_target == 0:
                self.turnRight()
            elif self.imu_target == 1:
                self.turnLeft()
        
        elif self.facing == 3:
            if self.imu_target == 3:
                #no turn
                self.imu_turning = False
                self.completeTurn = True

            elif self.imu_target == 0:
                self.turnRight()
            elif self.imu_target == 1:
                self.turnRight()
            elif self.imu_target == 2:
                self.turnLeft()





    def updatePos(self):
        now = time.time()
        #update current variables
         #do some check to ensure we aren't being triggered by the last gray section we saw
        if self.grayEntryTime < now - self.GRAY_COOLDOWN:
            #if gray detected:
            if (self.isGray[0] > self.minPixels) or (self.isGray[1] > self.minPixels)  or (self.isGray[2] > self.minPixels):
                
                self.grayEntryTime = now
                self.get_logger().info("Intersection detected!")


                
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
                    if (not self.current_destination and self.senseEntryTime < now - self.SENSE_COOLDOWN) or (self.current_destination):
                        #when sensing cooldown expires, look again.
                        self.senseEntryTime = now
                        self.current_destination = self.planDestination()


                        #we have directions to go somewhere
                        #self.current_destination has been set to either 1 directional number OR a LIST of directional numbers. 
                        #Check which one, if it is a list, we must iterate over it as it is a long path.
                        if type(self.current_destination) == list:
                            if(self.current_destination):
                                if type(self.current_destination[0]) == int:
                                    #if it is a single number from 0 to 3, then it is an immediate neighbour 
                                    #e.g. path = [2] i.e. go south
                                    self.imu_target = self.current_destination.pop(0)
                                    self.imu_turning = True
                                    self.startTurnBasedOnFacing()

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
                                    elif self.current_node.Ec == neigh_name:
                                        self.imu_target = 1
                                    elif self.current_node.Sc == neigh_name:
                                        self.imu_target = 2
                                    elif self.current_node.Wc == neigh_name:
                                        self.imu_target = 3

                                    
                                    self.imu_turning = True
                                    self.startTurnBasedOnFacing()

                                    #update sweeping setting
                                    if (self.current_node.name == 'A' and self.current_destination[0] == self.A.Nc) or (self.current_node.name == 'B' and self.current_destination[0] == self.B.Nc) or (self.current_node.name == 'C' and self.current_destination[0] == self.C.Ec) or (self.current_node.name == 'D' and self.current_destination[0] == self.D.Wc) or (self.current_node.name == 'E' and self.current_destination[0] == self.E.Nc) or (self.current_node.name == 'F' and self.current_destination[0] == self.F.Nc) or (self.current_node.name == 'F' and self.current_destination[0] == self.F.Wc) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Ec) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Sc) or (self.current_node.name == 'G' and self.current_destination[0] == self.G.Wc) :
                                        #when left first is better than right first
                                        self.skipZero = True
                                    else:
                                        self.skipZero = False


                        else : #not list
                            
                            #then it is a char from A to H, and it is an immediate neighbour 
                            #e.g. path = 'A'
                            if self.current_node.Nc == self.current_destination:
                                self.imu_target = 0
                            elif self.current_node.Ec == self.current_destination:
                                self.imu_target = 1
                            elif self.current_node.Sc == self.current_destination:
                                self.imu_target = 2
                            elif self.current_node.Wc == self.current_destination:
                                self.imu_target = 3

                            self.imu_turning = True
                            self.startTurnBasedOnFacing()

                            #update sweeping setting
                            if (self.current_node.name == 'A' and self.current_destination == self.A.Nc) or (self.current_node.name == 'B' and self.current_destination == self.B.Nc) or (self.current_node.name == 'C' and self.current_destination == self.C.Ec) or (self.current_node.name == 'D' and self.current_destination == self.D.Wc) or (self.current_node.name == 'E' and self.current_destination == self.E.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Wc) or (self.current_node.name == 'G' and self.current_destination == self.G.Ec) or (self.current_node.name == 'G' and self.current_destination == self.G.Sc) or (self.current_node.name == 'G' and self.current_destination == self.G.Wc) :
                                #when left first is better than right first
                                self.skipZero = True
                            else:
                                self.skipZero = False

                else : #not list
                    #update current_node and last_node
                    self.last_node = self.current_node
                    self.current_node = self.returnNode(self.current_destination)

                    #then it is a char from A to H, and it is an immediate neighbour 
                    #e.g. path = 'A'
                    if self.current_node.Nc == self.current_destination:
                        self.imu_target = 0
                    elif self.current_node.Ec == self.current_destination:
                        self.imu_target = 1
                    elif self.current_node.Sc == self.current_destination:
                        self.imu_target = 2
                    elif self.current_node.Wc == self.current_destination:
                        self.imu_target = 3

                            
                    self.imu_turning = True
                    self.startTurnBasedOnFacing()

                    #update sweeping setting
                    if (self.current_node.name == 'A' and self.current_destination == self.A.Nc) or (self.current_node.name == 'B' and self.current_destination == self.B.Nc) or (self.current_node.name == 'C' and self.current_destination == self.C.Ec) or (self.current_node.name == 'D' and self.current_destination == self.D.Wc) or (self.current_node.name == 'E' and self.current_destination == self.E.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Nc) or (self.current_node.name == 'F' and self.current_destination == self.F.Wc) or (self.current_node.name == 'G' and self.current_destination == self.G.Ec) or (self.current_node.name == 'G' and self.current_destination == self.G.Sc) or (self.current_node.name == 'G' and self.current_destination == self.G.Wc) :
                        #when left first is better than right first
                        self.skipZero = True
                    else:
                        self.skipZero = False
        


        


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

        if 0.0 <= yaw_deg < 45.0:
            self.facing = 0
        elif 45.0 <= yaw_deg < 135.0 :
            self.facing = 1
        elif 135.0 <= yaw_deg < 225.0 :
            self.facing = 2
        elif 225.0 <= yaw_deg < 360.0 :
            self.facing = 3
        
        #self.get_logger().info(f'Current Z Rotation (Yaw): {yaw_deg:.2f}°')

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

        elif self.imu_turning and self.facing == self.imu_target:
            #we have turned to face the general direction of where we needed to be,
            #but we may not have necessarily found a line yet
            #so let's turn on a switch saying keep turning, but once you find a line reset imu_turning.
            self.completeTurn = True
            self.imu_turning = False


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

            #if we were turning at an intersection and we found the particular line we were looking for, end that search and start following
            if(self.completeTurn):
                self.completeTurn = False

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