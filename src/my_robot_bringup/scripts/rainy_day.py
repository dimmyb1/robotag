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

                




    
    #Graph Functions
    def calculateProbabilities(self):
        

        #check if the likeliest option is a node
        
        #so, is it worth considering the node? or is there a likely edge?
        b = False
        cells = []
        consider = []
        
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