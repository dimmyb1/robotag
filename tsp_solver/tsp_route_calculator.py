#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from tsp_solver.tsp_solver_node import GraphTSP
from tsp_solver.brute_force import BruteForceTSP
import json
import time 

class TSPRouteServer(Node):
    def __init__(self, graph, targets):
        super().__init__('tsp_route_calculator')
        self.srv = self.create_service(Trigger, 'calculate_tsp_route', self.calculate_callback)
        self.get_logger().info("TSP Route Calculator ready.")
        self.graph = graph
        self.targets = targets

    def calculate_callback(self, _, response):
        start_time = time.time()

        solver = GraphTSP(self.graph)
        # start = 'PostOffice'
        start = 'PO'
        route = solver.nearest_neighbour_tsp(start, self.targets)
        optimized = solver.two_opt(route)
        self.get_logger().info("Using NN")
        # solver = BruteForceTSP(self.graph)
        # optimized = solver.brute_force_tsp(start, self.targets)
        # self.get_logger().info("Using BF")


        response.success = True
        self.get_logger().info(f"Time taken {time.time()-start_time}")
        response.message = json.dumps({
            'route': optimized,
            'targets_order': [n for n in optimized if n in self.targets],  
            'total_distance': solver.total_distance(optimized)
        })
        self.get_logger().info(f"Full path: {optimized}")
        self.get_logger().info(f"Targets order: {[n for n in optimized if n in self.targets]}")
        return response

def main():
    rclpy.init()
    LEFT = 1.0
    RIGHT = 1.0
    # graph = {
    #     'PostOffice': {'H1': 2, 'H2': 3},
    #     'H1': {'PostOffice': 2, 'H2': 1},
    #     'H2': {'H1': 4, 'H3': 2},
    #     'H3': {'H1': 3, 'H2': 1}, 
    #     # even though H2 is less from H3 the whole path would be longer therefore took H1
    # }
    # graph = {
    #     'PostOffice': {'H1': 2, 'H2': 3},
    #     'H1': {'PostOffice': 10, 'H2': 1},
    #     'H2': {'PostOffice':1,'H1': 4, 'H3': 2},
    #     'H3': {'H1': 1, 'H2': 3},
    #     # ['PostOffice', 'H1', 'H2', 'H3', 'H1', 'H2', 'PostOffice']

    # }

    # graph = {
    #     'PostOffice': {'H1': 4, 'H2': 6, 'H3': 8, 'H4': 5},
    #     'H1': {'PostOffice': 4, 'H2': 2, 'H5': 7, 'H6': 3},
    #     'H2': {'PostOffice': 6, 'H1': 2, 'H3': 4, 'H5': 5, 'H7': 6},
    #     'H3': {'PostOffice': 8, 'H2': 4, 'H4': 3, 'H6': 6, 'H8': 5},
    #     'H4': {'PostOffice': 5, 'H3': 3, 'H7': 4, 'H8': 6},
    #     'H5': {'H1': 7, 'H2': 5, 'H6': 4, 'H9': 8},
    #     'H6': {'H1': 3, 'H3': 6, 'H5': 4, 'H7': 5, 'H10': 6},
    #     'H7': {'H2': 6, 'H4': 4, 'H6': 5, 'H8': 3, 'H1': 5},
    #     'H8': {'H3': 5, 'H4': 6, 'H7': 3, 'H9': 4, 'H2': 7},
    #     'H9': {'H5': 8, 'H8': 4, 'H10': 3},
    #     'H10': {'H6': 6, 'H9': 3},
    # }


    # Full path: ['PostOffice', 'H4', 'H3', 'H6', 'H10', 'H6', 'H1', 'PostOffice']
    targets = ['H10', 'H6', 'H4']
    targets = ['H4', 'H6', 'H10']

    # Robot starts facing POST OFFICE
    graph = {
        'PO': {'HOUSE_1': RIGHT + 5 + RIGHT + 1 + LEFT, 
               'HOUSE_2': RIGHT + 3 + RIGHT + 11 + LEFT, 
               'HOUSE_3': RIGHT + 3 + RIGHT + 6 + LEFT + 1 + RIGHT, 
               'HOUSE_4': RIGHT + 3 + RIGHT + 3 + RIGHT, 
               'HOUSE_5': (2*RIGHT) + 9 + LEFT, 
               'HOUSE_6': (2*RIGHT) + 11 + RIGHT + 2 + LEFT, 
               'HOUSE_7': LEFT + 8, 
               'HOUSE_8':  LEFT + 5 + LEFT + 3 + LEFT, 
               'HOUSE_9': LEFT + 8 + LEFT + 9 + RIGHT, 
               'HOUSE_10': (2*LEFT) + 6 + RIGHT + 2 + LEFT + 2 + RIGHT + 2 + LEFT, 
               'CHARGER_0': LEFT + 1 + RIGHT, 
               'CHARGER_1': RIGHT + 3 + RIGHT + 8 + LEFT, 
               'CHARGER_2': LEFT + 8 + LEFT + 6 + RIGHT
        },

        'CHARGER_0': {
            'PO': LEFT + 1 + RIGHT,
            'HOUSE_1': RIGHT + 6 + RIGHT + 1 + LEFT,
            'HOUSE_2': RIGHT + 4 + RIGHT + 11 + LEFT,
            'HOUSE_3': RIGHT + 4 + RIGHT + 6 + LEFT + 1 + RIGHT,
            'HOUSE_4': RIGHT + 4 + RIGHT + 3 + RIGHT,
            'HOUSE_5': RIGHT + 1 + RIGHT + 9 + LEFT,
            'HOUSE_6': RIGHT + 1 + RIGHT + 11 + RIGHT + 2 + LEFT,
            'HOUSE_7': LEFT + 7,
            'HOUSE_8': LEFT + 4 + LEFT + 3 + LEFT,
            'HOUSE_9': LEFT + 7 + LEFT + 9 + RIGHT,
            'HOUSE_10': RIGHT + 4 + LEFT + 8 + LEFT + 1 + RIGHT,
            'CHARGER_1': RIGHT + 4 + RIGHT + 8 + LEFT,
            'CHARGER_2': LEFT + 7 + LEFT + 6 + RIGHT
        },
        
        'CHARGER_1': {
            'PO': LEFT + 8 + LEFT + 3 + LEFT,
            'HOUSE_1': LEFT + 2 + RIGHT + 2 + LEFT + 5 + RIGHT,
            'HOUSE_2': RIGHT + 3 + LEFT,
            'HOUSE_3': LEFT + 2 + RIGHT + 1 + RIGHT,
            'HOUSE_4': LEFT + 5 + LEFT,
            'HOUSE_5': LEFT + 1 + LEFT + 3 + LEFT + 2 + LEFT,
            'HOUSE_6': RIGHT + 3 + RIGHT + 5 + LEFT,
            'HOUSE_7': LEFT + 8 + LEFT + 11,
            'HOUSE_8': LEFT + 8 + LEFT + 8 + LEFT + 3 + LEFT,
            'HOUSE_9': RIGHT + 3 + RIGHT + 11 + RIGHT + 2 + LEFT,
            'HOUSE_10': LEFT + 1 + LEFT + 3 + RIGHT + 1 + LEFT + 2 + LEFT + 2 + RIGHT + 2 + LEFT,            
            'CHARGER_0': LEFT + 8 + LEFT + 4 + RIGHT,
            'CHARGER_2': RIGHT + 3 + RIGHT + 11 + RIGHT + 5 + RIGHT,
        },

        'CHARGER_2': {
            'PO': RIGHT + 6 + RIGHT + 8 + LEFT,
            'HOUSE_1': RIGHT + 6 + RIGHT + 13 + RIGHT + 1 + LEFT,
            'HOUSE_2': LEFT + 5 + LEFT + 11,
            'HOUSE_3': LEFT + 5 + RIGHT + 11 + LEFT + 5 + RIGHT + 1 + RIGHT,
            'HOUSE_4': RIGHT + 6 + RIGHT + 11 + RIGHT + 3 + RIGHT,
            'HOUSE_5': LEFT + 5 + LEFT + 8 + LEFT + 2 + RIGHT,
            'HOUSE_6': LEFT + 5 + LEFT + 6 + RIGHT,
            'HOUSE_7': RIGHT + 6 + LEFT,
            'HOUSE_8': RIGHT + 6 + RIGHT + 3 + RIGHT + 3 + LEFT,
            'HOUSE_9': LEFT + 3 + RIGHT,
            'HOUSE_10': RIGHT + 6 + LEFT + 3 + RIGHT + 8 + LEFT + 1 + RIGHT,
            'CHARGER_0': RIGHT + 6 + RIGHT + 7 + LEFT,
            'CHARGER_1': LEFT + 5 + LEFT + 11 + LEFT + 3 + RIGHT
        },


        'HOUSE_1': {
            'HOUSE_2':  RIGHT + 5 + RIGHT + 2 + LEFT + 5 + LEFT ,
            'HOUSE_3':  RIGHT + 5 + RIGHT + 1 + LEFT,
            'HOUSE_4':  LEFT + 1 + LEFT + 2 + LEFT + 3 + RIGHT,
            'HOUSE_5': RIGHT + 5 + RIGHT + 2 + LEFT + 1 + RIGHT+ 3 + LEFT + 2 + LEFT,
            'HOUSE_6':  RIGHT + 5 + RIGHT + 2 + RIGHT + 5 + RIGHT + 5 + LEFT,
            'HOUSE_7':  LEFT + 1 + LEFT + 13,
            'HOUSE_8':  LEFT + 1 + LEFT + 10 + LEFT + 3 + LEFT,
            'HOUSE_9':  LEFT + 1 + LEFT + 13 + LEFT + 9 + RIGHT,
            'HOUSE_10': LEFT + 1 + LEFT + 10 + LEFT + 8 + LEFT + 1 + RIGHT,
            'PO':       LEFT + 1 + LEFT + 5 + RIGHT,
            'CHARGER_0': LEFT + 1 + LEFT + 6 + RIGHT,
            'CHARGER_1': RIGHT + 5 + RIGHT + 2 + LEFT + 2 + LEFT,
            'CHARGER_2': LEFT + 1 + LEFT + 13 + LEFT + 6 + RIGHT
        },

        'HOUSE_2': {
            'HOUSE_1':  LEFT + 5 + RIGHT + 2 + LEFT + 5 + RIGHT,
            'HOUSE_3':  LEFT + 5 + RIGHT + 1 + RIGHT,
            'HOUSE_4':  LEFT + 8 + LEFT,
            'HOUSE_5':  (2*LEFT) + 3 + RIGHT + 2 + RIGHT,
            'HOUSE_6':  (2*LEFT) + 5 + LEFT,
            'HOUSE_7':  LEFT + 11 + LEFT + 11,
            'HOUSE_8':  (2*LEFT) + 3 + RIGHT + 5 + LEFT + 5 + RIGHT + 3 + RIGHT,
            'HOUSE_9':  (2*LEFT) + 11 + RIGHT + 2 + LEFT,
            'HOUSE_10': (2*LEFT) + 3 + RIGHT + 5 + LEFT + 2 + LEFT + 2 + RIGHT + 2 + LEFT,
            'PO':       LEFT + 11 + LEFT + 3 + RIGHT,
            'CHARGER_0': LEFT + 11 + LEFT + 4 + RIGHT,
            'CHARGER_1': LEFT + 3 + RIGHT,
            'CHARGER_2': (2*LEFT) + 11 + RIGHT + 5 + LEFT
        },

        'HOUSE_3': {
            'HOUSE_1':   LEFT + 1 + LEFT + 5 + RIGHT,
            'HOUSE_2':   RIGHT + 1 + LEFT + 5 + LEFT,
            'HOUSE_4':   RIGHT + 1 + RIGHT + 3 + RIGHT,
            'HOUSE_5':  RIGHT + 1 + LEFT + 1 + RIGHT + 3 + LEFT + 2 + LEFT,
            'HOUSE_6':  RIGHT + 1 + LEFT + 5 +  RIGHT + 5 + LEFT,
            'HOUSE_7':  RIGHT + 1 + RIGHT + 6 + LEFT + 11,
            'HOUSE_8':  RIGHT + 1 + RIGHT + 6 + LEFT + 8 + LEFT + 3 + LEFT,
            'HOUSE_9':  RIGHT + 1 + LEFT + 5 + RIGHT + 11 + RIGHT + 2 + LEFT,
            'HOUSE_10': RIGHT + 1 + LEFT + 1 + RIGHT + 3 + RIGHT + 1 + LEFT + 2 + LEFT + 2 + RIGHT + 2 + LEFT,
            'PO':        RIGHT +  1 + RIGHT + 6 + LEFT + 3 + RIGHT,
            'CHARGER_0': RIGHT + 1 + RIGHT + 6 + LEFT + 4 + RIGHT,
            'CHARGER_1': RIGHT + 1 + LEFT + 2 + LEFT,
            'CHARGER_2': RIGHT + 1 + LEFT + 5 + RIGHT + 11 + LEFT + 5 + LEFT
        },

        'HOUSE_4': {
            'HOUSE_1':  RIGHT + 3 + RIGHT + 2 + RIGHT + 1 + LEFT,
            'HOUSE_2':  LEFT + 8 + LEFT,
            'HOUSE_3':  LEFT + 3 + LEFT + 1 + LEFT,
            'HOUSE_5':  LEFT + 4 + RIGHT + 3 + LEFT + 2,
            'HOUSE_6':  LEFT + 8 + RIGHT + 5 + LEFT,
            'HOUSE_7':  RIGHT + 3 + LEFT + 11,
            'HOUSE_8':  RIGHT + 3 + LEFT + 8 + LEFT + 3 + LEFT,
            'HOUSE_9':  RIGHT + 3 + LEFT + 11 + LEFT + 9 + RIGHT,
            'HOUSE_10': LEFT + 4 + RIGHT + 3 + RIGHT + 1 + LEFT + 2 + LEFT + 2 + RIGHT + 2 + LEFT,
            'PO':        LEFT + 3 + LEFT + 3 + LEFT,
            'CHARGER_0': LEFT + 3 + LEFT + 4 + LEFT,
            'CHARGER_1': LEFT + 5 + LEFT,
            'CHARGER_2': RIGHT + 3 + LEFT + 11 + LEFT + 6 + RIGHT
        },

        'HOUSE_5': {
            'HOUSE_1':  LEFT + 2 + RIGHT + 3 + LEFT+ 1 + RIGHT + 2 + LEFT + 5 + RIGHT,
            'HOUSE_2':  RIGHT + 2 + LEFT + 3,
            'HOUSE_3':  LEFT + 2 + RIGHT + 3 + LEFT + 1 + RIGHT + 1 + RIGHT,
            'HOUSE_4':  LEFT + 2 + RIGHT + 3 + LEFT + 4 + LEFT,
            'HOUSE_6':  RIGHT + 2 + RIGHT + 2 + LEFT,
            'HOUSE_7':  LEFT + 9 + LEFT + 8,
            'HOUSE_8':  LEFT + 3 + LEFT + 5 + RIGHT + 3 + RIGHT,
            'HOUSE_9':  RIGHT + 2 + RIGHT + 8 + RIGHT + 2 + LEFT,
            'HOUSE_10': LEFT + 3 + LEFT + 2 + LEFT + 2 + RIGHT + 2 + LEFT,
            'PO':        LEFT + 9,
            'CHARGER_0': LEFT + 9 + LEFT + 1 + RIGHT,
            'CHARGER_1': LEFT + 2 + RIGHT + 3 + RIGHT + 1 + LEFT,      
            'CHARGER_2': RIGHT + 2 + RIGHT + 8 + RIGHT + 5 + LEFT
        },

        'HOUSE_6': {
            'HOUSE_1':  LEFT + 5 + LEFT + 5 + RIGHT + 2 + LEFT + 5 + RIGHT,
            'HOUSE_2':  LEFT + 5,
            'HOUSE_3':  LEFT + 5 + LEFT + 5 +  RIGHT + 1 + RIGHT,
            'HOUSE_4':  LEFT + 5 + LEFT + 8 + LEFT,
            'HOUSE_5':  LEFT + 2 + LEFT + 2 + RIGHT,
            'HOUSE_7':  RIGHT + 6 + RIGHT + 11 + LEFT,
            'HOUSE_8':  LEFT + 2 + LEFT + 5 + LEFT + 5 + RIGHT + 3 + LEFT,
            'HOUSE_9':  RIGHT + 6 + RIGHT + 2 + LEFT,
            'HOUSE_10': LEFT + 2 + LEFT + 5 + LEFT + 2 + LEFT + 2 + RIGHT + 2 + LEFT,
            'PO':       LEFT + 2 + LEFT + 11,
            'CHARGER_0': LEFT + 2 + LEFT + 11 + LEFT + 1 + RIGHT,
            'CHARGER_1': LEFT + 5 + LEFT + 3 + RIGHT,
            'CHARGER_2':  RIGHT + 6 + RIGHT + 5 + LEFT,
        },

        'HOUSE_7': {
            'HOUSE_1':  (2*LEFT) + 13 + RIGHT + 1 + LEFT,
            'HOUSE_2':  (2*LEFT) + 11 + RIGHT + 11 + LEFT,
            'HOUSE_3':  (2*LEFT) + 11 + RIGHT + 6 + LEFT + 1 + RIGHT,
            'HOUSE_4':  (2*LEFT) + 11 + RIGHT + 3 + RIGHT,
            'HOUSE_5':  (2*LEFT) + 8 + RIGHT + 9 + RIGHT,
            'HOUSE_6':   LEFT + 11 + LEFT + 6 + RIGHT,
            'HOUSE_8':  (2*LEFT) + 3 + RIGHT + 3 + LEFT,
            'HOUSE_9':  LEFT + 9 + RIGHT,
            'HOUSE_10': (2*LEFT) + 3 + RIGHT + 8 + LEFT + 1 + RIGHT,
            'PO':       (LEFT*2) + 8 + LEFT,
            'CHARGER_0': (LEFT*2) + 7 + LEFT,
            'CHARGER_1': (2*LEFT) + 11 + RIGHT + 8 + LEFT,
            'CHARGER_2': LEFT + 6 + RIGHT
        },

        'HOUSE_8': {
            'HOUSE_1':  LEFT + 3 + RIGHT + 10 + RIGHT + 1 + LEFT,
            'HOUSE_2':  RIGHT + 3 + LEFT + 5 + RIGHT + 5 + LEFT + 3,
            'HOUSE_3':  LEFT + 3 + RIGHT + 8 + RIGHT + 6 + RIGHT + 1 + RIGHT,
            'HOUSE_4':  LEFT + 3 + RIGHT + 8 + RIGHT + 3 + RIGHT,
            'HOUSE_5':  RIGHT + 3 + LEFT + 5 + RIGHT + 3 + LEFT,
            'HOUSE_6':  RIGHT + 3 + LEFT + 5 + RIGHT + 5 + RIGHT + 2 + LEFT,
            'HOUSE_7':  LEFT + 3 + LEFT + 3,
            'HOUSE_9':  LEFT + 3 + LEFT + 3 + LEFT + 9 + RIGHT,
            'HOUSE_10': RIGHT + 5 + LEFT + 1 + RIGHT,
            'PO':       RIGHT + 3 + RIGHT + 5 + LEFT,
            'CHARGER_0': RIGHT + 3 + RIGHT + 4 + LEFT,
            'CHARGER_1': LEFT + 3 + LEFT + 8 + RIGHT + 8 + LEFT,
            'CHARGER_2': LEFT + 3 + LEFT + 3 + LEFT + 6 + RIGHT
        },

        'HOUSE_9': {
            'HOUSE_1':  RIGHT + 9 + RIGHT + 13 + RIGHT + 1 + LEFT,
            'HOUSE_2':  LEFT + 2 + LEFT + 11,
            'HOUSE_3':  LEFT + 2 + LEFT + 11 + LEFT + 5 + RIGHT + 1 + RIGHT,
            'HOUSE_4':  RIGHT + 9 + RIGHT + 11 + RIGHT + 3 + RIGHT,
            'HOUSE_5':  LEFT + 2 + LEFT + 8 + LEFT + 2 + RIGHT,
            'HOUSE_6':  LEFT + 2 + LEFT + 6 + RIGHT,
            'HOUSE_7':  RIGHT + 9 + LEFT,
            'HOUSE_8':  RIGHT + 9 + RIGHT + 3 + RIGHT + 3 + LEFT,
            'HOUSE_10': RIGHT + 9 + RIGHT + 3 + RIGHT + 8 + LEFT + 1 + RIGHT,
            'PO':       RIGHT + 9 + RIGHT + 8 + LEFT,
            'CHARGER_0': RIGHT + 9 + RIGHT + 7 + LEFT,
            'CHARGER_1': LEFT + 2 + LEFT + 11 + LEFT + 3 + RIGHT,
            'CHARGER_2': RIGHT + 3 + LEFT,
        },

        'HOUSE_10': {
            'HOUSE_1':  RIGHT + 1 + RIGHT + 8 + RIGHT + 10 + RIGHT + 1 + LEFT,
            'HOUSE_2':  LEFT + 2 + LEFT + 2 + RIGHT+ 2 + RIGHT + 5 + LEFT + 3,
            'HOUSE_3':  LEFT + 2 + LEFT + 2 + RIGHT + 2 + RIGHT + 1 + LEFT + 3 + LEFT + 1 + RIGHT + 1 + RIGHT,
            'HOUSE_4':  LEFT + 2 + LEFT + 2 + RIGHT + 2 + RIGHT + 1 + LEFT + 3 + RIGHT + 4 + LEFT,
            'HOUSE_5':  LEFT + 2 + LEFT + 2 + RIGHT + 2 + RIGHT + 3 + LEFT,
            'HOUSE_6': LEFT + 2 + LEFT + 2 + RIGHT + 2 + RIGHT + 5 + RIGHT + 2 + LEFT,
            'HOUSE_7':  RIGHT + 1 + RIGHT + 8 + LEFT + 3,
            'HOUSE_8':  RIGHT + 1 + RIGHT + 5 + RIGHT,
            'HOUSE_9':  RIGHT + 1 + RIGHT + 8 + LEFT + 3 + LEFT + 9 + RIGHT,
            'PO':       LEFT + 2 + LEFT + 2 + RIGHT + 2 + LEFT + 6,
            'CHARGER_0': RIGHT + 1 + RIGHT + 8 + RIGHT + 4 + LEFT,
            'CHARGER_1': LEFT + 2 + LEFT + 2 + RIGHT + 2 + RIGHT + 1  + LEFT + 3 + RIGHT + 1 + LEFT,
            'CHARGER_2': RIGHT + 1 + RIGHT + 8 + LEFT + 3 + LEFT + 6 + RIGHT
        }
    }


    targets = ['HOUSE_6', 'HOUSE_4', 'HOUSE_10']

    # houses to visit -> WILL BE SOMEHOW PASSED TO HERE
    node = TSPRouteServer(graph, targets)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
