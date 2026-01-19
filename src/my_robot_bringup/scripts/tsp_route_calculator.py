#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from tsp_solver_node import GraphTSP
# from brute_force import BruteForceTSP
import json
import time 
from config import GRAPH

class TSPRouteServer(Node):
    def __init__(self, targets, graph=GRAPH):
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

# def main():
#     rclpy.init()
#     #for graph directory (shortest distance directory)
#     LEFT = 1.0
#     RIGHT = 1.0
#     #for directions directory (intersections directory)
#     B = 0 #BACKWARDS
#     L = 1 #LEFT
#     F = 2 #FORWARDS
#     R = 3 #RIGHT
#     turnRight = True
    

#     # graph = {
#     #     'PostOffice': {'H1': 2, 'H2': 3},
#     #     'H1': {'PostOffice': 2, 'H2': 1},
#     #     'H2': {'H1': 4, 'H3': 2},
#     #     'H3': {'H1': 3, 'H2': 1}, 
#     #     # even though H2 is less from H3 the whole path would be longer therefore took H1
#     # }
#     # graph = {
#     #     'PostOffice': {'H1': 2, 'H2': 3},
#     #     'H1': {'PostOffice': 10, 'H2': 1},
#     #     'H2': {'PostOffice':1,'H1': 4, 'H3': 2},
#     #     'H3': {'H1': 1, 'H2': 3},
#     #     # ['PostOffice', 'H1', 'H2', 'H3', 'H1', 'H2', 'PostOffice']

#     # }

#     # graph = {
#     #     'PostOffice': {'H1': 4, 'H2': 6, 'H3': 8, 'H4': 5},
#     #     'H1': {'PostOffice': 4, 'H2': 2, 'H5': 7, 'H6': 3},
#     #     'H2': {'PostOffice': 6, 'H1': 2, 'H3': 4, 'H5': 5, 'H7': 6},
#     #     'H3': {'PostOffice': 8, 'H2': 4, 'H4': 3, 'H6': 6, 'H8': 5},
#     #     'H4': {'PostOffice': 5, 'H3': 3, 'H7': 4, 'H8': 6},
#     #     'H5': {'H1': 7, 'H2': 5, 'H6': 4, 'H9': 8},
#     #     'H6': {'H1': 3, 'H3': 6, 'H5': 4, 'H7': 5, 'H10': 6},
#     #     'H7': {'H2': 6, 'H4': 4, 'H6': 5, 'H8': 3, 'H1': 5},
#     #     'H8': {'H3': 5, 'H4': 6, 'H7': 3, 'H9': 4, 'H2': 7},
#     #     'H9': {'H5': 8, 'H8': 4, 'H10': 3},
#     #     'H10': {'H6': 6, 'H9': 3},
#     # }


#     # Full path: ['PostOffice', 'H4', 'H3', 'H6', 'H10', 'H6', 'H1', 'PostOffice']
#     targets = ['H10', 'H6', 'H4']
#     targets = ['H4', 'H6', 'H10']
#     targets = ['HOUSE_6', 'HOUSE_4', 'HOUSE_10']

#     # houses to visit -> WILL BE SOMEHOW PASSED TO HERE
#     node = TSPRouteServer(targets)
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
