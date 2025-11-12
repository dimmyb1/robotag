#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from tsp_solver.tsp_solver_node import GraphTSP
import json

class TSPRouteService(Node):
    def __init__(self, graph, targets):
        super().__init__('tsp_route_calculator')
        self.srv = self.create_service(Trigger, 'calculate_tsp_route', self.calculate_callback)
        self.get_logger().info("TSP Route Calculator ready.")
        self.graph = graph
        self.targets = targets

    def calculate_callback(self, _, response):
        solver = GraphTSP(self.graph)
        start = 'PostOffice'
        route = solver.nearest_neighbour_tsp(start, self.targets)
        optimized = solver.two_opt(route)

        response.success = True
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
    graph = {
        'PostOffice': {'H1': 2, 'H2': 3},
        'H1': {'PostOffice': 2, 'H2': 1},
        'H2': {'H1': 4, 'H3': 2},
        'H3': {'H1': 3, 'H2': 1}, 
        # even though H2 is less from H3 the whole path would be longer therefore took H1
    }
    graph = {
        'PostOffice': {'H1': 2, 'H2': 3},
        'H1': {'PostOffice': 10, 'H2': 1},
        'H2': {'PostOffice':1,'H1': 4, 'H3': 2},
        'H3': {'H1': 1, 'H2': 3},
        # ['PostOffice', 'H1', 'H2', 'H3', 'H1', 'H2', 'PostOffice']

    }
    graph = {
        'PostOffice': {'H1': 3, 'H2': 4, 'H3': 8},
        'H1': {'PostOffice': 3, 'H2': 2, 'H4': 4},
        'H2': {'PostOffice': 4, 'H1': 2, 'H3': 3, 'H5': 5},
        'H3': {'PostOffice': 8, 'H2': 3, 'H6': 2},
        'H4': {'H1': 4, 'H5': 3, 'H7': 6},
        'H5': {'H2': 5, 'H4': 3, 'H6': 4, 'H8': 5},
        'H6': {'H3': 2, 'H5': 4, 'H9': 3},
        'H7': {'H4': 6, 'H8': 2, 'H10': 4},
        'H8': {'H5': 5, 'H7': 2, 'H9': 3},
        'H9': {'H6': 3, 'H8': 3, 'H10': 2},
        'H10': {'H7': 4, 'H9': 2, 'PostOffice': 7}
    }

    targets = ['H2', 'H5', 'H8', 'H10']  # houses to visit -> WILL BE SOMEHOW PASSED TO HERE
    node = TSPRouteService(graph, targets)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
