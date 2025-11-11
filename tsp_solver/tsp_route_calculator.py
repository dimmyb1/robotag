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

    def calculate_callback(self, request, response):
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
        'H2': {'PostOffice': 3, 'H1': 1, 'H3': 2},
        'H3': {'H1': 4, 'H2': 2},
    }
    targets = ['H1', 'H3']  # houses to visit -> WILL BE SOMEHOW PASSED TO HERE
    node = TSPRouteService(graph, targets)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
