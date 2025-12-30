#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
import json

class DeliverobotTSPClient(Node):
    def __init__(self):
        super().__init__('deliverobot_tsp_client')
        self.client = self.create_client(Trigger, 'calculate_tsp_route')

    def call_response(self):
        # Wait for service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for TSP service...')

        # Send request
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        response = future.result()
        data = json.loads(response.message)
        self.get_logger().info(f"TSP {data['targets_order']}")
        self.get_logger().info(f"Full Path (including intermediate nodes): {data['route']}")
        self.get_logger().info(f"Total Distance: {data['total_distance']}")

def main(args=None):
    rclpy.init(args=args)
    node = DeliverobotTSPClient()
    node.call_response()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
