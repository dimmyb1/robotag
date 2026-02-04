#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import Trigger
import json
import random
from rclpy.qos import QoSProfile, DurabilityPolicy

HOUSES = [f'HOUSE_{i}' for i in range(1, 11)]

class DeliverobotTSPClient(Node):
    def __init__(self):
        super().__init__('deliverobot_tsp_client')

        # Create TSP service client
        self.client = self.create_client(Trigger, 'calculate_tsp_route')

        # Generate 3 random targets
        self.targets = random.sample(HOUSES, 3)
        
        # Hardcoded targets for testing
        self.get_logger().info(f"Generated targets: {self.targets}")

        # Publish targets for getting the optimised path 
        self.publisher = self.create_publisher(String, 'tsp_targets', 10)
        # Publish the house to go to - to camera follower
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.nav_publisher = self.create_publisher(String, 'navigate_to_house', qos)

        self.targets_sent = False
        self.create_timer(0.5, self.publish_targets_once)

        # Start calling TSP service
        self.call_timer = self.create_timer(1.0, self.call_tsp_service_until_ready)

        self.current = "PO"
        self.route_index = 0

        # Subscribne to the navigation done to know if the camera follower arrived at the next house/building
        self.done_sub = self.create_subscription(
            String,
            'navigation_done',
            self.navigation_done_callback,
            10
        )

    def publish_targets_once(self):
        if self.targets_sent:
            return

        msg = String()
        msg.data = json.dumps(self.targets)
        self.publisher.publish(msg)
        # self.get_logger().info(f"Published targets to topic: {self.targets}")
        self.targets_sent = True
    
    def start_mission(self):
        self.get_logger().info("Starting mission")
        self.send_next_target()

    def call_tsp_service_until_ready(self):
        if not self.client.service_is_ready():
            self.get_logger().info("Waiting for TSP service...")
            return

        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.tsp_response_callback)

        # Stop timer after first call
        self.call_timer.cancel()

    def tsp_response_callback(self, future):
        response = future.result()

        if not response.success:
            self.get_logger().warn(response.message)
            return

        data = json.loads(response.message)
        self.optimized_route = data['route']
        self.get_logger().info(f"Optimized route received: {self.optimized_route}")
        self.start_mission()

    def go_to_house(self, start, target):
        self.get_logger().info(f"Navigating from {start} → {target}")
        msg = String()
        msg.data = json.dumps({'start': start, 'target': target})
        self.nav_publisher.publish(msg)
        # self.get_logger().info(f"Published navigation command: {start} → {target}")

    def send_next_target(self):
        if self.route_index >= len(self.optimized_route):
            self.get_logger().info("Mission complete!")
            self.destroy_node()
            rclpy.shutdown()
            return

        target = self.optimized_route[self.route_index]

        if target == self.current:
            self.route_index += 1
            self.send_next_target()
            return

        self.go_to_house(self.current, target)

    def navigation_done_callback(self, msg):
        data = json.loads(msg.data)
        reached = data['reached']

        self.get_logger().info(f"Confirmed arrival at {reached}")

        self.current = reached
        self.route_index += 1

        self.send_next_target()

def main(args=None):
    rclpy.init(args=args)
    node = DeliverobotTSPClient()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
