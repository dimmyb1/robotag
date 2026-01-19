#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json
import subprocess
import time

HOUSES = [f'HOUSE_{i}' for i in range(1, 11)]

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.publisher = self.create_publisher(String, 'tsp_targets', 10)
        self.subscription = self.create_subscription(
            String,
            'tsp_route',
            self.route_callback,
            10
        )
        self.optimized_targets = None

    def generate_targets(self):
        return random.sample(HOUSES, 3)

    def publish_targets(self, targets):
        msg = String()
        msg.data = json.dumps(targets)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published targets: {targets}")

    def route_callback(self, msg):
        self.optimized_targets = json.loads(msg.data)
        self.get_logger().info(f"Received optimized route: {self.optimized_targets}")

    def go_to_house(self, start, target):
        self.get_logger().info(f"Navigating from {start} → {target}")
        subprocess.run([
            'ros2', 'run', 'my_robot_bringup/scripts', 'camera_follower',
            '--ros-args',
            '-p', f'target_house:={target}',
            '-p', f'start:={start}'
        ])

    def run_mission(self):
        targets = self.generate_targets()
        self.publish_targets(targets)

        # Wait until the server publishes the optimized route
        while self.optimized_targets is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        current = 'PO'
        for house in self.optimized_targets:
            self.go_to_house(current, house)
            current = house
            time.sleep(2)

        self.go_to_house(current, 'PO')
        self.get_logger().info("Mission complete")

def main():
    rclpy.init()
    node = MissionController()
    node.run_mission()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
