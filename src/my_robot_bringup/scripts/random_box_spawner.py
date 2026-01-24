#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import random
import subprocess
import time

class RandomBoxSpawner(Node):
    def __init__(self):
        super().__init__('random_box_spawner')
        
        # Define predefined positions
        self.positions = [
            (-1.5, 6.45, 0.5),
            (-2.5, 1.5, 0.5),
            (-3.5, -6.5, 0.5),
            (4.5, -4.5, 0.5)
        ]
        
        # Wait for Gazebo to be ready
        self.get_logger().info('Waiting for Gazebo to fully initialize...')
        self.wait_for_gazebo()
        
        # Spawn the box
        self.spawn_random_box()
    
    def wait_for_gazebo(self):
        """Wait until Gazebo clock is publishing"""
        self.clock_received = False
        
        def clock_callback(msg):
            self.clock_received = True
        
        # Subscribe to clock topic
        clock_sub = self.create_subscription(
            Clock,
            '/clock',
            clock_callback,
            10
        )
        
        # Wait for clock messages
        max_wait = 30.0
        start_time = time.time()
        
        while not self.clock_received and (time.time() - start_time) < max_wait:
            rclpy.spin_once(self, timeout_sec=0.5)
            if not self.clock_received:
                elapsed = int(time.time() - start_time)
                self.get_logger().info(f'Waiting for Gazebo clock... ({elapsed}s)')
        
        if self.clock_received:
            self.get_logger().info('Gazebo clock detected! Waiting for GUI to fully load...')
            time.sleep(20.0)  # Extended wait for GUI to fully load
            self.get_logger().info('Ready to spawn!')
        else:
            self.get_logger().warn('Timeout waiting for Gazebo, attempting to spawn anyway...')
        
        # Clean up subscription
        self.destroy_subscription(clock_sub)
    
    def spawn_random_box(self):
        # Choose random position
        x, y, z = random.choice(self.positions)
        
        self.get_logger().info(f'Spawning grey box at position: ({x}, {y}, {z})')
        
        # Create SDF string
        sdf_string = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="random_grey_box">
    <pose>{x} {y} {z} 0 0 0</pose>
    <static>false</static>
    <link name="box_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.16666</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.16666</iyy>
          <iyz>0</iyz>
          <izz>0.16666</izz>
        </inertia>
      </inertial>
      <collision name="box_collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        
        # Escape the SDF string for the command line
        sdf_escaped = sdf_string.replace('"', '\\"').replace('\n', ' ')
        
        # Use gz service to spawn the model with SDF string
        try:
            cmd = [
                'gz', 'service',
                '-s', '/world/empty/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '5000',
                '--req', f'sdf: "{sdf_escaped}"'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info('Grey box spawned successfully!')
            else:
                self.get_logger().error(f'Failed to spawn box: {result.stderr}')
        except Exception as e:
            self.get_logger().error(f'Error spawning box: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = RandomBoxSpawner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()