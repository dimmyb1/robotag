#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import String
from config import BOX_POSITIONS
from rclpy.qos import QoSProfile, DurabilityPolicy

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
        # self.wait_for_gazebo()

        self.spawned = False
        self.spawned_house = None

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub = self.create_subscription(
            String,
            'spawn_box_for_house',
            self.spawn_box_callback,
            qos
        )

    def spawn_box_callback(self, msg):
      house = msg.data

      if self.spawned:
          self.get_logger().info(
              f"Box already spawned for {self.spawned_house}, "
              f"ignoring request for {house}"
          )
          return

      self.get_logger().info(f"Spawning box for {house}")

      self.spawn_box_for_house(house)

      self.spawned = True
      self.spawned_house = house
    
    def spawn_box_for_house(self, house):
      box_positions = BOX_POSITIONS

      if house not in box_positions:
          self.get_logger().error(f"Unknown house {house}")
          return

      x, y, z = box_positions[house]
      self.spawn_box_at(x, y, z)

    def spawn_box_at(self, x, y, z):        
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
          <ambient>0.5 0.0 0.5 1</ambient>
          <diffuse>0.5 0.0 0.5 1</diffuse>
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()