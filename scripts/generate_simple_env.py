#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity

class WallSpawner(Node):
    def __init__(self):
        super().__init__('generate_open_box')

        self.client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        # =========================
        # USER CONFIG (EDIT HERE)
        # =========================

        # Outer box
        x_min = 2.0
        x_max = 20.0
        y_min = -10.0
        y_max = 10.0

        # Inner wall
        inner_x = 7.0
        inner_y_min = -3.0
        inner_y_max = 3.0

        # Wall properties
        wall_thickness = 0.5
        wall_height = 5.0

        # =========================
        # BUILD WALLS
        # =========================

        # 1. Bottom wall (y = y_min)
        self.spawn_wall_segment(
            name="wall_bottom",
            x1=x_min, y1=y_min,
            x2=x_max, y2=y_min,
            thickness=wall_thickness,
            height=wall_height
        )

        # 2. Top wall (y = y_max)
        self.spawn_wall_segment(
            name="wall_top",
            x1=x_min, y1=y_max,
            x2=x_max, y2=y_max,
            thickness=wall_thickness,
            height=wall_height
        )

        # 3. Right wall (connects them)
        self.spawn_wall_segment(
            name="wall_right",
            x1=x_max, y1=y_min,
            x2=x_max, y2=y_max,
            thickness=wall_thickness,
            height=wall_height
        )

        # 4. Inner wall
        self.spawn_wall_segment(
            name="wall_inner",
            x1=inner_x, y1=inner_y_min,
            x2=inner_x, y2=inner_y_max,
            thickness=wall_thickness,
            height=wall_height
        )

        self.get_logger().info("Open box with inner wall spawned.")

    # =========================
    # Generic wall segment
    # =========================
    def spawn_wall_segment(self, name, x1, y1, x2, y2, thickness, height):

        dx = x2 - x1
        dy = y2 - y1

        length = (dx**2 + dy**2) ** 0.5

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        yaw = 0.0
        if length > 1e-6:
            import math
            yaw = math.atan2(dy, dx)

        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        if abs(yaw) > 1e-6:
            import math
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)

        pose = Pose()
        pose.position.x = cx
        pose.position.y = cy
        pose.position.z = height / 2.0
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # Box: length along x-axis in local frame → we align via yaw
        geometry = f"<box><size>{length} {thickness} {height}</size></box>"

        sdf = f"""
        <sdf version='1.6'>
          <model name='{name}'>
            <static>true</static>
            <link name='link'>
              <collision name='collision'>
                <geometry>{geometry}</geometry>
              </collision>
              <visual name='visual'>
                <geometry>{geometry}</geometry>
              </visual>
            </link>
          </model>
        </sdf>
        """

        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.robot_namespace = ""
        req.initial_pose = pose
        req.reference_frame = "world"

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

# =========================
# Main
# =========================
def main():
    rclpy.init()
    node = WallSpawner()
    rclpy.shutdown()

if __name__ == "__main__":
    main()