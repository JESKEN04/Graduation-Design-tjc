#!/usr/bin/env python3
from typing import Dict

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition


class FormationConsensusNode(Node):
    def __init__(self):
        super().__init__('formation_consensus_node')
        self.declare_parameter('uav_count', 3)
        self.declare_parameter('consensus_gain', 0.35)
        self.declare_parameter('publish_hz', 10.0)

        self.n = int(self.get_parameter('uav_count').value)
        self.k = float(self.get_parameter('consensus_gain').value)
        hz = float(self.get_parameter('publish_hz').value)

        self.positions: Dict[int, np.ndarray] = {}
        self.publishers = {}

        # 三角编队偏移
        self.offsets = {
            1: np.array([0.0, 0.0, -35.0]),
            2: np.array([8.0, 4.0, -35.0]),
            3: np.array([8.0, -4.0, -35.0]),
        }

        for i in range(1, self.n + 1):
            self.create_subscription(
                VehicleLocalPosition,
                f'/uav{i}/fmu/out/vehicle_local_position',
                lambda msg, idx=i: self._on_position(idx, msg),
                10,
            )
            self.publishers[i] = self.create_publisher(PoseStamped, f'/uav{i}/formation_setpoint', 10)

        self.timer = self.create_timer(1.0 / hz, self._publish_setpoints)
        self.get_logger().info('Formation consensus node started.')

    def _on_position(self, idx: int, msg: VehicleLocalPosition):
        self.positions[idx] = np.array([msg.x, msg.y, msg.z], dtype=float)

    def _publish_setpoints(self):
        if len(self.positions) < self.n:
            return

        centroid = np.mean(np.vstack([self.positions[i] for i in range(1, self.n + 1)]), axis=0)

        for i in range(1, self.n + 1):
            xi = self.positions[i]
            ui = np.zeros(3)
            for j in range(1, self.n + 1):
                if i == j:
                    continue
                desired = self.offsets[j] - self.offsets[i]
                ui += (self.positions[j] - xi - desired)

            target = xi + self.k * ui + 0.15 * (centroid + self.offsets[i] - xi)
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(target[0])
            msg.pose.position.y = float(target[1])
            msg.pose.position.z = float(target[2])
            self.publishers[i].publish(msg)


def main():
    rclpy.init()
    node = FormationConsensusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
