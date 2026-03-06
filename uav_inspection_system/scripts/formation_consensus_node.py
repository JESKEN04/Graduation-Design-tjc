#!/usr/bin/env python3
"""Graph-consensus formation command generator for 3 UAVs."""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class FormationConsensusNode(Node):
    def __init__(self):
        super().__init__('formation_consensus_node')
        self.declare_parameter('step', 0.08)
        self.step = float(self.get_parameter('step').value)

        self.L = np.array([[2., -1., -1.], [-1., 2., -1.], [-1., -1., 2.]], dtype=np.float64)
        self.offset = np.array([[0., 0., 0.], [6., 0., 0.], [3., 5.2, 0.]], dtype=np.float64)
        self.x = np.array([[-30., -8., 12.], [-34., -12., 12.], [-26., -12., 12.]], dtype=np.float64)

        self.pubs = [
            self.create_publisher(PoseStamped, f'/uav{i}/target_pose', 10)
            for i in range(1, 4)
        ]
        self.timer = self.create_timer(0.1, self.tick)

    def tick(self):
        # leader trajectory around powerline corridor
        t = self.get_clock().now().nanoseconds * 1e-9
        leader = np.array([-30.0 + 0.8 * t, 3.5 * np.sin(0.08 * t), 18.0 + 2.0 * np.sin(0.05 * t)])

        # consensus + formation keeping: dx = -L(x - offset) + leader anchor
        desired = np.vstack([leader + o for o in self.offset])
        dx = -(self.L @ (self.x - desired))
        self.x += self.step * dx

        for i in range(3):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = float(self.x[i, 0])
            msg.pose.position.y = float(self.x[i, 1])
            msg.pose.position.z = float(self.x[i, 2])
            msg.pose.orientation.w = 1.0
            self.pubs[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FormationConsensusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
