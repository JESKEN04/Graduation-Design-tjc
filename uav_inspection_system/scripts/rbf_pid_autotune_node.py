#!/usr/bin/env python3
"""Online RBF + RLS PID autotuner for multi-UAV inspection demo."""

import math
from dataclasses import dataclass
from typing import Dict

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


@dataclass
class RbfRlsModel:
    n_centers: int = 9
    sigma: float = 0.45
    forgetting: float = 0.995

    def __post_init__(self):
        axis = np.array([-1.0, 0.0, 1.0], dtype=np.float64)
        self.centers = np.array([(x, y) for x in axis for y in axis], dtype=np.float64)
        self.weights = np.zeros((self.n_centers, 3), dtype=np.float64)
        self.p = np.eye(self.n_centers, dtype=np.float64) * 200.0

    def _phi(self, err: np.ndarray) -> np.ndarray:
        d = self.centers - err[None, :]
        sq = np.sum(d * d, axis=1)
        return np.exp(-sq / (2.0 * self.sigma * self.sigma))

    def update(self, err: np.ndarray, target_gain_delta: np.ndarray) -> np.ndarray:
        phi = self._phi(err)
        pphi = self.p @ phi
        gain = pphi / (self.forgetting + phi.T @ pphi)
        pred = self.weights.T @ phi
        innovation = target_gain_delta - pred
        self.weights += np.outer(gain, innovation)
        self.p = (self.p - np.outer(gain, phi.T @ self.p)) / self.forgetting
        return pred + innovation


class RbfPidAutotuneNode(Node):
    """
    输入 telemetry: [ex, ey, ez, evx, evy, evz]
    输出 pid_gains: [kp, ki, kd]
    """

    def __init__(self):
        super().__init__('rbf_pid_autotune')
        self.declare_parameter('uav_count', 3)
        self.declare_parameter('base_kp', 0.60)
        self.declare_parameter('base_ki', 0.04)
        self.declare_parameter('base_kd', 0.28)

        self.uav_count = int(self.get_parameter('uav_count').value)
        self.base = np.array([
            float(self.get_parameter('base_kp').value),
            float(self.get_parameter('base_ki').value),
            float(self.get_parameter('base_kd').value),
        ], dtype=np.float64)

        self.models: Dict[int, RbfRlsModel] = {i: RbfRlsModel() for i in range(1, self.uav_count + 1)}
        self.gain_pubs = {}

        for i in range(1, self.uav_count + 1):
            self.create_subscription(
                Float32MultiArray,
                f'/uav{i}/telemetry',
                lambda msg, idx=i: self.telemetry_cb(idx, msg),
                10,
            )
            self.gain_pubs[i] = self.create_publisher(Float32MultiArray, f'/uav{i}/pid_gains', 10)

        self.get_logger().info('RBF + 最小二乘(P-RLS)自整定节点已启动')

    def telemetry_cb(self, idx: int, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            self.get_logger().warn(f'uav{idx} telemetry数据长度不足，期望6，当前{len(msg.data)}')
            return

        ex, ey, ez, evx, evy, evz = [float(v) for v in msg.data[:6]]
        pos_err = math.sqrt(ex * ex + ey * ey + ez * ez)
        vel_err = math.sqrt(evx * evx + evy * evy + evz * evz)

        # 用稳定性指标构造增益修正目标（可替换为更严格代价函数）
        target = np.array([
            min(0.45, 0.20 * pos_err + 0.05 * vel_err),
            min(0.06, 0.015 * pos_err),
            min(0.30, 0.10 * vel_err + 0.02 * pos_err),
        ], dtype=np.float64)

        model = self.models[idx]
        correction = model.update(np.array([pos_err, vel_err], dtype=np.float64), target)
        tuned = np.clip(self.base + correction, [0.3, 0.0, 0.1], [1.8, 0.3, 1.2])

        out = Float32MultiArray()
        out.data = tuned.astype(np.float32).tolist()
        self.gain_pubs[idx].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RbfPidAutotuneNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
