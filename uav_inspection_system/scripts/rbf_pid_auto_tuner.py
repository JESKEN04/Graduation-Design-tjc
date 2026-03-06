#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


@dataclass
class UavBuffer:
    phi: List[np.ndarray]
    targets: List[np.ndarray]
    weight: np.ndarray


class RBFPIDAutoTuner(Node):
    def __init__(self) -> None:
        super().__init__('rbf_pid_auto_tuner')
        self.declare_parameter('uav_count', 3)
        self.declare_parameter('sample_window', 250)
        self.declare_parameter('centers_per_dim', 3)
        self.declare_parameter('regularization', 0.08)
        self.declare_parameter('publish_topic_suffix', 'rbf_pid_correction')

        self.uav_count = int(self.get_parameter('uav_count').value)
        self.sample_window = int(self.get_parameter('sample_window').value)
        self.regularization = float(self.get_parameter('regularization').value)

        self.centers = np.linspace(-3.0, 3.0, int(self.get_parameter('centers_per_dim').value))
        self.sigma = 1.8

        self.buffers: Dict[int, UavBuffer] = {}
        self.last_att: Dict[int, VehicleAttitude] = {}
        self.last_pos: Dict[int, VehicleLocalPosition] = {}

        suffix = str(self.get_parameter('publish_topic_suffix').value)
        self.pub = {}
        for i in range(1, self.uav_count + 1):
            self.buffers[i] = UavBuffer(phi=[], targets=[], weight=np.zeros((len(self._rbf_feature(np.zeros(3))), 3)))
            self.create_subscription(VehicleLocalPosition, f'/uav{i}/fmu/out/vehicle_local_position', lambda msg, idx=i: self._on_pos(idx, msg), 10)
            self.create_subscription(VehicleAttitude, f'/uav{i}/fmu/out/vehicle_attitude', lambda msg, idx=i: self._on_att(idx, msg), 10)
            self.pub[i] = self.create_publisher(Float32MultiArray, f'/uav{i}/{suffix}', 10)

        self.timer = self.create_timer(0.2, self._fit_and_publish)
        self.get_logger().info('RBF PID auto tuner started.')

    def _on_pos(self, idx: int, msg: VehicleLocalPosition) -> None:
        self.last_pos[idx] = msg
        self._sample(idx)

    def _on_att(self, idx: int, msg: VehicleAttitude) -> None:
        self.last_att[idx] = msg
        self._sample(idx)

    def _sample(self, idx: int) -> None:
        if idx not in self.last_pos or idx not in self.last_att:
            return
        pos = self.last_pos[idx]
        vel = np.array([pos.vx, pos.vy, pos.vz], dtype=float)
        acc_est = np.array([pos.ax, pos.ay, pos.az], dtype=float)

        state = np.clip(np.hstack((vel[0], vel[1], vel[2])), -6.0, 6.0)
        phi = self._rbf_feature(state)

        # 希望将误差加速度映射为 PID 增益补偿的代理目标（P/I/D 分别对应 xyz）
        target = np.clip(np.array([
            0.18 * abs(acc_est[0]) + 0.07 * abs(vel[0]),
            0.18 * abs(acc_est[1]) + 0.07 * abs(vel[1]),
            0.12 * abs(acc_est[2]) + 0.06 * abs(vel[2]),
        ]), 0.0, 1.8)

        buf = self.buffers[idx]
        buf.phi.append(phi)
        buf.targets.append(target)
        if len(buf.phi) > self.sample_window:
            buf.phi.pop(0)
            buf.targets.pop(0)

    def _rbf_feature(self, state_xyz: np.ndarray) -> np.ndarray:
        feats = [1.0]
        for cx in self.centers:
            for cy in self.centers:
                for cz in self.centers:
                    d = (state_xyz[0] - cx) ** 2 + (state_xyz[1] - cy) ** 2 + (state_xyz[2] - cz) ** 2
                    feats.append(math.exp(-d / (2.0 * self.sigma**2)))
        return np.array(feats, dtype=float)

    def _fit_and_publish(self) -> None:
        for idx, buf in self.buffers.items():
            if len(buf.phi) < max(30, self.sample_window // 3):
                continue

            phi_mat = np.vstack(buf.phi)
            y_mat = np.vstack(buf.targets)

            reg_eye = self.regularization * np.eye(phi_mat.shape[1])
            inv = np.linalg.pinv(phi_mat.T @ phi_mat + reg_eye)
            buf.weight = inv @ phi_mat.T @ y_mat

            pred = phi_mat[-1] @ buf.weight
            msg = Float32MultiArray()
            msg.data = [float(pred[0]), float(pred[1]), float(pred[2])]
            self.pub[idx].publish(msg)


def main() -> None:
    rclpy.init()
    node = RBFPIDAutoTuner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
