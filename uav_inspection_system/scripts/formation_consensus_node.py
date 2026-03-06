#!/usr/bin/env python3


import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node



class FormationConsensusNode(Node):
    def __init__(self):
        super().__init__('formation_consensus_node')



if __name__ == '__main__':
    main()
