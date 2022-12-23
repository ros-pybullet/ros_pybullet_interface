import rclpy
from rclpy.node import Node
import pybullet as p

import numpy as np

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped


class DynamicObjectNode(Node):

    def __init__(self):
        super().__init__('dynamic_object_node')

        # Connect to PyBullet
        self.client_id = p.connect(p.SHARED_MEMORY)
        
        p.createMultiBody(
            baseMass=1,
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, rgbaColor=[1, 0, 0, 1]),
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE),
            basePosition=[0, 0, 2],
        )


def main():
    rclpy.init()
    node = DynamicObjectNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


    
