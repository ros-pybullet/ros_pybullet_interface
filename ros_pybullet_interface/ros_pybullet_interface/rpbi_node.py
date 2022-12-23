import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import pybullet as p

default_sampling_frequency = 20

class ActiveStatusPublisher:

    def __init__(self, node):
        node.declare_parameter('active_publish_frequency', default_sampling_frequency)
        active_publish_frequency = int(node.get_parameter(
            'active_publish_frequency').get_parameter_value().integer_value
        )
        self.pub = node.create_publisher(Bool, 'rpbi/active', qos_profile=10)
        node.create_timer(1./float(active_publish_frequency), self.active_status_timer_callback)
        self.node = node

    def active_status_timer_callback(self):
        self.pub.publish(Bool(data=self.node.active))


class ROSPyBulletInterfaceNode(Node):

    def __init__(self):

        # Initialize ROS
        super().__init__('rpbi_node')

        # Setup class attributes
        self.active = False

        # Connect to PyBullet
        connect_config = {}

        self.declare_parameter('connect_options', '')
        connect_options = self.get_parameter(
            'connect_options',
        ).get_parameter_value().string_value
        if connect_options:
            connect_config['options'] = connect_options

        self.client_id = p.connect(p.GUI_SERVER, **connect_config)

        # Start active status publisher
        ActiveStatusPublisher(self)

        p.setTimeStep(0.01)
        p.setGravity(0, 0, -10)
        p.setRealTimeSimulation(1)
        self.active = True


def main():
    rclpy.init()
    node = ROSPyBulletInterfaceNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
