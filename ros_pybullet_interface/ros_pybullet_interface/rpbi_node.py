from .node import RPBIBaseNode, main_func

from std_msgs.msg import Bool

class ActiveStatusPublisher:

    """Publishes whether the interface is active or not on the topic rpbi/active."""

    def __init__(self, node):
        """Class constructor."""

        # Class attributes
        self.node = node

        # Setup publisher
        self.pub = self.node.create_publisher(Bool, 'rpbi/active', qos_profile=10)

        # Start timer
        dt = 1./float(self.node.active_publish_frequency)
        self.node.create_timer(dt, self.active_status_timer_callback)

    def active_status_timer_callback(self):
        """Status publisher timer callback."""
        self.pub.publish(Bool(data=self.node.active))


class ROSPyBulletInterfaceNode(RPBIBaseNode):

    """The main ROS-PyBullet Interface server node."""

    def __init__(self):
        """Node constructor"""

        # Initialize ROS
        super().__init__('rpbi_node', server_node=True)

        # Setup class attributes
        self.active = False

        # Start active status publisher
        ActiveStatusPublisher(self)

        # Setup PyBullet
        set_time_step = self.setTimeStep
        if set_time_step is not None:
            p.setTimeStep(**set_time_step)

        set_gravity = self.setGravity
        if set_gravity is not None:
            p.setGravity(**set_gravity)

        # TEMP
        p.setRealTimeSimulation(1)
        # TEMP

        self.active = True


    # ----------------------------------
    # Properties/configuration

    @property
    def active_publish_frequency(self):
        """Sampling frequency that the active status is published."""
        return self.node.get('active_publish_frequency', 20)

    @property
    def setTimeStep(self):
        """https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.5h9ku1yf3u1v"""
        return self.config.get('setTimeStep')

    @property
    def setGravity(self):
        """https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.d6ihmmtes1id"""
        return self.config.get('setGravity')


def main():
    main_func(ROSPyBulletInterfaceNode)
