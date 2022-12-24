from .node import RPBIBaseNode, main_func


class VisualObjectNode(RPBIBaseNode):


    """This object should be used to simply visualize objects in PyBullet."""


    def __init__(self):
        """Node constructor"""

        # Initialize ROS node
        super().__init__('visual_object_node', setup_tf_listener=True)

        # Setup visual object
        create_multi_body = self.createMultiBody
        create_multi_body['baseVisualShapeIndex'] = self.p.createVisualShape(
            **self.createVisualShape
        )
        self.body_unique_id = self.p.createMultiBody(**create_multi_body)

        # Start timer
        if self.frame_id:
            dt = 1.0/float(self.frame_listener_frequency)
            self.create_timer(dt, self.frame_listener_timer_callback)


    # ----------------------------------
    # Properties/configuration


    @property
    def frame_id(self):
        """The frame that the node will use to update the object position/orientation."""
        return self.config.get('frame_id', '')


    @property
    def frame_listener_frequency(self):
        """Sampling frequency for the frame listener."""
        return self.config.get('frame_listener_frequency', 20)


    @property
    def createVisualShape(self):
        """https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.q1gn7v6o58bf"""
        create_visual_shape = self.get_required('createVisualShape')

        # Handle shape type
        shape_type = self.get_required('shapeType', config=create_visual_shape)
        if isinstance(shape_type, str):
            create_visual_shape['shapeType'] = getattr(self.p, shape_type)

        return create_visual_shape


    @property
    def createMultiBody(self):
        """https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.jgm6tud6blu3"""

        create_multi_body = self.config.get('createMultiBody', {})

        # Ensure the configuration doesn't contain baseCollisionShapeIndex
        try:
            del create_multi_body['baseCollisionShapeIndex']
        except KeyError as ex:
            pass

        # Ensure the configuration doesn't contain baseCollisionShapeIndex
        try:
            del create_multi_body['baseVisualShapeIndex']
        except KeyError as ex:
            pass

        return create_multi_body


    # ----------------------------------
    # Class methods


    def frame_listener_timer_callback(self):
        """Timer callback for the frame listener."""

        # Get pose
        pose = self.get_tf(self.frame_id)
        if pose is None: return

        # Extract data
        position, orientation = pose

        # Reset base position/orientation
        self.p.resetBasePositionAndOrientation(self.body_unique_id, position, orientation)


def main():
    """Main program."""
    main_func(VisualObjectNode)
