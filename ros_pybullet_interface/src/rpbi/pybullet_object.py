import numpy as np
from custom_ros_tools.config import replace_package
from abc import ABC, abstractmethod


class PybulletObject(ABC):


    """Base class for objects in Pybullet."""


    def __init__(self, pb, node, config):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node
        self.pubs = {}
        self.srvs = {}
        self.subs = {}
        self.timers = {}

        # Init config
        self.config = config
        self.name = self.config['name']
        del self.config['name']

        # Setup variables
        self.body_unique_id = None
        self.offset = None  # object offset in object base frame
        self.base = None  # object base frame defined in rpbi/world
        self.visual_shape_type = None # set when create_visual_shape is called

        # Initialize object
        self.init()

        self.node.loginfo(f'added pybullet object {self.name}')


    @abstractmethod
    def init(self):
        pass


    def create_visual_shape(self, config):
        """Exposes createVisualShape."""
        if isinstance(config['shapeType'], str):
            config['shapeType'] = getattr(self.pb, config['shapeType'])
        self.visual_shape_type = config['shapeType']
        if 'fileName' in config.keys():
            config['fileName'] = replace_package(config['fileName'])
        return self.pb.createVisualShape(**config)


    def create_collision_shape(self, config):
        """Exposes createCollisionShape."""
        if isinstance(config['shapeType'], str):
            config['shapeType'] = getattr(self.pb, config['shapeType'])
        if 'fileName' in config.keys():
            config['fileName'] = replace_package(config['fileName'])
        return self.pb.createCollisionShape(**config)


    def change_dynamics(self, config, link_index=-1):
        """Exposes changeDynamics."""
        config['bodyUniqueId'] = self.body_unique_id
        config['linkIndex'] = link_index
        if 'activationState' in config.keys():
            config['activationState'] = getattr(self.pb, config['activationState'])
        self.pb.changeDynamics(**config)

    def get_dynamics(self, link_index=-1):
        """Exposes changeDynamics."""
        config = {}
        config['bodyUniqueId'] = self.body_unique_id
        config['linkIndex'] = link_index
        dynamics_info = self.pb.getDynamicsInfo(**config)
        dynamics_dict = {
            "mass": dynamics_info[0],
            "lateralFriction": dynamics_info[1],
            "localInertiaDiagonal": dynamics_info[2],
            "restitution": dynamics_info[5],
            "rollingFriction": dynamics_info[6],
            "spinningFriction": dynamics_info[7],
            "contactDamping": dynamics_info[8],
            "contactStiffness": dynamics_info[9],
            "collisionMargin": dynamics_info[11]
        }
        return dynamics_dict


    def destroy(self):
        """Removes the object from Pybullet and closes any communication with ROS."""

        # Close all ROS communication
        for t in self.timers.values():
            t.shutdown()
        for s in self.subs.values():
            s.unregister()
        for p in self.pubs.values():
            p.unregister()
        for s in self.srvs.values():
            s.shutdown()

        # Remove object from pybullet
        self.pb.removeBody(self.body_unique_id)

class PybulletObjectArray:


    def __init__(self, pb, node, config, object_type, num_objects):

        # Set pybullet instance and ROS node
        self.pb = pb
        self.node = node

        # Create object array
        self.objects = []
        for i in range(num_objects):

            # Update config
            config_i = config.copy()
            config_i['name'] = config['name'] + str(i)
            if 'object_base_tf_frame_id' in config.keys():
                if config['object_base_tf_frame_id'] != 'rpbi/world':
                    config_i['object_base_tf_frame_id'] = config['object_base_tf_frame_id'] + str(i)
            if 'tf_frame_id' in config.keys():
                config_i['tf_frame_id'] = config['tf_frame_id'] + str(i)

            # Append object
            self.objects.append(object_type(pb, node, config_i))

    def destroy(self):
        for obj in self.objects:
            obj.destroy()
