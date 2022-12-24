import yaml
import rclpy
from rclpy.node import Node
import pybullet as p

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf2_ros import TransformBroadcaster


class ConfigurationError(Exception):
    pass


class RPBIBaseNode(Node):

    def __init__(
            self,
            node_name,
            server_node=False,
            setup_tf_listener=False,
            setup_tf_broadcaster=False,
    ):

        # Class attributes
        self.server_node = server_node
        self.p = p
        self.setup_tf_listener = setup_tf_listener
        self.setup_tf_broadcaster = setup_tf_broadcaster

        # Initialize ROS node
        super().__init__(node_name)

        # Get configuration filename
        self.declare_parameter('config_filename')
        config_filename = self.get_parameter('config_filename').get_parameter_value().string_value

        # Load configuration
        with open(config_filename, 'r') as config:
            self.config = yaml.load(config, Loader=yaml.FullLoader)

        # Setup PyBullet connection configuration
        connect_config = {}
        if server_node:
            connection_mode = self.p.GUI_SERVER  # only the rpbi_node.py should have this mode
            if 'options' in self.config.get('connect', {}):
                connect_config['options'] = self.config['connect']['options']
        else:
            connection_mode = self.p.SHARED_MEMORY

        # Connect to PyBullet
        self.client_id = self.p.connect(connection_mode, **connect_config)

        # Setup tf broadcaster
        self.tf_broadcaster = None
        if setup_tf_broadcaster:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Setup transform listener
        self.tf_buffer = None
        self.tf_listener = None
        if setup_tf_listener:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

    @property
    def name(self):
        name = self.config.get('name', '')
        if (not self.server_node) and (not name):
            # For non-server nodes a name must always be provided
            raise ConfigurationError("A name was not provided!")
        return name

    def get_required(self, label, config=None):

        # Use self.config by default
        if config is None:
            config = self.config

        # Extract required configuration parameter
        required = None
        try:
            required = config[label]
        except KeyError as ex:
            pass

        # Throw error when parameter was not extracted 
        if required is None:
            raise ConfigurationError(f"Missing configuration '{label}'")

        return required

    def get_tf(self, child_frame_id, parent_frame_id='rpbi/world'):

        try:

            # Lookup transform
            tf = self.tf_buffer.lookup_transform(
                parent_frame_id,
                child_frame_id,
                rclpy.time.Time(),
            )

        except TransformException as ex:

            # Handle error
            err_msg = f'Could not find transform {self.frame_id} in {parent_frame_id}:\n{ex}'
            self.get_logger().info(err_msg)
            return

        # Extract pose data
        tr = tf.transform.translation
        ro = tf.trassform.rotation
        position = [tr.x, tr.y, tr.z]
        quaternion = [ro.x, ro.y, ro.z, ro.w]

        return position, quaternion


    def set_tf(self, position, quaternion, child_frame_id, parent_frame_id='rpbi/world'):

        # Pack transform
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent_frame_id
        tf.child_frame_id = child_frame_id
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = position[2]
        tf.transform.rotation.x = quaternion[0]
        tf.transform.rotation.y = quaternion[1]
        tf.transform.rotation.z = quaternion[2]
        tf.transform.rotation.w = quaternion[3]

        # Send transform
        self.tf_broadcaster.sendTransform(tf)

    def close(self):
        self.p.disconnect()


def main_func(node_cls):
    """Main function"""

    # Initialize ROS node
    rclpy.init()
    node = node_cls()

    try:

        # Start spinnning
        rclpy.spin(node)

    except Exception as ex:

        # Report exception
        node.get_logger().error(f"Error thrown by node: {ex}")

    finally:

        # Disconnect from PyBullet/ROS
        node.close()
        node.destroy_node()
        rclpy.shutdown()
