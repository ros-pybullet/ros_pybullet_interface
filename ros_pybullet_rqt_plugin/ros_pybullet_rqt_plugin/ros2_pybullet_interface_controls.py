from rqt_gui_py.plugin import Plugin
from .pybullet_interface_controls_widget import PyBulletInterfaceControls
    
class Ros2PyBulletInterfaceControls(Plugin):

    def __init__(self, context):
        super(Ros2PyBulletInterfaceControls, self).__init__(context)
        self._node = context.node
        self._logger = self._node.get_logger().get_child('ros_pybullet_rqt_plugin.ros2_pybullet_interface_controls.Ros2PyBulletInterfaceControls')
        
        super(Ros2PyBulletInterfaceControls, self).__init__(context)
        self.setObjectName('Ros2PyBulletInterfaceControls')

        self._widget = PyBulletInterfaceControls(context.node, self)

        self._widget.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
