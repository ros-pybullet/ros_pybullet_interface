import os
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QProcess
from python_qt_binding.QtWidgets import QWidget
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from std_msgs.msg import Int64
from std_srvs.srv import Trigger

class PyBulletInterfaceControls(QWidget):

    def __init__(self, node, plugin=None):

        super(PyBulletInterfaceControls, self).__init__()

        self._node = node
        self.process  = QProcess(self)
        self._plugin = plugin

        _, package_path = get_resource('packages', 'ros_pybullet_rqt_plugin')
        ui_file = os.path.join(package_path, 'share', 'ros_pybullet_rqt_plugin', 'resource', 'PyBulletInterfaceControls_layout.ui')
        loadUi(ui_file, self)

        self.start_button.pressed.connect(self.start_button_handler)
        self.send_button.pressed.connect(self.send_button_handler)
        self.step_button.pressed.connect(self.step_button_handler)
        self.stop_button.pressed.connect(self.stop_button_handler)

        self.subscription = self._node.create_subscription(
            Int64,
            'rpbi/status',
            self.status_callback,
            10)
        

    def start_button_handler(self):
        self.get_srv_handler('rpbi/start', Trigger)

    def step_button_handler(self):
        self.get_srv_handler('rpbi/step', Trigger)

    def stop_button_handler(self):
        self.get_srv_handler('rpbi/stop', Trigger)

    def send_button_handler(self):
        pass

    def status_callback(self, msg):
        self.text_box.setText("Status: " + str(msg.data)) 

    def get_srv_handler(self, srv_name: str, srv_type: type):
        handler = None
        handler = self._node.create_client(srv_type, srv_name)
        handler.wait_for_service(timeout_sec = 2.0) # wait 2 seconds
        self._node.get_logger().info('Service ' + srv_name + ' not available.')
        return handler

    def start(self):
        pass

    def shutdown_plugin(self):
        pass
