import sys
from rqt_gui.main import Main

def main():
    main = Main()
    sys.exit(main.main(sys.argv, standalone='ros_pybullet_rqt_plugin.ros2_pybullet_interface_controls.Ros2PyBulletInterfaceControls'))

if __name__ == '__main__':
    main()