#!/usr/bin/env python3
import tkinter
from std_msgs.msg import Int64
from rpbi.ros_node import RosNode
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from ros_pybullet_interface.srv import ResetJointState
from custom_ros_tools.ros_comm import get_srv_handler

class Window(tkinter.Frame):

    def __init__(self, node, master):
        tkinter.Frame.__init__(self, master)
        self.master = master
        self.node = node

        # widget can take all window
        self.pack(fill=tkinter.BOTH, expand=1)

        # create button, link it to clickExitButton()
        self.start_button = tkinter.Button(self, text="Start", command=self.start_button_handler)
        self.step_button = tkinter.Button(self, text="Step", command=self.step_button_handler)
        self.stop_button = tkinter.Button(self, text="Stop", command=self.stop_button_handler)
        self.send_button = tkinter.Button(self, text='Send', command=self.send_button_handler)

        self.start_button.place(x=0, y=0)
        self.step_button.place(x=0, y=100)
        self.stop_button.place(x=0, y=200)
        self.send_button.place(x=100, y=0)

        self.text_box = tkinter.Text(self, height=100, width=200)
        self.text_box.place(x=100, y=300)
        self.text_box.insert(tkinter.END, "Initialization")

        self.master.wm_title("ROS-PyBullet Interface Controls")
        self.master.geometry("500x500")

    def start_button_handler(self):
        self.node.start_button_handler()

    def step_button_handler(self):
        self.node.step_button_handler()

    def stop_button_handler(self):
        self.node.stop_button_handler()

    def send_button_handler(self):
        self.node.send_robot()

class Node(RosNode):

    def __init__(self):
        super().__init__('rpbi_controls_node')
        self.on_shutdown(self.close)
        self.config = self.get_param('~config', {}).get('controls', {})
        self.Subscriber('rpbi/status', Int64, self.status_callback)
        self.start_button_handler = get_srv_handler('rpbi/start', Trigger, persistent=True)
        self.step_button_handler = get_srv_handler('rpbi/step', Trigger, persistent=True)
        self.stop_button_handler = get_srv_handler('rpbi/stop', Trigger, persistent=True)

        robot_name = self.config.get('robot_name', None)
        if robot_name:
            self.send_button_handler = get_srv_handler(f'rpbi/{robot_name}/move_to_initial_joint_state', ResetJointState, persistent=True)
        else:
            self.send_button_handler = None            
        self.root = tkinter.Tk()
        self.app = Window(self, self.root)

    def status_callback(self, msg):
        if not hasattr(self, 'app'): return
        self.app.text_box.delete("1.0","end")
        self.app.text_box.insert(tkinter.END, f"Status: {msg.data}")

    def send_robot(self, state=None):
        if self.send_button_handler:
            self.send_button_handler(JointState(), 6.0)

    def close(self):
        self.root.quit()

    def spin(self):
        self.root.mainloop()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
