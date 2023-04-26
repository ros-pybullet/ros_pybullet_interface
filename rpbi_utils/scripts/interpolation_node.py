#!/usr/bin/env python3
# license removed for brevity
import rospy

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS message types
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray

import rpbi_utils.interpolation as interpol_lib
from custom_ros_tools.config import load_config

# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------


class TrajManager:

    def __init__(self, mot_dim, interpol):

        self.nochange_win_len = interpol['nochange_window_length']
        self.inter_freq = 1.0/interpol['inter_dt']
        self.use_interp = interpol['use_interpolation']

        # number of dimentions of the trajectory plan
        # traj_planDim = mot_dim["number"]

        # information about the fixed and planned dimensions of the motion
        self.mot_dim = mot_dim

        # init struct for interpolated motion plan
        self.motion_interp_plan = np.array([])

    def get_next_waypt(self):
        """ Get function to access data from the trajectory class """

        traj_waypt = self.popFirstTrajElem()

        if traj_waypt is not None:
            return self.trans_traj_2_motion_6D(traj_waypt)
        else:
            return None

    def trans_traj_2_motion_6D(self, way_pt):
        """A function that maps dimensions of the Traj to 6D"""

        # translation
        if self.mot_dim['trans']['translation_x'] is not None:
            x = self.mot_dim['trans']['translation_x']
        else:
            x = way_pt[self.mot_dim['trans']['translation_x_index']]

        if self.mot_dim['trans']['translation_y'] is not None:
            y = self.mot_dim['trans']['translation_y']
        else:
            y = way_pt[self.mot_dim['trans']['translation_y_index']]

        if self.mot_dim['trans']['translation_z'] is not None:
            z = self.mot_dim['trans']['translation_z']
        else:
            z = way_pt[self.mot_dim['trans']['translation_z_index']]

        pos = np.array([x, y, z])

        # rotation
        # specify manually axis and angle --- used for fixed orientation
        if self.mot_dim['rotation']['rotation_repr'] == 'none':
            Ori_Rot = R.from_rotvec(np.deg2rad(
                self.mot_dim['rotation']['rotation_angle'])*np.array(self.mot_dim['rotation']['rotation_vec']))
        # specify manually axis and take angle from the planner  --- used for rotation around fixed axis
        elif self.mot_dim['rotation']['rotation_repr'] == 'theta':
            Ori_Rot = R.from_rotvec(np.array(
                self.mot_dim['rotation']['rotation_vec'])*way_pt[self.mot_dim['rotation']['rotation_vec_index'][0]])
        elif self.mot_dim['rotation']['rotation_repr'] == 'euler':
            # take euler angles directly from the planner
            idx = self.mot_dim['rotation']['rotation_vec_index']
            Ori_Rot = R.from_euler('ZYX', np.array(way_pt[idx[0]:idx[1]]))
        elif self.mot_dim['rotation']['rotation_repr'] == 'quat':
            # take quaternion directly from the planner
            idx = self.mot_dim['rotation']['rotation_vec_index']
            Ori_Rot = R.from_quat(np.array(way_pt[idx[0]:idx[1]]))

        Ori = Ori_Rot.as_quat()

        return np.hstack((pos, Ori))

    def popFirstTrajElem(self):
        """ Extract the 1st element of the motion struct to
        send to the simulation"""

        # if we use interpolation, we use the interpolated one
        # if self.use_interp:

        if self.motion_interp_plan.shape[1] == 0:
            rospy.logerr("All the trajectory data has been consumed")
            return None

        nextWaypt = self.motion_interp_plan[:, 0]
        self.motion_interp_plan = np.delete(self.motion_interp_plan, 0, 1)
        self.time_interp_plan = np.delete(self.time_interp_plan, 0)

        return nextWaypt

    def update_traj(self, new_traj):
        """ Implemented for receiding horizon and MPC loops
            Needs to be extensively tested, when a receiding horizon or MPC
            motion planner is available                    """

        # from which knot and on of the new traj, do we what to use?
        # ATTENTION: This should be become a parameter, when receiding horizon/MPC can be tested
        index_of_1st_knot = 1

        # # time is always the first row
        time_vec = new_traj[0, :]

        # get the index where to new trajectory data should be inserted
        insertIndex = self.find_insert_index(time_vec, index_of_1st_knot)

        # create new time vector
        time_vector = np.append(self.time_interp_plan[:insertIndex], self.time_interp_plan[insertIndex] +
                                time_vec[index_of_1st_knot:].reshape(1, time_vec[index_of_1st_knot:].shape[0]))
        time_vector = time_vector.reshape(1, time_vector.shape[0])

        numRows, _ = new_traj.shape
        midRow = self.mot_dim['number'] + 1

        # first half rows denote position
        traj_plan = np.hstack(
            (self.motion_interp_plan[:, :insertIndex],  new_traj[1:midRow, index_of_1st_knot:]))
        # second half rows denote velocity
        dtraj_plan_noaction_window = np.diff(self.motion_interp_plan[:, :insertIndex+1])
        dtraj_plan = np.hstack((dtraj_plan_noaction_window,  new_traj[midRow:, index_of_1st_knot:]))

        # interpolate
        if self.use_interp:
            self.time_interp_plan, self.motion_interp_plan = self.compute_interp_traj(
                time_vector, traj_plan, dtraj_plan)
        else:
            self.time_interp_plan = time_vector
            self.motion_interp_plan = traj_plan

    def find_insert_index(self, time_vector, index_1knot):

        # find where along the time axis should the data be added
        insertion_index = np.where(self.time_interp_plan > time_vector[index_1knot])
        #  if first knot of new traj is after the duration of the current trajectory
        if insertion_index[0].size == 0:
            insertion_index = -1
        else:
            #  if first knot of new traj is within the duration of the current trajectory
            insertion_index = insertion_index[0][0]
            if insertion_index < self.nochange_win_len:
                #  if first knot of new traj is within the nochange_window of the current trajectory
                rospy.logerr(
                    "The first new knot of the trajectory is timed to be within the nochange_window region! It will be overriden.")
                insertion_index = self.nochange_win_len

        return insertion_index

    def set_init_traj(self, new_traj):
        """ More comments are needed """

        # # time is always the first row
        time_vec = new_traj[0, :]
        time_vector = time_vec.reshape(1, time_vec.shape[0])

        numRows, _ = new_traj.shape
        midRow = self.mot_dim['number'] + 1

        # first half rows denote position
        traj_plan = new_traj[1:midRow, :]

        # second half rows denote velocity
        dtraj_plan = new_traj[midRow:, :]

        # interpolate
        if self.use_interp:
            self.time_interp_plan, self.motion_interp_plan = self.compute_interp_traj(
                time_vector, traj_plan, dtraj_plan)
        else:
            self.time_interp_plan = time_vector
            self.motion_interp_plan = traj_plan

    def compute_interp_traj(self, time_vector, traj_plan, dtraj_plan):
        """ Compute the interpolated trajectory from the planning traj

            ATTENTION: if the angular motion is provide in 3D, it should be quaternions
            to used slerp... Implementation pending....
                                                                             """
        temp_motion_interp_plan = np.empty((0))
        trajDim = traj_plan.shape[0]
        row_len = 0

        rot_repr = self.mot_dim['rotation']['rotation_repr']
        if rot_repr == 'none' or rot_repr == 'theta':
            # for each dimension of the motion compute the interpolated trajectory
            for i in range(trajDim):
                inter_seq_time, inter_seq_i = interpol_lib.interpolate_cubic_hermite_spline(
                    time_vector[0, :], traj_plan[i, :], dtraj_plan[i, :], sample_freq=self.inter_freq, plot_flag=False, plot_title="DimVsTime")
                temp_motion_interp_plan = np.append(temp_motion_interp_plan, inter_seq_i, axis=0)

            row_len = trajDim
        else:
            # for each translational dimension of the motion compute the interpolated trajectory
            # using position and derivatives
            if self.mot_dim['trans']['translation_x'] is None:
                i = self.mot_dim['trans']['translation_x_index']
                inter_seq_time, inter_seq_i = interpol_lib.interpolate_cubic_hermite_spline(
                    time_vector[0, :], traj_plan[i, :], dtraj_plan[i, :], sample_freq=self.inter_freq, plot_flag=False, plot_title="XVsTime")
                temp_motion_interp_plan = np.append(temp_motion_interp_plan, inter_seq_i, axis=0)
                row_len += 1

            if self.mot_dim['trans']['translation_y'] is None:
                i = self.mot_dim['trans']['translation_y_index']
                inter_seq_time, inter_seq_i = interpol_lib.interpolate_cubic_hermite_spline(
                    time_vector[0, :], traj_plan[i, :], dtraj_plan[i, :], sample_freq=self.inter_freq, plot_flag=False, plot_title="YVsTime")
                temp_motion_interp_plan = np.append(temp_motion_interp_plan, inter_seq_i, axis=0)
                row_len += 1

            if self.mot_dim['trans']['translation_z'] is None:
                i = self.mot_dim['trans']['translation_z_index']
                inter_seq_time, inter_seq_i = interpol_lib.interpolate_cubic_hermite_spline(
                    time_vector[0, :], traj_plan[i, :], dtraj_plan[i, :], sample_freq=self.inter_freq, plot_flag=False, plot_title="ZVsTime")
                temp_motion_interp_plan = np.append(temp_motion_interp_plan, inter_seq_i, axis=0)
                row_len += 1

            # for each rotational dimension of the motion compute the interpolated trajectory
            # using only the value of the Euler angles and neglect angular velocity
            if rot_repr == 'euler':
                rot_idx = self.mot_dim['rotation']['rotation_vec_index']
                for i in range(rot_idx[0], rot_idx[1]):
                    # inter_seq_time, inter_seq_i = interpol_lib.interpolate_poly_fit(time_vector[0,:],  traj_plan[i,:], polyOrder = 3, sample_freq=self.inter_freq, plot_flag=True, plot_title="EulerVsTime")
                    inter_seq_time, inter_seq_i = interpol_lib.interpolate_cubic_spline(
                        time_vector[0, :],  traj_plan[i, :], sample_freq=self.inter_freq, plot_flag=False, plot_title="EulerVsTime")
                    temp_motion_interp_plan = np.append(
                        temp_motion_interp_plan, inter_seq_i, axis=0)
                row_len += 3

            # for each rotational dimension of the motion compute the interpolated trajectory
            # using slerp for quaternions
            elif rot_repr == 'quat':
                # we need to do slerp
                rot_idx = self.mot_dim['rotation']['rotation_vec_index']
                # inter_seq_time, inter_seq_i = interpol_lib.interpolate_linearly_quaternions(time_vector[0,:],  traj_plan[rot_idx[0]:rot_idx[1],:], sample_freq=self.inter_freq)
                inter_seq_time, inter_seq_i = interpol_lib.interpolate_cubic_quaternions(
                    time_vector[0, :],  traj_plan[rot_idx[0]:rot_idx[1], :], sample_freq=self.inter_freq)
                for i in range(inter_seq_i.shape[1]):
                    temp_motion_interp_plan = np.append(
                        temp_motion_interp_plan, inter_seq_i[:, i], axis=0)
                row_len += 4

        # reshape to have a dimension per row
        col_len = inter_seq_i.shape[0]
        temp_motion_interp_plan = temp_motion_interp_plan.reshape(row_len, col_len)

        return inter_seq_time, temp_motion_interp_plan


class ROSTrajInterface(object):

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()
        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # Setup constants
        consuming_freq = rospy.get_param('~consuming_freq', 100)
        self.dt = 1.0/float(consuming_freq)

        # safety check
        if consuming_freq > 200:
            rospy.loginfo("%s: Shutting down to high FREQUENCY of consuming in node ", self.name)
            self.clean_shut_down()

        # Get ros parameters
        self.traj_config_file_name = rospy.get_param('~traj_config')

        # if interpolation is for a robot
        namespace = rospy.get_param('~namespace', '')

        # stream the interpolated data or not
        rospy.set_param('/stream_interpolated_motion_flag', True)

        #  traj_manager
        self.setup_traj_manager(self.traj_config_file_name, namespace)

        # Establish connection with planning node
        rospy.loginfo(f"{self.name}: Waiting for self.current_traj_topic topic")
        msgTraj = rospy.wait_for_message(self.current_traj_topic, Float64MultiArray)

        # single update of trajectory
        # To be used for play-back motion plans
        self.read_initial_traj_from_ROS(msgTraj)

        # Subscribe target trajectory callback
        # repetitive update of trajectory
        # To be used for receiding horizon and/or MPC motion plans
        # rospy.Subscriber(self.current_traj_topic, Float64MultiArray, self.read_current_traj_update_from_ROS)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def setup_traj_manager(self, config_file_name, namespace):

        # Load robot configuration
        config = load_config(config_file_name)

        # Extract data from configuration
        mot_dim = config['motion_dimensions']

        # Extract data from configuration
        interpol = config['interpolation']

        # set info about TF publisher
        self.msg_header_frame_id = config['communication']['publisher']['header_frame_id']
        self.msg_child_frame_id = f"{namespace}/{config['communication']['publisher']['msg_child_frame_id']}"

        # set info for listener
        self.current_traj_topic = "ros_pybullet_interface/waypt_traj"

        # Create trajectory manager instance
        self.traj_manag = TrajManager(mot_dim, interpol)

    def read_initial_traj_from_ROS(self, msg):
        # listener, that receives the initial trajectory
        # decode msg
        msg_data = self.decode_ROS_traj_msg(msg)

        #  call to initial setup of trajectory data
        self.traj_manag.set_init_traj(msg_data)

    def read_current_traj_update_from_ROS(self, msg):
        # listener, that receives the new trajectories and update the structure
        # decode msg
        msg_data = self.decode_ROS_traj_msg(msg)

        #  call to update the trajectory data
        self.traj_manag.update_traj(msg_data)

    def decode_ROS_traj_msg(self, msg):
        """ From Float64MultiArray type msg to numpy 2D array"""

        if msg.layout.dim[0].label == "rows":
            rows = msg.layout.dim[0].size
        if msg.layout.dim[1].label == "columns":
            columns = msg.layout.dim[1].size

        data = np.array(msg.data).reshape(rows, columns)

        return data

    def publish_next_waypt_to_ROS(self, event):
        """ Publish 6D information for the respective rigid body """

        # check if stream flag is active
        if rospy.get_param('/stream_interpolated_motion_flag') != True:
            return

        motion = self.traj_manag.get_next_waypt()

        # if the motion plan is not empty
        if motion is not None:

            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.msg_header_frame_id
            msg.child_frame_id = self.msg_child_frame_id
            msg.transform.translation.x = motion[0]
            msg.transform.translation.y = motion[1]
            msg.transform.translation.z = motion[2]
            msg.transform.rotation.x = motion[3]
            msg.transform.rotation.y = motion[4]
            msg.transform.rotation.z = motion[5]
            msg.transform.rotation.w = motion[6]  # NOTE: the ordering here may be wrong

            # Publish tf msg
            self.tf_broadcaster.sendTransform(msg)

        else:
            self.clean_shut_down()

    def clean_shut_down(self):
        print('')
        rospy.loginfo("%s: Shutting down interpolation node ", self.name)
        # Shut down write callback
        self.write_callback_timer.shutdown()
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node("ros_Traj_interface", anonymous=True)
        # Initialize node class
        ROS_traj_interface = ROSTrajInterface()

        rospy.loginfo("%s: node started.", ROS_traj_interface.name)

        # Create timer for periodic publisher
        dur = rospy.Duration(ROS_traj_interface.dt)

        ROS_traj_interface.write_callback_timer = rospy.Timer(
            dur, ROS_traj_interface.publish_next_waypt_to_ROS)

        # Ctrl-C will stop the script
        rospy.on_shutdown(ROS_traj_interface.clean_shut_down)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
