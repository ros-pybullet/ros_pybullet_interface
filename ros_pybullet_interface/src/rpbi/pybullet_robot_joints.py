import time
import numpy as np
from math import radians
from ros_pybullet_interface.msg import JointInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty

class Joint:

    """Joint class that hold joint information returned by pybullet.getJointInfo."""

    joint_types_str = ["JOINT_REVOLUTE", "JOINT_PRISMATIC", "JOINT_SPHERICAL", "JOINT_PLANAR", "JOINT_FIXED"]

    def __init__(self, pb_obj, jointIndex):

        # Setup
        self.ft_pub_key = None  # force/torque publisher key in pb_obj.pubs (if enabled)
        self.pb_obj = pb_obj
        self.info = self.pb_obj.pb.getJointInfo(self.pb_obj.body_unique_id, jointIndex)

        # Extract data
        self.jointIndex = self.info[0]
        self.jointName = self.info[1].decode('utf-8')
        self.jointType = self.info[2]
        self.jointTypeStr = Joint.joint_types_str[self.info[2]]
        self.qIndex = self.info[3]
        self.uIndex = self.info[4]
        self.flags = self.info[5]
        self.jointDamping = self.info[6]
        self.jointFriction = self.info[7]
        self.jointLowerLimit = self.info[8]
        self.jointUpperLimit = self.info[9]
        self.jointMaxForce = self.info[10]
        self.jointMaxVelocity = self.info[11]
        self.linkName = self.info[12].decode('utf-8')
        self.jointAxis = self.info[13]
        self.parentFramePos = self.info[14]
        self.parentFrameOrn = self.info[15]
        self.parentIndex = self.info[16]

        # wrench offset
        self.setzero = False
        self.setzero_cnt = 0
        self.total_setzero_cnt = 100
        self.wrench_offset = [0., 0., 0., 0., 0., 0.]
        self.wrench_measured = [0., 0., 0., 0., 0., 0.]

        # Parse the joint information as a ros_pybullet_interface/JointInfo ROS message
        self.joint_info_msg = JointInfo(
            jointIndex = self.jointIndex, jointName = self.jointName, jointType = self.jointType,
            jointTypeStr = self.jointTypeStr, qIndex = self.qIndex, uIndex = self.uIndex,
            flags = self.flags, jointDamping = self.jointDamping, jointFriction = self.jointFriction,
            jointLowerLimit = self.jointLowerLimit, jointUpperLimit = self.jointUpperLimit,
            jointMaxForce = self.jointMaxForce, jointMaxVelocity = self.jointMaxVelocity,
            linkName = self.linkName, parentIndex = self.parentIndex,
            is_fixed = self.is_fixed(), is_revolute = self.is_revolute(),
        )
        self.joint_info_msg.jointAxis.x = self.jointAxis[0]
        self.joint_info_msg.jointAxis.y = self.jointAxis[1]
        self.joint_info_msg.jointAxis.z = self.jointAxis[2]
        self.joint_info_msg.parentFramePos.x = self.parentFramePos[0]
        self.joint_info_msg.parentFramePos.y = self.parentFramePos[1]
        self.joint_info_msg.parentFramePos.z = self.parentFramePos[2]
        self.joint_info_msg.parentFrameOrn.x = self.parentFrameOrn[0]
        self.joint_info_msg.parentFrameOrn.y = self.parentFrameOrn[1]
        self.joint_info_msg.parentFrameOrn.z = self.parentFrameOrn[2]
        self.joint_info_msg.parentFrameOrn.w = self.parentFrameOrn[3]

    def in_limit(self, q):
        return self.jointLowerLimit <= q <= self.jointUpperLimit

    def is_revolute(self):
        return self.jointType == self.pb_obj.pb.JOINT_REVOLUTE

    def is_fixed(self):
        return self.jointType == self.pb_obj.pb.JOINT_FIXED

    def turn_off_collisions(self):
        self.pb_obj.pb.setCollisionFilterGroupMask(self.pb_obj.body_unique_id, self.jointIndex, 0, 0)

    def enable_ft_sensor(self):
        self.pb_obj.pb.enableJointForceTorqueSensor(self.pb_obj.body_unique_id, self.jointIndex, enableSensor=1)
        self.ft_pub_key = f'{self.pb_obj.name}_{self.jointName}_ft_sensor'
        self.pb_obj.pubs[self.ft_pub_key] = self.pb_obj.node.Publisher(f'rpbi/{self.pb_obj.name}/{self.jointName}/ft_sensor', WrenchStamped, queue_size=10)
        self.pb_obj.srvs[self.ft_pub_key] = self.pb_obj.node.Service(f'rpbi/{self.pb_obj.name}/{self.jointName}/Zero', Empty, self.zero_wrench)

    @property
    def ft_sensor_enabled(self):
        return self.ft_pub_key is not None

    def publish_wrench(self, joint_reaction_forces, frame_id):

        wrench_measured = [-wrench for wrench in joint_reaction_forces]

        if self.setzero:
            if self.setzero_cnt == 0:
                self.wrench_offset = [wrench for wrench in wrench_measured]
                self.setzero_cnt += 1
            elif self.setzero_cnt < self.total_setzero_cnt:
                self.wrench_offset = [offset + wrench for offset,wrench in zip(self.wrench_offset,wrench_measured)]
                self.setzero_cnt += 1
            else:
                self.wrench_offset = [offset/self.total_setzero_cnt for offset in self.wrench_offset]
                self.setzero_cnt = 0
                self.setzero = False
        else:
            # set and publish wrench
            msg = WrenchStamped()
            msg.header.stamp = self.pb_obj.node.time_now()
            msg.header.frame_id = frame_id
            msg.wrench.force.x = wrench_measured[0] - self.wrench_offset[0]
            msg.wrench.force.y = wrench_measured[1] - self.wrench_offset[1]
            msg.wrench.force.z = wrench_measured[2] - self.wrench_offset[2]
            msg.wrench.torque.x = wrench_measured[3] - self.wrench_offset[3]
            msg.wrench.torque.y = wrench_measured[4] - self.wrench_offset[4]
            msg.wrench.torque.z = wrench_measured[5] - self.wrench_offset[5]
            self.pb_obj.pubs[self.ft_pub_key].publish(msg)

    def zero_wrench(self, req):
        self.setzero = True
        return []

class Joints(list):

    def __init__(self, pb_obj):

        # Setup
        super().__init__()
        self.pb_obj = pb_obj

        # Setup joints
        self.num_joints = self.pb_obj.pb.getNumJoints(self.pb_obj.body_unique_id)
        for jointIndex in range(self.num_joints):
            self.append(Joint(pb_obj, jointIndex))

        # Set control mode
        self.control_mode = None
        if not self.pb_obj.is_visual_robot:
            cm = self.pb_obj.config['setJointMotorControlArray']['controlMode']
            if isinstance(cm, str):
                self.control_mode = getattr(self.pb_obj.pb, cm)
            elif isinstance(cm, int):
                self.control_mode = cm
            else:
                raise TypeError("did not recognize type for controlMode in configuration, expected either str of int")

        # Setup other class variables
        self.ndof = sum([not j.is_fixed() for j in self])
        self.names = [j.jointName for j in self]
        self.indices = [j.jointIndex for j in self]
        self.link_names = [j.linkName for j in self]

        # Set initial joint state
        self.reset(self.initial_joint_state)

        # Enable ft sensors
        # NOTE: if robot is visual then this data will not be published, even if this is set in config file
        if not self.pb_obj.is_visual_robot:
            for name in self.enabled_joint_force_torque_sensors:
                self[name].enable_ft_sensor()

        # Setup target joint state subscriber
        if not self.pb_obj.is_visual_robot:
            self.joint_state_target_callback = self.set_target
        else:
            self.joint_state_target_callback = self.reset
            self.turn_off_all_collisions()

        topic_name = f'rpbi/{self.pb_obj.name}/joint_states/target'
        self.pb_obj.subs['target_joint_state'] = self.pb_obj.node.Subscriber(topic_name, JointState, self.joint_state_target_callback)

        # Start joint state publisher
        if not self.pb_obj.is_visual_robot:
            self.start_joint_state_publisher()

        # Start logging joint limit violations (optional)
        # NOTE: this is only available for visual robots
        if self.do_log_joint_limit_violations:
            dt = self.pb_obj.node.Duration(1.0/float(self.log_joint_limit_violations_hz))
            self.pb_obj.timers['joint_limit_violations'] = self.pb_obj.node.Timer(dt, self.log_joint_limit_violations)

    @property
    def do_log_joint_limit_violations(self):
        return self.pb_obj.config.get('do_log_joint_limit_violations', True) and self.pb_obj.is_visual_robot

    @property
    def log_joint_limit_violations_hz(self):
        return self.pb_obj.config.get('log_joint_limit_violations_hz', 50)

    def log_joint_limit_violations(self, event):
        joint_states = self.pb_obj.pb.getJointStates(self.pb_obj.body_unique_id, self.indices)
        self._log_joint_limit_violations(joint_states)

    def _log_joint_limit_violations(self, joint_states):
        in_limit = [
            joint.in_limit(joint_states[0])
            for joint, joint_states in zip(self, joint_states)
            if not joint.is_fixed()
        ]
        if not all(in_limit):
            err_msg = f'------- joint limit violation for robot {self.pb_obj.name} -------'
            self.pb_obj.node.logerr(err_msg)

    @property
    def enabled_joint_force_torque_sensors(self):
        return self.pb_obj.config.get('enabled_joint_force_torque_sensors', []) # list of joint names

    def append(self, joint):
        super().append(joint)
        if len(self) > self.num_joints:
            raise ValueError("too many joints set, this error should not be thrown!")

    def reset(self, joint_state_msg):
        for name, position in zip(joint_state_msg.name, joint_state_msg.position):
            self.pb_obj.pb.resetJointState(
                self.pb_obj.body_unique_id, self.name_to_index(name), position,
            )

    def set_target(self, joint_state_msg):

        # Set target
        if self.control_mode == self.pb_obj.pb.POSITION_CONTROL:
            target = {'targetPositions': joint_state_msg.position}
        elif self.control_mode == self.pb_obj.pb.VELOCITY_CONTROL:
            target = {'targetVelocity': joint_state_msg.velocity}
        elif self.control_mode == self.pb_obj.pb.TORQUE_CONTROL:
            target = {'forces': joint_state_msg.effort}
        else:
            raise ValueError("did not recognize control mode!")

        # Set gains if supplied by user
        if 'positionGains' in self.pb_obj.config['setJointMotorControlArray']:
            target['positionGains'] = self.pb_obj.config['setJointMotorControlArray']['positionGains']
        if 'velocityGains' in self.pb_obj.config['setJointMotorControlArray']:
            target['velocityGains'] = self.pb_obj.config['setJointMotorControlArray']['velocityGains']

        # Set target
        self.pb_obj.pb.setJointMotorControlArray(
            self.pb_obj.body_unique_id,
            [self.name_to_index(name) for name in joint_state_msg.name],
            self.control_mode,
            **target,
        )

    def name_to_index(self, name):
        return self.names.index(name)

    def __getitem__(self, k):
        if isinstance(k, str):
            joint_index = self.name_to_index(k)
        elif isinstance(k, int):
            joint_index = k
        else:
            raise TypeError(f"index type is not recognized, expected int or str, got {type(k)}")
        return super().__getitem__(joint_index)

    @property
    def init_is_deg(self):
        return self.pb_obj.config.get('initial_revolute_joint_positions_are_deg', True)

    @property
    def initial_joint_state(self):

        init_joint_state = JointState(name=self.names, position=[0.0]*len(self))

        init_joint_pos = self.pb_obj.config.get('initial_joint_position', {})
        for i, name in enumerate(self.names):
            if name in init_joint_pos:
                pos = init_joint_pos[name]
                if self[name].is_revolute() and self.init_is_deg:
                    pos = radians(pos)
                init_joint_state.position[i] = pos

        return init_joint_state

    def turn_off_all_collisions(self):
        for j in self:
            j.turn_off_collisions()

    @property
    def joint_state_publisher_hz(self):
        return self.pb_obj.config.get('joint_state_publisher_hz', 50)

    def start_joint_state_publisher(self):

        # Setup publisher
        self.pb_obj.pubs['joint_state'] = self.pb_obj.node.Publisher(f'rpbi/{self.pb_obj.name}/joint_states', JointState, queue_size=10)

        # Start timer
        dt = self.pb_obj.node.Duration(1.0/float(self.joint_state_publisher_hz))
        self.pb_obj.timers['joint_state_publisher'] = self.pb_obj.node.Timer(dt, self._publish_joint_state)

    def _publish_joint_state(self, event):

        # Get joint states from pybullet
        joint_states = self.pb_obj.pb.getJointStates(self.pb_obj.body_unique_id, self.indices)

        # Publish ft sensor states
        for joint, joint_state in zip(self, joint_states):
            if joint.ft_sensor_enabled:
                joint.publish_wrench(joint_state[2], joint.linkName)

        # Publish joint state message
        self.pb_obj.pubs['joint_state'].publish(self.pack_joint_state_msg(joint_states))

    @staticmethod
    def position_from_joint_states(joint_states):
        return np.array([js[0] for js in joint_states])

    @staticmethod
    def velocity_from_joint_states(joint_states):
        return np.array([js[1] for js in joint_states])

    @staticmethod
    def applied_joint_motor_torque_from_joint_states(joint_states):
        return np.array([js[3] for js in joint_states])

    def get_current_joint_state_as_np(self):
        joint_states = self.pb_obj.pb.getJointStates(self.pb_obj.body_unique_id, self.indices)
        return self.position_from_joint_states(joint_states)

    def pack_joint_state_msg(self, joint_states):
        msg = JointState(
            name=self.names,
            position=self.position_from_joint_states(joint_states),
            velocity=self.velocity_from_joint_states(joint_states),
            effort=self.applied_joint_motor_torque_from_joint_states(joint_states),
        )
        msg.header.stamp = self.pb_obj.node.time_now()
        return msg

    def get_current_joint_state_as_msg(self):
        joint_states = self.pb_obj.pb.getJointStates(self.pb_obj.body_unique_id, self.indices)
        return self.pack_joint_state_msg(joint_states)

    def move_to_joint_state(self, target_joint_state_msg, duration):

        # Setup
        duration = np.clip(duration, 0.04, np.inf)  # prevent zero-division

        # Get current joint state as np array
        current_joint_state = self.get_current_joint_state_as_np()

        # Convert target_joint_state_msg to np array
        # NOTE: any missing target joint positions will be given the current joint state
        target_joint_state = current_joint_state.copy()
        for name, position in zip(target_joint_state_msg.name, target_joint_state_msg.position):

            # Only set non-fixed joint targets
            if not self[name].is_fixed():
                target_joint_state[self.name_to_index(name)] = position

        # Move the robot
        qprev = current_joint_state.copy()
        alpha = 0.0
        hz = 100  # expose to user?
        dt = 1.0/float(hz)
        rate = self.pb_obj.node.Rate(hz)
        t0 = time.time()
        while alpha < 1.0:
            qgoal = alpha*target_joint_state + (1.0-alpha)*current_joint_state
            dq = qgoal - qprev
            joint_state = JointState(name=self.names, position=qgoal, velocity=dq/dt)
            self.set_target(joint_state)  # service only available for non-visual robots
            qprev = qgoal.copy()
            rate.sleep()
            alpha = (time.time() - t0)/duration
