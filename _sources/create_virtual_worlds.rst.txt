Create Virutal Worlds
=====================

A virtual world can be created by listing the objects in the main configuration file (see the previous section).
In addition, PyBullet objects can be added programatically (see the next section of the documentation) or even from the command line using ``$ rosservice call``.
We give full details for the the available PyBullet object types in the following sub-sections.

For *every* PyBullet object type, the configuration file **must** contain a ``name`` tag [``str``].
The name given to the object **must** be unique with respect to all other PyBullet objects (even of different types).

Futuremore, all objects are defined with respect to a global base coordinate frame called ``rpbi/world``.

Robot
-----

A robot can be specified using a URDF file.
Currently, this is the only format accepted by the ROS-PyBullet Interface.
The parameters for specifying a robot are listed as follows.

* ``loadURDF`` (required), see the PyBullet documentation. Note the ``fileName`` can be given relative to a ROS package using curley brackets ``{ros_package}``. In addition, if the ``fileName`` is given as ``robot_description`` then the URDF file is retrieved from the ``robot_description`` ROS parameter - of course, this must be set in the launch file or a script somewhere.
* ``setJointMotorControlArray`` (required), see the PyBullet documentation. Only the ``controlMode`` is required to be specified. **Do not** specify ``jointIndices``, ``targetPositions``, ``targetVelocities``, ``forces``, or the ``physicsClientId``.
* ``initial_joint_positions`` [``dict[str: float]``], the initial joint positions for the robot. You can specify any joints you wish by giving the joint name and initial joint position value. Unspecified joints default to ``0.0``.
* ``initial_revolute_joint_positions_are_deg``, when ``true`` the initial joint positions for revolute joints are assumed to be given in degrees, otherwise radians. Default is ``True``.
* ``joint_state_publisher_hz`` [``int``], frequency that the robot joint states are published to ROS on the topic ``rpbi/NAME/joint_states`` with type ``sensor_msgs/JointState`` where ``NAME`` is the PyBullet object name.
* ``broadcast_link_states`` [``bool``], when true the robot links for the robot are broadcast to ROS as ``tf2`` frames.
* ``broadcast_link_states_hz`` [``int``], the frequency that the links of the robot are broadcast to ROS.
* ``enabled_joint_force_torque_sensors`` [``list[str]``], list of joint names that have Force-Torque sensors enabled. Names of joints should correspond to those defined in the URDF. When these sensors are enabled, they are published to ROS with topic name ``rpbi/NAME/JOINTNAME/ft_sensor`` with type ``geometry_msgs/WrenchStamped`` where ``NAME`` is the PyBullet object name, and ``JOINTNAME`` is the given joint name.
* ``is_visual_robot`` [``bool``], when ``true`` the robot is treated a visual object, i.e. it will not react to other objects in the environment and other objects will not react to the robot. This can be useful for debugging and also visualizing a real robot. The default value is ``false``.
* ``do_log_joint_limit_violations`` [``bool``], when the robot is a visual robot if ``true`` then the joint limit violations are reported to the terminal.
* ``log_joint_limit_violations_hz`` [``int``], the frequency that the joint limit violations are checked.
* ``start_ik_callback`` [``bool``], when true a subscriber is started for the topic ``rpbi/NAME/ik`` of message type ``ros_pybullet_interface/CalculateInverseKinematicsProblem`` where ``NAME`` is the name of the Pybullet object. This allows you to implement task space controller. Rather than streaming target joint states to the robot, you can stream goal states (defined as a ``ros_pybullet_interface/CalculateInverseKinematicsProblem`` message).  This option can only be used when the robot is in the ``POSITION_CONTROL`` or ``VELOCITY_CONTROL`` control modes.
* ``color_alpha`` [``float``], the alpha value for the robot in range ``[0.0, 1.0]``. This allows you to make the robot transparent. By default, this option is not used.

You can move the robot in several ways.
The most common way is to stream target joint states by publishing to the topic ``rpbi/NAME/joint_states/target`` where ``NAME`` is the name of the Pybullet object.
Note, that joint state messages must include the ``name`` parameter, i.e. a list of joint names that specify the order of the ``position``/``velocity``/``effort`` attributes.
Another way to generate motion is to stream task space targets, see the tag ``start_ik_callback`` above.
Finally, several services are provided that will move the robot to desired states - see below.

Several ROS services are started when a PyBullet robot is instantiated.
These are listed as follows.
*Note*, in the following ``NAME`` is the name of the Pybullet object.

* ``rpbi/NAME/robot_info`` [``ros_pybullet_interface/RobotInfo``], returns information about the robot (i.e. the name, link/joint names, body unique ID, number of joints, number of degrees of freedom, joint information from PyBullet ``pybullet.getJointInfo`` method, see the `documentation <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.la294ocbo43o>`_, enabled Force-Torque sensors, and the current joint state).
* ``rpbi/NAME/ik`` [``ros_pybullet_interface/CalculateInverseKinematics``], compute a single IK. The target joint state is returned.

The following ROS services are only created for robots that are *not visual* (i.e. the tag ``is_visual_robot``, see above, is ommited or set to ``false``).

* ``rpbi/NAME/move_to_joint_state`` [``ros_pybullet_interface/ResetJointState``], given a target joint state and duration the robot is moved from the current state to the goal. The duration (in seconds) is the time it will take for the robot to move from the current state to the goal state. *Note*, there is no collision avoidance.
* ``rpbi/NAME/move_to_init_joint_state`` [``ros_pybullet_interface/ResetJointState``], moves the robot to the initial joint state specified in the yaml configuration file under the tag ``initial_joint_positions`` (see above). *Note*, we re-use the ``ros_pybullet_interface/ResetJointState`` service type here. That means when you call the service you will need to include the duration (i.e. time it takes for the robot to move from the current configuration to the goal) and an empty ``sensor_msgs/JointState`` message - the joint state message will be ignored. *Note*, there is no collision avoidance.
* ``rpbi/NAME/move_to_eff_state`` [``ros_pybullet_interface/ResetEffState``], given a task space target and a duration the robot is moved from the current configuration to a goal configuration (computed using PyBullet's Inverse Kinematics feature). *Note*, there is no collision avoidnace.


Collision Object
----------------

For the collision object, other objects will react to it, but it will remain unaffected.
Objects such as walls, doors, ceilings, floors should be modelled using this object type.
The parameters to setup this object are listed as follows.

* ``createVisualShape`` (required), see PyBullet documentation.
* ``createCollisionShape`` (required), see PyBullet documentation.
* ``changeDynamics``, see PyBullet documentation.
* ``object_tf``, if unspecified the default is the identity.

  * ``tf_id`` [``str``], the ``tf2`` frame ID that defines the camera pose. This frame **must** be defined with respect to the ``rpbi/world`` frame.
  * ``hz`` [``int``], the frequency that the pose is queried. Default is ``30``.

Dynamic Object
--------------

You can simulate virtual objects using a dynamic object.
In this case, the objects motion is completely defined by Pybullet.
The parameters for this object type are as follows.

* ``createVisualShape`` (required), see PyBullet documentation.
* ``createCollisionShape`` (required), see PyBullet documentation.
* ``changeDynamics`` (required), see PyBullet documentation.
* ``baseMass`` [``float``] (required), mass of the base.
* ``basePosition`` [``list[float]``], base position in the ``rpbi/world`` frame.
* ``baseOrientation`` [``list[float]``], base orientation in the ``rpbi/world`` frame (as a quaternion).
* ``resetBaseVelocity``, see PyBullet documentation. Note, the ``bodyUniqueId`` does not need to be passed. This will specify the initial velocity of the object.
* ``broadcast_hz`` [``int``], this is the frequency that the object pose is broadcast to ``tf2``. Default is ``0`` (i.e. the pose is not broadcast). The frame is always published with respect to the ``rpbi/world`` frame and given the name ``rpbi/NAME`` where ``NAME`` is the name of the PyBullet object.

Visual Object
-------------

A visual object is used primarily for visualizing real world objects or for debugging.
These simply visualize objects, other objects will not react to this object and it will not react to other objects.
To specify this object the following parameters can be used.

* ``createVisualShape`` (required), see the PyBullet documentation. Note the ``fileName`` can be given relative to a ROS package using curley brackets ``{ros_package}``. Also, the ``shapeType`` parameter can be passed as a string.
* ``object_tf``, if unspecified the default is the identity.

  * ``tf_id`` [``str``], the ``tf2`` frame ID that defines the camera pose. This frame **must** be defined with respect to the ``rpbi/world`` frame.
  * ``hz`` [``int``], the frequency that the pose is queried. Default is ``30``.



Soft bodies
-----------

PyBullet also implements deformable object and cloth simulation.
Soft bodies can be setup using the ``pybullet.loadSoftBody`` method, or from a URDF file.
For the URDF, see the next section.
When using the ``loadSoftBody`` approach, you can to specify the following tags.

* ``loadSoftBody`` (required), see PyBullet documentation.
* ``createSoftBodyAnchor`` [``list[list[float/int]]``], pin vertices of a deformable object to the world. *Note*, the PyBullet documentation for ``createSoftBodyAnchor`` is limited. It is not clear what is exactly the interface. The soft body unique ID will be passed automatically, but any other parameters must be supplied. Some potential resources:

  * `https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_anchor.py <https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/deformable_anchor.py>`_
  * `https://github.com/bulletphysics/bullet3/discussions/4088 <https://github.com/bulletphysics/bullet3/discussions/4088>`_
  * `https://github.com/bulletphysics/bullet3/blob/7dee3436e747958e7088dfdcea0e4ae031ce619e/examples/pybullet/pybullet.c#L2280-L2326 <https://github.com/bulletphysics/bullet3/blob/7dee3436e747958e7088dfdcea0e4ae031ce619e/examples/pybullet/pybullet.c#L2280-L2326>`_


Loading from URDF
-----------------

This interface allows you to load objects directly from a URDF.
The only required tag is as follows.

* ``loadURDF`` (required), see the PyBullet documentation.

*Note*, for this object type there is no ROS communication available.
Future work will include updated feature set for this object type.
