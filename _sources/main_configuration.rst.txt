.. _mainconfig:

Main Configuration
==================

The ROS-PyBullet Interface is launched from a ROS ``.launch`` file.
Configuring the interface amounts to passing a ``.yaml`` file as follows.

.. code-block:: xml

		<node pkg="ros_pybullet_interface" name="ros_pybullet_interface" type="ros_pybullet_interface_node.py" output="screen">
		  <rosparam param="config" file="path/to/config.yaml"/>
		</node>

In the example above, the ``config.yaml`` file specifies the virtual world.
This is the file where you can list what objects and robots to load, specify visualization options, and configure PyBullet physics parameters.
		
Configuration files
-------------------

We use `yaml <https://yaml.org/>`_ as the format for configuration files.
Many of the tags used are directly passed to PyBullet functions.
For example, take the `pybullet.setGravity <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.d6ihmmtes1id>`_ method.
In the ROS-PyBullet Interface node, during initialization of the PyBullet instance, if the user provides the ``setGravity`` tag, as follows, in the main yaml configuration file (full details in next section) then the ``pybullet.setGravity`` method is called.

.. code-block:: yaml

		setGravity:
		  gravX: 0.0
		  gravY: 0.0
		  gravZ: -9.81

Notice in the above example that the tag and sub-tags match the interface for the PyBullet method - this is deliberate.
*Under-the-hood* of the interface, the values given in the configuration file are directly passed to the corresponding method in the PyBullet library.
Please note that in all cases for the library functions, most methods take an optional argument ``physicsClientId`` (for cases when multiple PyBullet servers are setup), this **should not** be passed - only a single PyBullet server can be created.
In all cases, the default values for the PyBullet library methods are used.
There are method in the PyBullet library that take a ``flags`` option.
In this case, you can pass the strings (e.g. in the ``loadURDF`` method, you can specify``flags`` as ``URDF_MERGE_FIXED_LINKS|URDF_USE_INERTIA_FROM_FILE``).
Note that not all methods are exposed in this way.

The PyBullet library uses the ``camelCase`` style for its methods/parameters, we use this style for methods/parameters that directly correspond to a library function.
There are some tags that are used to generate different behavior with regards to the interface itself.
In this case, we use ``snake_case`` style to differentiate these parameters.
The benefit for this style-guide is that it allows you to easily tell the difference between parameters that are linked to PyBullet and those that are linked to the interface.
Furthermore, for the ``camelCase`` functions/parameters, you can look these up in the `PyBullet Quickstart Guide <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3>`_.
Note, there are some exceptions to the rules however these are documented when necessary.

Our basic "hello world example" is launched using `basic_example_kuka_lwr.launch <https://github.com/ros-pybullet/ros_pybullet_interface/blob/main/rpbi_examples/launch/basic_example_kuka_lwr.launch>`_.
The main configuration file, `config.yaml <https://github.com/ros-pybullet/ros_pybullet_interface/blob/main/rpbi_examples/configs/basic_example_kuka_lwr/config.yaml>`_, is given as follows.

.. code-block:: yaml

		#
		# Basic example
		#


		#
		# Pybullet instance
		#
		connect:
		  connection_mode: 'GUI'
		  options: '--mp4=$HOME/basic_example_kuka_lwr.mp4'

		setGravity:
		  gravX: 0.0
		  gravY: 0.0
		  gravZ: -9.81

		timeStep: 0.01
		start_pybullet_after_initialization: true
		status_hz: 50

		#
		# Pybullet visualizer
		#
		configureDebugVisualizer:
		  enable: 0
		  flag: 'COV_ENABLE_GUI'

		resetDebugVisualizerCamera:
		  cameraDistance: 2.0
		  cameraYaw: 0.0
		  cameraPitch: -45.0
		  cameraTargetPosition: [0.0, 0.0, 0.0]

		#
		# Pybullet objects
		#
		collision_objects:
		  - "{rpbi_examples}/configs/floor.yaml"
		robots:
		  - "{rpbi_examples}/configs/basic_example_kuka_lwr/kuka_lwr.yaml"


		#
		# Sensors
		#

		rgbd_sensor:
		  name: 'rgbd_sensor'
		  hz: 30
		  # this will project the depth to a point cloud
		  # pointcloud: True
		  intrinsics:
		    width: 640
		    height: 480
		    fov: 40
		    range: [0.01, 10000]
		  object_tf:
		    tf_id: 'rpbi/camera'		

You will notice that there are four main sections in the main configuration file:
PyBullet instance, PyBullet visualizer, PyBullet objects, and sensors.
Details for each of these are given in the following sub-sections.

Note, the order in which these sections or the parameters themselves are listed does not necessarily need to be in any particular ordering.
However, we suggest you follow this convention so that configuration files are more readable.
		    
PyBullet Instance
-----------------

This section of the main configuration file allows you to setup the main PyBullet instance.
The type of settings you can set in this section relate to physical parameter (e.g. gravity), or time (e.g. simulator time-step), etc.
Some parameters are expected, and others are optional.
The full list of possible parameters are listed as follows.

* ``connect`` (required), see PyBullet documentation. Note, the ``connection_mode`` can be passed as a string. Also note, a very useful feature is recording videos of the interface - see the ``options`` parameter.
* ``setAdditionalSearchPath``, see PyBullet documentation. Note, you can pass the string ``"pybullet_data_path"``, this will add the additional search path given by `pybullet_data.getDataPath()`. Also note, that a list of paths can be given, these will all get added.
* ``resetSimulation``, see PyBullet documentation.
* ``setGravity``, see PyBullet documentation.
* ``timeStep`` [``float``], Each time PyBullet is stepped time-step will proceed with by this duration (secs). Default is ``0.02``.
* ``setPhysicsEngineParameter``, see PyBullet documentation.
* ``step_pybullet_manually`` [``bool``], this is always `true` when the connection mode is ``DIRECT``. Otherwise, you can specify PyBullet to be stepped manually inside a ROS Timer at the rate specified by the ``timeStep`` parameter. Otherwise, PyBullet will step itself internally. Differences have been observed, however it is not clear exactly what is happening inside the Bullet simulator source code.
* ``status_hz`` [``int``], this is the frequency that the status publisher is broadcast to ROS.

Visualizer
----------

The main GUI visualization camera can be adjusted in this section.
Parameters that correspond to the visualization are listed as follows.

* ``configureDebugVisualizer``, see the PyBullet documentation.
* ``resetDebugVisualizerCamera``, see the PyBullet documentation. Note, the pose of the camera can be adjusted by publishing new states to the ROS topic ``rpbi/reset_debug_visualizer_camera`` using the message type ``ros_pybullet_interface/ResetDebugVisualizerCamera``.

PyBullet Objects
----------------

There are several object types that are supported by the ROS-PyBullet Interface:
robots,
collision objects,
dynamic objects,
visual objects,
soft bodies, and
objects loaded directly from a URDF file.
This section of the main configuration file allows you to specify all the objects you want in your virtual world by listing the path to the filename.
You can specify these as follows.

.. code-block:: yaml

		robots:
		  - "{ros_package}/path/to/robot.yaml"

		collision_objects:
		  - "absolute/path/to/collision_obj.yaml"

		dynamic_objects:
		  - "{ros_package}/path/to/dynamic_obj.yaml"

		visual_objects:
		  - "{ros_package}/path/to/visual_obj.yaml"

		soft_objects:
		  - "{ros_package}/path/to/soft_body.yaml"

		urdfs:
		  - "{ros_package}/path/to/urdf_obj.yaml"
	
*Note*:

* all the above tags are optional,
* multiple objects can be listed for each object type, and
* each filename can be specified with an absolute path (see ``collision_objects`` above), or by a relative path to a ROS package using curly brackets ``{ros_package}`` (as above in all other examples).

All the object types have a different required/optional settings that must be given in the specified yaml configuration files.
The details for all these are given in the next section of the documentation.

Sensors
-------

There are two main types of sensors that can be simulated in the ROS-PyBullet Interface: Force-Torque sensors, and RGBD cameras.
The Force-Torque sensors must be connected to a robot link, see the following section of the documentation for details on how to setup this sensor.
An RGBD camera can also be specified.
Currently, the interface is limited to only a single camera.

If desired, the RGBD camera can be specified in the main configuration file by adding the tag ``rgbd_sensor`` (as in the basic Kuka LWR example above).
The parameters used to configure the RGBD camera are listed as follows.

* ``name`` (required), the name of the sensor. Each PyBullet object is given a name, all these must be unique - more details are given in the next section of the documentation.
* ``intrinsics``, camera intrinsic parameters
  
  * ``width`` [``int``], width of camera image. Default is ``640``.
  * ``height`` [``int``], height of the camera image. Default is ``480``.
  * ``fov`` [``int``], field of view. Default is ``40``.
  * ``range`` [``list[float]``], depth range. Default is ``[0.01, 100.0]``.

* ``pointcloud`` [``bool``], when ``true`` the depth camera is projected as a point cloud and published to ROS. *Note*, due to the computation required this will slow the simulation. Standard ROS packages can efficiently compute this outside the simulator (as in the examples). It is recommended that you **do not** use this option. We originally added it for experimentation. Default is ``false``.
* ``hz`` [``int``], frequency that the RGBD sensor is updated. Default is ``30``.
* ``object_tf`` (required), the pose of the camera must be attached to a ``tf2`` (transform) frame

  * ``tf_id`` [``str``] (required), the ``tf2`` frame ID that defines the camera pose. This frame **must** be defined with respect to the ``rpbi/world`` frame.
  * ``hz`` [``int``], the frequency that the pose is queried. Default is ``30``.
