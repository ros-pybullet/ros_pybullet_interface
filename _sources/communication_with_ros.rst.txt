Communication with ROS
======================

In this section, we discuss generally the kind of communication that the interface has with ROS.
This is mainly in terms of topics/services.
For PyBullet object specific ROS communication, see previous sections.

Services
--------

Several services are instantiated when the ROS-PyBullet Interface node is launched.
These allow you to either programatically interact with PyBullet from your own code, or from the command line.
These serives are detailed as follows.

``rpbi/start`` (``std_srv/Trigger``)
####################################

The simulation is started, if it has been stopped.


``rpbi/step`` (``std_srv/Trigger``)
###################################

If the simulation is stopped, then it is stepped by one time-step.
The duration of the time-step is given by the ``timeStep`` parameter in the main configuration file.


``rpbi/stop`` (``std_srv/Trigger``)
###################################

The simulation is stopped, if it is running.

``rpbi/get_debug_visualizer_camera`` (``ros_pybullet_interface/GetDebugVisualizerCamera``)
##########################################################################################

When this service is called, the response returns the current parameters for the main visualizer: i.e.
``cameraDistance``,
``cameraYaw``,
``cameraPitch``, and
``cameraTargetPosition``.

``rpbi/add_pybullet_object`` (``ros_pybullet_interface/AddPybulletObject``)
###########################################################################

An object is added to PyBullet.
The service allows you to either load from file or pass the configuration for the object.
The input for the service expects a ``ros_pybullet_interface/PybulletObject`` message - see `here <https://github.com/ros-pybullet/ros_pybullet_interface/blob/main/ros_pybullet_interface/msg/PybulletObject.msg>`_.

For both cases (i.e. load from filename or configuration), an ``object_type`` must be given.
Either ``PybulletObject.VISUAL``, ``PybulletObject.COLLISION``, ``PybulletObject.DYNAMIC``, ``PybulletObject.ROBOT``, ``PybulletObject.SOFT``, or ``PybulletObject.URDF``.

*Load from filename*: the ``PybulletObject.filename`` variable must be set.
You can specify reletive filenames by giving a ROS package name in the format ``{ros_package}/path/to/file.yaml``.

*Load from configuration*: if you want to pass the configuration in the service then you need to send it as a string, that is ultimately a yaml file. In Python, the best way to do this is to specify the configration in a ``dict`` (in the same way a yaml file is loaded) and convert it to a string using the ``config_to_str`` method provided in ``custom_ros_tools.config``.
See example below.

.. code-block:: python

		from custom_ros_tools.config import config_to_str
		from ros_pybullet_interface.msg import PybulletObject

		# make config
		config = {}
		# ...

		# Setup request
		req = PybulletObject(config=config_to_str(config))		

**Note**: loading from a filename takes precedence - if you want to load by passing the configuration then the ``filename`` parameter must not be set.
		
``rpbi/remove_pybullet_object`` (``cob_srv/SetString``)
#######################################################

Given the PyBullet object name as the only parameter, the object is removed from PyBullet, and all ROS communication for that object is closed.

PyBullet Status
---------------

From when the ROS-PyBullet node is launched, the state of PyBullet is published to the topic ``rpbi/status`` (with type ``std_msgs/Int64``).
By default, this is published at 50Hz, however you specify the frequency through the ``status_hz`` parameter in the main configuration file.
The message on the topic indicates whether the simulator is running or not.
If it is running, then the value of the message is ``1``, otherwise it is ``0``.

Time and ROS Clock
------------------

It is possible to syncronize the ROS clock time with the PyBullet simulation time.
This means that when you call ``rospy.Time.now()`` in Python or ``ros::Time::now()`` in C++, the value will be equal to the simulation time in PyBullet.
This can only be done when the ROS-Pybullet Interface node is configured to use manual time stepping (the reason for this is because `the Pybullet simulator time is not currently exposed <https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12438>`_).
To synconize the PyBullet simulator time and ROS clock time, set the Boolean ROS parameter ``use_sim_time`` to ``true``.
