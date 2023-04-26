Installation
============

The following describes how to install the ROS-PyBullet Interface.

Requirements
############


* ROS Noetic
  
   * Melodic, and past version should work fine (only Melodic has been tested) so long as you configure ROS to use Python 3
   * ROS2 is currently unsupported, however this is in-development
   * `catkin_tools <https://catkin-tools.readthedocs.io/en/latest/>`_
   * `rosinstall <http://wiki.ros.org/rosinstall>`_
   * `rosdep <http://wiki.ros.org/rosdep>`_

* Ubuntu 20.04

  * Other versions should work, provided Python 3 is installed
    
* Python 3

* ``urdf_parser_py``, see `here <http://wiki.ros.org/urdfdom_py>`_.
  

From binaries
#############

Currently, this is *in-progress*.

From source
###########

1. `Create a catkin workspace <https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace>`_ or use an existing workspace. `catkin_tools <https://catkin-tools.readthedocs.io/en/latest/>`_ is the preferred build system.
2. ``cd`` to the ``src`` directory of your catkin workspace.
3. Clone this repository: ``$ git clone https://github.com/ros-pybullet/ros_pybullet_interface.git``
4. Install source dependencies: ``$ rosinstall . --catkin --nobuild``
5. Install binary dependencies: ``$ rosdep update ; rosdep install --from-paths ./ -iry``
6. Compile the workspace: ``$ catkin build -s``
7. Source the workspace: ``$ source $(catkin locate)/devel/setup.bash``

Now you should be able to run the :ref:`examples <examples>`.
