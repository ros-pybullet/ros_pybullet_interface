.. _develop:

Development
===========

Contributing
------------

We are more than happy to accept bug fixes, new features, suggestions, comments, or any other form of feedback.
If you have an issue using the interface, or would like a new feature added please `submit an issue <https://github.com/ros-pybullet/ros_pybullet_interface/issues>`_.
For pull requests, please `fork the repository <https://github.com/ros-pybullet/ros_pybullet_interface/fork>`_, create a new branch, and submit your pull request.

Future work
-----------

* Port GUI controls to RQT.
* Add additional features to GUI interface, e.g. move to custom joint states.
* Support loading from SDF and MuJoCo configuration files.
* Update ROS communication features for loading objects from URDF configuration files.

Known issues
------------

* Objects with alpha color values less than 1.0 are rendedered in RGB images but not the depth image for the RGBD sensor simulation. To make sure the depth image contains the object, ensure the alpha value is set to 1.0.
