cmake_minimum_required(VERSION 3.0.2)
project(ros_pybullet_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  cob_srvs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  ResetDebugVisualizerCamera.msg
  JointInfo.msg
  CalculateInverseKinematicsProblem.msg
  PybulletObject.msg
  ObjectDynamics.msg
  KeyboardEvent.msg
  MouseEvent.msg
)

add_service_files(
  FILES
  RobotInfo.srv
  ResetJointState.srv
  ResetEffState.srv
  CalculateInverseKinematics.srv
  GetDebugVisualizerCamera.srv
  AddPybulletObject.srv
  GetObjectDynamics.srv
  ChangeObjectDynamics.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/ros_pybullet_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
