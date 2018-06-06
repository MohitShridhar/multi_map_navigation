cmake_minimum_required(VERSION 2.8.3)
project(multi_map_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  map_server
  visualization_msgs
  move_base_msgs
  nav_msgs
  message_generation
  std_msgs
  nodelet
  rviz
  tf
)

catkin_python_setup()
# NAVIGATION ----------------

# generate action files
add_action_files(DIRECTORY action FILES MultiMapNavigation.action MultiMapNavigationTransition.action MultiMapNavigationTargetElevator.action MultiMapServer.action)

#generate generate_messages
add_message_files(DIRECTORY msg FILES elevatorRequest.msg)

# generate service files
add_service_files(DIRECTORY srv FILES ReinitManager.srv)

# messages dependencies
generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs)


catkin_package(
  CATKIN_DEPENDS rospy actionlib visualization_msgs move_base_msgs nav_msgs std_msgs message_runtime nodelet rviz tf map_server
)
