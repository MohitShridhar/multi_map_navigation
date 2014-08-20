cmake_minimum_required(VERSION 2.8.3)
project(multi_map_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  map_store
  visualization_msgs
  move_base_msgs
  nav_msgs
  warehouse_ros
  message_generation
  std_msgs
)


add_action_files(DIRECTORY action FILES MultiMapNavigation.action MultiMapNavigationTransition.action MultiMapNavigationTargetElevator.action)
add_service_files(DIRECTORY srv FILES SetMap.srv ReinitManager.srv)
add_message_files(DIRECTORY msg FILES)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs) 


catkin_package(
  CATKIN_DEPENDS rospy actionlib map_store visualization_msgs move_base_msgs nav_msgs warehouse_ros std_msgs message_runtime
)

install(
  DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

