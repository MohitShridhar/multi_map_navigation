cmake_minimum_required(VERSION 2.8.3)
project(multi_map_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  map_store
  map_server
  visualization_msgs
  move_base_msgs
  nav_msgs
  warehouse_ros
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

# generate service files
add_service_files(DIRECTORY srv FILES SetMap.srv ReinitManager.srv)

# generate message files
add_message_files(DIRECTORY msg FILES)

# messages dependencies
generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs)



# RVIZ PLUGINS --------------

# catkin dependencies
catkin_package(
  CATKIN_DEPENDS rospy actionlib map_store visualization_msgs move_base_msgs nav_msgs warehouse_ros std_msgs message_runtime nodelet rviz tf map_server
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_definitions(-DHAVE_NEW_YAMLCPP)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/plugins/navigation/navigation_panel.h
  src/plugins/navigation/multi_map_pose_tool.h
  src/plugins/map_manager/map_manager_panel.h
  src/plugins/map_alignment_tools/base_comm_pose_tool.h
  src/plugins/map_alignment_tools/secondary_comm_pose_tool.h
)

set(SOURCE_FILES
  src/plugins/navigation/navigation_panel.cpp
  src/plugins/navigation/multi_map_pose_tool.cpp
  src/plugins/map_manager/map_manager_panel.cpp
  src/plugins/map_manager/map_server.cpp
  src/plugins/map_alignment_tools/base_comm_pose_tool.cpp
  src/plugins/map_alignment_tools/secondary_comm_pose_tool.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(DIRECTORY src/scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
