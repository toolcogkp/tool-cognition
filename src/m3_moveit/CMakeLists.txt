cmake_minimum_required(VERSION 2.8.3)
project(m3_moveit)

find_package(catkin REQUIRED COMPONENTS 
	moveit_ros_planning 
	moveit_ros_planning_interface 
	moveit_core pluginlib 
    industrial_trajectory_filters_mod
	sensor_msgs
	moveit_msgs
	control_msgs
	message_generation
	actionlib
	actionlib_msgs
	cmake_modules
	std_msgs
)
#cmake_modules

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)

set(ACT_FILES
    MoveitExec.action
    MoveitSingle.action
    MoveitFingers.action
    MoveitDual.action
    MoveitWholeBody.action
    NeckTilt.action
   
	MoveitPickPlace.action
	MoveitCollide.action
)

add_action_files(
	DIRECTORY planning/action
	FILES	${ACT_FILES}
)

generate_messages(
	DEPENDENCIES actionlib_msgs moveit_msgs geometry_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    actionlib_msgs
    control_msgs
    industrial_trajectory_filters_mod
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(planning)

