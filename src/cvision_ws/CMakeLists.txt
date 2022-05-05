cmake_minimum_required(VERSION 2.8.3)
project(findtoolontable)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation
  roscpp
  geometry_msgs
  geometric_shapes
  sensor_msgs
  std_msgs
	shape_msgs
	eigen_conversions
)

find_package(Boost REQUIRED COMPONENTS )
find_package(PCL 1.7 REQUIRED COMPONENTS)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_message_files(
    FILES
	  vision_msgs.msg
		pcl_vector.msg
)

generate_messages(
   DEPENDENCIES
	  sensor_msgs
 	  std_msgs
		geometry_msgs
		shape_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


