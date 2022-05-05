cmake_minimum_required(VERSION 2.8.3)
project(monte_carlo)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  m3_moveit
  roscpp
  sensor_msgs
  std_msgs
  tf
  eigen_conversions
  message_generation
  kdl_parser
  findtoolontalbe
)

find_package(Curses REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  calib.msg
)

add_service_files(
	FILES
	tf_switch.srv
)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	    ${PCL_INCLUDE_DIRS}
)

file(GLOB_RECURSE HEADERS include/*.hpp include/*.h)

add_executable(
	tf_broadcaster
	${HEADERS} # for qtcreator...
	src/dynamic_tf_broadcaster.cpp
)

target_link_libraries(
	tf_broadcaster
	${catkin_LIBRARIES}
)

add_dependencies(
	tf_broadcaster
	m3_moveit_generate_messages_cpp
)

#real robot
add_executable(
	monte_carlo_pipeline
    ${HEADERS} # for qtcreator...
	src/tool_expt_pipeline.cpp
	src/machine.cpp
        #src/perceive.cpp
        
	src/manipulate.cpp
	src/manipulate_mcst.cpp
	src/manipulate_action.cpp
	src/manipulate_vision.cpp
	
	#monte carlo tree
	src/create_tool.cpp     #standalone
	src/gjk.cpp             #standalone
	src/node.cpp            #node depends on kld & moveit_ik
	src/search.cpp          #search depends on node, kdl, moveit_ik, gjk, create_tool
	src/tool_expt.cpp       #tool_expt requires all the four above
	
	src/recorder.cpp
	
	src/search2.cpp
	src/node2.cpp
	src/tool_expt2.cpp
	src/manipulate_vision_2.cpp
	src/manipulate_mcst_2.cpp
	
	src/manipulate_mcst_2_left.cpp
	src/manipulate_mcst_3_left.cpp
	
	src/bucket_pick.cpp
	src/dustpan_pick.cpp
	
	src/manipulate_reachable.cpp
)
target_link_libraries(
	monte_carlo_pipeline
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_dependencies(
	monte_carlo_pipeline
	m3_moveit_generate_messages_cpp
)

#simulation
add_executable(
	monte_carlo_pipeline_sim
	${HEADERS} # for qtcreator...
	src/tool_expt_pipeline.cpp
	src/machine_sim.cpp
        
	src/manipulate.cpp
	src/manipulate_mcst.cpp
	src/manipulate_action.cpp
	src/manipulate_vision.cpp
	
	# monte carlo tree
	src/create_tool.cpp     #standalone
	src/gjk.cpp             #standalone
	src/node.cpp            #node depends on kld & moveit_ik
	src/search.cpp          #search depends on node, kdl, moveit_ik, gjk, create_tool
	src/tool_expt.cpp        #tool_expt requires all the four above

    src/recorder.cpp
	
	src/search2.cpp
	src/node2.cpp	
	src/tool_expt2.cpp
	src/manipulate_vision_2.cpp
	src/manipulate_mcst_2.cpp

	src/manipulate_mcst_2_left.cpp
	src/manipulate_mcst_3_left.cpp
	
	src/bucket_pick.cpp
	src/dustpan_pick.cpp
	
	src/manipulate_reachable.cpp
)
target_link_libraries(
	monte_carlo_pipeline_sim
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(
	monte_carlo_pipeline_sim
	m3_moveit_generate_messages_cpp
)


# home robots
add_executable(
	monte_carlo_home
    src/home.cpp
)
target_link_libraries(
  monte_carlo_home
  ${catkin_LIBRARIES}
)

