This is the package for the Olivia III robot to plan and perform tool manipulation given inputs from an external perception module and known functionality features. It is provided as supplementary material as part of the review process for the submitted manuscript "A framework for tool cognition in robots without prior tool learning or observation" by K.P. Tee, S. Cheong, J. Li, and G. Ganesh.

Process description:
====================
1. It subscribes to messages from the perception module which gives information about the object, target, obstacle, and tool candidates.
2. The task is to push the object to the target on a table.
3. Based on the object, target, and obstacle locations, if the robot is unable to find a motion plan with its limb/body, it turns to a tool rack beside it to find potential tools. 
4. Then it creates the MCTS tree and performs the search to find the best augmentation, i.e. combination of grasp and push (functionality) points on the tool, which enable the task to be completed.
5. Finally, it generates the joint motion trajectories and sends them to an external control module to execute the motion on the robot.  


Key source file descriptions:
=============================
- tool_expt_pipeline.cpp : entry point to start the tool experiment.  
- machine.cpp : finite state machine for the entire pipeline.  
- manipulate.cpp : start of manipulation pipeline, various manipulation actions, and callback function for object, target, and obstacle.  
- manipulate_vision.cpp : callback function for tool candidates for no-obstacle case.  
- manipulate_vision_2.cpp : callback function for tool candidates for with-obstacle case.  
- manipulate_mcts_2_left.cpp : checks object, target, tool locations and starts MCTS pipeline
- tool_expt2.cpp : handles the MCTS pipeline, from creating tool representation to conducting the search 
- create_tool.cpp : create tool representation, based on perception data, to facilitate MCTS search for best augmentation.
- search.cpp and node.cpp : MCTS for no-obstacle case
- search2.cpp and node2.cpp : MCTS for with-obstacle case with via point options



Command line:
=============
> rosrun monte_carlo monte_carlo_pipeline _task_mode:=13  


Environment: 
============ 
- Ubuntu 16.04
- ROS kinetic
- C++
- Olivia III Robot


Dependencies provided:  
======================
- cvision_ws : tool perception ros messages 
- kaist_msgs : Olivia III robot ros messages
- m3_moveit : moveit action server for related motion planning tasks

External Dependencies (not provided):  
=====================================
- ros-kinetic-moveit-core
- moveit_object_handling (https://github.com/JenniferBuehler/moveit-pkgs/tree/master/moveit_object_handling)  
- gazebo_ros_link_attacher (https://github.com/pal-robotics/gazebo_ros_link_attacher)
- gazebo_test_tools (https://github.com/JenniferBuehler/gazebo-pkgs/tree/master/gazebo_test_tools)
- object_msgs (https://github.com/JenniferBuehler/general-message-pkgs/tree/master/object_msgs)
- industrial_trajectory_filters (https://github.com/ros-industrial/industrial_core/tree/kinetic-devel/industrial_trajectory_filters)

