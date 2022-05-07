/**
 * @file manipulate.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief Copyright 2019 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * manipulate state in the state machine
 * @version 0.1
 * @date 2019-02-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef MANIPULATE_HPP_INCLUDED
#define MANIPULATE_HPP_INCLUDED

// state machine
#include <tool_expt/machine.h>

// eigen
#include <Eigen/Core>
#include <Eigen/SVD>

// messages
#include <sensor_msgs/JointState.h>
#include <sound_play/sound_play.h>

// action client
#include <actionlib/client/simple_action_client.h>
#include <m3_moveit/MoveitSingleAction.h>
#include <m3_moveit/MoveitFingersAction.h>
#include <m3_moveit/MoveitDualAction.h>
#include <m3_moveit/MoveitWholeBodyAction.h>

#include "neck_pan/NeckPanAction.h"
#include "neck_pan/NeckPanFeedback.h"
#include "neck_pan/NeckPanResult.h"
#include <m3_moveit/NeckTiltAction.h>

#include <m3_moveit/MoveitPickPlaceAction.h>
#include <m3_moveit/MoveitCollideAction.h>

#include "monte_carlo/tf_switch.h"

// tool experiment objects
#include <tool_expt/create_tool.h>
#include <tool_expt/search.h>
#include <tool_expt/tool_expt.h>
#include <tool_expt/moveit_ik.h>
#include <tool_expt/kdl_ik.h>
#include "tool_expt/vision_data.h"

#include <tool_expt/search2.h>
#include <tool_expt/tool_expt2.h>
#include <tool_expt/definitions.h>

// pcl
#include <geometric_shapes/shape_operations.h>
#include <pcl_conversions/pcl_conversions.h>

// visualization messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace sc = boost::statechart;
namespace mpl = boost::mpl;



//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

/**
 * @brief State machine Event structure: Manipulation
 * This event is invoked by Olivia state machine when in the Active state.
 * Perform sequences of action to manipulate object to a given target goal.
 * 
 * State machine will handle reaction based on target position and perceived object position.
 * The objective is to bring the object to the target.
 * 
 * when the object is out of reach for Olivia's arm, perform search for viable tool in surroundings,
 * use the monte carlo search tree algorithm to determine how to use the percieved tool as an extension of its arm,
 * and finally executes the collision-free planning.
 * 
 */
struct Manipulate : sc::state< Manipulate, Active > 
{
public:
      /// list of state machine transition reaction
      typedef mpl::list<
      sc::custom_reaction< EvStart >,
      sc::custom_reaction< EvPerceiveObject >,
      sc::custom_reaction< EvObjectFrontOfTargetRightOfRobot >,
      sc::custom_reaction< EvObjectBackOfTargetRightOfRobot >,
      sc::custom_reaction< EvObjectRightOfTarget >,
      sc::custom_reaction< EvObjectFrontRightOfTargetTool >,
      sc::custom_reaction< EvObjectBackRightOfTargetTool >,
      sc::custom_reaction< EvObjectFrontOfTargetRightOfRobotTool >,
      sc::custom_reaction< EvObjectFrontOfTargetLeftOfRobotTool >,
      sc::custom_reaction< EvObjectRightOfTargetTool >,
      sc::custom_reaction< EvObjectBackOfTargetRightOfRobotTool >,
      sc::custom_reaction< EvReady >,
      sc::custom_reaction< EvTargetNofObject >,
      sc::custom_reaction< EvTargetSofObject >,
      sc::custom_reaction< EvTargetOmniObject >,
      sc::custom_reaction< EvMonteCarloExpt >,
      sc::custom_reaction< EvMonteCarloExpt2 >,
      sc::custom_reaction< EvMonteCarloExpt2_left >,
      sc::custom_reaction< EvMonteCarloExpt3_left >
      > reactions;

      /**
       * @brief Construct a new Manipulate object
       * 
       * @param ctx 
       */
      Manipulate(my_context ctx);
      /**
       * @brief Destroy the Manipulate object
       * 
       */
      ~Manipulate();

//state machine reactions
//----------------------------------------------------------------------
      sc::result react( const EvStart & );
      sc::result react( const EvPerceiveObject & );
      sc::result react( const EvObjectFrontOfTargetRightOfRobot & );
      sc::result react( const EvObjectBackOfTargetRightOfRobot & );
      sc::result react( const EvObjectRightOfTarget & );
      sc::result react( const EvObjectFrontRightOfTargetTool & );
      sc::result react( const EvObjectBackRightOfTargetTool & );
      sc::result react( const EvObjectFrontOfTargetRightOfRobotTool & );
      sc::result react( const EvObjectFrontOfTargetLeftOfRobotTool & );
      sc::result react( const EvObjectRightOfTargetTool & );
      sc::result react( const EvObjectBackOfTargetRightOfRobotTool & );
      sc::result react( const EvReady & );
      sc::result react( const EvTargetNofObject & );
      sc::result react( const EvTargetSofObject & );
      sc::result react( const EvTargetOmniObject & );
      sc::result react( const EvMonteCarloExpt & );
      sc::result react( const EvMonteCarloExpt2 & );
      sc::result react( const EvMonteCarloExpt2_left & );
      sc::result react( const EvMonteCarloExpt3_left & );

private:

//parameters and variables
//----------------------------------------------------------------------
      // ros nodehandler
      ros::NodeHandle n;

      // speech module
      sound_play::SoundClient speech;

      // subscribers
      ros::Subscriber sub_object_printout;
      ros::Subscriber sub_object;
      ros::Subscriber sub_tool_handle;
      ros::Subscriber sub_bendtool;
      ros::Subscriber sub_tool_in_hand;
      ros::Subscriber sub_fake_tool_transform;
      ros::Subscriber sub_fake_target;
      ros::Subscriber sub_calib_next;
      ros::Subscriber sub_calib_tool_poses;
      ros::Subscriber sub_tool_decision;
      ros::Subscriber sub_joint_states;
      ros::Subscriber sub_sticklength;
      ros::Subscriber sub_neck_pan;

      // publishers
      ros::Publisher pub_calib;
      ros::Publisher pub_req_object_target;
      ros::Publisher pub_req_tool_handle;
      ros::Publisher pub_table;

      // poses 
      geometry_msgs::PoseStamped object_in_base, target_in_base, tool_in_base, bendtool_in_base;
      geometry_msgs::PoseStamped object_in_base_live, target_in_base_live, tool_in_base_live, bendtool_in_base_live;
      geometry_msgs::PoseStamped obj_tar_center;
      geometry_msgs::PoseArray::Ptr poses_in_camframe;
      geometry_msgs::PoseStamped pose_grasped_tool_in_base_live;
      geometry_msgs::Pose obj2hand;
      geometry_msgs::Pose omnistraight;
      geometry_msgs::Pose fake_tool_transform, fake_tool_transform_live;
      geometry_msgs::Pose bend_tool_transform, bend_tool_transform_live;
      geometry_msgs::PoseArray calib_tool_poses_live;
      geometry_msgs::Point pullback_offset_right_hand, pullback_offset_left_hand, pushsideways_offset_right_hand, pushsideways_offset_left_hand, pushforward_offset_right_hand, pushforward_offset_left_hand, bendtool_offset_right_hand, tool_offset_right_hand, pushforward_offset_right_tool, pullback_offset_right_tool, pushsideways_offset_right_tool;
      geometry_msgs::Quaternion pullback_orientation_right_hand, pullback_orientation_left_hand, pushsideways_orientation_right_hand, pushsideways_orientation_left_hand, pushforward_orientation_right_hand, pushforward_orientation_left_hand, bendtool_orientation_right_hand, tool_orientation_right_hand, pushforward_orientation_right_tool, pullback_orientation_right_tool, pushsideways_orientation_right_tool;
      geometry_msgs::Point object_to_hand_position_offset, object_to_tool_position_offset;
      geometry_msgs::Quaternion object_to_hand_orientation_offset;
      std::vector<geometry_msgs::Point> boundary_forw, boundary_back, boundary_side;

      geometry_msgs::Point forward_offset_right_hand;
      geometry_msgs::Quaternion forward_orientation_right_hand;
      geometry_msgs::Point forward_offset_2_right_hand;
      geometry_msgs::Quaternion forward_orientation_2_right_hand;

      geometry_msgs::Point backward_offset_right_hand;
      geometry_msgs::Quaternion backward_orientation_right_hand;
      geometry_msgs::Point backward_offset_2_right_hand;
      geometry_msgs::Quaternion backward_orientation_2_right_hand;

      geometry_msgs::Point omni_offset_right_hand;
      geometry_msgs::Quaternion omni_orientation_right_hand;

      geometry_msgs::Point omni_offset_right_hand2;
      geometry_msgs::Quaternion omni_orientation_right_hand2;

      geometry_msgs::Point omni_offset_target;
      geometry_msgs::Quaternion omni_orientation_target;

      tf::TransformListener tf_listener;
      sensor_msgs::JointState joint_state;

      bool object_target_received;
      bool tool_handle_received;
      bool bendtool_received;
      bool fake_tool_transform_received;
      bool fake_target_received;
      bool bend_tool_transform_received;
      bool bend_target_received;
      bool using_tool;
      bool tool_decision_received;
      bool sticklength_received;
      bool calib_tool_poses_received;
      int calib_next;
      int is_suitable_tool;
      int functionality;
      double fake_target_x ;
      double fake_target_y ;
      double TABLE_HEIGHT, PUCK_HEIGHT, PUCK_RADIUS, TOOL_HANDLE_HEIGHT, HAND_THICKNESS, LIFT_TOOL_HEIGHT, MOVE_WAIST_ANGLE; 
      double WAIST_INIT, WAIST_TOOL;
      double BENDTOOL_RADIUS, LIFT_BENDTOOL_HEIGHT;
      double sleep_time;
      double waist_yaw, base_yaw;
      double sticklength, min_augment;

      double OBST_RADIUS;
      double init_x, init_y, init_z;

//callback functions
//----------------------------------------------------------------------

      /**
       * @brief subscriber callback to output object data on screen
       * 
       * @param msg 
       */
      void object_printout_callBack(const geometry_msgs::PoseArray::Ptr& msg);
      /**
       * @brief subscriber callback to obtain target pose with respect to object from perception module
       * 
       * @param msg 
       */
      void object_target_positions_callBack(const geometry_msgs::PoseArray::Ptr& msg);

      /**
       * @brief subscriber callback to obtain position of tool handle from perception module
       * 
       * @param msg 
       */
      void tool_handle_position_callBack(const geometry_msgs::Pose::Ptr& msg);

      /**
       * @brief subscriber callback to obtain position of bendtool handle from perception module
       * 
       * @param msg 
       */
      void bendtool_position_callBack(const geometry_msgs::Pose::Ptr& msg);

      /**
       * @brief subscriber callback to obtain tf of a fake tool
       * 
       * @param msg 
       */
      void fake_tool_transform_callBack(const geometry_msgs::PoseArray::Ptr& msg);

      /**
       * @brief subscriber callback to obtain position of a fake target
       * 
       * @param msg 
       */
      void fake_target_callBack(const geometry_msgs::PoseArray::Ptr& msg);

      /**
       * @brief subscriber callback to calibrate tool pose
       * 
       * @param msg 
       */
      void calib_tool_poses_callBack(const geometry_msgs::PoseArray::Ptr& msg);

      /**
       * @brief subscriber callback to obtain decision for which tool to use
       * 
       * @param msg 
       */
      void tool_decision_callBack(const std_msgs::Int32::Ptr& msg);

      /**
       * @brief subscriber callback to perform calibration for next tool
       * 
       * @param msg 
       */
      void calib_next_callBack(const std_msgs::Int32::Ptr& msg);

      /**
       * @brief subscriber callback to obtain joint states of Olivia robot
       * 
       * @param msg 
       */
      void joint_states_callBack(const sensor_msgs::JointState::Ptr& msg);

      /**
       * @brief subscriber callback to obtain length of perceived tool/stick
       * 
       * @param msg 
       */
      void sticklength_callBack(const std_msgs::Float64::Ptr& msg);

      /**
       * @brief subscriber callback to obtain joint state of Olivia neck
       * The neck pan motor for Olivia was separately added to allow rotation of the neck
       * Hence, the joint state was obtained individually here
       * 
       * @param msg 
       */
      void neck_pan_state_callBack(const sensor_msgs::JointState::Ptr& msg);

//functions
//----------------------------------------------------------------------
      
      /**
       * @brief Function to fill moveit action goal with the joint name of group used on Olivia
       * 
       * @return m3_moveit::MoveitWholeBodyGoal moveit action goal
       */
      m3_moveit::MoveitWholeBodyGoal fillJointNamesFull();

      /**
       * @brief Function to fill moveit action goal with all the joint names on Olivia
       * 
       * @return m3_moveit::MoveitWholeBodyGoal moveit action goal
       */
      m3_moveit::MoveitWholeBodyGoal fillNamesFull();

      /**
       * @brief Set the Joint Targets To Current Position -> used as default  
       * 
       * @return m3_moveit::MoveitWholeBodyGoal moveit action goal
       */
      m3_moveit::MoveitWholeBodyGoal SetJointTargetsToCurrent();

      /**
       * @brief Function to compute the transform matrix of Tool object to Olivia's Hand
       * Basically, this computes how to hold the tool in olivia's hand
       * 
       * @param calib_hand_poses the center of hand pose of Olivia
       * @param calib_tool_poses the perceived Tool object pose
       * @return Eigen::MatrixXd transformation matrix of Tool with respect to Hand
       */
      Eigen::MatrixXd ComputeTool2HandTransform(const geometry_msgs::PoseArray calib_hand_poses, const geometry_msgs::PoseArray calib_tool_poses);
      
      /**
       * @brief Function to get only the joint names of the fingers in a single Olivia's hand
       * 
       * @param hand the name of the olivia hand: left or right
       * @return std::vector<std::string> the joint names
       */
      std::vector<std::string> fillJointNamesFingers(std::string hand);

      /**
       * @brief Function to get only the joint names of the finger encoders in a single Olivia's hand
       * 
       * @param hand the name of the olivia hand: left or right
       * @return std::vector<std::string> the joint names
       */
      std::vector<std::string> fillJointNamesFingersEncoders(std::string hand);

      /**
       * @brief generate the boundary data for different hand poses 
       * The boundary data depends on the functionality that is required to be performed.
       * Such as pushing or pulling and etc.
       * 
       * @param functionality which functionality (eg 1:forw, 2:back , 3:side )
       * @return std::vector<geometry_msgs::Point> boundary data as geometry points
       */
      std::vector<geometry_msgs::Point> load_boundary(const int functionality);

      /**
       * @brief Function to change a given pose from its current frame to another frame
       * 
       * @param pose_msg given pose
       * @param from_frame current frame
       * @param to_frame target frame
       * @return geometry_msgs::PoseStamped pose in target frame
       */
      geometry_msgs::PoseStamped transformPose(geometry_msgs::Pose pose_msg, std::string from_frame, std::string to_frame);
      
      /**
       * @brief Function to check if given pose is reachable by Olivia's arm
       * 
       * @param pose given pose
       * @return true the given pose is reachable by Olivia
       * @return false the given pose is not reachable -> tool is required
       */
      bool isReachable(geometry_msgs::Pose pose);

      /**
       * @brief Function to request a sequence of motion to calibrate the pose of the tool
       * 
       * @param hand which olivia hand
       * @return true calibration successful
       * @return false calibration unsuccessful
       */
      bool ToolInHandCalibration(std::string hand);

      /**
       * @brief Function to do a sweep with Olivia's neck to detect the object's location
       * 
       */
      void CheckObjectLocation();

      /**
       * @brief Function to do a sweep with Olivia's neck to search for viable tool
       * 
       */
      void CheckToolLocation();

      /**
       * @brief Function to do a sweep with Olivia's neck to search for viable bendtool
       * 
       */
      void CheckBendToolLocation();

      /**
       * @brief Function to obtain the transform matrix of a fake tool
       * 
       */
      void CheckFakeToolTransform();

      /**
       * @brief Function to obtain the transform matrix of a bend tool
       * 
       */
      void CheckBendToolTransform();

      /**
       * @brief Function to call moveit to rotate the Waist to target angle
       * 
       * @param angle_deg target angle
       */
      void MoveWaist(double angle_deg);

      /**
       * @brief Function to call moveit to rotate the Waist by an incremental angle from current
       * 
       * @param angle_deg amount of angle to move
       */
      void MoveWaistFromCurrent(double angle_deg);

      /**
       * @brief Function to call moveit to move the waist & the trunk of Olivia.
       * There are 2 joints involved, waist and trunk.
       * 
       * @param angle_deg target waist angle
       * @param angle_deg_2 target trunk angle
       */
      void MoveWaistTrunkFromCurrent(double angle_deg, double angle_deg_2);

      /**
       * @brief In gazebo simulation only. Function call for gazebo to respawn Puck object
       * 
       */
      void ResetPuck();
      /**
       * @brief In gazebo simulation only. Function call for gazebo to respawn Tool object
       * 
       */
      void ResetTool();

      /**
       * @brief Function to compute a circle
       * 
       */
      void DrawCircle();

      /**
       * @brief Function call for moveit to move hand/end-effector to a hardcoded position.
       * Prototype function to test moveit only. Used for maintenance/hardware debugging check.
       */
      void TestHandOrient();

      /**
       * @brief Function to call moveit to send Olivia into 'Home' pose
       * 
       */
      void Home();

      /**
       * @brief Function to call moveit to send Olivia into 'Home' pose but at a given Waist orientation
       * Please check moveit srdf file for joint angles of stated 'Home' pose.
       * 
       * @param waist_val target waist angle
       * @return int 
       */
      int Home(double waist_val);

      /**
       * @brief Function to call moveit to send Olivia into 'Home2' pose but at a given Waist orientation
       * Please check moveit srdf file for joint angles of stated 'Home2' pose.
       * 
       * @param waist_val target waist angle, default angle=0
       * @return int 
       */
      int Home2(double waist_val=0);
      
      /**
       * @brief Function to call speech library to vocalise a text 
       * 
       * @param string text to speak
       */
      void Speech(std::string string);

      /**
       * @brief return sign of number
       * 
       * @param num input
       * @return double output( +1.0, -1.0, 0 )
       */
      double signum(const double num);

      /**
       * @brief Function to move Olivia's hand by a given amount from it's current position
       * 
       * @param link name of link/end-effector
       * @param moveX translation in x
       * @param moveY translation in y
       * @param moveZ translation in z
       * @param Npts number of waypoints interpolation
       * @param pose_array_target return the set of target poses 
       */
      void translateHandFromCurrent(const std::string link, const std::vector<double> moveX, const std::vector<double> moveY, const std::vector<double> moveZ, const int Npts, geometry_msgs::PoseArray& pose_array_target);
      
      /**
       * @brief Get the Current Transforms of Olivia's left and right arm
       * 
       * @param tfQuat_left quaternion of left arm
       * @param tfQuat_right quaternion of right arm
       * @param tfVec_left linear translation of left arm
       * @param tfVec_right linear translation of right arm
       */
      void getCurrentTransforms(tf::Quaternion& tfQuat_left, tf::Quaternion& tfQuat_right,  tf::Vector3& tfVec_left, tf::Vector3& tfVec_right);
      
      /**
       * @brief Get the Current Transform of Olivia's given link
       * 
       * @param link desired link name
       * @param tfQuat quaternion of link
       * @param tfVec linear translation of link
       */
      void getCurrentTransform(const std::string link, tf::Quaternion& tfQuat, tf::Vector3& tfVec);

      /**
       * @brief Get the Current Transform of Olivia's given link, return in geometry pose
       * 
       * @param link desired link_name
       * @param output geometry pose
       * @return true successful
       * @return false failed
       */
      bool getCurrentTransform(const std::string link, geometry_msgs::PoseStamped& output);

      /**
       * @brief Action Sequence to pick up Tool with the right arm of Olivia
       * 
       * @return int result: -1=failed
       */
      int PickupToolRightArm();

      /**
       * @brief Action Sequence to pick up Bend Tool -> first test case
       * 
       * @return int result: -1=failed
       */
      int PickupBendTool();

      /**
       * @brief Action Sequence to pick up a Bend Tool with variable length
       * 
       * @param stick_length length of stick
       * @param head_length length of head
       * @param head_angle_deg angle of head on bendtool
       * @return int result: -1=failed
       */
      int BendTool(double stick_length, double head_length, double head_angle_deg);

      /**
       * @brief Function to open the fingers of Olivia
       * 
       * @param side left or right side of Olivia
       * @param mode  0= wtout tool, 1= wt tool
       * @return int result: -1=failed
       */
      int OpenFingers(std::string side, int mode);

      /**
       * @brief Function to close fingers of Olivia
       * 
       * @param side left or right side of Olivia
       * @param grasp_type 0=none, 1=tool, 2=bendtool
       * @param mode  0= wtout tool, 1= wt tool
       * @return int result: -1=failed
       */
      int CloseFingers(std::string side, int grasp_type, int mode);

//Actions
//------------------------------------------------------------------------//

      /**
       * @brief Action sequence to pull back left arm
       * 
       * @return int result: -1=failed
       */
      int ActionPullBackLeftArm();
      /**
       * @brief Action sequence to pull back right arm
       * 
       * @return int result: -1=failed
       */
      int ActionPullBackRightArm();
      /**
       * @brief Action sequence to pull back right arm with tool in hand
       * 
       * @return int result: -1=failed
       */
      int ActionPullBackRightArmTool();
      /**
       * @brief Action sequence to pull back arm with bendtool in hand
       * 
       * @return int result: -1=failed
       */
      int ActionPullBackBendTool();
      /**
       * @brief Action sequence to push sideways with left arm
       * 
       * @return int result: -1=failed
       */
      int ActionPushSidewaysLeftArm();
      /**
       * @brief Action sequence to push sidewyas with right arm
       * 
       * @return int result: -1=failed
       */
      int ActionPushSidewaysRightArm();
      /**
       * @brief Action sequence to push forward left direction with left arm
       * 
       * @return int result: -1=failed
       */
      int ActionPushForwardLeftArm();
      /**
       * @brief Action sequence to push forward with right arm
       * 
       * @param using_tool true=with tool, false=without tool
       * @return int result: -1=failed
       */
      int ActionPushForwardRightArm(bool using_tool);
      /**
       * @brief Action sequence to push forward right with arm.
       * special sequence test 
       * 
       * @return int result: -1=failed
       */
      int ActionPushForwardRightArmSpecial();
      /**
       * @brief Action sequence to Return Tool back to where it was obtained.
       * For right arm
       * @return int result: -1=failed
       */
      int ActionReturnToolRightArm();
       /**
       * @brief Action sequence to Return Tool back to where it was obtained.
       * For right arm,
       * special sequence test 
       * @return int result: -1=failed
       */
      int ActionReturnToolRightArmSpecial();
      /**
       * @brief Action sequence to Return BendTool back to where it was obtained.
       * 
       * @return int result: -1=failed
       */
      int ActionReturnBendTool();
      /**
       * @brief Action sequence to push forward in North direction
       * 
       * @return int result: -1=failed
       */
      int ActionPushForwardN();
      /**
       * @brief Action sequence to push forward in North direction with tool
       * 
       * @return int result: -1=failed
       */
      int ActionPushForwardN_Tool();
      /**
       * @brief Action sequence to pull object towards south direction
       * 
       * @return int result: -1=failed
       */
      int ActionPushbackS();
      /**
       * @brief Action sequence to pull object towards south direction with tool
       * 
       * @return int result: -1=failed
       */
      int ActionPushbackS_Tool();
      /**
       * @brief Action sequence to pull/push object in any direction with right arm
       * 
       * @return int result: -1=failed
       */
      int ActionOmniRight();
      /**
       * @brief Action sequence to pull/push object in any direction with Tool in right arm
       * 
       * @return int result: -1=failed
       */
      int ActionOmniRight_Tool();

//------------------------------------------------------------------------//

      /**
       * @brief Execution Handler function for call action sequence for North push
       * 
       * @return int result: -1=failed
       */
      int ExNorth();
      /**
       * @brief Execution Handler function for call action sequence for North push
       * This one also checks for target within reach or not.
       * Decides if tool is required and what action to take
       * 
       * @param object_within_reach true: yes, false: no
       * @param target_within_reach true: yes, false: no
       * @return int result: -1=failed
       */
      int ExNorthTool(bool object_within_reach, bool target_within_reach);

      /**
       * @brief  Execution Handler function for call action sequence for South pull
       * 
       * @return int result: -1=failed
       */
      int ExSouth();
      /**
       * @brief Execution Handler function for call action sequence for South pull
       * This one also checks for target within reach or not.
       * Decides if tool is required and what action to take
       * 
       * @param object_within_reach true: yes, false: no
       * @param target_within_reach true: yes, false: no
       * @return int result: -1=failed
       */
      int ExSouthTool(bool object_within_reach, bool target_within_reach);

      /**
       * @brief Execution Handler function for call action sequence for omnidirectional pull/push
       * 
       * @param repoflag perform a reposition of arm: true=yes, false=no
       * @return int int result: -1=failed
       */
      int ExOmni(bool repoflag);
      /**
       * @brief Execution Handler function for call action sequence for omnidirectional pull/push
       * This one also checks for target within reach or not.
       * Decides if tool is required and what action to take
       * 
       * @param object_within_reach true: yes, false: no
       * @param target_within_reach true: yes, false: no
       * @return int result: -1=failed
       */
      int ExOmniTool(bool object_within_reach, bool target_within_reach);

// newer functions
//------------------------------------------------------------------------//

      /**
       * @brief Function to perform pinching with Olivia fingers
       * Compute the encoder value to close based on input tool size
       * 
       * @param side left or right
       * @param diameter_in_cm diameter of close to
       * @param mode 0= wtout tool, 1= wt tool
       * @return int result -1=failed
       */
      int pinchFingers(std::string side, double diameter_in_cm, int mode);
       /**
       * @brief Function to perform pinching with Olivia fingers but with tighter grip force
       * Compute the encoder value to close based on input tool size but tighter (larger gain)
       * 
       * @param side left or right
       * @param diameter_in_cm diameter of close to
       * @param mode 0= wtout tool, 1= wt tool
       * @return int result -1=failed
       */
      int powerFingers(std::string side, double diameter_in_cm, int mode);
      /**
       * @brief Function to move fingers based on fraction between minimum and maximum joint values
       * 
       * @param side left or right
       * @param fraction 0-100% of hand opening
       * @param mode  0= wtout tool, 1= wt tool
       * @return int result -1=failed
       */
      int MoveFingers(std::string side, double fraction, int mode);

      /**
       * @brief Function to tilt neck to desired angle
       * 
       * @param neck_tilt_angle desired neck tilt angle
       * @return int result -1=failed
       */
      int NeckTilt(double neck_tilt_angle);
      /**
       * @brief Function to pan the neck to desired angle
       * 
       * @param pan_angle desired neck pan angle
       * @param pan_speed speed of motor
       * @return int result -1=failed
       */
      int NeckPan(double pan_angle, double pan_speed);
      /**
       * @brief Funtion to compute what angle to turn the neck to face the detected object center
       * 
       * @return int result -1=failed
       */
      int TurnHeadToObjTarCenter();
      /**
       * @brief Function to perform both neck panning and tilting at the same time
       * 
       * @param neck_tilt_angle desired angle
       * @param neck_pan_angle desired angle
       * @return int result -1=failed
       */
      int TurnHeadTiltPan(double neck_tilt_angle, double neck_pan_angle);

      /**
       * @brief Action sequence to perform both neck pan and tilt
       * 
       * @param state the desired state
       * @param result -1=failed
       */
      void ActionClientNeckPanDoneCallback(const actionlib::SimpleClientGoalState &state, const neck_pan::NeckPanResultConstPtr &result);

      /**
       * @brief Function to compute the minimum augmentation required for object to reach target
       * Augmentation length refers to the length of the tool required to push object to the target.
       * Need to search for tool of this length
       * 
       * @param boundary boundary
       * @param object the object pose
       * @param target the desired target pose
       * @param object_within_reach true=yes, false=no
       * @param target_within_reach true=yes, false=no
       * @return double the length/distance in meters
       */
      double compute_min_aug(const std::vector<geometry_msgs::Point> boundary, const geometry_msgs::Point object, const geometry_msgs::Point target, const bool object_within_reach, const bool target_within_reach);

      bool joint_updated;
      double angle, radian; 
      double neck_pan_angle, neck_tilt_angle;

      double current_pan_angle;
      double current_tilt_angle;

      /**
       * @brief Keyboard input function
       * 
       * @return true 
       * @return false 
       */
      bool kbhit();

      /**
       * @brief Create_Tool object. Please refer to create_tool.h
       * 
       */
      Create_Tool m_create_tool;

      /**
       * @brief Tool_Expt object. Please refer to tool_expt.h
       * 
       */
      Tool_Expt m_tool_expt;

      /**
       * @brief MCT search object. Please refer to search.h
       * 
       */
      MCT_Search m_mct_search;

      /**
       * @brief Moveit IK object. Please refer to moveit_ik.h
       * 
       */
      MOVEIT_IK m_moveit_ik;
      /**
       * @brief KDL IK object. Please refer to kdl_ik.h
       * 
       */
      KDL_IK m_kdl_ik;

      //results
      vector< Affine3d > m_result;  //_T_real_grasp
      Affine3d m_result_grab;
      double m_result_atk;
      double max_score;

      Affine3d create_affine(double roll, double pitch, double yaw, double x, double y, double z);
      Affine3d create_affine(double roll, double pitch, double yaw, Vector3d xyz);
      vector<double> joint_values;
      vector<double> joint_values_l;

      /**
       * @brief Obtain object and target position
       * 
       * @return int result: -1=failed
       */
      int findObjectTargetLocation();
      /**
       * @brief Obtain Tool and target position
       * 
       * @return int result: -1=failed
       */
      int findToolTargetLocation();

      /**
       * @brief perform mct search and tool usage experiment
       * This function performs the computation and planning
       * 
       * @return int 
       */
      int mct_tool_experiment();

      /**
       * @brief perform experiment
       * This function performs the execution of the Olivia robot.
       * Followed after solution is found by mct_tool_experiment() function.
       * 
       * @return int 
       */
      int perform_experiment();


//moveit planning scene functions
//----------------------------------------------------------------------//

      /**
       * @brief Action client result callback.
       * Non-blocking implementation of action client 
       * 
       * @param state goal
       * @param result 
       */
      void doneCb(const actionlib::SimpleClientGoalState& state, const m3_moveit::MoveitSingleResultConstPtr& result);
      /**
       * @brief Action client active callback.
       * Non-blocking implementation of action client. Not used.
       */
      void activeCb();
      /**
       * @brief Action client feedback callback.
       * Non-blocking implementation of action client.
       * 
       * @param feedback 
       */
      void feedbackCb(const m3_moveit::MoveitSingleFeedbackConstPtr& feedback);
      /**
       * @brief Check for user input.
       * When moveit planning is done, for dangerously long objects/tool,
       * user is required to provide an input to continue with execution.
       * 
       * @return int 
       */
      int checkUserInput();

      /**
       * @brief Moveit Action client call to attach a collision object to the given arm
       * in the robot model in the planning scene for collision-free planning
       * 
       * @param arm_name left or right
       * @param obj_name the name of the collision object that exist in the planning scene
       * @param cobj data of collision object (eg mesh)
       * @return int result: -1=failed
       */
      int ActionAttachObj(string arm_name, string obj_name, moveit_msgs::CollisionObject cobj);
      /**
       * @brief Moveit Action Client to detact a previously attached object from the given arm
       * 
       * @param arm_name left or right
       * @param obj_name the name of the collision object that exist in the planning scene
       * @return int result: -1=failed
       */
      int ActionDetachObj(string arm_name, string obj_name);
      /**
       * @brief Moveit Action Client to remove an object from the planning scene
       * 
       * @param obj_id the object name
       * @return int result: -1=failed
       */
      int ActionRmObj(string obj_id);
      /**
       * @brief Moveit Action Client to add an object into the planning scene
       * 
       * @param obj_id the object name
       * @param cobj the object data (eg mesh)
       * @return int result: -1=failed
       */
      int ActionAddObj(string obj_id, moveit_msgs::CollisionObject cobj);

//manipulate_action newer
//----------------------------------------------------------------------//

      /**
       * @brief Action to go to a position. Old test function.
       * 
       * @return int result: -1=failed
       */
      int ActionGoToPos();
      /**
       * @brief Action to go to a position.
       * 
       * @param this_pose pose in eigen
       * @param plan_only true=do not execute, finish when planning is done.
       * @param action_name the moveit planning function to use
       * @param replan true=if planning failed, continue to re-attempt planning
       * @param link_name desired link name
       * @return int result: -1=failed
       */
      int ActionGoToPos(Affine3d this_pose, bool plan_only=false, string action_name="task_single_right", bool replan=false, string link_name="ee");
      /**
       * @brief Action to go to a position via multiple waypoints.
       * 
       * @param these_poses set of poses in eigen
       * @param plan_only true=do not execute, finish when planning is done.
       * @param action_name the moveit planning function to use
       * @param replan true=if planning failed, continue to re-attempt planning
       * @param link_name desired link name
       * @return int result: -1=failed
       */
      int ActionGoToPos(std::vector< Affine3d > these_poses, bool plan_only=false, string action_name="task_single_right", bool replan=false, string link_name="ee");
      /**
       * @brief Action to go to a position.
       * 
       * @param action_pos translation of desired pose
       * @param action_quat quaternion of desired pose
       * @param plan_only true=do not execute, finish when planning is done.
       * @param action_name the moveit planning function to use
       * @param replan true=if planning failed, continue to re-attempt planning
       * @param link_name desired link name
       * @return int result: -1=failed
       */
      int ActionGoToPos(Vector3d action_pos, Quaterniond action_quat, bool plan_only=false, string action_name="cart_single_right", bool replan=false, string link_name="ee");
      /**
       * @brief Action to go to a position.
       * Caertesian space planning of moveit is able to execute a plan that is not 100% successfully planned.
       * If planning > desired fraction, execute the incomplete plan anyway.
       * 
       * @param these_poses set of poses in eigen
       * @param fraction the allowed fraction of movement
       * @param plan_only true=do not execute, finish when planning is done.
       * @param action_name the moveit planning function to use
       * @param replan true=if planning failed, continue to re-attempt planning
       * @param link_name desired link name
       * @return int result: -1=failed
       */
      int ActionGoToPos(std::vector< Affine3d > these_poses, double* fraction, bool plan_only=false, string action_name="cart_single_right", bool replan=false, string link_name="ee");

      /**
       * @brief Action to add collision object into planning scene
       * Used at prototyping stage to add known objects into the planning scene.
       * Such as table or chair and etc.
       * 
       * @param ids set of index/name to identify objects
       * @param meshfiles set of directory path for objects
       * @param poses set of poses to place objects
       * @return int result: -1=failed
       */
      int addCollision(vector< string > ids, vector< string > meshfiles, vector< geometry_msgs::Pose > poses);
      /**
       * @brief Action to add tool collision into planning scene.
       * Only used during gazebo simulation.
       * @return int result: -1=failed
       */
      int addToolCollision();
      /**
       * @brief Action to add a puck collision into planning scene.
       * Only used during gazebo simulation.
       * @param radius puck radius
       * @return int result: -1=failed
       */
      int addPuckCollision(double radius = 0.04);
      /**
       * @brief Action to add a table collision into planning scene.
       * 
       * @return int result: -1=failed
       */
      int addTableCollision();
      /**
       * @brief Action to add a tool rack collision into planning scene.
       * 
       * @return int result: -1=failed
       */
      int addRackCollision();

      /**
       * @brief Computation function to provide pose based on interpolation
       * When using Caertesian space planning of moveit, and planning is not 100% to target, 
       * do this to quickly obtain a good estimate of pose before execution
       * 
       * @param fraction percentage of plan
       * @param start start or current pose of plan
       * @param end end or target pose of plan
       * @return Affine3d estimated pose at given fraction of plan
       */
      Affine3d EstimatePose(double fraction, Affine3d start, Affine3d end);
      /**
       * @brief Compute if Re-adjustment is required.
       * Usually used when moveit failed to compute a plan and need to shift the hand slightly,
       * in hopes of allowing planning of higher success rate.
       * 
       * @param idx the index
       * @param gohere set of eigen pose targets
       * @param fraction allowable fraction
       * @param start_TF starting transform pose
       * @param end_TF ending transform pose
       * @return true successful
       * @return false failed
       */
      bool ReAdjustment(int idx, vector< Affine3d > gohere, double fraction, Affine3d start_TF, Affine3d end_TF);

      /// keep track of a list of collision items
      vector< moveit_msgs::CollisionObject > collision_list;
      moveit_msgs::CollisionObject puck_collision;
      moveit_msgs::CollisionObject tool_collision;
      moveit_msgs::CollisionObject table_collision;
      moveit_msgs::CollisionObject table2_collision;
      moveit_msgs::CollisionObject table3_collision;

      moveit_msgs::CollisionObject rack_collision;

      ros::Publisher display_rs_publisher, input_pub, markers_pub, camera_swap_publisher;
      moveit_msgs::DisplayRobotState drs;


//vision
//----------------------------------------------------------------------//
      geometry_msgs::PoseStamped vtool_in_base;

      /**
       * @brief Execution Function for perception module
       * 
       * @return int 
       */
      int vision_cb_test();
      /**
       * @brief Action sequence to finish experiment when using perception module
       * Return tool to rack and etc.
       * 
       * @param tool_in_hand true=yes, false=no
       * @param okay experiment was success or not
       * @param wflag use additional waypoints or not
       */
      void finish_v(bool tool_in_hand, bool okay=true, bool wflag=false);

      ros::Subscriber sub_vision;
      /**
       * @brief Callback function to receive data from perception module
       * 
       * @param msg 
       */
      void vision_data_callBack(const findtoolontalbe::vision_msgs::Ptr &msg);
      bool detected;

      //vision data
      int tool_num;
      double current_tool_width;
      vision_data v_data;
      std::vector< vision_each > v_list;
      std::vector< shape_msgs::Mesh > toolMeshes;

      moveit_msgs::CollisionObject vtool_collision;
      /**
       * @brief Function to add Perceived object from perception module into the moveit planning scene
       * 
       * @param co_mesh the segmented mesh of object
       * @return int result: -1=failed
       */
      int addVisualToolCollision(shape_msgs::Mesh co_mesh);

      /**
       * @brief Display function to print out computed poses.
       * For debugging check.
       * 
       * @return int result: -1=failed
       */
      int display_poses_matrix();
      /**
       * @brief Display function to print out perceived tool matrix
       * 
       * @return int result: -1=failed
       */
      int display_tool_matrix();
      Affine3d obj_TF, tar_TF, tool_TF;

      Affine3d home_pose, home_Tool;

      vector< Affine3d > grabposes, returnposes, home_tool_poses;
      vector< Affine3d > home_path_poses;
      /**
       * @brief Action sequences to return Tool to where it was obtained
       * 
       * @return int result: -1=failed
       */
      int returnTool();

      //tf_switch_dynamic_broadcaster
      ros::ServiceClient client_tf_switch;
      /**
       * @brief this is used to switch tf frames between cameras
       * This switches between frames of olivia eyes, overhead camera for table,
       * and a camera that looks at the tool rack.
       * Significantly reduce effort and time required for experiment 
       * to scan and search for objects/tools/obstacles on table/rack
       * 
       * @param num 
       * @return int 
       */
      int ActionSwitchTF(int num);

      visualization_msgs::MarkerArray marker_List;
      int marker_id;
      /**
       * @brief Function to add markers to RVIZ to provide visual aids
       * 
       * @param pose desired pose
       * @param name name of marker
       */
      void AddMarker(Affine3d pose, string name); //rviz
      /**
       * @brief publish all the markers to rviz
       * 
       */
      void publishMarker(); //rviz
      /**
       * @brief remove all the markers from rviz
       * 
       */
      void clearMarker();

//-----------------------------with obstacle-------------------------------------------//
      bool obstacle_detected;
      geometry_msgs::PoseStamped obstacle_in_base, obstacle_in_base_live;
      geometry_msgs::PoseStamped rack_in_base;

      /**
       * @brief Function to respawn obstacle in Gazebo simulation only
       * 
       */
      void ResetObstacle();
      /**
       * @brief Function to add obstacle and collision into planning scene
       * 
       * @param radius size of obstacle
       * @return int result: -1=failed
       */
      int addObstacleCollision(double radius = 0.04);
      Affine3d obst_TF;
      moveit_msgs::CollisionObject obstacle_collision;

      /**
       * @brief Updated function to detect object and target with overhead camera
       * 
       * @return int result: -1=failed
       */
      int findObjectTargetLocation2();
      /**
       * @brief Updated function to detect tool and target location with camera looking at tool rack
       * 
       * @return int result: -1=failed
       */
      int findToolTargetLocation2();
      /**
       * @brief Experiment 2 for mcts with tool
       * Added an overhead camera for table, and a camera that looks at the tool rack.
       * Significantly reduce effort and time required for experiment 
       * to scan and search for objects/tools/obstacles on table/rack.
       * An improvement from just depending on Olivia's eyes alone.
       * 
       * @return int result: -1=failed
       */
      int mct_tool_experiment2();
      /**
       * @brief Function to execute plan if mcts solution is successfully found.
       * 
       * @return int result: -1=failed
       */
      int perform_experiment2();

      /**
       * @brief Display more information
       * 
       * @return int result: -1=failed
       */
      int display_poses_matrix2();
      /**
       * @brief Updated function from updated perception module data format
       * 
       * @return int result: -1=failed
       */
      int findToolTargetLocation3();

      MCT_Search2 m_mct_search2;
      Tool_Expt_2 m_tool_expt2;
      /**
       * @brief Function to perform mcts demo with updated functions
       * 
       * @return int 
       */
      int mcts_demo_2();
      Vector3d m_result_via_pt;
      bool m_obstruct_flag;

      /**
       * @brief Function to export out all the data to csv file
       * 
       * @return int 
       */
      int write2file();

      /**
       * @brief Function to find object and target location on the left side of olivia
       * 
       * @return int result: -1=failed
       */
      int findObjectTargetLocation2_left();
      /**
       * @brief Function to perform the mcts demo but on the reverse left side
       * 
       * @return int result: -1=failed
       */
      int mct_tool_experiment2_left();
      /**
       * @brief Function to execute plan if mcts solution is successfully found.
       * For left side and left arm.
       * 
       * @return int result: -1=failed
       */
      int perform_experiment2_left();
      /**
       * @brief Action sequence to finish experiment when using perception module
       * Return tool to rack and etc. Left side and left arm
       * 
       * @param tool_in_hand true=yes, false=no
       * @param okay experiment was success or not
       * @param wflag use additional waypoints or not
       */
      void finish_v_l(bool tool_in_hand, bool okay=true, bool wflag=false);

      /**
       * @brief MCTS experiment version3 with addition of obstacles.
       * For left side and left arm.
       * 
       * @return int 
       */
      int mct_tool_experiment3_left();

      //---------------------------------
      int goIntoDiffPos(double HEAD_SIZE, double HANDLE_LENGTH, double HEIGHT_LENGTH);
      int recursivePos(double HEAD_SIZE, double HANDLE_LENGTH, double HEIGHT_LENGTH);

      /**
       * @brief Function to obtain tool pose
       * 
       * @return int result: -1=failed
       */
      int findToolLoca();
      /**
       * @brief Action function to move Olivia neck, waist and trunk at same time to desired values
       * 
       * @param tilt_angle desired tilt angle
       * @param pan_angle desired pan angle
       * @param pan_speed panning speed 
       * @param angle_deg desired waist angle
       * @param angle_deg_2 desired trunk angle
       * @return int result: -1=failed
       */
      int neck_and_waist_trunk(double tilt_angle, double pan_angle, double pan_speed, double angle_deg, double angle_deg_2);

      //----------------------------reachable
      /**
       * @brief Function to check if object is reachable.
       * If using robot arm to perform Pick And Place task, computation changes to detect if graspable.
       * 
       * @param PICK_FLAG is robot doing direct pick and place
       * @return true reachable
       * @return false not reachable
       */
      bool EvReachable(bool PICK_FLAG);
      /**
       * @brief Updated function to check if object is reachable
       * 
       * @return true reachable
       * @return false not reachable
       */
      bool EvReachable2();

      /**
       * @brief compute the pseudo inverse of given matrix 
       * 
       * @tparam _Matrix_Type_ 
       * @param a 
       * @param epsilon 
       * @return _Matrix_Type_ 
       */
      template<typename _Matrix_Type_>
      _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
      {
            Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
            double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
            return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
      }
};


/* ======================================================================== */
// sc::simple_state class accepts up to four parameters
// 1st param: name of simple_state
// 2nd param: defines the state machine/context that this state belongs to
// 3rd param: inner initial state, if any
// 4th param: specifies whether and what kind of history is kept
/* ======================================================================== */
struct ReadyState : sc::simple_state< ReadyState, Active > 
{
public:
  ReadyState(){}
  ~ReadyState(){}

  typedef sc::custom_reaction< EvStart > reactions;
  sc::result react( const EvStart & ) 
  {
     return transit< Startup >();
  }
};

#endif //MANIPULATE_HPP_INCLUDED

