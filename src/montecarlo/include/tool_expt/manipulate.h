#ifndef MANIPULATE_HPP_INCLUDED
#define MANIPULATE_HPP_INCLUDED

#include <tool_expt/machine.h>
#include <actionlib/client/simple_action_client.h>
#include <m3_moveit/MoveitSingleAction.h>
#include <m3_moveit/MoveitFingersAction.h>
#include <m3_moveit/MoveitDualAction.h>
#include <m3_moveit/MoveitWholeBodyAction.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <sensor_msgs/JointState.h>
#include <sound_play/sound_play.h>

#include "neck_pan/NeckPanAction.h"
#include "neck_pan/NeckPanFeedback.h"
#include "neck_pan/NeckPanResult.h"
#include <m3_moveit/NeckTiltAction.h>

#include <m3_moveit/MoveitPickPlaceAction.h>
#include <m3_moveit/MoveitCollideAction.h>

#include <tool_expt/create_tool.h>
#include <tool_expt/search.h>
#include <tool_expt/tool_expt.h>
#include <tool_expt/moveit_ik.h>
#include <tool_expt/kdl_ik.h>

#include <geometric_shapes/shape_operations.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tool_expt/vision_data.h"
#include "monte_carlo/tf_switch.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tool_expt/search2.h>
#include <tool_expt/tool_expt2.h>
#include <tool_expt/definitions.h>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;



//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

struct Manipulate : sc::state< Manipulate, Active > 
{
      public:
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

      Manipulate(my_context ctx);
      ~Manipulate();

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
      ros::NodeHandle n;

      sound_play::SoundClient speech;

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

      ros::Publisher pub_calib;
      ros::Publisher pub_req_object_target;
      ros::Publisher pub_req_tool_handle;
      ros::Publisher pub_table;

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
      void object_printout_callBack(const geometry_msgs::PoseArray::Ptr& msg);
      void object_target_positions_callBack(const geometry_msgs::PoseArray::Ptr& msg);
      void tool_handle_position_callBack(const geometry_msgs::Pose::Ptr& msg);
      void bendtool_position_callBack(const geometry_msgs::Pose::Ptr& msg);
      void fake_tool_transform_callBack(const geometry_msgs::PoseArray::Ptr& msg);
      void fake_target_callBack(const geometry_msgs::PoseArray::Ptr& msg);
      void calib_tool_poses_callBack(const geometry_msgs::PoseArray::Ptr& msg);
      void tool_decision_callBack(const std_msgs::Int32::Ptr& msg);
      void calib_next_callBack(const std_msgs::Int32::Ptr& msg);
      void joint_states_callBack(const sensor_msgs::JointState::Ptr& msg);
      void sticklength_callBack(const std_msgs::Float64::Ptr& msg);
      void neck_pan_state_callBack(const sensor_msgs::JointState::Ptr& msg);

            //functions
            //----------------------------------------------------------------------
      m3_moveit::MoveitWholeBodyGoal fillJointNamesFull();
      m3_moveit::MoveitWholeBodyGoal fillNamesFull();
      m3_moveit::MoveitWholeBodyGoal SetJointTargetsToCurrent();
      Eigen::MatrixXd ComputeTool2HandTransform(const geometry_msgs::PoseArray calib_hand_poses, const geometry_msgs::PoseArray calib_tool_poses);
      std::vector<std::string> fillJointNamesFingers(std::string hand);
      std::vector<std::string> fillJointNamesFingersEncoders(std::string hand);
      std::vector<geometry_msgs::Point> load_boundary(const int functionality);
      geometry_msgs::PoseStamped transformPose(geometry_msgs::Pose pose_msg, std::string from_frame, std::string to_frame);
      bool isReachable(geometry_msgs::Pose pose);
      bool ToolInHandCalibration(std::string hand);
      void CheckObjectLocation();
      void CheckToolLocation();
      void CheckBendToolLocation();
      void CheckFakeToolTransform();
      void CheckBendToolTransform();
      void MoveWaist(double angle_deg);
      void MoveWaistFromCurrent(double angle_deg);
      void MoveWaistTrunkFromCurrent(double angle_deg, double angle_deg_2);

      void ResetPuck();
      void ResetTool();

      void DrawCircle();
      void TestHandOrient();
      void Home();
      int Home(double waist_val);
      int Home2(double waist_val=0);
      
      void Speech(std::string string);
      double signum(const double num);
      void translateHandFromCurrent(const std::string link, const std::vector<double> moveX, const std::vector<double> moveY, const std::vector<double> moveZ, const int Npts, geometry_msgs::PoseArray& pose_array_target);
      void getCurrentTransforms(tf::Quaternion& tfQuat_left, tf::Quaternion& tfQuat_right,  tf::Vector3& tfVec_left, tf::Vector3& tfVec_right);
      void getCurrentTransform(const std::string link, tf::Quaternion& tfQuat, tf::Vector3& tfVec);
      bool getCurrentTransform(const std::string link, geometry_msgs::PoseStamped& output);

      int PickupToolRightArm();
      int PickupBendTool();
      int BendTool(double stick_length, double head_length, double head_angle_deg);
      int OpenFingers(std::string side, int mode);
      int CloseFingers(std::string side, int grasp_type, int mode);
      int ActionPullBackLeftArm();
      int ActionPullBackRightArm();
      int ActionPullBackRightArmTool();
      int ActionPullBackBendTool();
      int ActionPushSidewaysLeftArm();
      int ActionPushSidewaysRightArm();
      int ActionPushForwardLeftArm();
      int ActionPushForwardRightArm(bool using_tool);
      int ActionPushForwardRightArmSpecial();
      int ActionReturnToolRightArm();
      int ActionReturnToolRightArmSpecial();
      int ActionReturnBendTool();

      int ActionPushForwardN();
      int ExNorth();
      int ExNorthTool(bool object_within_reach, bool target_within_reach);
      int ActionPushForwardN_Tool();

      int ActionPushbackS();
      int ExSouth();
      int ExSouthTool(bool object_within_reach, bool target_within_reach);
      int ActionPushbackS_Tool();

      int ActionOmniRight();
      int ExOmni(bool repoflag);
      int ExOmniTool(bool object_within_reach, bool target_within_reach);
      int ActionOmniRight_Tool();

      int pinchFingers(std::string side, double diameter_in_cm, int mode);
      int powerFingers(std::string side, double diameter_in_cm, int mode);
      int MoveFingers(std::string side, double fraction, int mode);

      int NeckTilt(double neck_tilt_angle);
      int NeckPan(double pan_angle, double pan_speed);
      int TurnHeadToObjTarCenter();
      int TurnHeadTiltPan(double neck_tilt_angle, double neck_pan_angle);

      void ActionClientNeckPanDoneCallback(const actionlib::SimpleClientGoalState &state, const neck_pan::NeckPanResultConstPtr &result);

      double compute_min_aug(const std::vector<geometry_msgs::Point> boundary, const geometry_msgs::Point object, const geometry_msgs::Point target, const bool object_within_reach, const bool target_within_reach);


      bool joint_updated;
      double angle, radian; 
      double neck_pan_angle, neck_tilt_angle;

      double current_pan_angle;
      double current_tilt_angle;

      bool kbhit();

      Create_Tool m_create_tool;
      Tool_Expt m_tool_expt;

      MCT_Search m_mct_search;
      MOVEIT_IK m_moveit_ik;
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

      int findObjectTargetLocation();
      int findToolTargetLocation();
      int mct_tool_experiment();
      int perform_experiment();



      //manipulate_action
      //----------------------------------------------------------------------
      void doneCb(const actionlib::SimpleClientGoalState& state, const m3_moveit::MoveitSingleResultConstPtr& result);
      void activeCb();
      void feedbackCb(const m3_moveit::MoveitSingleFeedbackConstPtr& feedback);
      int checkUserInput();

      int ActionAttachObj(string arm_name, string obj_name, moveit_msgs::CollisionObject cobj);
      int ActionDetachObj(string arm_name, string obj_name);
      int ActionRmObj(string obj_id);
      int ActionAddObj(string obj_id, moveit_msgs::CollisionObject cobj);

      int ActionGoToPos();
      int ActionGoToPos(Affine3d this_pose, bool plan_only=false, string action_name="task_single_right", bool replan=false, string link_name="ee");
      int ActionGoToPos(std::vector< Affine3d > these_poses, bool plan_only=false, string action_name="task_single_right", bool replan=false, string link_name="ee");
      int ActionGoToPos(Vector3d action_pos, Quaterniond action_quat, bool plan_only=false, string action_name="cart_single_right", bool replan=false, string link_name="ee");

      int ActionGoToPos(std::vector< Affine3d > these_poses, double* fraction, bool plan_only=false, string action_name="cart_single_right", bool replan=false, string link_name="ee");

      int addCollision(vector< string > ids, vector< string > meshfiles, vector< geometry_msgs::Pose > poses);
      int addToolCollision();
      int addPuckCollision(double radius = 0.04);
      int addTableCollision();

      int addRackCollision();

      Affine3d EstimatePose(double fraction, Affine3d start, Affine3d end);
      bool ReAdjustment(int idx, vector< Affine3d > gohere, double fraction, Affine3d start_TF, Affine3d end_TF);

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
      //----------------------------------------------------------------------
      geometry_msgs::PoseStamped vtool_in_base;

      int vision_cb_test();
      void finish_v(bool tool_in_hand, bool okay=true, bool wflag=false);

      ros::Subscriber sub_vision;
      void vision_data_callBack(const findtoolontalbe::vision_msgs::Ptr &msg);
      bool detected;

      //vision data
      int tool_num;
      double current_tool_width;
      vision_data v_data;
      std::vector< vision_each > v_list;
      std::vector< shape_msgs::Mesh > toolMeshes;

      moveit_msgs::CollisionObject vtool_collision;
      int addVisualToolCollision(shape_msgs::Mesh co_mesh);

      int display_poses_matrix();
      int display_tool_matrix();
      Affine3d obj_TF, tar_TF, tool_TF;

      Affine3d home_pose, home_Tool;

      vector< Affine3d > grabposes, returnposes, home_tool_poses;
      vector< Affine3d > home_path_poses;
      int returnTool();

      //tf_switch_dynamic_broadcaster
      ros::ServiceClient client_tf_switch;
      int ActionSwitchTF(int num);

      visualization_msgs::MarkerArray marker_List;
      int marker_id;
      void AddMarker(Affine3d pose, string name); //rviz
      void publishMarker(); //rviz
      void clearMarker();




      //-----------------------------with obstacle
      bool obstacle_detected;
      geometry_msgs::PoseStamped obstacle_in_base, obstacle_in_base_live;
      geometry_msgs::PoseStamped rack_in_base;
      void ResetObstacle();
      int addObstacleCollision(double radius = 0.04);
      Affine3d obst_TF;
      moveit_msgs::CollisionObject obstacle_collision;

      int findObjectTargetLocation2();
      int findToolTargetLocation2();
      int mct_tool_experiment2();
      int perform_experiment2();

      int display_poses_matrix2();
      int findToolTargetLocation3();

      MCT_Search2 m_mct_search2;
      Tool_Expt_2 m_tool_expt2;
      int mcts_demo_2();
      Vector3d m_result_via_pt;
      bool m_obstruct_flag;

      int write2file();


      int findObjectTargetLocation2_left();
      int mct_tool_experiment2_left();
      int perform_experiment2_left();
      void finish_v_l(bool tool_in_hand, bool okay=true, bool wflag=false);

      int mct_tool_experiment3_left();

      //---------------------------------
      int goIntoDiffPos(double HEAD_SIZE, double HANDLE_LENGTH, double HEIGHT_LENGTH);
      int recursivePos(double HEAD_SIZE, double HANDLE_LENGTH, double HEIGHT_LENGTH);

      int findToolLoca();
      int neck_and_waist_trunk(double tilt_angle, double pan_angle, double pan_speed, double angle_deg, double angle_deg_2);

      //----------------------------reachable
      bool EvReachable(bool PICK_FLAG);
      bool EvReachable2();

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

#endif

