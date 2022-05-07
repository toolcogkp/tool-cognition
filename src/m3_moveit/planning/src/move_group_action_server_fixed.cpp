/**
 * @file move_group_action_server_fixed.cpp
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief
 * Moveit action server. All the actions of olivia is planned and execute here.
 * version 2.0, revised
 * @version 0.1
 * @date 2019-02-10
 *
 * @copyright Copyright (c) 2019
 *
 */

//=========================================================================================================//

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "geometry_msgs/Pose.h"

#include <actionlib/server/simple_action_server.h>
#include <m3_moveit/MoveitSingleAction.h>
#include <m3_moveit/MoveitFingersAction.h>
#include <m3_moveit/MoveitDualAction.h>
#include <m3_moveit/MoveitWholeBodyAction.h>

#include <m3_moveit/MoveitPickPlaceAction.h>
#include <m3_moveit/MoveitCollideAction.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <kaist_msgs/PlannedMoveRequest.h>
#include <kaist_msgs/FingerMoveRequest.h>
#include <kaist_msgs/JointEncoderStatus.h>

#include <moveit/robot_state/attached_body.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <boost/tokenizer.hpp>

// industrial trajectory filter
#include "industrial_trajectory_filters/uniform_sample_filter.h"

// messages
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <moveit/collision_detection_fcl/collision_common.h>
#include <moveit/robot_state/conversions.h>

#include <m3_moveit/NeckTiltAction.h>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

//=========================================================================================================//

// namespace
using namespace std;
using std::cin;
using std::cout;
using std::endl;

//=========================================================================================================//
// constant parameters
const double HAND_ORIENTATION_DIST_TOL = 0.001;
const double JIGGLE_CONST = 0.005;
const int MAX_ATTEMPTS = 6;
bool constrain_hand_orientation = false;
bool is_hand_closed, is_hand_opened, is_hand_grasping_tool, close_hand, open_hand, grasp_tool, release_tool;
double finger_proximal_closed_limit;
double finger_proximal_opened_limit;

double open_fingers_currentmode_value = -50.0;
double close_fingers_currentmode_value = 50.0;
double stop_fingers_currentmode_value = 0.0;
double duration_b4_stopping_fingers;
double duration_b4_stopping_fingers_no_tool = 6.0;
double duration_b4_stopping_fingers_with_tool;
int tool_type;

double tol_finger = 0.5;
double finger_target_right1, finger_target_right2, finger_live_right1, finger_live_right2;
double finger_target_left1, finger_target_left2, finger_live_left1, finger_live_left2;
bool rf1_target_reached = false;
bool rf2_target_reached = false;

bool finger_1_target_received = false;
bool finger_2_target_received = false;

bool finger1_reached = false;
bool finger2_reached = false;

kaist_msgs::FingerMoveRequest finger_cmd;
ros::Publisher fingers_pub;
ros::Publisher whole_body_pub;
ros::Publisher test_pub;

//=========================================================================================================//

/**
 * @brief Moveit action server
 *
 */
class M3MoveGroup
{
public:
    /**
     * @brief Construct a new M3MoveGroup object
     *
     * @param name
     */
    M3MoveGroup(std::string name) : group(name),
                                    rgrp_name("torso_and_right_arm"),
                                    group_right("torso_and_right_arm"),
                                    lgrp_name("torso_and_left_arm"),
                                    group_left("torso_and_left_arm"),

                                    cart_single_as_(nh, "cart_single", boost::bind(&M3MoveGroup::CartSingleActionCB, this, _1), false),
                                    fingers_as_(nh, "fingers", boost::bind(&M3MoveGroup::FingersActionCB, this, _1), false),
                                    fingers_encoders_as_(nh, "fingers_encoders", boost::bind(&M3MoveGroup::FingersEncodersActionCB, this, _1), false),
                                    task_single_as_(nh, "task_single", boost::bind(&M3MoveGroup::TaskSingleActionCB, this, _1), false),
                                    task_dual_as_(nh, "task_dual", boost::bind(&M3MoveGroup::TaskDualActionCB, this, _1), false),
                                    mixed_whole_body_as_(nh, "mixed_whole_body", boost::bind(&M3MoveGroup::MixedWholeBodyActionCB, this, _1), false),
                                    joint_whole_body_as_(nh, "joint_whole_body", boost::bind(&M3MoveGroup::JointWholeBodyActionCB, this, _1), false),
                                    neck_tilt_as_(nh, "neck_single", boost::bind(&M3MoveGroup::NeckSingleActionCB, this, _1), false),

                                    attach_action_as_(nh, "attach_object", boost::bind(&M3MoveGroup::AttachRightActionCB, this, _1), false),
                                    detach_action_as_(nh, "detach_object", boost::bind(&M3MoveGroup::DetachRightActionCB, this, _1), false),

                                    add_collision_as_(nh, "add_collision", boost::bind(&M3MoveGroup::AddCollisionActionCB, this, _1), false),
                                    remove_collision_as_(nh, "remove_collision", boost::bind(&M3MoveGroup::RemoveCollisionActionCB, this, _1), false),

                                    task_single_right_as_(nh, "task_single_right", boost::bind(&M3MoveGroup::TaskSingleRightActionCB, this, _1), false),
                                    cart_single_right_as_(nh, "cart_single_right", boost::bind(&M3MoveGroup::CartSingleRightActionCB, this, _1), false),

                                    counter(0)
    {
        attachedlist.clear();

        is_hand_closed = true;
        is_hand_opened = false;
        is_hand_grasping_tool = false;
        close_hand = false;
        open_hand = false;
        grasp_tool = false;
        release_tool = false;

        group_right.setPlanningTime(2); // double seconds
        group_left.setPlanningTime(2);  // double seconds

        geometry_msgs::PoseStamped actual;
        actual = group.getCurrentPose();
        get_planning_scene = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
        planning_scene_diff_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

        sub_collision_object = nh.subscribe("collision_object", 1, &M3MoveGroup::CollisionObjectCallback, this);
        sub_finger_encoder = nh.subscribe("JointEncoder", 1, &M3MoveGroup::FingerEncoderCallback, this);

        fingers_pub = nh.advertise<kaist_msgs::FingerMoveRequest>("/FingerMoveRequst", 5);
        whole_body_pub = nh.advertise<kaist_msgs::PlannedMoveRequest>("/PlannedMoveRequest", 5);

        sub_finger_timeout = nh.subscribe("/fingerFail", 1, &M3MoveGroup::fingerTimeoutCallback, this);

        input_sub = nh.subscribe("/user/input", 1, &M3MoveGroup::InputCallback, this);

        // GRASPING PARAMETERS
        rightHandLinkNames.push_back("handmount_RIGHT");
        rightHandLinkNames.push_back("palm");
        rightHandLinkNames.push_back("index_proximal_RIGHT");
        rightHandLinkNames.push_back("index_medial_RIGHT");
        rightHandLinkNames.push_back("index_distal_RIGHT");

        rightHandLinkNames.push_back("middle_proximal_RIGHT");
        rightHandLinkNames.push_back("middle_medial_RIGHT");
        rightHandLinkNames.push_back("middle_distal_RIGHT");

        rightHandLinkNames.push_back("ring_proximal_RIGHT");
        rightHandLinkNames.push_back("ring_medial_RIGHT");
        rightHandLinkNames.push_back("ring_distal_RIGHT");

        rightHandLinkNames.push_back("last_proximal_RIGHT");
        rightHandLinkNames.push_back("last_medial_RIGHT");
        rightHandLinkNames.push_back("last_distal_RIGHT");

        leftHandLinkNames.push_back("handmount_LEFT");
        leftHandLinkNames.push_back("l_palm");
        leftHandLinkNames.push_back("index_proximal_LEFT");
        leftHandLinkNames.push_back("index_medial_LEFT");
        leftHandLinkNames.push_back("index_distal_LEFT");

        leftHandLinkNames.push_back("middle_proximal_LEFT");
        leftHandLinkNames.push_back("middle_medial_LEFT");
        leftHandLinkNames.push_back("middle_distal_LEFT");

        leftHandLinkNames.push_back("ring_proximal_LEFT");
        leftHandLinkNames.push_back("ring_medial_LEFT");
        leftHandLinkNames.push_back("ring_distal_LEFT");

        leftHandLinkNames.push_back("last_proximal_LEFT");
        leftHandLinkNames.push_back("last_medial_LEFT");
        leftHandLinkNames.push_back("last_distal_LEFT");

        // start action server
        cart_single_as_.start();
        fingers_as_.start();
        fingers_encoders_as_.start();
        task_single_as_.start();
        task_dual_as_.start();
        mixed_whole_body_as_.start();
        joint_whole_body_as_.start();
        neck_tilt_as_.start();

        attach_action_as_.start();
        detach_action_as_.start();
        add_collision_as_.start();
        remove_collision_as_.start();

        task_single_right_as_.start();
        cart_single_right_as_.start();
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------
    bool replan = false;
    bool responseFlag = false;
    bool quitFlag = false;

    /**
     * @brief user input function
     *
     * @param input
     */
    void InputCallback(const std_msgs::Int32ConstPtr &input)
    {
        replan = true;
        responseFlag = true;
        quitFlag = false;

        // char in = (char) input->data;
        cout << "-----------------------------" << endl;
        ROS_WARN_STREAM("user input recieved: " << input->data);

        switch (input->data)
        {
        case 1:
        {
            ROS_INFO("Replan");
            replan = true;
            break;
        }
        case 2:
        {
            ROS_INFO("Quit");
            quitFlag = true;
            break;
        }
        case 3:
        {
            ROS_INFO("Execute");
            replan = false;
            break;
        }
        default:
        {
            ROS_WARN("unknown input -> ignored");
            responseFlag = false;
            break;
        }
        }
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief action server timeout callback for finger
     * 
     * @param timeout 
     */
    void fingerTimeoutCallback(const std_msgs::Bool timeout)
    {

        ROS_INFO("finger 10secs action server timeout: stopping finger cmd");
        std::vector<std::string> finger;
        finger.push_back("rf1");
        finger.push_back("rf2");
        stop_fingers(finger);

        rf1_target_reached = false;
        finger_1_target_received = false;
        rf2_target_reached = false;
        finger_2_target_received = false;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Using ros timer to check for finger timeout
     * 
     * @param e timer event
     */
    void fingerTimerTimeoutCallback(const ros::TimerEvent &e)
    {

        ROS_INFO("finger 9secs timer timeout: stopping finger cmd");
        std::vector<std::string> finger;
        finger.push_back("rf1");
        finger.push_back("rf2");
        stop_fingers(finger);

        rf1_target_reached = false;
        finger_1_target_received = false;
        rf2_target_reached = false;
        finger_2_target_received = false;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Action server to add collision object into moveit planning scene
     * 
     * @param obj collision object
     */
    void CollisionObjectCallback(const moveit_msgs::CollisionObjectConstPtr &obj)
    {

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(*obj);
        planning_scene_interface.addCollisionObjects(collision_objects); // TODO: remove old collision objects
        ROS_INFO("Added collision object into the planning scene");
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief callback test for fingers
     * 
     * @param e 
     */
    void callback_testpub(const ros::TimerEvent &e)
    {
        kaist_msgs::JointEncoderStatus test_msg;
        test_msg.names.push_back("rf1");
        test_msg.positions.push_back(finger_target_right1);
        test_msg.names.push_back("rf2");
        test_msg.positions.push_back(finger_target_right2);
        test_msg.names.push_back("lf1");
        test_msg.positions.push_back(finger_target_left1);
        test_msg.names.push_back("lf2");
        test_msg.positions.push_back(finger_target_left2);
        test_pub.publish(test_msg);
    }

    /**
     * @brief callback to stop fingers. Time based control of olivia fingers
     * 
     * @param e 
     */
    void callback_stop_fingers(const ros::TimerEvent &e)
    {
        std::vector<std::string> fingers;
        fingers.push_back("rf1");
        fingers.push_back("rf2");
        fingers.push_back("lf1");
        fingers.push_back("lf2");
        stop_fingers(fingers);
    }

    /**
     * @brief time-based control to stop olivia finger1
     * 
     * @param e 
     */
    void callback_stop_fingers1(const ros::TimerEvent &e)
    {
        std::vector<std::string> fingers;
        fingers.push_back("rf1");
        fingers.push_back("lf1");
        stop_fingers(fingers);
    }

    /**
     * @brief time based control to stop olivia finger2
     * 
     * @param e 
     */
    void callback_stop_fingers2(const ros::TimerEvent &e)
    {
        std::vector<std::string> fingers;
        fingers.push_back("rf2");
        fingers.push_back("lf2");
        stop_fingers(fingers);
    }

    /**
     * @brief time based control to stop all olivia fingers
     * 
     * @param names 
     */
    void stop_fingers(std::vector<std::string> names)
    {
        finger_cmd.names.clear();
        finger_cmd.values.clear();

        for (int i = 0; i < names.size(); i++)
        {
            finger_cmd.names.push_back(names.at(i));
            finger_cmd.values.push_back(stop_fingers_currentmode_value);
        }

        cout << "finger cmd 1=:" << finger_cmd.values[0] << endl;
        cout << "finger cmd 2=:" << finger_cmd.values[1] << endl;

        fingers_pub.publish(finger_cmd);
        cout << " finger stop command published.\n"
             << endl;
    }

    /**
     * @brief encoder position based control to stop olivia fingers
     * 
     * @param joint 
     */
    void FingerEncoderCallback(const kaist_msgs::JointEncoderStatusConstPtr &joint)
    {
        // if(finger_target_received){
        std::vector<std::string> finger;
        std_msgs::String msg;

        // ROS_INFO("encoder callback");

        if (rf1_target_reached)
        {

            ROS_INFO(">>> rf1 target reached <<<\n");
            std_msgs::Bool msg;
            msg.data = true;
            finger1_reached = true;

            rf1_target_reached = false;
            finger_1_target_received = false;
        }

        if (rf2_target_reached)
        {

            ROS_INFO(">>> rf2 target reached <<<\n");
            std_msgs::Bool msg;
            msg.data = true;
            finger2_reached = true;

            rf2_target_reached = false;
            finger_2_target_received = false;
        }

        for (int i = 0; i < joint->names.size(); i++)
        {

            if (joint->names.at(i) == finger2_name)
            {
                finger_live_right2 = joint->positions.at(i);

                if (finger_2_target_received)
                {

                    cout << "finger_live_right2 = " << finger_live_right2 << endl;
                    cout << "finger target right 2 =" << finger_target_right2 << endl;
                    cout << "finger cmd 2=:" << finger_cmd.values[1] << "\n"
                         << endl;

                    if (fabs(finger_live_right2 - finger_target_right2) <= tol_finger)
                    {
                        finger.clear();
                        finger.push_back(finger2_name);
                        cout << ">>> stopping f2..." << endl;
                        stop_fingers(finger);
                        rf2_target_reached = true;
                    }
                }
            }

            if (joint->names.at(i) == finger1_name)
            {
                finger_live_right1 = joint->positions.at(i);

                if (finger_1_target_received)
                {

                    cout << "finger_live_right1 = " << finger_live_right1 << endl;
                    cout << "finger target right 1 =" << finger_target_right1 << endl;
                    cout << "finger cmd 1=:" << finger_cmd.values[0] << "\n"
                         << endl;

                    if (fabs(finger_live_right1 - finger_target_right1) <= tol_finger)
                    {
                        finger.clear();
                        finger.push_back(finger1_name);
                        cout << ">>> stopping f1..." << endl;
                        stop_fingers(finger);
                        rf1_target_reached = true;
                    }
                }
            }
        }

        // ROS_INFO("--- encoder cb end ---\n");
        // }
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief action server to control olivia fingers using encoders
     * 
     * @param goal target goal
     */
    void FingersEncodersActionCB(const m3_moveit::MoveitFingersGoalConstPtr &goal)
    {

        m3_moveit::MoveitFingersResult result;
        cout << "FingersEncodersActionCB" << endl;
        finger_1_target_received = true;
        finger_2_target_received = true;

        finger1_name = "rf1"; // default
        finger2_name = "rf2"; // default
        if (goal->side == "left")
        {
            finger1_name = "lf1";
            finger2_name = "lf2";
        }

        for (int i = 0; i < 10; i++)
        {
            // cout << "waiting for finger targets to be reached" << endl;
            ros::spinOnce();
            sleep(0.002);
        }
        sleep(0.01);

        rf1_target_reached = false;
        rf2_target_reached = false;

        finger_cmd.requesterid = 0;
        finger_cmd.names.clear();
        finger_cmd.values.clear();

        finger1_reached = false;
        finger2_reached = false;

        finger_target_right1 = goal->trajectory.points[0].positions[0];
        finger_target_right2 = goal->trajectory.points[0].positions[1];

        cout << "finger target 1 =" << finger_target_right1 << endl;
        cout << "finger target 2 =" << finger_target_right2 << endl;

        finger_cmd.names.push_back(finger1_name);
        if (-(finger_live_right1 - finger_target_right1) >= tol_finger)
        { // negative
            finger_cmd.values.push_back(open_fingers_currentmode_value);
        }
        else if ((finger_live_right1 - finger_target_right1) >= tol_finger)
        {
            finger_cmd.values.push_back(close_fingers_currentmode_value);
        }
        else
        {
            finger_1_target_received = false;
            finger_cmd.values.push_back(stop_fingers_currentmode_value);
        }

        finger_cmd.names.push_back(finger2_name);
        if (-(finger_live_right2 - finger_target_right2) >= tol_finger)
        { // negative
            finger_cmd.values.push_back(open_fingers_currentmode_value);
        }
        else if ((finger_live_right2 - finger_target_right2) >= tol_finger)
        {
            finger_cmd.values.push_back(close_fingers_currentmode_value);
        }
        else
        {
            finger_2_target_received = false;
            finger_cmd.values.push_back(stop_fingers_currentmode_value);
        }

        if (!finger_1_target_received && !finger_2_target_received)
        {
            cout << ">>> both fingers within tolerance range of " << tol_finger << " to target" << endl;
            cout << "-------------------------" << endl;
            cout << " both fingers am here!!" << endl;

            result.error_code = result.SUCCESSFUL;
            fingers_encoders_as_.setSucceeded(result);
            return;
        }

        // send to real robot
        //----------------------------------
        cout << "finger cmd = " << finger_cmd.values[0] << endl;
        cout << "finger cmd 2= " << finger_cmd.values[1] << endl;
        fingers_pub.publish(finger_cmd);

        ros::Timer timer1 = nh.createTimer(ros::Duration(9.0), &M3MoveGroup::fingerTimerTimeoutCallback, this, true);

        cout << "Executing fingers motion ...\n";

        result.error_code = result.FAIL;

        ros::Rate r(15);
        while (!(finger1_reached && finger2_reached) && ros::ok())
        {
            // cout << "waiting for finger targets to be reached" << endl;
            ros::spinOnce();
            r.sleep();
        }

        cout << "-------------------------" << endl;
        cout << "am here!! -> execution done" << endl;

        result.error_code = result.SUCCESSFUL;
        fingers_encoders_as_.setSucceeded(result);
        return;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Action server to control olivia fingers
     * 
     * @param goal 
     */
    void FingersActionCB(const m3_moveit::MoveitFingersGoalConstPtr &goal)
    {

        nh.getParam("/tool_type", tool_type);
        switch (tool_type)
        {
        case 1:
        {
            duration_b4_stopping_fingers_with_tool = 5.0;
            break;
        }
        case 2:
        {
            duration_b4_stopping_fingers_with_tool = 3.0;
            break;
        }
        case 3:
        {
            duration_b4_stopping_fingers_with_tool = 5.0;
            break;
        }
        case 4:
        {
            duration_b4_stopping_fingers_with_tool = 4.0;
            break;
        }
        case 5:
        {
            duration_b4_stopping_fingers_with_tool = 2.0;
            break;
        }
        default:
        {
            duration_b4_stopping_fingers_with_tool = 5.0;
            break;
        }
        }

        int feature_learning_sim = 0;
        nh.getParam("feature_learning_sim", feature_learning_sim);
        if (feature_learning_sim)
        {
            finger_proximal_closed_limit = 0.0;
            finger_proximal_opened_limit = 1.3;
        }
        else
        {
            finger_proximal_closed_limit = 1.0;
            finger_proximal_opened_limit = 0.1;
        }

        std::string group_name;

        group_name = "right_fingers"; // default
        if (goal->side == "left")
            group_name = "left_fingers";

        moveit::planning_interface::MoveGroup group_fingers(group_name);
        m3_moveit::MoveitFingersResult result;
        result.error_code = result.SUCCESSFUL;

        group_fingers.setStartStateToCurrentState();
        std::vector<double> group_variable_values;

        for (int i = 0; i < goal->trajectory.points.size(); i++)
        {

            if (is_hand_closed)
            { // state
                if (goal->trajectory.points[i].positions[0] > finger_proximal_closed_limit)
                {
                    open_hand = true; // action
                    close_hand = false;
                    grasp_tool = false;
                    release_tool = false;
                }
                else
                {
                    close_hand = open_hand = false; // no action
                    grasp_tool = release_tool = false;
                    std::cout << "hand already close! Abort action to close hand." << std::endl;
                }
            }
            else if (is_hand_opened)
            {
                if (goal->trajectory.points[i].positions[0] < finger_proximal_opened_limit)
                {
                    if (goal->grasp_tool)
                    {
                        grasp_tool = true;
                        open_hand = false;
                        close_hand = false;
                        release_tool = false;
                    }
                    else
                    {
                        close_hand = true; // action
                        open_hand = false;
                        grasp_tool = false;
                        release_tool = false;
                    }
                }
                else
                {
                    open_hand = close_hand = false; // no action
                    grasp_tool = release_tool = false;
                    std::cout << "hand already open! Abort action to open hand." << std::endl;
                }
            }
            else if (is_hand_grasping_tool)
            {
                if (goal->trajectory.points[i].positions[0] > finger_proximal_closed_limit)
                {
                    open_hand = false;
                    close_hand = false;
                    grasp_tool = false;
                    release_tool = true; // action
                }
                else
                {
                    close_hand = open_hand = false; // no action
                    grasp_tool = release_tool = false;
                    std::cout << "hand already close! Abort action to close hand." << std::endl;
                }
            }
            std::cout << "is_hand_closed = " << is_hand_closed << std::endl;
            std::cout << "is_hand_opened = " << is_hand_opened << std::endl;
            std::cout << "is_hand_grasping_tool = " << is_hand_grasping_tool << std::endl;
            std::cout << "open_hand = " << open_hand << std::endl;
            std::cout << "close_hand = " << close_hand << std::endl;
            std::cout << "grasp_tool = " << grasp_tool << std::endl;
            std::cout << "release_tool = " << release_tool << std::endl;

            if (open_hand || close_hand || release_tool || grasp_tool)
            {
                group_fingers.getCurrentState()->copyJointGroupPositions(group_fingers.getCurrentState()->getRobotModel()->getJointModelGroup(group_fingers.getName()), group_variable_values);

                for (int j = 0; j < group_variable_values.size(); j++)
                    group_variable_values[j] = goal->trajectory.points[i].positions[j];

                group_fingers.setJointValueTarget(group_variable_values);

                std::cout << "Executing fingers motion ...\n";

                // send to real robot
                //----------------------------------
                finger_cmd.requesterid = 0;
                finger_cmd.names.clear();
                finger_cmd.values.clear();
                duration_b4_stopping_fingers = duration_b4_stopping_fingers_no_tool;
                if (open_hand || release_tool)
                {
                    if (group_name == "left_fingers")
                    {
                        finger_cmd.names.push_back("lf1");
                        finger_cmd.names.push_back("lf2");
                    }
                    else
                    {
                        finger_cmd.names.push_back("rf1");
                        finger_cmd.names.push_back("rf2");
                    }
                    finger_cmd.values.push_back(open_fingers_currentmode_value);
                    finger_cmd.values.push_back(open_fingers_currentmode_value);
                    if (release_tool)
                        duration_b4_stopping_fingers = duration_b4_stopping_fingers_with_tool;
                }
                else if (close_hand || grasp_tool)
                {
                    if (group_name == "left_fingers")
                    {
                        finger_cmd.names.push_back("lf1");
                        finger_cmd.names.push_back("lf2");
                    }
                    else
                    {
                        finger_cmd.names.push_back("rf1");
                        finger_cmd.names.push_back("rf2");
                    }
                    finger_cmd.values.push_back(close_fingers_currentmode_value);
                    finger_cmd.values.push_back(close_fingers_currentmode_value);
                    if (grasp_tool)
                        duration_b4_stopping_fingers = duration_b4_stopping_fingers_with_tool;
                }

                cout << "duration_b4_stopping_fingers_with_tool = " << duration_b4_stopping_fingers_with_tool << endl;
                if (goal->mode == 0)
                    duration_b4_stopping_fingers = duration_b4_stopping_fingers_no_tool;
                else if (goal->mode == 1)
                    duration_b4_stopping_fingers = duration_b4_stopping_fingers_with_tool;

                fingers_pub.publish(finger_cmd);
                ros::Timer timer1 = nh.createTimer(ros::Duration(duration_b4_stopping_fingers), &M3MoveGroup::callback_stop_fingers1, this, true);
                ros::Timer timer2 = nh.createTimer(ros::Duration(duration_b4_stopping_fingers + 1.0), &M3MoveGroup::callback_stop_fingers2, this, true);

                // send to gazebo
                //----------------------------------
                moveit::planning_interface::MoveGroup::Plan plan;
                if (!group_fingers.plan(plan))
                { // real robot fingers not set up for encoder feedback
                    result.error_code = result.FAIL;
                }
                else
                {
                    if (is_hand_closed && open_hand)
                    {
                        is_hand_closed = false;
                        is_hand_opened = true;
                        is_hand_grasping_tool = false;
                    }
                    else if (is_hand_opened && close_hand)
                    {
                        is_hand_closed = true;
                        is_hand_opened = false;
                        is_hand_grasping_tool = false;
                    }
                    else if (is_hand_opened && grasp_tool)
                    {
                        is_hand_closed = false;
                        is_hand_opened = false;
                        is_hand_grasping_tool = true;
                    }
                    else if (is_hand_grasping_tool && release_tool)
                    {
                        is_hand_closed = false;
                        is_hand_opened = true;
                        is_hand_grasping_tool = false;
                    }
                    open_hand = close_hand = false;
                    grasp_tool = release_tool = false;

                    bool sim_robot;
                    nh.getParam("/sim_robot", sim_robot);
                    std::cout << "sim_robot = " << sim_robot << std::endl;
                    if (sim_robot)
                    {
                        if (!group_fingers.execute(plan)) // causes joint trajectory action controller to segfault. To fix!
                        {
                            result.error_code = result.FAIL;
                        }
                    }
                    else
                        sleep(duration_b4_stopping_fingers + 1.0); // wait for real robot finger motion to complete
                }

                group_variable_values.clear();
            }
        }

        std::cout << "is_hand_closed = " << is_hand_closed << std::endl;
        std::cout << "is_hand_opened = " << is_hand_opened << std::endl;
        std::cout << "is_hand_grasping_tool = " << is_hand_grasping_tool << std::endl;
        std::cout << "open_hand = " << open_hand << std::endl;
        std::cout << "close_hand = " << close_hand << std::endl;
        std::cout << "grasp_tool = " << grasp_tool << std::endl;
        std::cout << "release_tool = " << release_tool << std::endl;

        fingers_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------
    // use as reference only
    /**
     * @brief Moveit action server: planning using cartesian space for single arm
     * 
     * @param goal target
     */
    void CartSingleActionCB(const m3_moveit::MoveitSingleGoalConstPtr &goal)
    {

        std::cout << "\n\nCART SINGLE ACTION!!\n\n"
                  << std::endl;

        // to prevent bug with attached items, refrain from creating new movegroup within functions
        bool leftFlag = false;

        std::string group_name;
        group_name = "right_arm"; // default
        if (goal->link_name == "l_ee")
        {
            leftFlag = true;
            group_name = "left_arm";
        }

        if (leftFlag)
            group_left.clearPoseTargets();
        else
            group_right.clearPoseTargets();

        m3_moveit::MoveitSingleResult result;
        result.error_code = result.SUCCESSFUL;
        result.fraction = 0.0;

        std::vector<geometry_msgs::Pose> waypoints;
        for (int i = 0; i < goal->end_eff.poses.size(); i++)
        {
            waypoints.push_back(goal->end_eff.poses[i]);
        }
        moveit_msgs::RobotTrajectory trajectory;

        double fraction;
        if (leftFlag)
            fraction = group_left.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        else
            fraction = group_right.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        ROS_INFO("\n========================================");
        if (fraction > 0.95)
            ROS_INFO("fraction = %f\n", fraction);
        else
            ROS_ERROR("fraction = %f\n", fraction);

        if (goal->plan_only)
        {
            ROS_INFO("Plan only.");
            if (fraction <= 0.95)
                result.error_code = result.FAIL;
        }
        else
        {
            if (fraction > 0.95)
            {
                ROS_INFO("time parameterization\n");
                // Adding time parameterization to planned trajectory
                if (leftFlag)
                {
                    robot_trajectory::RobotTrajectory rt(group_left.getCurrentState()->getRobotModel(), group_name);
                    rt.setRobotTrajectoryMsg(*group_left.getCurrentState(), trajectory);
                    trajectory_processing::IterativeParabolicTimeParameterization iptp;
                    bool success = iptp.computeTimeStamps(rt);
                    ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                    rt.getRobotTrajectoryMsg(trajectory);
                }
                else
                {
                    robot_trajectory::RobotTrajectory rt(group_right.getCurrentState()->getRobotModel(), group_name);
                    rt.setRobotTrajectoryMsg(*group_right.getCurrentState(), trajectory);
                    trajectory_processing::IterativeParabolicTimeParameterization iptp;
                    bool success = iptp.computeTimeStamps(rt);
                    ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                    rt.getRobotTrajectoryMsg(trajectory);
                }

                // Filtering the time-parameterized trajectory with Uniform Sampler. Sampling time set in "uniform_sample_filter.cpp" to 0.005s for Olivia3.
                industrial_trajectory_filters::MessageAdapter t_in;
                t_in.request.trajectory = trajectory.joint_trajectory;
                industrial_trajectory_filters::MessageAdapter t_out;
                industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
                adapter.update(t_in, t_out);
                trajectory.joint_trajectory = t_out.request.trajectory;

                moveit::planning_interface::MoveGroup::Plan plan;
                std::cout << "Executing single arm-only motion (cart)...\n";

                Send2OliviaSingleArm(trajectory, goal->link_name);

                plan.trajectory_ = trajectory;

                if (leftFlag)
                {
                    group_left.setStartStateToCurrentState();
                    group_left.clearPoseTargets();
                    if (!group_left.execute(plan))
                    {
                        cout << ">>>>> actual execution failed <<<<<" << endl;
                        result.error_code = result.FAIL;
                    }
                }
                else
                {
                    group_right.setStartStateToCurrentState();
                    group_right.clearPoseTargets();
                    if (!group_right.execute(plan))
                    {
                        cout << ">>>>> actual execution failed <<<<<" << endl;
                        result.error_code = result.FAIL;
                    }
                }
            }
            else
                result.error_code = result.FAIL;
        }

        cart_single_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: Using cartesian space planning for single right arm
     * 
     * @param goal target
     */
    void CartSingleRightActionCB(const m3_moveit::MoveitSingleGoalConstPtr &goal)
    {

        std::cout << "\n\nCART SINGLE RIGHT ACTION!!\n\n"
                  << std::endl;

        // to prevent bug with attached items, refrain from creating new movegroup within functions
        bool leftFlag = false;

        std::string group_name;
        group_name = rgrp_name; // default

        // left
        if (goal->link_name == "l_ee")
        {
            leftFlag = true;
            group_name = lgrp_name;
        }

        if (leftFlag)
            group_left.clearPoseTargets();
        else
            group_right.clearPoseTargets();

        m3_moveit::MoveitSingleResult result;
        result.error_code = result.SUCCESSFUL;
        std::vector<geometry_msgs::Pose> waypoints;

        m3_moveit::MoveitSingleFeedback feedback;
        feedback.state = 0;
        cart_single_right_as_.publishFeedback(feedback);

        double fraction = 0;
        result.fraction = 0.0;

        if (goal->plan_only)
        {
            feedback.state = 3;
            cart_single_right_as_.publishFeedback(feedback);

            for (int i = 0; i < goal->end_eff.poses.size(); i++)
            {
                waypoints.push_back(goal->end_eff.poses[i]);
            }
            moveit_msgs::RobotTrajectory trajectory;

            if (leftFlag)
                group_left.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
            else
                group_right.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

            ROS_WARN_STREAM(">>> Fraction = " << fraction * 100 << "%");

            ROS_INFO("Plan only.");
            if (fraction <= 0.50)
                result.error_code = result.FAIL;
            result.fraction = fraction;
        }
        else
        {
            bool doreplan = goal->replan;
            bool planSucceed = false;
            // double fraction = 0;

            replan = doreplan;

            for (int i = 0; i < goal->end_eff.poses.size(); i++)
            {
                waypoints.push_back(goal->end_eff.poses[i]);
            }

            moveit_msgs::RobotTrajectory trajectory;

            do
            {
                planSucceed = false;
                quitFlag = false;
                responseFlag = false;
                fraction = 0;

                if (leftFlag)
                    fraction = group_left.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
                else
                    fraction = group_right.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

                // cout << trajectory <<endl;
                ROS_WARN_STREAM(">>> Fraction = " << fraction * 100 << "%");

                if (fraction <= 0.0)
                {
                    std::cout << "Planning failed." << std::endl;
                }
                else
                {
                    planSucceed = true;
                }

                if (replan)
                {
                    feedback.state = 4;
                    cart_single_right_as_.publishFeedback(feedback);

                    ros::Rate r(20); // in hz
                    ROS_INFO("waiting for user response...");
                    while (!responseFlag)
                    {
                        // wait
                        ros::spinOnce();
                        r.sleep();
                    }
                }

                // allows replan if execute a failed plan by mistake
                if (!replan && !planSucceed)
                    replan = true;

            } while (doreplan && replan && !quitFlag);

            if (!planSucceed || quitFlag) // quit action
            {
                ROS_ERROR_STREAM("planning failed");
                result.error_code = result.FAIL;
                cart_single_right_as_.setSucceeded(result);
                return;
            }
            else // execute
            {
                result.fraction = fraction;
                // Adding time parameterization to planned trajectory
                bool success = false;

                if (leftFlag)
                {
                    robot_trajectory::RobotTrajectory rt(group_left.getCurrentState()->getRobotModel(), lgrp_name);
                    rt.setRobotTrajectoryMsg(*group_left.getCurrentState(), trajectory);
                    trajectory_processing::IterativeParabolicTimeParameterization iptp;
                    success = iptp.computeTimeStamps(rt);
                    ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                    rt.getRobotTrajectoryMsg(trajectory);
                }
                else
                {
                    robot_trajectory::RobotTrajectory rt(group_right.getCurrentState()->getRobotModel(), rgrp_name);
                    rt.setRobotTrajectoryMsg(*group_right.getCurrentState(), trajectory);
                    trajectory_processing::IterativeParabolicTimeParameterization iptp;
                    success = iptp.computeTimeStamps(rt);
                    ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                    rt.getRobotTrajectoryMsg(trajectory);
                }

                // Filtering the time-parameterized trajectory with Uniform Sampler. Sampling time set in "uniform_sample_filter.cpp" to 0.005s for Olivia3.
                industrial_trajectory_filters::MessageAdapter t_in;
                t_in.request.trajectory = trajectory.joint_trajectory;
                industrial_trajectory_filters::MessageAdapter t_out;
                industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
                adapter.update(t_in, t_out);
                trajectory.joint_trajectory = t_out.request.trajectory;

                moveit::planning_interface::MoveGroup::Plan plan;
                std::cout << "Executing single arm-only motion (cart)...\n";

                Send2OliviaSingleArm(trajectory, goal->link_name);

                plan.trajectory_ = trajectory;
                if (leftFlag)
                {
                    if (!group_left.execute(plan))
                    {
                        ROS_ERROR_STREAM("execution failed");
                        result.error_code = result.FAIL;
                    }
                }
                else
                {
                    if (!group_right.execute(plan))
                    {
                        ROS_ERROR_STREAM("execution failed");
                        result.error_code = result.FAIL;
                    }
                }
            }
        }

        cart_single_right_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------
    
    /**
     * @brief radian to degree conversion
     * 
     * @param rad 
     * @return double degree
     */
    double r2d(double rad)
    {
        return rad * 180.0 / 3.14159;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Remap and convert joint angles to Olivia 
     * differences in real robot and urdf,
     * this is to remap the joints before sending to real robot
     * 
     * @param input joint values
     * @return std::vector<double> real joint values
     */
    std::vector<double> ConvertToOliviaCoordinates(const std::vector<double> input)
    {

        std::vector<double> output;
        output.resize(18);
        // torso
        output[0] = r2d(input[0]);

        output[1] = r2d(input[1]);
        output[2] = r2d(input[2]);

        // left arm
        output[3] = -r2d(input[3]);
        output[4] = -r2d(input[4]) - 15.0;
        output[5] = -r2d(input[5]);
        output[6] = -r2d(input[6]) + 20.0;
        output[7] = -r2d(input[7]);
        output[8] = -r2d(input[8]);
        output[9] = -r2d(input[9]);

        // neck pitch
        output[10] = r2d(input[10]);

        // right arm
        output[11] = -r2d(input[11]);
        output[12] = -r2d(input[12]) + 15.0;
        output[13] = -r2d(input[13]);
        output[14] = -r2d(input[14]) + 20.0;
        output[15] = -r2d(input[15]);
        output[16] = -r2d(input[16]);
        output[17] = -r2d(input[17]);

        return output;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Remap and convert joint angles to Olivia 
     * A separate dynamixel is used for the head.
     * Joint value need to be send separately
     * 
     * @param input joint values
     * @return std::vector<double> real joint values 
     */
    std::vector<double> ConvertToOliviaHead(const std::vector<double> input)
    {
        std::vector<double> output;
        output.resize(1);
        // head only
        output[0] = r2d(input[0]);
        return output;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Remap and convert joint angles to Olivia (for the arm only)
     * 
     * @param input joint values
     * @param arm name (left or right)
     * @return std::vector<double> real joint values
     */
    std::vector<double> ConvertOliviaArm(const std::vector<double> input, std::string arm)
    {
        std::vector<double> output;
        output.resize(7);
        double coeff;
        if (arm == "right")
            coeff = 1.0;
        else if (arm == "left")
            coeff = -1.0;
        output[0] = -r2d(input[0]);
        output[1] = -r2d(input[1]) + coeff * 15.0;
        output[2] = -r2d(input[2]);
        output[3] = -r2d(input[3]) + 20.0;
        output[4] = -r2d(input[4]);
        output[5] = -r2d(input[5]);
        output[6] = -r2d(input[6]);

        return output;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief send joint values to kaist daemon to move real olivia
     * 
     * @param trajectory solution from moveit
     * @param link_name end-effector name
     */
    void Send2OliviaSingleArm(moveit_msgs::RobotTrajectory trajectory, std::string link_name)
    {
        kaist_msgs::PlannedMoveRequest joint_traj_cmd;

        moveit_msgs::RobotTrajectory trajectory_whole;
        std::vector<double> current_joint_values, current_joint_values_0, new_arm_joint_values;
        group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), current_joint_values_0);

        // mapping from moveit to Olivia joint coordinates
        current_joint_values = ConvertToOliviaCoordinates(current_joint_values_0);

        trajectory_whole.joint_trajectory.points.resize(trajectory.joint_trajectory.points.size());
        trajectory_whole.joint_trajectory.joint_names.push_back("wsy");
        trajectory_whole.joint_trajectory.joint_names.push_back("wsp");
        trajectory_whole.joint_trajectory.joint_names.push_back("trp");

        trajectory_whole.joint_trajectory.joint_names.push_back("lsp");
        trajectory_whole.joint_trajectory.joint_names.push_back("lsr");
        trajectory_whole.joint_trajectory.joint_names.push_back("lsy");
        trajectory_whole.joint_trajectory.joint_names.push_back("leb");
        trajectory_whole.joint_trajectory.joint_names.push_back("lwy");
        trajectory_whole.joint_trajectory.joint_names.push_back("lwp");
        trajectory_whole.joint_trajectory.joint_names.push_back("lw2");

        trajectory_whole.joint_trajectory.joint_names.push_back("nkp");

        trajectory_whole.joint_trajectory.joint_names.push_back("rsp");
        trajectory_whole.joint_trajectory.joint_names.push_back("rsr");
        trajectory_whole.joint_trajectory.joint_names.push_back("rsy");
        trajectory_whole.joint_trajectory.joint_names.push_back("reb");
        trajectory_whole.joint_trajectory.joint_names.push_back("rwy");
        trajectory_whole.joint_trajectory.joint_names.push_back("rwp");
        trajectory_whole.joint_trajectory.joint_names.push_back("rw2");

        for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
        {

            trajectory_whole.joint_trajectory.points[i].positions.resize(current_joint_values.size());
            trajectory_whole.joint_trajectory.points[i].velocities.resize(current_joint_values.size());
            trajectory_whole.joint_trajectory.points[i].accelerations.resize(current_joint_values.size());
            trajectory_whole.joint_trajectory.points[i].positions = current_joint_values;
            trajectory_whole.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i].time_from_start;

            std::vector<double> arm_only;
            arm_only.resize(7);
            for (int k = 0; k < 7; k++)
                arm_only[k] = trajectory.joint_trajectory.points[i].positions[k + 3];

            if (link_name == "l_ee")
            {
                new_arm_joint_values = ConvertOliviaArm(arm_only, "left");
                for (int j = 3; j < 10; j++)
                {
                    trajectory_whole.joint_trajectory.points[i].positions[j] = new_arm_joint_values[j - 3];
                }
            }
            else if (link_name == "ee")
            {
                new_arm_joint_values = ConvertOliviaArm(arm_only, "right");
                for (int j = 11; j < 18; j++)
                {
                    trajectory_whole.joint_trajectory.points[i].positions[j] = new_arm_joint_values[j - 11];
                }
            }

            // write the waist trajectory
            trajectory_whole.joint_trajectory.points[i].positions[0] = r2d(trajectory.joint_trajectory.points[i].positions[0]);
            // write waist pitch trajectory
            trajectory_whole.joint_trajectory.points[i].positions[1] = r2d(trajectory.joint_trajectory.points[i].positions[1]);
            // write trunk trajectory
            trajectory_whole.joint_trajectory.points[i].positions[2] = r2d(trajectory.joint_trajectory.points[i].positions[2]);
        }
        joint_traj_cmd.names = trajectory_whole.joint_trajectory.joint_names;
        joint_traj_cmd.points = trajectory_whole.joint_trajectory.points;

        // set the period for movement execution
        size_t num_pts = trajectory_whole.joint_trajectory.points.size();
        float time_sec = (trajectory_whole.joint_trajectory.points[num_pts - 1].time_from_start - trajectory_whole.joint_trajectory.points[0].time_from_start).toSec();
        joint_traj_cmd.timems = 1.2 * time_sec * 1000;

        // publish to olivia robot. Upon receiving trajectory, olivia robot will perform movement.
        whole_body_pub.publish(joint_traj_cmd);
        std::cout << "Published trajectory to Olivia." << std::endl;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------
    // use as ref
    /**
     * @brief send joint values to kaist daemon to move real olivia (for whole body)
     * 
     * @param trajectory solution from moveit
     */
    void Send2OliviaWholeBody(moveit_msgs::RobotTrajectory trajectory)
    {
        kaist_msgs::PlannedMoveRequest joint_traj_cmd;

        moveit_msgs::RobotTrajectory trajectory_whole;
        std::vector<double> current_joint_values;
        //      group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), current_joint_values_0);

        trajectory_whole.joint_trajectory.points.resize(trajectory.joint_trajectory.points.size());
        trajectory_whole.joint_trajectory.joint_names.push_back("wsy");
        trajectory_whole.joint_trajectory.joint_names.push_back("wsp");
        trajectory_whole.joint_trajectory.joint_names.push_back("trp");

        trajectory_whole.joint_trajectory.joint_names.push_back("lsp");
        trajectory_whole.joint_trajectory.joint_names.push_back("lsr");
        trajectory_whole.joint_trajectory.joint_names.push_back("lsy");
        trajectory_whole.joint_trajectory.joint_names.push_back("leb");
        trajectory_whole.joint_trajectory.joint_names.push_back("lwy");
        trajectory_whole.joint_trajectory.joint_names.push_back("lwp");
        trajectory_whole.joint_trajectory.joint_names.push_back("lw2");

        trajectory_whole.joint_trajectory.joint_names.push_back("nkp");

        trajectory_whole.joint_trajectory.joint_names.push_back("rsp");
        trajectory_whole.joint_trajectory.joint_names.push_back("rsr");
        trajectory_whole.joint_trajectory.joint_names.push_back("rsy");
        trajectory_whole.joint_trajectory.joint_names.push_back("reb");
        trajectory_whole.joint_trajectory.joint_names.push_back("rwy");
        trajectory_whole.joint_trajectory.joint_names.push_back("rwp");
        trajectory_whole.joint_trajectory.joint_names.push_back("rw2");

        //      for(int i=0; i<trajectory.joint_trajectory.joint_names.size(); i++)
        //          cout << trajectory.joint_trajectory.joint_names.at(i) << endl;

        for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
        {
            trajectory_whole.joint_trajectory.points[i].positions.resize(current_joint_values.size());
            trajectory_whole.joint_trajectory.points[i].velocities.resize(current_joint_values.size());
            trajectory_whole.joint_trajectory.points[i].accelerations.resize(current_joint_values.size());

            // mapping from moveit to Olivia joint coordinates
            current_joint_values = ConvertToOliviaCoordinates(trajectory.joint_trajectory.points[i].positions);

            trajectory_whole.joint_trajectory.points[i].positions = current_joint_values;
            trajectory_whole.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i].time_from_start;
        }
        joint_traj_cmd.names = trajectory_whole.joint_trajectory.joint_names;
        joint_traj_cmd.points = trajectory_whole.joint_trajectory.points;

        // set the period for movement execution
        size_t num_pts = trajectory_whole.joint_trajectory.points.size();
        float time_sec = (trajectory_whole.joint_trajectory.points[num_pts - 1].time_from_start - trajectory_whole.joint_trajectory.points[0].time_from_start).toSec();
        joint_traj_cmd.timems = 1.2 * time_sec * 1000;

        // publish to olivia robot. Upon receiving trajectory, olivia robot will perform movement.
        whole_body_pub.publish(joint_traj_cmd);
        std::cout << "Published trajectory to Olivia." << std::endl;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief send joint values to dynamixel to move real olivia head only
     * 
     * @param trajectory target
     */
    void Send2OliviaHead(moveit_msgs::RobotTrajectory trajectory)
    {
        kaist_msgs::PlannedMoveRequest joint_traj_cmd;

        moveit_msgs::RobotTrajectory trajectory_head;
        std::vector<double> current_joint_values;

        trajectory_head.joint_trajectory.points.resize(trajectory.joint_trajectory.points.size());
        trajectory_head.joint_trajectory.joint_names.push_back("nkp");

        for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
        {

            trajectory_head.joint_trajectory.points[i].positions.resize(current_joint_values.size());
            trajectory_head.joint_trajectory.points[i].velocities.resize(current_joint_values.size());
            trajectory_head.joint_trajectory.points[i].accelerations.resize(current_joint_values.size());

            // mapping from moveit to Olivia joint coordinates
            current_joint_values = ConvertToOliviaHead(trajectory.joint_trajectory.points[i].positions);

            trajectory_head.joint_trajectory.points[i].positions = current_joint_values;
            trajectory_head.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[0].time_from_start;
        }

        joint_traj_cmd.names = trajectory_head.joint_trajectory.joint_names;
        joint_traj_cmd.points = trajectory_head.joint_trajectory.points;

        // set the period for movement execution
        size_t num_pts = trajectory_head.joint_trajectory.points.size();
        float time_sec = (trajectory_head.joint_trajectory.points[num_pts - 1].time_from_start - trajectory_head.joint_trajectory.points[0].time_from_start).toSec();
        joint_traj_cmd.timems = 1.2 * time_sec * 1000;

        // publish to olivia robot. Upon receiving trajectory, olivia robot will perform movement.
        whole_body_pub.publish(joint_traj_cmd);
        std::cout << "Published trajectory to Olivia." << std::endl;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: moveit group task planning, 
     * movement less predictable, but higher rate of success
     * 
     * @param goal target
     */
    void TaskSingleActionCB(const m3_moveit::MoveitSingleGoalConstPtr &goal)
    {

        ROS_WARN("TaskSingleActionCB");

        constrain_hand_orientation = goal->constrain_hand_orientation;
        std::cout << "constrain_hand_orientation = " << constrain_hand_orientation << std::endl;
        group_right.clearPoseTargets();
        m3_moveit::MoveitSingleResult result;
        result.error_code = result.SUCCESSFUL;
        result.fraction = 0.0;

        for (int i = 0; i < goal->end_eff.poses.size(); i++)
        {
            if (goal->plan_only)
            {
                moveit::planning_interface::MoveGroup::Plan plan;
                if (!group_right.plan(plan))
                    result.error_code = result.FAIL;
            }
            else
            {
                group_right.setStartStateToCurrentState();
                group_right.clearPoseTargets();

                if (!goal->link_name.empty())
                    group_right.setPoseTarget(goal->end_eff.poses[i], goal->link_name);

                moveit::planning_interface::MoveGroup::Plan plan;
                int iAttempt = 1;
                bool planSucceed = false;
                while (iAttempt <= MAX_ATTEMPTS && !planSucceed)
                {
                    if (!group_right.plan(plan))
                    {
                        std::cout << "Planning failed. Trying again " << MAX_ATTEMPTS - iAttempt << " times..." << std::endl;
                        JiggleTarget(iAttempt, goal->end_eff.poses[i], goal->link_name);
                    }
                    else
                    {
                        if (constrain_hand_orientation)
                        {
                            planSucceed = CheckHandOrientation(plan, goal->end_eff.poses[i], goal->link_name);
                        }
                        else
                            planSucceed = true;
                        if (!planSucceed)
                        {
                            std::cout << "CheckHandOrientation failed. Trying again " << MAX_ATTEMPTS - iAttempt << " times..." << std::endl;
                            JiggleTarget(iAttempt, goal->end_eff.poses[i], goal->link_name);
                        }
                    }
                    if (!planSucceed)
                        iAttempt += 1;
                }

                if (!planSucceed) // prevent error or core dump
                {
                    ROS_ERROR_STREAM("planning failed");
                    result.error_code = result.FAIL;
                    task_single_as_.setSucceeded(result);
                    return;
                }
                else
                {
                    // Adding time parameterization to planned trajectory
                    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
                    robot_trajectory::RobotTrajectory rt(group_right.getCurrentState()->getRobotModel(), "right_arm");
                    rt.setRobotTrajectoryMsg(*group_right.getCurrentState(), trajectory);
                    trajectory_processing::IterativeParabolicTimeParameterization iptp;
                    bool success = iptp.computeTimeStamps(rt);
                    ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                    rt.getRobotTrajectoryMsg(trajectory);

                    // Filtering the time-parameterized trajectory with Uniform Sampler. Sampling time set in "uniform_sample_filter.cpp" to 0.005s for Olivia3.
                    industrial_trajectory_filters::MessageAdapter t_in;
                    t_in.request.trajectory = trajectory.joint_trajectory;
                    industrial_trajectory_filters::MessageAdapter t_out;
                    industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
                    adapter.update(t_in, t_out);
                    trajectory.joint_trajectory = t_out.request.trajectory;

                    Send2OliviaSingleArm(trajectory, goal->link_name);

                    plan.trajectory_ = trajectory;
                    group_right.setStartStateToCurrentState();

                    if (planSucceed)
                    {
                        std::cout << "Executing arm-only motion (task space)...\n";
                        if (!group_right.execute(plan))
                            result.error_code = result.FAIL;
                    }
                    else
                        result.error_code = result.FAIL;

                } // end planning succeed
            }     // end plan only
        }         // end for each waypoint
        task_single_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: moveit group task planning (for right single arm only)
     * 
     * @param goal target
     */
    void TaskSingleRightActionCB(const m3_moveit::MoveitSingleGoalConstPtr &goal)
    {

        std::cout << "\n\nTASK SINGLE RIGHT ACTION!!\n\n"
                  << std::endl;

        constrain_hand_orientation = goal->constrain_hand_orientation;
        std::cout << "constrain_hand_orientation = " << constrain_hand_orientation << std::endl;

        // to prevent bug with attached items, refrain from creating new movegroup within functions
        bool leftFlag = false;

        std::string group_name;
        group_name = rgrp_name; // default

        // left
        if (goal->link_name == "l_ee")
        {
            leftFlag = true;
            // group_name = "left_arm";
            group_name = lgrp_name;
            cout << "doing LEFT HAND!!!" << endl;
        }

        if (leftFlag)
        {
            group_left.clearPoseTargets();
            group_left.setStartStateToCurrentState();
        }
        else
        {
            group_right.clearPoseTargets();
            group_right.setStartStateToCurrentState();
        }
        sleep(2.0);

        m3_moveit::MoveitSingleResult result;
        result.error_code = result.SUCCESSFUL;
        result.fraction = 0.0;

        m3_moveit::MoveitSingleFeedback feedback;
        feedback.state = 0;
        task_single_right_as_.publishFeedback(feedback);

        moveit::planning_interface::MoveGroup::Plan plan;

        for (int i = 0; i < goal->end_eff.poses.size(); i++)
        {

            if (goal->plan_only)
            {
                if (leftFlag)
                {
                    group_left.clearPoseTargets();
                    if (!goal->link_name.empty())
                    {
                        group_left.setPoseTarget(goal->end_eff.poses[i], goal->link_name);

                        feedback.state = 3;
                        task_single_right_as_.publishFeedback(feedback);

                        if (!group_left.plan(plan))
                            result.error_code = result.FAIL;
                    }
                }
                else
                {
                    group_right.clearPoseTargets();
                    if (!goal->link_name.empty())
                    {
                        group_right.setPoseTarget(goal->end_eff.poses[i], goal->link_name);

                        feedback.state = 3;
                        task_single_right_as_.publishFeedback(feedback);

                        if (!group_right.plan(plan))
                            result.error_code = result.FAIL;
                    }
                }
            } // end plan only
            else
            {
                bool doreplan = goal->replan;
                bool planSucceed;

                replan = doreplan;

                do
                {
                    planSucceed = false;
                    quitFlag = false;
                    responseFlag = false;

                    if (leftFlag)
                    {
                        group_left.clearPoseTargets();
                        if (!goal->link_name.empty())
                        {
                            group_left.setPoseTarget(goal->end_eff.poses[i], goal->link_name);
                            group_left.setStartStateToCurrentState();
                            // group_left.setGoalOrientationTolerance(0.02); //in radians

                            if (!group_left.plan(plan))
                                std::cout << "Planning failed." << std::endl;
                            else
                                planSucceed = true;
                        }
                    } // end left
                    else
                    {
                        group_right.clearPoseTargets();
                        if (!goal->link_name.empty())
                        {
                            group_right.setPoseTarget(goal->end_eff.poses[i], goal->link_name);
                            group_right.setStartStateToCurrentState();
                            // group_right.setGoalOrientationTolerance(0.02); //in radians

                            if (!group_right.plan(plan))
                                std::cout << "Planning failed." << std::endl;
                            else
                                planSucceed = true;
                        }
                    } // end right

                    if (replan)
                    {
                        feedback.state = 4;
                        task_single_right_as_.publishFeedback(feedback);

                        ros::Rate r(20); // in hz
                        ROS_INFO("waiting for user response...");
                        while (!responseFlag)
                        {
                            // wait
                            ros::spinOnce();
                            r.sleep();
                        }
                    }

                    // if execute, but plan failed, get to replan
                    if (!replan && !planSucceed)
                        replan = true;

                } while (doreplan && replan && !quitFlag);

                if (!planSucceed || quitFlag) // prevent error or core dump
                {
                    ROS_ERROR_STREAM("planning failed");
                    result.error_code = result.FAIL;
                    task_single_right_as_.setSucceeded(result);
                    return;
                }
                else
                {
                    bool success = false;
                    // Adding time parameterization to planned trajectory
                    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;

                    if (leftFlag)
                    {
                        robot_trajectory::RobotTrajectory rt(group_left.getCurrentState()->getRobotModel(), lgrp_name);
                        rt.setRobotTrajectoryMsg(*group_left.getCurrentState(), trajectory);
                        trajectory_processing::IterativeParabolicTimeParameterization iptp;
                        success = iptp.computeTimeStamps(rt);
                        ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                        rt.getRobotTrajectoryMsg(trajectory);
                    }
                    else
                    {
                        robot_trajectory::RobotTrajectory rt(group_right.getCurrentState()->getRobotModel(), rgrp_name);
                        rt.setRobotTrajectoryMsg(*group_right.getCurrentState(), trajectory);
                        trajectory_processing::IterativeParabolicTimeParameterization iptp;
                        success = iptp.computeTimeStamps(rt);
                        ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                        rt.getRobotTrajectoryMsg(trajectory);
                    }

                    // Filtering the time-parameterized trajectory with Uniform Sampler. Sampling time set in "uniform_sample_filter.cpp" to 0.005s for Olivia3.
                    industrial_trajectory_filters::MessageAdapter t_in;
                    t_in.request.trajectory = trajectory.joint_trajectory;
                    industrial_trajectory_filters::MessageAdapter t_out;
                    industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
                    adapter.update(t_in, t_out);
                    trajectory.joint_trajectory = t_out.request.trajectory;

                    for (int i = 0; i < trajectory.joint_trajectory.points.size(); i++)
                    {
                        trajectory.joint_trajectory.points[i].time_from_start *= 1.5;
                        for (int j = 0; j < trajectory.joint_trajectory.joint_names.size(); j++)
                        {
                            trajectory.joint_trajectory.points[i].velocities[j] /= 1.5;
                            trajectory.joint_trajectory.points[i].accelerations[j] /= pow(1.5, 2);
                        }
                    }

                    Send2OliviaSingleArm(trajectory, goal->link_name);

                    plan.trajectory_ = trajectory;
                    std::cout << "Executing arm-only motion (task space)...\n";

                    if (leftFlag)
                    {
                        if (!group_left.execute(plan))
                        {
                            ROS_ERROR_STREAM("execution failed");
                            result.error_code = result.FAIL;
                        }
                    }
                    else
                    {
                        if (!group_right.execute(plan))
                        {
                            ROS_ERROR_STREAM("execution failed");
                            result.error_code = result.FAIL;
                        }
                    }

                } // end planSucceed
            }     // end plan only else

        } // end for loop

        task_single_right_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: attaching objects in planning scene to arm (works for both left or right)
     * 
     * @param goal target
     */
    void AttachRightActionCB(const m3_moveit::MoveitPickPlaceGoalConstPtr &goal)
    {
        ROS_INFO_STREAM("\n\nATTACH OBJ CB");

        m3_moveit::MoveitPickPlaceResult result;
        bool success = false;

        if (goal->arm_name == "right")
        {
            success = group_right.attachObject(goal->object_name, "ee", rightHandLinkNames);
        }
        else if (goal->arm_name == "left")
        {
            success = group_left.attachObject(goal->object_name, "l_ee", leftHandLinkNames);
        }
        else
        {
            ROS_ERROR_STREAM("unknown arm name");
            result.error_code = result.FAIL;
            attach_action_as_.setSucceeded(result);
            return;
        }
        sleep(2.0);

        if (success)
        {
            std::cout << "object attached!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            attachedlist.push_back(goal->object_name);

            result.error_code = result.SUCCESSFUL;
            attach_action_as_.setSucceeded(result);
        }
        else
        {
            ROS_ERROR_STREAM("FAILED TO ATTACH");
            result.error_code = result.FAIL;
            attach_action_as_.setSucceeded(result);
        }
    }

    /**
     * @brief Moveit Action server: detach objects in planning scene from arm (works for both left or right)
     * 
     * @param goal target
     */
    void DetachRightActionCB(const m3_moveit::MoveitPickPlaceGoalConstPtr &goal)
    {
        ROS_INFO_STREAM("\n\nDETACH OBJ CB");

        m3_moveit::MoveitPickPlaceResult result;
        bool success = false;

        if (goal->arm_name == "right")
        {
            success = group_right.detachObject(goal->object_name);
        }
        else if (goal->arm_name == "left")
        {
            success = group_left.detachObject(goal->object_name);
        }
        else
        {
            ROS_ERROR_STREAM("unknown arm name");
            result.error_code = result.FAIL;
            detach_action_as_.setSucceeded(result);
            return;
        }
        sleep(2.0);

        if (success)
        {
            std::cout << "object detached!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            int idx = -1;
            for (int i = 0; i < attachedlist.size(); i++)
                if (attachedlist.at(i) == goal->object_name)
                    idx = i;

            if (idx > -1)
                attachedlist.erase(attachedlist.begin() + idx);

            result.error_code = result.SUCCESSFUL;
            detach_action_as_.setSucceeded(result);
        }
        else
        {
            ROS_ERROR_STREAM("FAILED TO DETACH");
            result.error_code = result.FAIL;
            detach_action_as_.setSucceeded(result);
        }
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: Remove objects from planning scene
     * 
     * @param goal target
     */
    void RemoveCollisionActionCB(const m3_moveit::MoveitCollideGoalConstPtr &goal)
    {
        std::cout << "\n\nRemoveCollisionActionCB ACTION!!\n\n"
                  << std::endl;

        m3_moveit::MoveitCollideResult result;
        result.error_code = result.FAIL;

        std::vector<std::string> object_ids;
        object_ids.push_back(goal->object_name);
        planning_scene_interface.removeCollisionObjects(object_ids);
        sleep(2.0);

        vector<string> list;
        list = planning_scene_interface.getKnownObjectNames();
        cout << "known object: \n[";
        for (int i = 0; i < list.size(); i++)
            cout << " " << list.at(i);
        cout << " ]" << endl;

        cout << "done" << endl;
        sleep(2.0);

        result.error_code = result.SUCCESSFUL;
        remove_collision_as_.setSucceeded(result);

        // check update scenes
        for (int i = 0; i < 5; i++)
        {
            usleep(100000);
            ros::spinOnce();
        }
    }

    /**
     * @brief Moveit Action server: add object into planning scene for collision-free planning
     * 
     * @param goal target
     */
    void AddCollisionActionCB(const m3_moveit::MoveitCollideGoalConstPtr &goal)
    {
        std::cout << "\n\nAddCollisionActionCB ACTION!!\n\n"
                  << std::endl;

        m3_moveit::MoveitCollideResult result;
        result.error_code = result.FAIL;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(goal->collision_object);
        planning_scene_interface.addCollisionObjects(collision_objects);
        sleep(2.0);

        vector<string> list;
        list = planning_scene_interface.getKnownObjectNames();
        cout << "known object: \n[";
        for (int i = 0; i < list.size(); i++)
            cout << " " << list.at(i);
        cout << " ]" << endl;

        bool notFound = true;
        ros::Rate r(20); // in hz
        while (notFound)
        {
            // wait
            ros::spinOnce();
            for (int i = 0; i < list.size(); i++)
            {
                if (list.at(i) == goal->object_name)
                {
                    notFound = false;
                    break;
                }
            }
            if (notFound)
                ROS_WARN_STREAM("w8ing for " << goal->object_name << " to appear in scene");
            r.sleep();
        }

        cout << "done" << endl;
        result.error_code = result.SUCCESSFUL;
        add_collision_as_.setSucceeded(result);

        // check update scenes
        for (int i = 0; i < 5; i++)
        {
            usleep(100000);
            ros::spinOnce();
        }
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: group task planning for both arms
     * 
     * @param goal 
     */
    void TaskDualActionCB(const m3_moveit::MoveitDualGoalConstPtr &goal)
    {
        constrain_hand_orientation = goal->constrain_hand_orientation;
        std::cout << "constrain_hand_orientation = " << constrain_hand_orientation << std::endl;
        group.clearPoseTargets();
        m3_moveit::MoveitDualResult result;
        result.error_code = result.SUCCESSFUL;

        if (goal->plan_only)
        {
            moveit::planning_interface::MoveGroup::Plan plan;
            if (!group.plan(plan))
                result.error_code = result.FAIL;
        }
        else
        {
            group.setStartStateToCurrentState();
            group.clearPoseTargets();

            if (!goal->left_link_name.empty())
                group.setPoseTarget(goal->left_end_eff, goal->left_link_name);
            if (!goal->right_link_name.empty())
                group.setPoseTarget(goal->right_end_eff, goal->right_link_name);

            moveit::planning_interface::MoveGroup::Plan plan;
            int iAttempt = 1;
            bool planSucceed = false;
            while (iAttempt <= MAX_ATTEMPTS && !planSucceed)
            {
                if (!group.plan(plan))
                {
                    std::cout << "Planning failed. Trying again " << MAX_ATTEMPTS - iAttempt << " times..." << std::endl;
                    JiggleTarget(iAttempt, goal->left_end_eff, goal->right_end_eff, goal->left_link_name, goal->right_link_name);
                }
                else
                {
                    if (constrain_hand_orientation)
                    {
                        planSucceed = CheckHandOrientation(plan, goal->left_end_eff, goal->right_end_eff);
                    }
                    else
                        planSucceed = true;
                    if (!planSucceed)
                    {
                        std::cout << "CheckHandOrientation failed. Trying again " << MAX_ATTEMPTS - iAttempt << " times..." << std::endl;
                        JiggleTarget(iAttempt, goal->left_end_eff, goal->right_end_eff, goal->left_link_name, goal->right_link_name);
                    }
                }
                if (!planSucceed)
                    iAttempt += 1;
            }

            // Adding time parameterization to planned trajectory
            moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
            robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "whole_body");
            rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            bool success = iptp.computeTimeStamps(rt);
            ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
            rt.getRobotTrajectoryMsg(trajectory);

            // Filtering the time-parameterized trajectory with Uniform Sampler. Sampling time set in "uniform_sample_filter.cpp" to 0.005s for Olivia3.
            industrial_trajectory_filters::MessageAdapter t_in;
            t_in.request.trajectory = trajectory.joint_trajectory;
            industrial_trajectory_filters::MessageAdapter t_out;
            industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
            adapter.update(t_in, t_out);
            trajectory.joint_trajectory = t_out.request.trajectory;

            plan.trajectory_ = trajectory;
            group.setStartStateToCurrentState();

            if (planSucceed)
            {
                std::cout << "Executing arms-only motion (task space)...\n";
                if (!group.execute(plan))
                    result.error_code = result.FAIL;
            }
            else
                result.error_code = result.FAIL;
        }
        task_dual_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: joint space planning for whole body
     * 
     * @param goal target
     */
    void MixedWholeBodyActionCB(const m3_moveit::MoveitWholeBodyGoalConstPtr &goal)
    {
        constrain_hand_orientation = goal->constrain_hand_orientation;
        std::cout << "constrain_hand_orientation = " << constrain_hand_orientation << std::endl;
        group.clearPoseTargets();
        m3_moveit::MoveitWholeBodyResult result;
        result.error_code = result.SUCCESSFUL;

        moveit_msgs::RobotState robotstate;
        std::vector<double> group_variable_values;

        ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        moveit_msgs::DisplayTrajectory display_trajectory;

        int nJointsTorso = goal->torso_trajectory.joint_names.size();
        int nJointsLeft = goal->left_trajectory.joint_names.size();
        int nJointsHead = goal->head_trajectory.joint_names.size();
        int nJointsRight = goal->right_trajectory.joint_names.size();
        int nJoints = nJointsTorso + nJointsLeft + nJointsHead + nJointsRight;

        robotstate.joint_state.position.resize(nJoints);
        for (int j = 0; j < nJointsTorso; j++)
            robotstate.joint_state.name.push_back(goal->torso_trajectory.joint_names[j]);
        for (int j = 0; j < nJointsLeft; j++)
            robotstate.joint_state.name.push_back(goal->left_trajectory.joint_names[j]);
        for (int j = 0; j < nJointsHead; j++)
            robotstate.joint_state.name.push_back(goal->head_trajectory.joint_names[j]);
        for (int j = 0; j < nJointsRight; j++)
            robotstate.joint_state.name.push_back(goal->right_trajectory.joint_names[j]);

        for (int i = 0; i < goal->torso_trajectory.points.size(); i++)
        {

            group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
            for (int j = 0; j < nJointsTorso; j++)
                group_variable_values[j] = goal->torso_trajectory.points[i].positions[j];

            for (int j = 0; j < group_variable_values.size(); j++)
            {
                robotstate.joint_state.position[j] = group_variable_values[j];
            }

            // TODO: what happens if size of pose array does not match that of joint array ?
            group.setPoseTarget(goal->left_end_eff.poses[i], goal->left_link_name);
            group.setPoseTarget(goal->right_end_eff.poses[i], goal->right_link_name);

            int iAttempt = 1;
            bool planSucceed = false;
            moveit::planning_interface::MoveGroup::Plan plan;
            moveit::planning_interface::MoveGroup::Plan planFinal;
            while (iAttempt <= MAX_ATTEMPTS && !planSucceed)
            {
                group.setStartState(robotstate);

                if (!group.plan(plan))
                {
                    result.error_code = result.FAIL;
                    std::cout << "Planning failed. Trying again " << MAX_ATTEMPTS - iAttempt << " times..." << std::endl;
                    JiggleTarget(iAttempt, goal->left_end_eff.poses[i], goal->right_end_eff.poses[i], goal->left_link_name, goal->right_link_name);
                }
                else
                {
                    int last = plan.trajectory_.joint_trajectory.points.size() - 1;
                    for (int j = nJointsTorso; j < group_variable_values.size(); j++)
                        group_variable_values[j] = plan.trajectory_.joint_trajectory.points[last].positions[j]; // pick the last trajectory point
                    int iStart = nJointsTorso + nJointsLeft;
                    int iEnd = iStart + nJointsHead;
                    for (int j = iStart; j < iEnd; j++)
                        group_variable_values[j] = goal->head_trajectory.points[i].positions[j - iStart];

                    group.setStartStateToCurrentState();
                    group.setJointValueTarget(group_variable_values);

                    if (!group.plan(planFinal))
                    {
                        result.error_code = result.FAIL;
                        std::cout << "Final planning failed. Trying again " << MAX_ATTEMPTS - iAttempt << " times..." << std::endl;
                        JiggleTarget(iAttempt, goal->left_end_eff.poses[i], goal->right_end_eff.poses[i], goal->left_link_name, goal->right_link_name);
                    }
                    else
                    {

                        if (constrain_hand_orientation)
                        {
                            planSucceed = CheckHandOrientation(planFinal, goal->left_end_eff.poses[i], goal->right_end_eff.poses[i]);
                        }
                        else
                            planSucceed = true;
                        if (!planSucceed)
                        {
                            std::cout << "CheckHandOrientation failed. Jiggle target and try again " << MAX_ATTEMPTS - iAttempt << " times..." << std::endl;
                            JiggleTarget(iAttempt, goal->left_end_eff.poses[i], goal->right_end_eff.poses[i], goal->left_link_name, goal->right_link_name);
                        }
                    }
                }
                if (!planSucceed)
                    iAttempt += 1;
            }

            if (planSucceed)
            {
                std::cout << "Executing whole_body motion (mixed mode)....\n";
                if (!group.execute(planFinal))
                    result.error_code = result.FAIL;
            }
            else
                result.error_code = result.FAIL;

            group.clearPoseTargets();
            group_variable_values.clear();
        }

        mixed_whole_body_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: joint space planning 
     * 
     * @param goal target
     */
    void JointWholeBodyActionCB(const m3_moveit::MoveitWholeBodyGoalConstPtr &goal)
    {

        ROS_WARN_STREAM("JointWholeBodyActionCB");
        group.clearPoseTargets();
        m3_moveit::MoveitWholeBodyResult result;
        result.error_code = result.SUCCESSFUL;
        moveit::planning_interface::MoveGroup::Plan plan;
        group.clearPoseTargets();
        group.setStartStateToCurrentState();
        sleep(2.0);

        std::vector<double> group_variable_values;
        for (int i = 0; i < goal->left_trajectory.points.size(); i++)
        {
            group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
            group_variable_values[0] = goal->torso_trajectory.points[i].positions[0];
            group_variable_values[1] = goal->torso_trajectory.points[i].positions[1];
            group_variable_values[2] = goal->torso_trajectory.points[i].positions[2];

            group_variable_values[3] = goal->left_trajectory.points[i].positions[0];
            group_variable_values[4] = goal->left_trajectory.points[i].positions[1];
            group_variable_values[5] = goal->left_trajectory.points[i].positions[2];
            group_variable_values[6] = goal->left_trajectory.points[i].positions[3];
            group_variable_values[7] = goal->left_trajectory.points[i].positions[4];
            group_variable_values[8] = goal->left_trajectory.points[i].positions[5];
            group_variable_values[9] = goal->left_trajectory.points[i].positions[6];

            // neck
            group_variable_values[10] = goal->head_trajectory.points[i].positions[0];

            group_variable_values[11] = goal->right_trajectory.points[i].positions[0];
            group_variable_values[12] = goal->right_trajectory.points[i].positions[1];
            group_variable_values[13] = goal->right_trajectory.points[i].positions[2];
            group_variable_values[14] = goal->right_trajectory.points[i].positions[3];
            group_variable_values[15] = goal->right_trajectory.points[i].positions[4];
            group_variable_values[16] = goal->right_trajectory.points[i].positions[5];
            group_variable_values[17] = goal->right_trajectory.points[i].positions[6];

            group.setJointValueTarget(group_variable_values);
            sleep(0.5);

            if (goal->plan_only)
            {
                if (!group.plan(plan))
                    result.error_code = result.FAIL;
            }
            else
            {
                if (!group.plan(plan))
                {
                    result.error_code = result.FAIL;
                    std::cout << "planning failed" << std::endl;
                }
                else
                {
                    // Adding time parameterization to planned trajectory
                    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
                    robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "whole_body");
                    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
                    trajectory_processing::IterativeParabolicTimeParameterization iptp;
                    bool success = iptp.computeTimeStamps(rt);
                    ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                    rt.getRobotTrajectoryMsg(trajectory);

                    // Filtering the time-parameterized trajectory with Uniform Sampler. Sampling time set in "uniform_sample_filter.cpp" to 0.005s for Olivia3.
                    industrial_trajectory_filters::MessageAdapter t_in;
                    t_in.request.trajectory = trajectory.joint_trajectory;
                    industrial_trajectory_filters::MessageAdapter t_out;
                    industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
                    adapter.update(t_in, t_out);
                    trajectory.joint_trajectory = t_out.request.trajectory;

                    Send2OliviaWholeBody(trajectory);

                    plan.trajectory_ = trajectory;
                    group.setStartStateToCurrentState();
                    sleep(2.0);

                    std::cout << "Executing whole-body motion (joint space)...\n";
                    if (!group.execute(plan))
                        result.error_code = result.FAIL;
                }

                group_variable_values.clear();
            }
        }
        joint_whole_body_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Moveit Action server: Moving the neck only
     * 
     * @param goal target
     */
    void NeckSingleActionCB(const m3_moveit::NeckTiltGoalConstPtr &goal)
    {

        ROS_WARN_STREAM("NeckSingleActionCB");

        moveit::planning_interface::MoveGroup group_head("head");
        group_head.clearPoseTargets();

        m3_moveit::NeckTiltResult result;
        result.error_code = result.SUCCESSFUL;

        std::vector<double> head_variable_values;
        group_head.setStartStateToCurrentState();

        for (int i = 0; i < goal->head_trajectory.points.size(); i++)
        {
            group_head.getCurrentState()->copyJointGroupPositions(group_head.getCurrentState()->getRobotModel()->getJointModelGroup(group_head.getName()), head_variable_values);

            for (int j = 0; j < head_variable_values.size(); j++)
                head_variable_values[j] = goal->head_trajectory.points[i].positions[j];

            group_head.setJointValueTarget(head_variable_values);

            moveit::planning_interface::MoveGroup::Plan plan;

            if (!group_head.plan(plan))
            {
                result.error_code = result.FAIL;
                std::cout << "planning failed" << std::endl;
            }
            else
            {

                // Adding time parameterization to planned trajectory
                ROS_INFO("time parameterization\n");
                moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;

                robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "head");
                rt.setRobotTrajectoryMsg(*group_head.getCurrentState(), trajectory);
                trajectory_processing::IterativeParabolicTimeParameterization iptp;
                bool success = iptp.computeTimeStamps(rt);
                ROS_INFO("Computed time stamp %s", success ? "SUCCEEDED" : "FAILED");
                rt.getRobotTrajectoryMsg(trajectory);

                // ROS_INFO("Filtering");
                //  Filtering the time-parameterized trajectory with Uniform Sampler. Sampling time set in "uniform_sample_filter.cpp" to 0.005s for Olivia3.
                industrial_trajectory_filters::MessageAdapter t_in;
                t_in.request.trajectory = trajectory.joint_trajectory;
                industrial_trajectory_filters::MessageAdapter t_out;
                industrial_trajectory_filters::UniformSampleFilterAdapter adapter;
                adapter.update(t_in, t_out);
                trajectory.joint_trajectory = t_out.request.trajectory;

                Send2OliviaHead(trajectory);

                plan.trajectory_ = trajectory;
                group_head.setStartStateToCurrentState();

                std::cout << "Executing head motion (joint space)...\n";
                if (!group_head.execute(plan))
                    result.error_code = result.FAIL;

                head_variable_values.clear();
            }
        }
        neck_tilt_as_.setSucceeded(result);
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief check the planning scene kinematics models for hand orientation
     * 
     * @param plan moveit solution
     * @param pose_left left arm
     * @param pose_right right arm
     * @return true success
     * @return false failed
     */
    bool CheckHandOrientation(const moveit::planning_interface::MoveGroup::Plan plan, const geometry_msgs::Pose pose_left, const geometry_msgs::Pose pose_right)
    {

        bool planSucceed = false;
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(group.getCurrentState()->getRobotModel()));
        const Eigen::Affine3d &end_effector_stateL = kinematic_state->getGlobalLinkTransform("l_ee");
        const Eigen::Affine3d &end_effector_stateR = kinematic_state->getGlobalLinkTransform("ee");
        std::vector<double> joint_values;

        std::cout << "No. planned trajectory points = " << plan.trajectory_.joint_trajectory.points.size() << std::endl;

        for (int j = 0; j < plan.trajectory_.joint_trajectory.points.size(); j++)
        {
            joint_values.resize(plan.trajectory_.joint_trajectory.points[j].positions.size());
            for (int k = 0; k < plan.trajectory_.joint_trajectory.points[j].positions.size(); k++)
                joint_values[k] = plan.trajectory_.joint_trajectory.points[j].positions[k];
            kinematic_state->setJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), joint_values);
            kinematic_state->update();

            Eigen::Matrix3d matL = end_effector_stateL.rotation();
            Eigen::Quaterniond qL(matL);
            Eigen::Matrix3d matR = end_effector_stateR.rotation();
            Eigen::Quaterniond qR(matR);

            std::vector<double> quat_planL, quat_goalL, quat_planR, quat_goalR;
            quat_planL.resize(4);
            quat_planL[0] = qL.w();
            quat_planL[1] = qL.x();
            quat_planL[2] = qL.y();
            quat_planL[3] = qL.z();
            quat_goalL.resize(4);
            quat_goalL[0] = pose_left.orientation.w;
            quat_goalL[1] = pose_left.orientation.x;
            quat_goalL[2] = pose_left.orientation.y;
            quat_goalL[3] = pose_left.orientation.z;
            quat_planR.resize(4);
            quat_planR[0] = qR.w();
            quat_planR[1] = qR.x();
            quat_planR[2] = qR.y();
            quat_planR[3] = qR.z();
            quat_goalR.resize(4);
            quat_goalR[0] = pose_right.orientation.w;
            quat_goalR[1] = pose_right.orientation.x;
            quat_goalR[2] = pose_right.orientation.y;
            quat_goalR[3] = pose_right.orientation.z;

            if (j == 0)
                planSucceed = true;
            if (GetQuaternionDistance(quat_planL, quat_goalL) > HAND_ORIENTATION_DIST_TOL || GetQuaternionDistance(quat_planR, quat_goalR) > HAND_ORIENTATION_DIST_TOL)
            {
                planSucceed = false;
                std::cout << "Hand orientation difference between goal and plan (" << j << "th point) exceeded tolerance!!" << std::endl;
            }
        }
        if (planSucceed)
            std::cout << "Hand orientations throughout plan is close to goal." << std::endl;
        std::cout << "=================================================================== " << std::endl;
        return planSucceed;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief check the hand orientation in moveit planning
     * 
     * @param plan moveit solution
     * @param pose desired pose
     * @param name link_name
     * @return true success
     * @return false failed
     */
    bool CheckHandOrientation(const moveit::planning_interface::MoveGroup::Plan plan, const geometry_msgs::Pose pose, std::string name)
    {

        bool planSucceed = false;
        robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(group.getCurrentState()->getRobotModel()));
        const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(name);
        std::vector<double> joint_values;

        std::cout << "No. planned trajectory points = " << plan.trajectory_.joint_trajectory.points.size() << std::endl;

        for (int j = 0; j < plan.trajectory_.joint_trajectory.points.size(); j++)
        {
            joint_values.resize(plan.trajectory_.joint_trajectory.points[j].positions.size());
            for (int k = 0; k < plan.trajectory_.joint_trajectory.points[j].positions.size(); k++)
                joint_values[k] = plan.trajectory_.joint_trajectory.points[j].positions[k];
            kinematic_state->setJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), joint_values);
            kinematic_state->update();

            Eigen::Matrix3d mat = end_effector_state.rotation();
            Eigen::Quaterniond q(mat);

            std::vector<double> quat_plan, quat_goal;
            quat_plan.resize(4);
            quat_plan[0] = q.w();
            quat_plan[1] = q.x();
            quat_plan[2] = q.y();
            quat_plan[3] = q.z();
            quat_goal.resize(4);
            quat_goal[0] = pose.orientation.w;
            quat_goal[1] = pose.orientation.x;
            quat_goal[2] = pose.orientation.y;
            quat_goal[3] = pose.orientation.z;

            if (j == 0)
                planSucceed = true;
            if (GetQuaternionDistance(quat_plan, quat_goal) > HAND_ORIENTATION_DIST_TOL)
            {
                planSucceed = false;
                std::cout << "Hand orientation difference between goal and plan (" << j << "th point) exceeded tolerance!!" << std::endl;
            }
        }
        if (planSucceed)
            std::cout << "Hand orientations throughout plan is close to goal." << std::endl;
        std::cout << "=================================================================== " << std::endl;
        return planSucceed;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Get the delta distance between 2 quaternion
     * 
     * @param quat1 input1
     * @param quat2 input2
     * @return double distance
     */
    double GetQuaternionDistance(std::vector<double> quat1, std::vector<double> quat2)
    {
        double sum = 0.0;
        for (int k = 0; k < 4; k++)
        {
            sum += quat1[k] * quat2[k];
        }
        return 1.0 - sum * sum;
    }

    //------------------------------------------------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------------------------------------------------

    /**
     * @brief Attempt to jiggle the target pose to achieve higher success rate when moveit fails to plan
     * 
     * @param iAttempt number of jiggles
     * @param pose current pose
     * @param name link_name
     */
    void JiggleTarget(int iAttempt, geometry_msgs::Pose pose, std::string name)
    {
        geometry_msgs::Pose new_target;
        double jiggleX = JIGGLE_CONST * iAttempt;
        double jiggleY = JIGGLE_CONST * iAttempt * (2 * (iAttempt % 2) - 1);
        if (jiggleX > 0.05)
            jiggleX = 0.05;
        else if (jiggleX < -0.05)
            jiggleX = -0.05;
        if (jiggleY > 0.03)
            jiggleY = 0.03;
        else if (jiggleY < -0.03)
            jiggleY = -0.03;
        new_target = pose;
        new_target.position.x += jiggleX;
        new_target.position.y += jiggleY;

        group.setPoseTarget(new_target, name);

        std::cout << "old_target = " << pose.position;
        std::cout << "new_target = " << new_target.position << std::endl;
    }

    /**
     * @brief Attempt to jiggle the target pose to achieve higher success rate when moveit fails to plan
     * 
     * @param iAttempt number of jiggles
     * @param pose_left left arm
     * @param pose_right right arm
     * @param name_left link_name of left
     * @param name_right link_name of right
     */
    void JiggleTarget(int iAttempt, geometry_msgs::Pose pose_left, geometry_msgs::Pose pose_right, std::string name_left, std::string name_right)
    {
        geometry_msgs::Pose new_target_left, new_target_right;
        double jiggleX = JIGGLE_CONST * iAttempt;
        double jiggleY = JIGGLE_CONST * iAttempt * (2 * (iAttempt % 2) - 1);
        if (jiggleX > 0.05)
            jiggleX = 0.05;
        else if (jiggleX < -0.05)
            jiggleX = -0.05;
        if (jiggleY > 0.03)
            jiggleY = 0.03;
        else if (jiggleY < -0.03)
            jiggleY = -0.03;
        new_target_left = pose_left;
        new_target_left.position.x += jiggleX;
        new_target_left.position.y += jiggleY;
        new_target_right = pose_right;
        new_target_right.position.x += jiggleX;
        new_target_right.position.y += jiggleY;
        group.setPoseTarget(new_target_left, name_left);
        group.setPoseTarget(new_target_right, name_right);
        std::cout << "old_target_left = " << pose_left.position;
        std::cout << "new_target_left = " << new_target_left.position << std::endl;
        std::cout << "old_target_right = " << pose_right.position;
        std::cout << "new_target_right = " << new_target_right.position << std::endl;
    }

private:
    double speed;
    int counter;

    ros::NodeHandle nh;
    ros::Subscriber sub, input_sub;

    string input;

    actionlib::SimpleActionServer<m3_moveit::MoveitSingleAction> cart_single_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitFingersAction> fingers_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitFingersAction> fingers_encoders_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitSingleAction> task_single_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitDualAction> task_dual_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitWholeBodyAction> mixed_whole_body_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitWholeBodyAction> joint_whole_body_as_;
    actionlib::SimpleActionServer<m3_moveit::NeckTiltAction> neck_tilt_as_;

    actionlib::SimpleActionServer<m3_moveit::MoveitPickPlaceAction> attach_action_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitPickPlaceAction> detach_action_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitCollideAction> add_collision_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitCollideAction> remove_collision_as_;

    actionlib::SimpleActionServer<m3_moveit::MoveitSingleAction> task_single_right_as_;
    actionlib::SimpleActionServer<m3_moveit::MoveitSingleAction> cart_single_right_as_;

    ros::Publisher planning_scene_diff_pub;
    ros::ServiceClient get_planning_scene;
    ros::Subscriber sub_collision_object;
    ros::Subscriber sub_finger_encoder;
    ros::Subscriber sub_finger_timeout;

    moveit::planning_interface::MoveGroup group;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit::planning_interface::MoveGroup group_right;
    moveit::planning_interface::MoveGroup group_left;

    std::string lgrp_name, rgrp_name;

    std::vector<std::string> attachedlist;

    std::vector<std::string> rightHandLinkNames, leftHandLinkNames;
    std::string rightpalmLinkName = "handmount_RIGHT";
    std::string leftpalmLinkName = "handmount_LEFT";

    std::string finger1_name, finger2_name;
};

//------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief main program, this moveit action server will be running in background.
 * In tool experiment, when Olivia robot needs to move, it will do the planning and execution upon goal recieved. 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_action_server");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string name;
    nh.getParam("group", name);
    M3MoveGroup m3_group(name);

    ros::Rate r(20); // in hz
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
    }
    ROS_INFO("end of moveit");
    ros::shutdown();
    return 0;
}
