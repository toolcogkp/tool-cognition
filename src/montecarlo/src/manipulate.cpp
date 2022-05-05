#include <tool_expt/manipulate.h>
#include <tool_expt/calib.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/SetLinkState.h>

#include <geometric_shapes/shape_operations.h>

//#define USE_SPEECH

using namespace std;
using namespace Eigen;

bool failFlag = false;
int failcount = 0;
int maxFail = 0;        

const double objtohand = 0.105 + 0.005; 
const double side_objtohand = 0.12;
const double forehand_finetune = 0.03;
const double backhand_finetune = 0.01;

double diameter_size = 8.3; 
const double tool_diameter = 3.0; 

int task_mode;
bool neck_pan_flag = false;

ros::Publisher fingerFailPub;

Manipulate::Manipulate(my_context ctx ) : my_base( ctx ) 
{
    object_target_received = false; tool_handle_received = false; tool_decision_received = false; sticklength_received = false;
    obstacle_detected = false;
    
    is_suitable_tool = 0; calib_next = 0;

    neck_pan_angle = 0;
    neck_tilt_angle = 0;
    current_pan_angle = 0;
    current_tilt_angle = 0;
 
    n.getParam("/task_mode", task_mode);
    n.getParam("/diameter", diameter_size);
    n.getParam("/pan_angle", neck_pan_angle);
    n.getParam("/tilt_angle", neck_tilt_angle);

    //Subscribers
    //------------
    sub_object = n.subscribe("/object_target", 1, &Manipulate::object_target_positions_callBack, this);
    sub_fake_tool_transform = n.subscribe("/fake_tool_transform", 1, &Manipulate::fake_tool_transform_callBack, this);
    sub_fake_target = n.subscribe("/fake_target", 1, &Manipulate::fake_target_callBack, this);
    sub_calib_tool_poses = n.subscribe("/calib_tool_poses", 1, &Manipulate::calib_tool_poses_callBack, this);
    sub_calib_next = n.subscribe("/calib_next", 1, &Manipulate::calib_next_callBack, this);
    sub_tool_decision = n.subscribe("/tool_decision", 1, &Manipulate::tool_decision_callBack, this);
    sub_joint_states = n.subscribe("/joint_states", 1 , &Manipulate::joint_states_callBack, this);

    sub_tool_handle = n.subscribe("/tool_handle", 1, &Manipulate::tool_handle_position_callBack, this);
    sub_bendtool = n.subscribe("/bendtool_pose", 1, &Manipulate::bendtool_position_callBack, this);
    sub_sticklength = n.subscribe("/sticklength",1, &Manipulate::sticklength_callBack, this);

    joint_updated = false;

    markers_pub = n.advertise<visualization_msgs::MarkerArray>("/rviz_markers", 10);
    marker_List.markers.clear();
    marker_id = 0;

    sub_vision = n.subscribe("/vision/data", 1, &Manipulate::vision_data_callBack, this);
    detected = true; // stop detection unless triggered

    client_tf_switch =  n.serviceClient<monte_carlo::tf_switch>("tf_switch");

    //Publishers
    //----------
    pub_calib = n.advertise<tool_expt::calib>("/calib",10);
    pub_req_object_target = n.advertise<std_msgs::Int32>("/request_object_target",10);
    pub_req_tool_handle = n.advertise<std_msgs::Int32>("/request_tool_handle",10);

    fingerFailPub = n.advertise<std_msgs::Bool>("/fingerFail", 5);

    display_rs_publisher = n.advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1, true);
    input_pub = n.advertise<std_msgs::Int32>("/user/input", 1, true);

    camera_swap_publisher = n.advertise<std_msgs::Int32>("/camera_olivia/swap", 1, true);

    functionality = 0;
    fake_target_received = false;
    using_tool = false; //default
    sleep_time = 1.0; //sleep to wait for motion to complete

    TABLE_HEIGHT = 0.74;
    PUCK_HEIGHT = 0.09;
    PUCK_RADIUS = 0.04;
    TOOL_HANDLE_HEIGHT = 0.12;
    HAND_THICKNESS = 0.11;
    LIFT_TOOL_HEIGHT = 0.1;//0.07
    MOVE_WAIST_ANGLE = -45.0;
    
    // WAIST_INIT = 30.0;//45.0
    WAIST_INIT = 0.0;//45.0

    WAIST_TOOL = -60.0;//-60.0
    
    waist_yaw = 0.0; //rad
    base_yaw = -3.14159/4.0;//rad

    BENDTOOL_RADIUS = 0.029;
    LIFT_BENDTOOL_HEIGHT = 0.2;

    const double OFFSET_LONG = PUCK_RADIUS  + 0.5*HAND_THICKNESS;
    const double OFFSET_SHORT = 0.0;
    const double OFFSET_LAT = PUCK_RADIUS  + 0.5*HAND_THICKNESS;
    const double OFFSET_HT = 0.0;



    //offsets for hand
    //-----------------------------------
    pullback_offset_right_hand.x = OFFSET_LAT;
    pullback_offset_right_hand.y = 0.0;//-OFFSET_SHORT;
    pullback_offset_right_hand.z = OFFSET_HT;
    pullback_orientation_right_hand.w = 0.0;
    pullback_orientation_right_hand.x = 0.0;
    pullback_orientation_right_hand.y = 0.70711;
    pullback_orientation_right_hand.z = 0.70711;

    pullback_offset_left_hand.x = OFFSET_LAT;
    pullback_offset_left_hand.y = OFFSET_SHORT;
    pullback_offset_left_hand.z = OFFSET_HT;
    pullback_orientation_left_hand.w = 0.0;
    pullback_orientation_left_hand.x = 0.0;
    pullback_orientation_left_hand.y = -0.70711;
    pullback_orientation_left_hand.z = 0.70711;

    pushsideways_offset_right_hand.x = -OFFSET_SHORT;
    pushsideways_offset_right_hand.y = -OFFSET_LAT;
    pushsideways_offset_right_hand.z = OFFSET_HT;
    pushsideways_orientation_right_hand.w = 0.5;
    pushsideways_orientation_right_hand.x = 0.5;
    pushsideways_orientation_right_hand.y = 0.5;
    pushsideways_orientation_right_hand.z = 0.5;

    pushsideways_offset_left_hand.x = -OFFSET_SHORT;
    pushsideways_offset_left_hand.y = OFFSET_LAT;
    pushsideways_offset_left_hand.z = OFFSET_HT;
    pushsideways_orientation_left_hand.w = 0.5;
    pushsideways_orientation_left_hand.x = -0.5;
    pushsideways_orientation_left_hand.y = 0.5;
    pushsideways_orientation_left_hand.z = -0.5;

    pushforward_offset_right_hand.x = -OFFSET_LONG;
    pushforward_offset_right_hand.y = 0.0;
    pushforward_offset_right_hand.z = OFFSET_HT;
    pushforward_orientation_right_hand.w = 0.5;
    pushforward_orientation_right_hand.x = 0.5;
    pushforward_orientation_right_hand.y = 0.5;
    pushforward_orientation_right_hand.z = 0.5;

    pushforward_offset_left_hand.x = pushforward_offset_right_hand.x;
    pushforward_offset_left_hand.y = pushforward_offset_right_hand.y;
    pushforward_offset_left_hand.z = pushforward_offset_right_hand.z;
    pushforward_orientation_left_hand.w = 0.5;
    pushforward_orientation_left_hand.x = -0.5;
    pushforward_orientation_left_hand.y = 0.5;
    pushforward_orientation_left_hand.z = -0.5;


    //offsets for tool
    //-----------------------------------
    tool_offset_right_hand.x = 0.0;
    tool_offset_right_hand.y = 0.0;
    tool_offset_right_hand.z = 0.0;
    tool_orientation_right_hand.w = 0.70711;
    tool_orientation_right_hand.x = sqrt(1.0-0.70711*0.70711);
    tool_orientation_right_hand.y = 0.0;
    tool_orientation_right_hand.z = 0.0;

    bendtool_offset_right_hand.x = 0.0;
    bendtool_offset_right_hand.y = 0.0;
    bendtool_offset_right_hand.z = OFFSET_LONG+0.05;
    bendtool_orientation_right_hand.w = 0.0;
    bendtool_orientation_right_hand.x = 0.70711;
    bendtool_orientation_right_hand.y = sqrt(1.0-0.70711*0.70711);
    bendtool_orientation_right_hand.z = 0.0;

    //functionality = 2 //back
    pullback_offset_right_tool.x = PUCK_RADIUS;
    pullback_offset_right_tool.y = 0.0;
    pullback_offset_right_tool.z = 0.0;
    pullback_orientation_right_tool.w = 0.5;
    pullback_orientation_right_tool.x = 0.5;
    pullback_orientation_right_tool.y = 0.5;
    pullback_orientation_right_tool.z = 0.5;

    //functionality = 3 //side
    pushsideways_offset_right_tool.x = 0.0;
    pushsideways_offset_right_tool.y = -PUCK_RADIUS;
    pushsideways_offset_right_tool.z = 0.0;
    pushsideways_orientation_right_tool.w = 0.5;
    pushsideways_orientation_right_tool.x = 0.5;
    pushsideways_orientation_right_tool.y = 0.5;
    pushsideways_orientation_right_tool.z = 0.5;

    //functionality = 1 //for
    pushforward_offset_right_tool.x = -PUCK_RADIUS;
    pushforward_offset_right_tool.y = 0.0;
    pushforward_offset_right_tool.z = 0.0;
    pushforward_orientation_right_tool.w = 0.5;
    pushforward_orientation_right_tool.x = 0.5;
    pushforward_orientation_right_tool.y = 0.5;
    pushforward_orientation_right_tool.z = 0.5;
    
    //-----------------------------------
    
    cout << "task_mode =" << task_mode << endl;

    switch(task_mode)
    {
        case 1:{
            WAIST_INIT = 0;
            cout << "go to home... press enter to continue" << endl;
            cin.ignore();
            Home();
            break;
        }
        case 0:{
            
            WAIST_INIT = -30.0;
            Home();
            //default task/demo
            Speech("Starting the demonstration.");

            Speech("My task is to push ... the yellow object ... ");
            Speech("To the red goal ... on the table.");
            sleep(2.0);
    
            post_event(EvPerceiveObject());
            break;
        }
        case 2:{
            //for position testing only
            Home();

            MoveWaist(-15);
            sleep(sleep_time);

            cout << "press enter to continue" << endl;
            cin.ignore();

            object_in_base.pose.position.x = 0.697172;
            object_in_base.pose.position.y = -0.399337;
            object_in_base.pose.position.z = 0.79;
            target_in_base.pose.position.x = 0.727579;
            target_in_base.pose.position.y = -0.143182;
            target_in_base.pose.position.z = 0.79;
            cout << "going to position" << endl;
            radian = 173*PI/180.0;
            object_to_hand_position_offset.x =  -0.40025;
            object_to_hand_position_offset.y = -0.0215499;
            object_to_hand_position_offset.z = 0.05;
            object_to_hand_orientation_offset.x = 0.528649;
            object_to_hand_orientation_offset.y = 0.469606;
            object_to_hand_orientation_offset.z = 0.469606;
            object_to_hand_orientation_offset.w = 0.528649;

            cout << "press enter to continue" << endl;
            cin.ignore();
            Home();
            break;
        }
        case 3:{
            cout << "open fingers\n" << endl;
            OpenFingers("left", 0 );
            sleep(sleep_time);
            break;
        }
        case 4:{
            cout << "close fingers\n" <<endl;
            CloseFingers("left",0,0);
            sleep(sleep_time);
            break;
        }
        case 5:{
            cout << "pinching obj of diameter size= " << diameter_size << "\n" << endl;
            
            if(diameter_size < 0){
                cout << "input diameter size is negative. quitting." << endl; 
            }
            else if(diameter_size > 15.0){
                cout << "input diameter size is too large. this is in cm. quitting." << endl;
            }
            else{
                pinchFingers("right", diameter_size, 0);
                sleep(sleep_time);
            }

            break;
        }
        case 6:{
            cout << "power grabbing obj of diameter size= " << diameter_size << "\n" << endl;
            
            if(diameter_size < 0){
                cout << "input diameter size is negative. quitting." << endl; 
            }
            else if(diameter_size > 15.0){
                cout << "input diameter size is too large. this is in cm. quitting." << endl;
            }
            else{
                powerFingers("right", diameter_size, 0);
                sleep(sleep_time);
            }

            break;
        }

        #ifdef USE_NECK_PAN
            case 7:{
                cout << "neck_panning to "<< neck_pan_angle << "degs" << endl;
                NeckPan(neck_pan_angle, 0.5);

                sleep(sleep_time);
                for(int i =0; i < 10; i++)
                    ros::spinOnce();
                cout << "current pan angle = " << current_pan_angle << endl;
                            cout << "current tilt angle = " << current_tilt_angle << endl;
                break;
            }
            case 8:{
                cout << "neck tilt to "<< neck_tilt_angle << "degs" << endl;
                NeckTilt(neck_tilt_angle);

                sleep(sleep_time);
                for(int i =0; i < 10; i++)
                    ros::spinOnce();
                cout << "current pan angle = " << current_pan_angle << endl;
                            cout << "current tilt angle = " << current_tilt_angle << endl;
                break;
            }
            case 9:{
                cout << "neck tilt to "<< neck_tilt_angle << "degs" << endl;
                cout << "neck_panning to "<< neck_pan_angle << "degs" << endl;
                TurnHeadTiltPan(neck_tilt_angle, neck_pan_angle);

                sleep(sleep_time);
                for(int i =0; i < 10; i++)
                    ros::spinOnce();
                cout << "current pan angle = " << current_pan_angle << endl;
                            cout << "current tilt angle = " << current_tilt_angle << endl;
                break;
            }
            case 10:{
                //testing if /tf is correct
                cout << "testing if /tf is correct >> enter to continue";
                cin.ignore();

                std_msgs::Int32 req_object_target_msg;
                req_object_target_msg.data = 1;
                pub_req_object_target.publish(req_object_target_msg);

                while(ros::ok() && !object_target_received){
                    usleep(500000);
                    ros::spinOnce();
                }
                object_target_received = false;
                
                if(ros::ok()){
                    cout << "Object and target detected ." << endl;
                    //snapshot of object and target positions at this moment for current manipulation trial. Not changed by "object_target_positions_callback".
                    object_in_base = object_in_base_live;
                    target_in_base = target_in_base_live;

                    std::cout << "==========================" << std::endl;
                    std::cout << "object location: "<< object_in_base.pose.position.x 
                    << ", " << object_in_base.pose.position.y << ", " 
                    << object_in_base.pose.position.z << std::endl;

                    std::cout << "target location: "<< target_in_base.pose.position.x 
                    << ", " << target_in_base.pose.position.y << ", " 
                    << target_in_base.pose.position.z << std::endl;

                    cout << "==========================================" << endl;
                }

                break;
            }
        #endif

        case 11:{

            cout << "monte carlo search tool experiment -> press enter to continue_";
            cin.ignore();

            //start event
            post_event(EvMonteCarloExpt());

            break;
        }
        
        case 12:{

            cout << "monte carlo search tool experiment" << endl;
            cout << "WITH OBSTACLE ";
            cout << "-> press enter to continue_";
            cin.ignore();

            //start event
            post_event(EvMonteCarloExpt2());

            break;
        }
        
        case 13:{

            cout << "monte carlo search tool experiment" << endl;
            cout << "WITH OBSTACLE ";
            cout << "BUT LEFT SIDE!!! ";
            cout << "-> press enter to continue_";
            cin.ignore();

            //start event
            post_event(EvMonteCarloExpt2_left());

            break;
        }


        case 16:{   
            cout << "go to home2 LOWER TORSO... press enter to continue" << endl;
            cin.ignore();
            Home2();
            break;
        }

        case 17:{

            cout << "monte carlo search tool experiment" << endl;
            cout << "WITH OBSTACLE ";
            cout << "BUT LEFT SIDE!!! ";
            cout << "\nVERSION 3.0!!! \n";
            cout << "-> press enter to continue_";
            cin.ignore();

            //start event
            post_event(EvMonteCarloExpt3_left());

            break;
        }

        case 18:
        {
            cout << "\nTESTING GO TO POSITION! \n";

            cout << "-> press enter to continue_";
            cin.ignore();

            Affine3d gohere;
            Matrix4d pos;
            pos <<  0,  0, 1, 0.57,
                   -1,  0, 0, 0.45,
                    0, -1, 0, 1.0,
                    0,  0, 0, 1;

            gohere.matrix() = pos;       

            if (ActionGoToPos(gohere, false, "task_single_right", true, "l_ee" ) >= 0)
                ROS_INFO_STREAM("movement okay~!\n");
            else
                ROS_ERROR_STREAM("failed to go to position");
            sleep(sleep_time);

            break;
        }

        case 19:
        {
            cout << "\nTESTING WAIST NECK TRUNK! \n";

            cout << "-> press enter to continue_";
            cin.ignore();

            WAIST_TOOL = 60.0;
            double TRUNK_PITCH_VAL = 65.0;
            double NECK_TILT_VAL = -13.0;
            double NECK_PAN_VAL = 0.4;
            double NECK_SPEED_VAL = 0.1;
    
            neck_and_waist_trunk( NECK_TILT_VAL, NECK_PAN_VAL, NECK_SPEED_VAL, WAIST_TOOL, TRUNK_PITCH_VAL);
            cout << "finished" << endl;
            break;
        }


        case 20:
        {
            cout << "\nTESTING PICK & PLACE! \n";
            cout << "-> press enter to continue_";
            cin.ignore();

            bool PICK_FLAG = true;
            //start event
            EvReachable(PICK_FLAG);
            break;
        }
        case 21:
        {
            cout << "\nTESTING OPENED HAND PUSH! \n";
            cout << "-> press enter to continue_";
            cin.ignore();

            bool PICK_FLAG = false;
            //start event
            EvReachable(PICK_FLAG);
            break;
        }
        case 22:
        {
            cout << "\nTESTING CLOSED HAND PUSH! \n";
            cout << "-> press enter to continue_";
            cin.ignore();

            //start event
            EvReachable2();
            break;
        }


        default:{
            std::cout << "Invalid task mode!\n";
            break;
        }

    }
}

Manipulate::~Manipulate() {
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::TestHandOrient(){
    //for testing only
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;

    target.link_name = "ee";

    tf::Quaternion tfQuat;
    tf::Vector3 tfVec;
    getCurrentTransform(target.link_name, tfQuat, tfVec);

    cout << tfQuat.getW() << endl;
    cout << tfQuat.getX() << endl;
    cout << tfQuat.getY() << endl;
    cout << tfQuat.getZ() << endl;

    target.end_eff.poses.resize(1);
    target.end_eff.poses[0].position.x = tfVec.getX();
    target.end_eff.poses[0].position.y = tfVec.getY();
    target.end_eff.poses[0].position.z = tfVec.getZ();

    target.end_eff.poses[0].orientation.w = 0.0;
    target.end_eff.poses[0].orientation.x = 0.0;
    target.end_eff.poses[0].orientation.y = 1.0;
    target.end_eff.poses[0].orientation.z = 0.0;
    cout << "test hand pose..." << endl;
    cout <<  target.end_eff.poses[0] << endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "----------------------------" << endl;
        cout << "test hand pose failed!" << endl;
        cout << "----------------------------" << endl;
    }

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::Home(){

    //======================================//
    //  move robot to start position        //
    //======================================//
    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;

    target_joint.torso_trajectory.joint_names.push_back("waist_yaw");
    target_joint.torso_trajectory.joint_names.push_back("waist_pitch");
    target_joint.torso_trajectory.joint_names.push_back("trunk_pitch");

    target_joint.right_trajectory.joint_names.push_back("right_shoulder_pitch");
    target_joint.right_trajectory.joint_names.push_back("right_shoulder_roll");
    target_joint.right_trajectory.joint_names.push_back("right_shoulder_yaw");
    target_joint.right_trajectory.joint_names.push_back("right_elbow");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_yaw");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_pitch");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_yaw2");

    target_joint.left_trajectory.joint_names.push_back("left_shoulder_pitch");
    target_joint.left_trajectory.joint_names.push_back("left_shoulder_roll");
    target_joint.left_trajectory.joint_names.push_back("left_shoulder_yaw");
    target_joint.left_trajectory.joint_names.push_back("left_elbow");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_yaw");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_pitch");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_yaw2");

    target_joint.head_trajectory.joint_names.push_back("neck_tilt");

    trajectory_msgs::JointTrajectoryPoint torso_joint;
    torso_joint.positions.resize(target_joint.torso_trajectory.joint_names.size());
    torso_joint.positions[0] = WAIST_INIT/180*3.14159265;//30.0
    torso_joint.positions[1] = -16.98/180*3.14159265;
    torso_joint.positions[2] = 47.44/180*3.14159265;
    target_joint.torso_trajectory.points.push_back(torso_joint);

    trajectory_msgs::JointTrajectoryPoint head_joint;
    head_joint.positions.resize(target_joint.head_trajectory.joint_names.size());
    head_joint.positions[0] = 13.3/180*3.14159265;
    target_joint.head_trajectory.points.push_back(head_joint);

    trajectory_msgs::JointTrajectoryPoint right_joint;
    right_joint.positions.resize(target_joint.right_trajectory.joint_names.size());

    right_joint.positions[0] = -39.49/180*3.14159265;
    right_joint.positions[1] = 27.88/180*3.14159265;
    right_joint.positions[2] = 38.56/180*3.14159265;
    right_joint.positions[3] = 144.10/180*3.14159265;
    right_joint.positions[4] = 71.18/180*3.14159265;
    right_joint.positions[5] = 8.96/180*3.14159265;
    right_joint.positions[6] = 64.62/180*3.14159265;
    target_joint.right_trajectory.points.push_back(right_joint);

    trajectory_msgs::JointTrajectoryPoint left_joint;
    left_joint.positions.resize(target_joint.left_trajectory.joint_names.size());
    left_joint.positions[0] =  right_joint.positions[0];
    left_joint.positions[1] = -right_joint.positions[1];
    left_joint.positions[2] = -right_joint.positions[2];
    left_joint.positions[3] =  right_joint.positions[3];
    left_joint.positions[4] = -right_joint.positions[4];
    left_joint.positions[5] =  right_joint.positions[5];
    left_joint.positions[6] = -right_joint.positions[6];
    target_joint.left_trajectory.points.push_back(left_joint);


    
    Speech("Moving to the starting position.");
    cout << "Moving to the home position...\n";
    moveit_j_ac.sendGoal(target_joint);
    moveit_j_ac.waitForResult(ros::Duration(30.0));
    result_joint = moveit_j_ac.getResult();
    sleep(sleep_time);
    if(result_joint->error_code == result_joint->FAIL) {
        std::cout << "Unable to reach home position\n";
    }else{
        cout << "Reached home position.\n";
    }

    #ifdef USE_NECK_PAN
    NeckPan(0.0, 0.5);
    cout << "neck pan reset to 0 deg position" << endl;
    #endif
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::Home(double waist_val){

    //======================================//
    //  move robot to start position        //
    //======================================//
    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;

    target_joint.torso_trajectory.joint_names.push_back("waist_yaw");
    target_joint.torso_trajectory.joint_names.push_back("waist_pitch");
    target_joint.torso_trajectory.joint_names.push_back("trunk_pitch");

    target_joint.right_trajectory.joint_names.push_back("right_shoulder_pitch");
    target_joint.right_trajectory.joint_names.push_back("right_shoulder_roll");
    target_joint.right_trajectory.joint_names.push_back("right_shoulder_yaw");
    target_joint.right_trajectory.joint_names.push_back("right_elbow");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_yaw");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_pitch");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_yaw2");

    target_joint.left_trajectory.joint_names.push_back("left_shoulder_pitch");
    target_joint.left_trajectory.joint_names.push_back("left_shoulder_roll");
    target_joint.left_trajectory.joint_names.push_back("left_shoulder_yaw");
    target_joint.left_trajectory.joint_names.push_back("left_elbow");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_yaw");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_pitch");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_yaw2");

    target_joint.head_trajectory.joint_names.push_back("neck_tilt");

    trajectory_msgs::JointTrajectoryPoint torso_joint;
    torso_joint.positions.resize(target_joint.torso_trajectory.joint_names.size());
    torso_joint.positions[0] = waist_val/180*3.14159265;
    torso_joint.positions[1] = -16.98/180*3.14159265;
    torso_joint.positions[2] = 47.44/180*3.14159265;
    target_joint.torso_trajectory.points.push_back(torso_joint);

    trajectory_msgs::JointTrajectoryPoint head_joint;
    head_joint.positions.resize(target_joint.head_trajectory.joint_names.size());
    head_joint.positions[0] = 13.3/180*3.14159265;
    target_joint.head_trajectory.points.push_back(head_joint);

    trajectory_msgs::JointTrajectoryPoint right_joint;
    right_joint.positions.resize(target_joint.right_trajectory.joint_names.size());

    right_joint.positions[0] = -39.49/180*3.14159265;
    right_joint.positions[1] = 27.88/180*3.14159265;
    right_joint.positions[2] = 38.56/180*3.14159265;
    right_joint.positions[3] = 144.10/180*3.14159265;
    right_joint.positions[4] = 71.18/180*3.14159265;
    right_joint.positions[5] = 8.96/180*3.14159265;
    right_joint.positions[6] = 64.62/180*3.14159265;
    target_joint.right_trajectory.points.push_back(right_joint);

    trajectory_msgs::JointTrajectoryPoint left_joint;
    left_joint.positions.resize(target_joint.left_trajectory.joint_names.size());
    left_joint.positions[0] =  right_joint.positions[0];
    left_joint.positions[1] = -right_joint.positions[1];
    left_joint.positions[2] = -right_joint.positions[2];
    left_joint.positions[3] =  right_joint.positions[3];
    left_joint.positions[4] = -right_joint.positions[4];
    left_joint.positions[5] =  right_joint.positions[5];
    left_joint.positions[6] = -right_joint.positions[6];
    target_joint.left_trajectory.points.push_back(left_joint);


    
    Speech("Moving to the starting position.");
    cout << "Moving to the home position...\n";
    moveit_j_ac.sendGoal(target_joint);
    moveit_j_ac.waitForResult(ros::Duration(30.0));
    result_joint = moveit_j_ac.getResult();
    sleep(sleep_time);
    if(result_joint->error_code == result_joint->FAIL) {
        std::cout << "Unable to reach home position\n";
        return -1;
    }else{
        cout << "Reached home position.\n";
    }

    #ifdef USE_NECK_PAN
    NeckPan(0.0, 0.5);
    cout << "neck pan reset to 0 deg position" << endl;
    #endif

    return 0;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::Home2(double waist_val){

    //======================================//
    //  move robot to start position        //
    //======================================//
    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;

    target_joint.torso_trajectory.joint_names.push_back("waist_yaw");
    target_joint.torso_trajectory.joint_names.push_back("waist_pitch");
    target_joint.torso_trajectory.joint_names.push_back("trunk_pitch");

    target_joint.right_trajectory.joint_names.push_back("right_shoulder_pitch");
    target_joint.right_trajectory.joint_names.push_back("right_shoulder_roll");
    target_joint.right_trajectory.joint_names.push_back("right_shoulder_yaw");
    target_joint.right_trajectory.joint_names.push_back("right_elbow");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_yaw");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_pitch");
    target_joint.right_trajectory.joint_names.push_back("right_wrist_yaw2");

    target_joint.left_trajectory.joint_names.push_back("left_shoulder_pitch");
    target_joint.left_trajectory.joint_names.push_back("left_shoulder_roll");
    target_joint.left_trajectory.joint_names.push_back("left_shoulder_yaw");
    target_joint.left_trajectory.joint_names.push_back("left_elbow");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_yaw");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_pitch");
    target_joint.left_trajectory.joint_names.push_back("left_wrist_yaw2");

    target_joint.head_trajectory.joint_names.push_back("neck_tilt");

    trajectory_msgs::JointTrajectoryPoint torso_joint;
    torso_joint.positions.resize(target_joint.torso_trajectory.joint_names.size());
    torso_joint.positions[0] = waist_val/180.0*3.14159265;
    torso_joint.positions[1] = -30.0/180.0 *3.14159265;
    torso_joint.positions[2] = 80.0/180.0 *3.14159265;
    target_joint.torso_trajectory.points.push_back(torso_joint);

    trajectory_msgs::JointTrajectoryPoint head_joint;
    head_joint.positions.resize(target_joint.head_trajectory.joint_names.size());
    head_joint.positions[0] = -9.0/180*3.14159265;
    target_joint.head_trajectory.points.push_back(head_joint);

    trajectory_msgs::JointTrajectoryPoint right_joint;
    right_joint.positions.resize(target_joint.right_trajectory.joint_names.size());
    
    right_joint.positions[0] = -0.63552052;
    right_joint.positions[1] =  0.55662557;
    right_joint.positions[2] =  0.94779608;
    right_joint.positions[3] =  2.52891371;
    right_joint.positions[4] =  1.17775187;
    right_joint.positions[5] =  0.28117340;
    right_joint.positions[6] =  1.43947069;

    target_joint.right_trajectory.points.push_back(right_joint);

    trajectory_msgs::JointTrajectoryPoint left_joint;
    left_joint.positions.resize(target_joint.left_trajectory.joint_names.size());
    left_joint.positions[0] =  right_joint.positions[0];
    left_joint.positions[1] = -right_joint.positions[1];
    left_joint.positions[2] = -right_joint.positions[2];
    left_joint.positions[3] =  right_joint.positions[3];
    left_joint.positions[4] = -right_joint.positions[4];
    left_joint.positions[5] =  right_joint.positions[5];
    left_joint.positions[6] = -right_joint.positions[6];
    target_joint.left_trajectory.points.push_back(left_joint);


    
    Speech("Moving to the starting position.");
    cout << "Moving to the home position...\n";
    moveit_j_ac.sendGoal(target_joint);
    moveit_j_ac.waitForResult(ros::Duration(30.0));
    result_joint = moveit_j_ac.getResult();
    sleep(sleep_time);
    if(result_joint->error_code == result_joint->FAIL) {
        std::cout << "Unable to reach home position\n";
        return -1;
    }else{
        cout << "Reached home position.\n";
        //current_tilt_angle = 13.3;
    }

    #ifdef USE_NECK_PAN
    NeckPan(0.0, 0.5);
    cout << "neck pan reset to 0 deg position" << endl;
    #endif

    return 0;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::Speech(std::string string){
  #ifdef USE_SPEECH
    speech.say(string,"voice_cmu_us_slt_arctic_clunits");
    sleep(3.0);
  #endif
}

void Manipulate::ResetPuck(){

    int randmax = 100;
    double varmax = 0.02;
    double noiseX =  varmax* (2.0*(rand() % randmax + 1)/((double)randmax) - 1.0);
    double noiseY =  varmax* (2.0*(rand() % randmax + 1)/((double)randmax) - 1.0);

    cout<< "noiseX = "  << noiseX << endl;
    cout<< "noiseY = "  << noiseY << endl;

    geometry_msgs::Pose puck_pose;
    puck_pose.position.x = object_in_base.pose.position.x + noiseX ;
    puck_pose.position.y = object_in_base.pose.position.y + noiseY;
    puck_pose.position.z = object_in_base.pose.position.z;

    puck_pose.orientation.x = 0.0;
    puck_pose.orientation.y = 0.0;
    puck_pose.orientation.z = 0.0;
    puck_pose.orientation.w = 1.0;

    geometry_msgs::Twist puck_twist;
    puck_twist.linear.x = 0.0;
    puck_twist.linear.y = 0.0;
    puck_twist.linear.z = 0.0;
    puck_twist.angular.x = 0.0;
    puck_twist.angular.y = 0.0;
    puck_twist.angular.z = 0.0;

    gazebo_msgs::LinkState linkstate;
    linkstate.link_name = (std::string) "puck_link";
    linkstate.reference_frame = (std::string) "world";
    linkstate.pose = puck_pose;
    linkstate.twist = puck_twist;

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    gazebo_msgs::SetLinkState setlinkstate;
    setlinkstate.request.link_state = linkstate;
    client.call(setlinkstate);

    cout << "Puck is reset to original position (with noise).\n";
    cout << "puck position x = " << puck_pose.position.x <<  endl;
    cout << "puck position y = " << puck_pose.position.y <<  endl;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::ResetObstacle()
{

    geometry_msgs::Pose obstacle_pose;
    obstacle_pose.position.x = obstacle_in_base.pose.position.x;
    obstacle_pose.position.y = obstacle_in_base.pose.position.y;
    obstacle_pose.position.z = obstacle_in_base.pose.position.z;
    obstacle_pose.orientation.x = 0.0;
    obstacle_pose.orientation.y = 0.0;
    obstacle_pose.orientation.z = 0.0;
    obstacle_pose.orientation.w = 1.0;

    geometry_msgs::Twist obstacle_twist;
    obstacle_twist.linear.x = 0.0;
    obstacle_twist.linear.y = 0.0;
    obstacle_twist.linear.z = 0.0;
    obstacle_twist.angular.x = 0.0;
    obstacle_twist.angular.y = 0.0;
    obstacle_twist.angular.z = 0.0;

    gazebo_msgs::LinkState linkstate;
    linkstate.link_name = (std::string) "obstacle_link";
    linkstate.reference_frame = (std::string) "world";
    linkstate.pose = obstacle_pose;
    linkstate.twist = obstacle_twist;

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    gazebo_msgs::SetLinkState setlinkstate;
    setlinkstate.request.link_state = linkstate;
    client.call(setlinkstate);

    cout << "Obstacle is reset to original position.\n";
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::ResetTool()
{

    geometry_msgs::Pose tool_pose;
    tool_pose.position = tool_in_base.pose.position;
    tool_pose.orientation = tool_in_base.pose.orientation;

    geometry_msgs::Twist tool_twist;
    tool_twist.linear.x = 0.0;
    tool_twist.linear.y = 0.0;
    tool_twist.linear.z = 0.0;
    tool_twist.angular.x = 0.0;
    tool_twist.angular.y = 0.0;
    tool_twist.angular.z = 0.0;

    gazebo_msgs::LinkState linkstate;
    linkstate.link_name = (std::string) "tool_link";
    linkstate.reference_frame = (std::string) "world";
    linkstate.pose = tool_pose;
    linkstate.twist = tool_twist;

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    gazebo_msgs::SetLinkState setlinkstate;
    setlinkstate.request.link_state = linkstate;
    client.call(setlinkstate);

    cout << "tool is reset to original position.\n";
}



//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::DrawCircle(){
    //for testing only
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;

    int N=100;
    double radius = 0.05;
    target.end_eff.poses.resize(N);
    target.link_name = "ee";

    tf::Quaternion tfQuat;
    tf::Vector3 tfVec;
    getCurrentTransform(target.link_name, tfQuat, tfVec);

    for(int i=0; i<N; i++){
        target.end_eff.poses[i].position.x = tfVec.getX()+0.07;
        target.end_eff.poses[i].position.y = tfVec.getY()+0.05 - radius + radius*cos(2.0*3.14159*(double)i/(double)N);
        target.end_eff.poses[i].position.z = tfVec.getZ()-0.05 + radius*sin(2.0*3.14159*(double)i/(double)N);
        target.end_eff.poses[i].orientation.w = 0.5;
        target.end_eff.poses[i].orientation.x = 0.5;
        target.end_eff.poses[i].orientation.y = 0.5;
        target.end_eff.poses[i].orientation.z = 0.5;

    }
    cout << "drawing circle..." << endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "----------------------------" << endl;
        cout << "draw circle failed!" << endl;
        cout << "----------------------------" << endl;
    }

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvPerceiveObject & ) {
    CheckObjectLocation();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::CheckToolLocation(){
    tool_handle_received = false;
    std_msgs::Int32 tool_msg;
    tool_msg.data = functionality;
    pub_req_tool_handle.publish(tool_msg);
    cout << "Waiting for tool to be detected..." << endl;

    double increment = -5.0;
    while(ros::ok() && !(tool_handle_received && sticklength_received)){

        //below is to pan and search for the tool
        for(int i =0; i < 10; i++){
            usleep(500000);
            ros::spinOnce();
        }

        if((current_pan_angle + increment) > 0.0 || (current_pan_angle + increment) < -30.0)
            increment = increment * -1.0;
        else{
            if(!(tool_handle_received && sticklength_received)){
                cout << "panning neck to " << current_pan_angle+increment << " for searching" << endl;
                
                #ifdef USE_NECK_PAN
                    NeckPan(current_pan_angle + increment, 0.2);
                #endif
            }
        }
    }

    if(ros::ok()){
        Speech("Yes, I have detected a potential tool!");
        cout << "Tool handle detected." << endl;
        tool_handle_received = false;
        sticklength_received = false;
        tool_in_base = tool_in_base_live;

        tool_in_base.pose.position.z = TABLE_HEIGHT + TOOL_HANDLE_HEIGHT - 0.01;
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::CheckBendToolLocation(){
    bendtool_received = false;
    cout << "Waiting for bend tool to be detected..." << endl;

    while(ros::ok() && !bendtool_received){
        usleep(500000);
        ros::spinOnce();
    }
    if(ros::ok()){
        cout << "Bend Tool detected." << endl;
        bendtool_received = false;
        bendtool_in_base = bendtool_in_base_live;
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::CheckObjectLocation(){

    std_msgs::Int32 req_object_target_msg;
    req_object_target_msg.data = 1;

    
    Speech("Please place the object and the goal...");
    sleep(0.5);
    Speech("And let me know when you are ready...");
    cout << "Place object and target. When ready, press Enter to Continue";
    cin.ignore();
    
    Speech("I am detecting the object... and the goal ...");
    pub_req_object_target.publish(req_object_target_msg);               
    cout << "Waiting for object and target to be detected..." << endl;

    double increment = -5.0;
    while(ros::ok() && !object_target_received){
        
        for(int i =0; i < 10; i++){
            usleep(500000);
            ros::spinOnce();
        }

    }
    
    object_target_received = false;



    if(ros::ok()){

        Speech("Found them...");
        cout << "Object and target detected ." << endl;
        
        //snapshot of object and target positions at this moment for current manipulation trial. Not changed by "object_target_positions_callback".
        object_in_base = object_in_base_live;
        target_in_base = target_in_base_live;

	    if(obstacle_detected)
	        obstacle_in_base = obstacle_in_base_live;
	    else
	    {
	        ROS_INFO_STREAM("Obstacle NOT detected!!! \n");

	        obstacle_in_base.pose.position.x = 5.0;   
	        obstacle_in_base.pose.position.y = 5.0;
	    }

        object_in_base.pose.position.z = 0.88 - 0.07;          
        target_in_base.pose.position.z = 0.88 - 0.07;     
        obstacle_in_base.pose.position.z = 0.88 - 0.07;     

        object_in_base.pose.orientation.x = object_in_base.pose.orientation.y = object_in_base.pose.orientation.z = 0;
        object_in_base.pose.orientation.w = 1;
        target_in_base.pose.orientation.x = target_in_base.pose.orientation.y = target_in_base.pose.orientation.z = 0;
        target_in_base.pose.orientation.w = 1;
        obstacle_in_base.pose.orientation.x = obstacle_in_base.pose.orientation.y = obstacle_in_base.pose.orientation.z = 0;
        obstacle_in_base.pose.orientation.w = 1;
	    //---------------------------


        double ex = fabs(object_in_base.pose.position.x - target_in_base.pose.position.x);
        double ey = fabs(object_in_base.pose.position.y - target_in_base.pose.position.y);
        double emin = ex;
        if(ey < emin)
            emin = ey;
        if(object_in_base.pose.position.z>0.9 || target_in_base.pose.position.z<0.72){
            Speech("Sorry");
            Speech("The object... or the goal... is not properly placed...");
            sleep(1.0);
            Speech( "Please align them ... along the X ... or Y ... direction.");
            sleep(1.5);
            cout << "Invalid object goal configuration." << endl;
            post_event(EvPerceiveObject());
            return;
        }

        bool sim_robot = false;
        n.getParam("/sim_robot",sim_robot);
        if(sim_robot)
        {
            ResetPuck();
            ResetObstacle();
        }

        if(!using_tool){

            float target_tol = 0.05;
            std::cout << "==========================" << std::endl;
            std::cout << "object location: "<< object_in_base.pose.position.x 
                        << ", " << object_in_base.pose.position.y << ", " 
                        << object_in_base.pose.position.z << std::endl;

            std::cout << "target location: "<< target_in_base.pose.position.x 
                        << ", " << target_in_base.pose.position.y << ", " 
                        << target_in_base.pose.position.z << std::endl;

            cout << "==========================================" << endl;
            cout << ">>>>> cal angle of obj to target <<<<<" << endl;
            //calculate the angle of object to target
            double y_dif = -(target_in_base.pose.position.y - object_in_base.pose.position.y);
            double x_dif = (target_in_base.pose.position.x - object_in_base.pose.position.x);
            cout << "x dif=" << x_dif << ", y_dif=" << y_dif << endl; 
            //south angle is negative
            radian = std::atan2(x_dif,y_dif);       //atan2 is -180 to 180 range; 
            //round off to int, double hard to sort into range
            angle = round(radian * 180.0 / PI); //in degree;

            ROS_WARN("\nangle obj2tar: deg= %f , radian= %f", angle, radian);

            cout << "==========================================" << endl;

            // TurnHeadToObjTarCenter();

             //transit< ReadyState >();

            if(fabs(y_dif) < target_tol && fabs(x_dif) < target_tol)
            {
                std::cout << "Object already at target!!!" <<std::endl;
                post_event(EvReady());
            }
            else{
                if(angle > 160.0){
                    cout << "move in Western direction" << endl;       
                    post_event(EvTargetNofObject());
                }
                else if(angle >= 20.0 && angle <= 160.0){
                    cout << "move in Northern direction" << endl;       
                    post_event(EvTargetOmniObject());
                }
                else if(angle < 20.0 && angle >=0){
                    cout << "move in Eastern direction" << endl;       
                    post_event(EvTargetNofObject());
                }
                else if(angle < 0 && angle >= -20.0){
                    cout << "move in Eastern direction" << endl;       
                    post_event(EvTargetSofObject());
                }
                else if(angle < -20.0 && angle >= -160.0){
                    cout << "move in Southern direction" << endl;       
                    post_event(EvTargetSofObject());
                }
                else{
                    cout << "move in Western direction" << endl;       
                    post_event(EvTargetSofObject());
                }
            }//check target location to object
        }//end !usetool


    }//end ros::ok
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

vector<geometry_msgs::Point> Manipulate::load_boundary(const int functionality){
    ifstream file;
    string filename;
    //functionality-- 1:forw, 2:back , 3:side
    switch(functionality){
        case 1:{
            filename = "/home/olivia/catkin_workspace/src/drc_hubo_apps/tool_expt/src/boundary/boundary_forw.txt";
            cout << "loading boundary data for push-forward hand pose..." << endl;
            break;
        }
        case 2:{
            filename = "/home/olivia/catkin_workspace/src/drc_hubo_apps/tool_expt/src/boundary/boundary_back.txt";
            cout << "loading boundary data for pull-back hand pose..." << endl;
            break;
        }
        case 3:{
            filename = "/home/olivia/catkin_workspace/src/drc_hubo_apps/tool_expt/src/boundary/boundary_side.txt";
            cout << "loading boundary data for push-sideways hand pose..." << endl;
            break;
        }
        default:{
            cout << "Unknown task. Cannot load boundary data."<< endl;
        }
    }
    vector<geometry_msgs::Point> boundary;
    geometry_msgs::Point boundary_pt;
    file.open(filename.c_str());

    if (file.is_open() ){
        cout << filename << " is open. Reading file." <<endl;
        string line;

        while(getline(file, line)){
            stringstream ss(line);
            vector<double> tmpvec;
            double dat;
            while(ss >> dat)
                tmpvec.push_back(dat);
            boundary_pt.x = tmpvec.at(0);
            boundary_pt.y = tmpvec.at(1);
            boundary_pt.z = tmpvec.at(2);
            boundary.push_back(boundary_pt);
        }
        file.close();
        cout << "Finished reading and closed file. Boundary data loaded successfully. " << endl;
    }
    else{
        cout << "unable to open " << filename.c_str() <<endl;
//        return NULL;
    }

    return boundary;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

double distance_btw_pts(const geometry_msgs::Point P1, const geometry_msgs::Point P2){
    return sqrt(pow(P1.x-P2.x, 2) + pow(P1.y-P2.y, 2) + pow(P1.z-P2.z, 2));
}



//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

double Manipulate::compute_min_aug(const vector<geometry_msgs::Point> boundary, const geometry_msgs::Point object, const geometry_msgs::Point target, const bool object_within_reach, const bool target_within_reach)
{
    double min_aug = 0.0;
    double min_aug_obj = 1e6;
    double min_aug_tar = 1e6;
    double dist;

    if(!object_within_reach || !target_within_reach){
        for(int i=0; i<boundary.size(); i++){
            if(!object_within_reach){
                dist = distance_btw_pts(boundary.at(i), object);
                if(dist<min_aug_obj)
                    min_aug_obj = dist;
            }
            if(!target_within_reach){
                dist = distance_btw_pts(boundary.at(i), target);
                if(dist<min_aug_tar)
                    min_aug_tar = dist;
            }


        }
        min_aug = min_aug_obj;
        if(min_aug > min_aug_tar)
            min_aug = min_aug_tar;
        cout << "min_aug_obj = " << min_aug_obj << endl;
        cout << "min_aug_tar = " << min_aug_tar << endl;
        cout << "min_aug = " << min_aug << endl;
    }

    return min_aug;

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

m3_moveit::MoveitWholeBodyGoal Manipulate::SetJointTargetsToCurrent(){

    usleep(500000);
    ros::spinOnce();

    if(!joint_state.name.empty()){
        m3_moveit::MoveitWholeBodyGoal target_joint;
        target_joint = fillJointNamesFull();
        trajectory_msgs::JointTrajectoryPoint torso_joint;
        torso_joint.positions.resize(target_joint.torso_trajectory.joint_names.size());
        trajectory_msgs::JointTrajectoryPoint head_joint;
        head_joint.positions.resize(target_joint.head_trajectory.joint_names.size());
        trajectory_msgs::JointTrajectoryPoint right_joint;
        right_joint.positions.resize(target_joint.right_trajectory.joint_names.size());
        trajectory_msgs::JointTrajectoryPoint left_joint;
        left_joint.positions.resize(target_joint.left_trajectory.joint_names.size());


        for(int i=0; i<joint_state.name.size(); i++){
            if (joint_state.name.at(i) == "waist_yaw")
                torso_joint.positions[0] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "waist_pitch")
                torso_joint.positions[1] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "trunk_pitch")
                torso_joint.positions[2] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "right_shoulder_pitch")
                right_joint.positions[0] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "right_shoulder_roll")
                right_joint.positions[1] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "right_shoulder_yaw")
                right_joint.positions[2] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "right_elbow")
                right_joint.positions[3] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "right_wrist_yaw")
                right_joint.positions[4] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "right_wrist_pitch")
                right_joint.positions[5] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "right_wrist_yaw2")
                right_joint.positions[6] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "left_shoulder_pitch")
                left_joint.positions[0] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "left_shoulder_roll")
                left_joint.positions[1] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "left_shoulder_yaw")
                left_joint.positions[2] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "left_elbow")
                left_joint.positions[3] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "left_wrist_yaw")
                left_joint.positions[4] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "left_wrist_pitch")
                left_joint.positions[5] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "left_wrist_yaw2")
                left_joint.positions[6] = joint_state.position.at(i);
            else if (joint_state.name.at(i) == "neck_tilt")
                head_joint.positions[0] = joint_state.position.at(i);
        }

        target_joint.torso_trajectory.points.push_back(torso_joint);
        target_joint.head_trajectory.points.push_back(head_joint);
        target_joint.right_trajectory.points.push_back(right_joint);
        target_joint.left_trajectory.points.push_back(left_joint);

        return target_joint;
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::MoveWaistFromCurrent(double angle_deg){

    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;

    target_joint = SetJointTargetsToCurrent();

    if(!joint_state.name.empty()){
        target_joint.torso_trajectory.points[0].positions[0] = (angle_deg/180*3.14159265);

        cout << "Moving to waist yaw angle from current...\n";
        moveit_j_ac.sendGoal(target_joint);
        moveit_j_ac.waitForResult(ros::Duration(30.0));
        result_joint = moveit_j_ac.getResult();
        sleep(sleep_time);
        if(result_joint->error_code == result_joint->FAIL) {
            std::cout << "Unable to reach waist yaw angle!\n";
        }else{
            cout << "Reached waist yaw angle.\n";
        }
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::MoveWaistTrunkFromCurrent(double angle_deg, double angle_deg_2){


    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;

    target_joint = SetJointTargetsToCurrent();

    if(!joint_state.name.empty()){
        target_joint.torso_trajectory.points[0].positions[0] = (angle_deg/180.0*3.14159265);
        target_joint.torso_trajectory.points[0].positions[2] = (angle_deg_2/180.0*3.14159265);

        cout << "Moving to waist yaw angle from current...\n";
        moveit_j_ac.sendGoal(target_joint);
        moveit_j_ac.waitForResult(ros::Duration(30.0));
        result_joint = moveit_j_ac.getResult();
        sleep(sleep_time);
        if(result_joint->error_code == result_joint->FAIL) {
            std::cout << "Unable to reach waist yaw angle!\n";
        }else{
            cout << "Reached waist yaw angle.\n";
        }
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::MoveWaist(double angle_deg){
    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;

    target_joint = fillJointNamesFull();

    trajectory_msgs::JointTrajectoryPoint torso_joint;
    torso_joint.positions.resize(target_joint.torso_trajectory.joint_names.size());
    torso_joint.positions[0] = angle_deg/180*3.14159265;
    torso_joint.positions[1] = -16.9/180*3.14159265;
    torso_joint.positions[2] = 47.44/180*3.14159265;
    target_joint.torso_trajectory.points.push_back(torso_joint);

    trajectory_msgs::JointTrajectoryPoint head_joint;
    head_joint.positions.resize(target_joint.head_trajectory.joint_names.size());
    head_joint.positions[0] = 13.3/180*3.14159265;
    target_joint.head_trajectory.points.push_back(head_joint);

    trajectory_msgs::JointTrajectoryPoint right_joint;
    right_joint.positions.resize(target_joint.right_trajectory.joint_names.size());

    right_joint.positions[0] = -39.49/180*3.14159265;
    right_joint.positions[1] = 27.88/180*3.14159265;
    right_joint.positions[2] = 38.56/180*3.14159265;
    right_joint.positions[3] = 144.10/180*3.14159265;
    right_joint.positions[4] = 71.18/180*3.14159265;
    right_joint.positions[5] = 8.96/180*3.14159265;
    right_joint.positions[6] = 64.62/180*3.14159265;
    target_joint.right_trajectory.points.push_back(right_joint);

    trajectory_msgs::JointTrajectoryPoint left_joint;
    left_joint.positions.resize(target_joint.left_trajectory.joint_names.size());
    left_joint.positions[0] =  right_joint.positions[0];
    left_joint.positions[1] = -right_joint.positions[1];
    left_joint.positions[2] = -right_joint.positions[2];
    left_joint.positions[3] =  right_joint.positions[3];
    left_joint.positions[4] = -right_joint.positions[4];
    left_joint.positions[5] =  right_joint.positions[5];
    left_joint.positions[6] = -right_joint.positions[6];
    target_joint.left_trajectory.points.push_back(left_joint);

    cout << "Moving to waist yaw angle!!!...\n";
    moveit_j_ac.sendGoal(target_joint);
    moveit_j_ac.waitForResult(ros::Duration(30.0));
    result_joint = moveit_j_ac.getResult();
    sleep(sleep_time);
    if(result_joint->error_code == result_joint->FAIL) {
        std::cout << "Unable to reach waist yaw angle!\n";
    }else{
        cout << "Reached waist yaw angle.\n";
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


bool Manipulate::isReachable(geometry_msgs::Pose pose){

//====================TODO : Change to compute not only pullback reachability, but also the other 2. Possibly use reach boundaries. ====================

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;

    if(pose.position.x > 0.0){
        target.end_eff.poses.resize(1);
        if(pose.position.y <= 0.0){
            target.link_name = "ee";
            target.end_eff.poses[0].position = pullback_offset_right_hand;
            target.end_eff.poses[0].orientation = pullback_orientation_right_hand;
        }
        else if(pose.position.y > 0.0){
            target.link_name = "l_ee";
            target.end_eff.poses[0].position = pullback_offset_left_hand;
            target.end_eff.poses[0].orientation = pullback_orientation_left_hand;
        }
        target.end_eff.poses[0].position.x += object_in_base.pose.position.x ;
        target.end_eff.poses[0].position.y += object_in_base.pose.position.y ;
        target.end_eff.poses[0].position.z += object_in_base.pose.position.z ;
        target.plan_only = true;
        ac.sendGoal(target);
        target.plan_only = false;
        ac.waitForResult(ros::Duration(20.0));
        result = ac.getResult();
        if(result->error_code == result->FAIL){
            cout << "----------------------------" << endl;
            cout << "Not reachable by robot hand:" << endl;
            cout << pose << endl;
            cout << "----------------------------" << endl;
            using_tool = true;
            return false;
        }
        else
            return true;

    }
    else{
        std::cout << "Object or target is behind the robot. Not handling this case. Aborting." <<std::endl;
        return false;
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::OpenFingers(const string side, int mode){

    //mode in action lib: 0= stop fingers wtout tool; 1= stop fingers wt tool
    #ifdef USE_FINGER_ENC
    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers_encoders", true);
    #else
    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
    #endif
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    target_fingers.trajectory.joint_names = fillJointNamesFingers(side);
    target_fingers.side = side;

    trajectory_msgs::JointTrajectoryPoint finger_joints;
    finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());
        
    #ifdef USE_FINGER_ENC
    finger_joints.positions[0] = 33.0;
    finger_joints.positions[1] = 33.0;
    finger_joints.positions[2] = 0.0;
    finger_joints.positions[3] = -33.0;
    finger_joints.positions[4] = -33.0;
    finger_joints.positions[5] = 0.0;
    #else 
    //use below position if not using encoder
    finger_joints.positions[0] = 1.3;
    finger_joints.positions[1] = 1.59;
    finger_joints.positions[2] = 0.0;
    finger_joints.positions[3] = -1.3;
    finger_joints.positions[4] = -1.59;
    finger_joints.positions[5] = 0.0;
    #endif
    

    target_fingers.trajectory.points.push_back(finger_joints);

    std::cout << "Opening fingers in " << side << " hand..." << std::endl;

    target_fingers.mode = mode;
    ac_fingers.sendGoal(target_fingers);
    ac_fingers.waitForResult(ros::Duration(20.0)); //will w8 till receive result

    result_fingers = ac_fingers.getResult();
    if(result_fingers->error_code == result_fingers->FAIL){
        cout << "Opening fingers in " << side << " hand FAILED!" <<  std::endl;
        
        std_msgs::Bool failed;
        failed.data = true;
        fingerFailPub.publish(failed);
        
        return -1;
    }
    else{
        cout << "Opening fingers in " << side << " DONE" <<  std::endl;
        return 1;
    }
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::CloseFingers(const string side, int grasp_type, int mode){

    //grasp_type: 0=none, 1=tool, 2=bendtool
    //mode_type: 0= wtout tool, 1= wt tool
    trajectory_msgs::JointTrajectoryPoint finger_joints;
    #ifdef USE_FINGER_ENC
    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers_encoders", true);
    #else
    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
    #endif
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    target_fingers.trajectory.joint_names = fillJointNamesFingers(side);
    target_fingers.side = side;

    finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());
    
    #ifdef USE_FINGER_ENC
    finger_joints.positions[0] = 1.0;
    finger_joints.positions[1] = 1.0;
    finger_joints.positions[2] = 0.0;
    finger_joints.positions[3] = -finger_joints.positions[0];
    finger_joints.positions[4] = -finger_joints.positions[1];
    finger_joints.positions[5] = -finger_joints.positions[2];
    #else
    finger_joints.positions[0] = 0.0;
    finger_joints.positions[1] = 0.0;
    finger_joints.positions[2] = 0.0;
    finger_joints.positions[3] = -finger_joints.positions[0];
    finger_joints.positions[4] = -finger_joints.positions[1];
    finger_joints.positions[5] = -finger_joints.positions[2];
    #endif

    target_fingers.trajectory.points.clear();
    target_fingers.trajectory.points.push_back(finger_joints);
    if(grasp_type==1 || grasp_type==2)
        target_fingers.grasp_tool = true;   //set grasp_tool to true check action cb
    target_fingers.grasp_type = grasp_type;

    std::cout << "Closing fingers in " << side << " hand... " << std::endl;

    target_fingers.mode = mode;
    ac_fingers.sendGoal(target_fingers);
    ac_fingers.waitForResult(ros::Duration(20.0));
    result_fingers = ac_fingers.getResult();
    if(result_fingers->error_code == result_fingers->FAIL){
        cout << "Closing fingers in " << side << "  hand FAILED!\n";
        
        std_msgs::Bool failed;
        failed.data = true;
        fingerFailPub.publish(failed);
        
        return -1;
    }
    else{
        cout << "Closing fingers in " << side << "  hand DONE. \n";
        return 1;
    }
}

//------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::MoveFingers(const string side, double fraction, int mode){

        //mode_type: 0= wtout tool, 1= wt tool
    if(fraction>1){
        fraction=1;
        std::cout << "fraction cannot be greater than 1. Setting to 1." << std::endl;
    }
    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    target_fingers.trajectory.joint_names = fillJointNamesFingers(side);
    target_fingers.side = side;
    

    trajectory_msgs::JointTrajectoryPoint finger_joints;
    finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());
    finger_joints.positions[0] = 1.3*fraction;
    finger_joints.positions[1] = 1.59*fraction;
    finger_joints.positions[2] = 0.6*fraction;
    finger_joints.positions[3] = -finger_joints.positions[0];
    finger_joints.positions[4] = -finger_joints.positions[1];
    finger_joints.positions[5] = -finger_joints.positions[2];

    target_fingers.trajectory.points.push_back(finger_joints);


    std::cout << "Moving fingers in " << side << " hand to " << fraction*100.0 << " percent" << std::endl;
    for(int i; i<target_fingers.trajectory.joint_names.size(); i++)
        std::cout << target_fingers.trajectory.joint_names[i] << std::endl;

    target_fingers.mode = mode;
    ac_fingers.sendGoal(target_fingers);
    ac_fingers.waitForResult(ros::Duration(20.0));
    result_fingers = ac_fingers.getResult();
    if(result_fingers->error_code == result_fingers->FAIL){
        cout << "Moving fingers in " << side << " hand FAILED!" <<  std::endl;
        return -1;
    }
    else{
        cout << "Moving fingers in " << side << " DONE" <<  std::endl;
        return 1;
    }
}


//------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::pinchFingers(const string side, double diameter_in_cm, int mode){

    cout << ">> Pinching <<" << endl;

    //mode_type: 0= wtout tool, 1= wt tool
    #ifdef USE_FINGER_ENC

    //linear interpolation
    if( diameter_in_cm < 3.0 || diameter_in_cm > 11.0){
        cout << "using linear interpolation for obj diameter of 3cm to 11cm" << endl;
        cout << "The input object diameter is outside of linear interpolation range" << endl;
        return -1;
    }
    
    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers_encoders", true);
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    target_fingers.trajectory.joint_names = fillJointNamesFingers(side);
    target_fingers.side = side;
    
    double encoder_val = 0.7373452465 * diameter_in_cm + 20.289836858;
    //encoder_val += 0.2; //be more conservative
    
    std::cout << "Moving fingers in " << side << " hand to encoder value = " << encoder_val << " " << std::endl;
    
    trajectory_msgs::JointTrajectoryPoint finger_joints;
    finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());
    
    finger_joints.positions[0] = encoder_val;
    finger_joints.positions[1] = encoder_val;
    finger_joints.positions[2] = 0.0;
    finger_joints.positions[3] = -finger_joints.positions[0];
    finger_joints.positions[4] = -finger_joints.positions[1];
    finger_joints.positions[5] = -finger_joints.positions[2];

    target_fingers.trajectory.points.push_back(finger_joints);

    target_fingers.mode = mode;
    ac_fingers.sendGoal(target_fingers);
    ac_fingers.waitForResult(ros::Duration(20.0));
    result_fingers = ac_fingers.getResult();
    
    if(result_fingers->error_code == result_fingers->FAIL){
        cout << "pinching using fingers in " << side << " hand FAILED!" <<  std::endl;
        
        std_msgs::Bool failed;
        failed.data = true;
        fingerFailPub.publish(failed);
        
        return -1;
    }
    else{
        cout << "pinching using fingers in " << side << " DONE" <<  std::endl;
        return 1;
    }
    
    #else 
        cout << "this pinching function is using encoder action server, check define USE_FINGER_ENC code -> quitting" << endl;
        return -1;
    #endif
}


//------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::powerFingers(const string side, double diameter_in_cm, int mode){

    cout << ">> Power Grabbing <<" << endl;

    //mode_type: 0= wtout tool, 1= wt tool
    #ifdef USE_FINGER_ENC

    //linear interpolation
    if( diameter_in_cm < 2.0 || diameter_in_cm > 10.0){
        cout << "using linear interpolation for obj diameter of 2cm to 10cm" << endl;
        cout << "The input object diameter is outside of linear interpolation range" << endl;
        return -1;
    }
    
    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers_encoders", true);
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    target_fingers.trajectory.joint_names = fillJointNamesFingers(side);
    target_fingers.side = side;
    
    double encoder_val;
    if(diameter_in_cm >= 2.0 && diameter_in_cm < 5.0){                             //linear interpolation
        encoder_val = 2.1192930781 * diameter_in_cm - 1.2493372607;
    }
    else{                                                                          //logarithmic interpolation
        encoder_val = 25.3635690023 * log(diameter_in_cm) - 31.2123226127;
    }
    encoder_val += 0.25; //be more conservative
    
    std::cout << "Moving fingers in " << side << " hand to encoder value = " << encoder_val << " " << std::endl;
    
    trajectory_msgs::JointTrajectoryPoint finger_joints;
    finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());
    
    finger_joints.positions[0] = encoder_val;
    finger_joints.positions[1] = encoder_val;
    finger_joints.positions[2] = 0.0;
    finger_joints.positions[3] = -finger_joints.positions[0];
    finger_joints.positions[4] = -finger_joints.positions[1];
    finger_joints.positions[5] = -finger_joints.positions[2];

    target_fingers.trajectory.points.push_back(finger_joints);

    target_fingers.mode = mode;
    ac_fingers.sendGoal(target_fingers);
    ac_fingers.waitForResult(ros::Duration(20.0));
    result_fingers = ac_fingers.getResult();
    
    if(result_fingers->error_code == result_fingers->FAIL){
        cout << "power grabbing using fingers in " << side << " hand FAILED!" <<  std::endl;
        
        std_msgs::Bool failed;
        failed.data = true;
        fingerFailPub.publish(failed);
        
        return -1;
    }
    else{
        cout << "power grabbing using fingers in " << side << " DONE" <<  std::endl;
        return 1;
    }
    
    #else 
        cout << "this powergrab function is using encoder action server, check define USE_FINGER_ENC code -> quitting" << endl;
        return -1;
    #endif
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::ActionClientNeckPanDoneCallback(const actionlib::SimpleClientGoalState &state, const neck_pan::NeckPanResultConstPtr &result)
{
    std::stringstream ss;
    ss << "NeckPan finished in state [" << state.toString() << "] - ";
    if (result->result == neck_pan::NeckPanResult::K_RESULT_OK)
    {
        ss << "Successful";
        ROS_INFO_STREAM(ss.str());
    }
    else
    {
        ss << "FAILED!";
        ROS_ERROR_STREAM(ss.str());
    }

}








//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// in radian
int Manipulate::NeckPan(double pan_angle, double pan_speed){
    //note max pan speed  = 1.0
    // pan_angle = pan_angle * PI / 180.0; //convert to radian
    if(pan_angle > 1.22 || pan_angle < -1.22){
        cout << "neck PAN angle outside of 1.22 to -1.22 rad range >>> saturates" << endl;

       if(pan_angle > 1.22)
         pan_angle = 1.22;
        else if(pan_angle < -1.22)
            pan_angle = -1.22;
        else
            return -1; 
    }

    if(pan_speed > 1.0)
        pan_speed = 1.0;
    else if(pan_speed < 0)
        pan_speed = 0;

    actionlib::SimpleActionClient<neck_pan::NeckPanAction> neck_ac("neck_pan", true);
    neck_ac.waitForServer(ros::Duration(20.0));

    neck_pan::NeckPanGoal goal;
    goal.angle = pan_angle;
    goal.speed = pan_speed;

    cout << "Moving to neck pan angle = "<< pan_angle << "rads\n";

    neck_ac.sendGoal(goal);
    //boost::bind(&Manipulate::ActionClientNeckPanDoneCallback, this, _1, _2));
    neck_ac.waitForResult(ros::Duration(10.0));
    neck_pan::NeckPanResultConstPtr result  = neck_ac.getResult();

    if (result->result == neck_pan::NeckPanResult::K_RESULT_OK)
    {
        cout << "neck PANNING done!" << endl;
        //current_pan_angle = pan_angle;
        return 1;
    }
    else
    {
        cout << "neck PANNING failed" << endl;
        return -1;
    }

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//work in progress

int Manipulate::NeckTilt(double tilt_angle){

    if(tilt_angle > 13.3 || tilt_angle < -13.3){
        cout << "neck tilt angle outside of 13.3 to -13.3deg range >>> saturate" << endl;
        if(tilt_angle > 13.3)
            tilt_angle = 13.3;
        else if(tilt_angle < -13.3)
            tilt_angle = -13.3;
        else
            return -1;
    }


    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;

    target_joint = SetJointTargetsToCurrent();
    target_joint.head_trajectory.points[0].positions[0] =  tilt_angle/180*3.14159265; 

    cout << "Moving to neck tilt angle = "<< tilt_angle << "degs\n";
    moveit_j_ac.sendGoal(target_joint);
    moveit_j_ac.waitForResult(ros::Duration(30.0));
    result_joint = moveit_j_ac.getResult();
    sleep(sleep_time);
    
    if(result_joint->error_code == result_joint->FAIL) {
        std::cout << "neck TILTING failed\n";
        return -1;
    }else{
        cout << "neck TILTING done\n";
    }

    return 1;
    
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::TurnHeadToObjTarCenter(){

    double temp_x =  obj_tar_center.pose.position.x;
    double temp_y =  obj_tar_center.pose.position.y;
    double temp_z =  obj_tar_center.pose.position.z;
    double temp =  sqrt(temp_x*temp_x + temp_y*temp_y);

    cout << "coordinates mean = " << temp_x << " , " << temp_y << " , " << temp_z << " , " << temp << endl;

    cout << "tilt increment = " << atan2(temp_x, temp_y)*180.0/PI << endl;
    cout << "pan increment = " << atan2(temp_z, temp)*180.0/PI << endl;

    for(int i= 0; i < 10; i++)
        ros::spinOnce();
    
    cout << "current tilt angle = " << current_tilt_angle << endl;
    cout << "current pan angle = " << current_pan_angle << endl;

    neck_tilt_angle = current_tilt_angle + atan2(temp_x, temp_y)*180.0/PI; 
    neck_pan_angle = current_pan_angle + atan2(temp_z, temp)*180.0/PI;

    cout << "neck tilt angle = " << neck_tilt_angle << endl;
    cout << "neck pan angle = " << neck_pan_angle << endl;

    cout << "moving head to face obj and target >> press enter to continue";
    cin.ignore();

    //calculation is done for performing tilt first before pan;
    if(NeckTilt(neck_tilt_angle)< 0){
        return -1;
    }
    sleep(sleep_time);
    
    #ifdef USE_NECK_PAN
        if(NeckPan(neck_pan_angle, 0.5)<0){
            return -1;
        }
        sleep(sleep_time);
    #endif

    for(int i= 0; i < 10; i++)
        ros::spinOnce();

    cout << "current tilt angle = " << current_tilt_angle << endl;
    cout << "current pan angle = " << current_pan_angle << endl;

    cout << "moving head to face obj and target Done" << endl;
    //cin.ignore();

    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//both in radian
int Manipulate::TurnHeadTiltPan(double neck_tilt_angle, double neck_pan_angle){

    for(int i= 0; i < 10; i++)
        ros::spinOnce();

        cout << "current tilt angle = " << current_tilt_angle << endl;
    cout << "current pan angle = " << current_pan_angle << endl;

    cout << "move head to tilt= " << neck_tilt_angle << " and pan= " << neck_pan_angle << " >> press enter to continue";
    // cin.ignore();

    //must perform tilt first before pan;
    if(NeckTilt(neck_tilt_angle)< 0){
        return -1;
    }
    sleep(sleep_time);
    
    #ifdef USE_NECK_PAN
        if(NeckPan(neck_pan_angle, 0.1)<0){
            return -1;
        }
        sleep(sleep_time);
    #endif

    for(int i= 0; i < 10; i++)
        ros::spinOnce();

        cout << "current tilt angle = " << current_tilt_angle << endl;
    cout << "current pan angle = " << current_pan_angle << endl;

    cout << "move head Done" << endl;
    //cin.ignore();

    return 1;
}




//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::BendTool(double stick_length, double head_length, double head_angle_deg){
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac_left("cart_single", true);
    ac_left.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target_left;
    m3_moveit::MoveitSingleResultConstPtr result_left;
    target_left.link_name = "l_ee";

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac_right("cart_single", true);
    ac_right.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target_right;
    m3_moveit::MoveitSingleResultConstPtr result_right;
    target_right.link_name = "ee";


    std::cout << "BendTool" << std::endl;
    std::cout << "==========================" << std::endl;


    geometry_msgs::Point init_right;
    init_right.x = 0.3;
    init_right.y = -0.25;
    init_right.z = 0.95;

    // Right hand pre-positioning
    //=============================================
    int Npts = 1;
    target_right.end_eff.poses.resize(Npts);

    target_right.end_eff.poses[0].position.x = init_right.x;
    target_right.end_eff.poses[0].position.y = init_right.y;
    target_right.end_eff.poses[0].position.z = init_right.z;
    target_right.end_eff.poses[0].orientation.w = 0.70711;
    target_right.end_eff.poses[0].orientation.x = 0.0;
    target_right.end_eff.poses[0].orientation.y = sqrt(1.0 - 0.70711*0.70711);
    target_right.end_eff.poses[0].orientation.z = 0.0;

    ac_right.sendGoal(target_right);
    ac_right.waitForResult(ros::Duration(20.0));
    result_right = ac_right.getResult();
    if(result_right->error_code == result_right->FAIL){
        cout << "Right hand pre-positioning FAILED!\n";
        return -1;
    }
    else
        cout << "Right hand pre-positioning DONE. \n";
    sleep(sleep_time);

    double hand_width = 0.11;

    // Left hand pre-positioning for grasp
    //=============================================
    Npts = 1;
    target_left.end_eff.poses.resize(Npts);
    target_left.end_eff.poses[0].position.x = target_right.end_eff.poses[0].position.x - 0.17; //0.13
    target_left.end_eff.poses[0].position.y = target_right.end_eff.poses[0].position.y + stick_length - 0.5*hand_width; //0.35;
    target_left.end_eff.poses[0].position.z = target_right.end_eff.poses[0].position.z; //0.95;
    target_left.end_eff.poses[0].orientation.w = 0.70711;
    target_left.end_eff.poses[0].orientation.x = 0.0;
    target_left.end_eff.poses[0].orientation.y = sqrt(1.0 - 0.70711*0.70711);
    target_left.end_eff.poses[0].orientation.z = 0.0;

    ac_left.sendGoal(target_left);
    ac_left.waitForResult(ros::Duration(20.0));
    result_left = ac_left.getResult();
    if(result_left->error_code == result_left->FAIL){
        cout << "Left hand pre-positioning for grasp FAILED!\n";
        return -1;
    }
    else
        cout << "Left hand pre-positioning for grasp DONE. \n";
    sleep(sleep_time);


    //Open left fingers
    //============================================
    int ret;
    ret = OpenFingers("left",0);
    if(ret !=0){
        cout << "Open left fingers FAILED! \n";
        return -1;
    }


    // Left hand moving in for grasp
    //=============================================
    target_left.end_eff.poses[0].position.x = target_right.end_eff.poses[0].position.x;//0.3

    ac_left.sendGoal(target_left);
    ac_left.waitForResult(ros::Duration(20.0));
    result_left = ac_left.getResult();
    if(result_left->error_code == result_left->FAIL){
        cout << "Left hand moving in for grasp FAILED!\n";
        return -1;
    }
    else
        cout << "Left hand moving in for grasp DONE. \n";
    sleep(sleep_time);



    //Grasp bendtool with left
    //============================================
    int grasp_type = 2; // for bendtool
    ret = CloseFingers("left", grasp_type,1);

    if(ret ==0)
        cout << "Left grasp bendtool DONE. \n";
    else{
        cout << "Left grasp bendtool FAILED! \n";
        return -1;
    }


    //Open right fingers
    //============================================
    ret = OpenFingers("right",1);
    if(ret !=0){
        cout << "Open right fingers FAILED! \n";
        return -1;
    }


    target_right.end_eff.poses[0].position.y = target_left.end_eff.poses[0].position.y - head_length - hand_width; //0.0;

    ac_right.sendGoal(target_right);
    ac_right.waitForResult(ros::Duration(20.0));
    result_right = ac_right.getResult();
    if(result_right->error_code == result_right->FAIL){
        cout << "Right hand moving left for regrasp FAILED!\n";
        return -1;
    }
    else
        cout << "Right hand moving left for regrasp DONE. \n";
    sleep(sleep_time);


    //Regrasp bendtool with right
    //============================================
    grasp_type = 2; // for bendtool
    ret = CloseFingers("right", grasp_type,1);

    if(ret ==0)
        cout << "Regrasp bendtool DONE. \n";
    else
        cout << "Regrasp bendtool FAILED! \n";


    //Bend with left hand
    //============================================
    geometry_msgs::Point bend_center;

    double bend_radius= head_length; 
    bend_center.x=target_left.end_eff.poses[0].position.x;
    bend_center.y=target_left.end_eff.poses[0].position.y-bend_radius;
    bend_center.z=target_left.end_eff.poses[0].position.z;

    double angle_step_deg = 20.0; 
    double angle_step = angle_step_deg/180.0*3.14159;
    double steps = head_angle_deg/angle_step_deg + 2.0;
    int N_steps = floor(steps); 

    cout << "N_steps = " << N_steps << endl;

    geometry_msgs::PoseArray hand_pose_bend;
    geometry_msgs::Pose hand_pose_tmp = target_left.end_eff.poses[0];
    Eigen::Affine3d T_rotx, T_hand;
    T_rotx = Eigen::Affine3d(Eigen::AngleAxisd(-angle_step, Eigen::Vector3d(1, 0, 0)));
    hand_pose_bend.poses.resize(N_steps);
    for(int i=0; i<N_steps; i++){
        hand_pose_bend.poses[i].position.x = bend_center.x;
        hand_pose_bend.poses[i].position.y = bend_center.y + bend_radius*cos((i+1)*angle_step);
        hand_pose_bend.poses[i].position.z = bend_center.z - bend_radius*sin((i+1)*angle_step);

        tf::poseMsgToEigen(hand_pose_tmp, T_hand);
        T_hand = T_rotx*T_hand;
        tf::poseEigenToMsg(T_hand, hand_pose_tmp);
        hand_pose_bend.poses[i].orientation = hand_pose_tmp.orientation;
    }

    target_left.end_eff.poses.resize(N_steps);
    target_left.end_eff.poses = hand_pose_bend.poses;
    cout << "target_left.end_eff.poses= " << target_left.end_eff.poses[0] << endl;
    ac_left.sendGoal(target_left);
    ac_left.waitForResult(ros::Duration(20.0));
    result_left = ac_left.getResult();
    if(result_left->error_code == result_left->FAIL){
        cout << "Bend with left hand FAILED!\n";
        return -1;
    }
    else
        cout << "Bend with left hand DONE. \n";
    sleep(sleep_time);


    //Open right fingers
    //============================================
    ret = OpenFingers("right",1);
    if(ret !=0){
        cout << "Open right fingers FAILED! \n";
        return -1;
    }

    // Right hand moving right for regrasp
    //=============================================
    target_right.end_eff.poses[0].position.y = init_right.y; //-0.25;

    ac_right.sendGoal(target_right);
    ac_right.waitForResult(ros::Duration(20.0));
    result_right = ac_right.getResult();
    if(result_right->error_code == result_right->FAIL){
        cout << "Right hand moving right for regrasp FAILED!\n";
        return -1;
    }
    else
        cout << "Right hand moving right for regrasp DONE. \n";
    sleep(sleep_time);

    //Regrasp bendtool with right
    //============================================
    grasp_type = 2; // for bendtool
    ret = CloseFingers("right", grasp_type,1);

    if(ret ==0)
        cout << "Regrasp bendtool DONE. \n";
    else
        cout << "Regrasp bendtool FAILED! \n";

    //Open left fingers
    //============================================
    ret = OpenFingers("left",1);
    if(ret !=0){
        cout << "Open left fingers FAILED! \n";
        return -1;
    }


    // Left hand extract
    //=============================================
    Npts = 1;
    target_left.end_eff.poses.resize(Npts);

    target_left.end_eff.poses[0].position.x = 0.13;
    target_left.end_eff.poses[0].position.y = 0.2;
    target_left.end_eff.poses[0].position.z = 0.85;
    target_left.end_eff.poses[0].orientation.w = 0.5;
    target_left.end_eff.poses[0].orientation.x = -0.5;
    target_left.end_eff.poses[0].orientation.y = 0.5;
    target_left.end_eff.poses[0].orientation.z = -0.5;

    ac_left.sendGoal(target_left);
    ac_left.waitForResult(ros::Duration(20.0));
    result_left = ac_left.getResult();
    if(result_left->error_code == result_left->FAIL){
        cout << "Left hand extract FAILED!\n";
        return -1;
    }
    else
        cout << "Left hand extract DONE. \n";
    sleep(sleep_time);

    //Close left hand
    //============================================
    grasp_type = 2; // for bendtool
    ret = CloseFingers("left", grasp_type,0);

    if(ret ==0)
        cout << "Close left hand DONE. \n";
    else{
        cout << "Close left handFAILED! \n";
        return -1;
    }

    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;
    target_joint = SetJointTargetsToCurrent();
    
    trajectory_msgs::JointTrajectoryPoint right_joint;
    right_joint.positions.resize(target_joint.right_trajectory.joint_names.size());
    right_joint.positions[0] = -39.49/180*3.14159265;
    right_joint.positions[1] = 27.88/180*3.14159265;
    right_joint.positions[2] = 38.56/180*3.14159265;
    right_joint.positions[3] = 144.10/180*3.14159265;
    right_joint.positions[4] = 71.18/180*3.14159265;
    right_joint.positions[5] = 8.96/180*3.14159265;
    right_joint.positions[6] = 64.62/180*3.14159265;
    target_joint.right_trajectory.points[0] = right_joint;

    cout << "Moving to neutral pose...\n";
    moveit_j_ac.sendGoal(target_joint);
    moveit_j_ac.waitForResult(ros::Duration(30.0));
    result_joint = moveit_j_ac.getResult();
    sleep(sleep_time*2);
    if(result_joint->error_code == result_joint->FAIL) {
        std::cout << "Unable to move to neutral pose!\n";
    }else{
        cout << "Moved to neutral pose.\n";
    }
    sleep(sleep_time);
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::PickupBendTool(){

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    std::cout << "PickupBendTool" << std::endl;
    std::cout << "==========================" << std::endl;


    // Pre-positioning
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);
    target.end_eff.poses[0].position = bendtool_offset_right_hand;
    target.end_eff.poses[0].position.x += bendtool_in_base.pose.position.x+0.05;
    target.end_eff.poses[0].position.y += bendtool_in_base.pose.position.y-0.3;
    target.end_eff.poses[0].position.z += bendtool_in_base.pose.position.z+0.05;
    target.end_eff.poses[0].orientation.w = 0.0;
    target.end_eff.poses[0].orientation.x = 1.0;
    target.end_eff.poses[0].orientation.y = 0.0;
    target.end_eff.poses[0].orientation.z = 0.0;


    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Pre-positioning FAILED!\n";
        return -1;
    }
    else
        cout << "Pre-positioning DONE. \n";




    //Open right fingers
    //============================================
    int ret = OpenFingers("right",0);
    if(ret !=0){
        cout << "Open right fingers FAILED! \n";
        return -1;
    }

    // Pre-positioning with fingers open
    //=============================================
    Npts = 1;
    target.end_eff.poses.resize(Npts);
    target.end_eff.poses[0].position = bendtool_offset_right_hand;
//    target.end_eff.poses[0].orientation = bendtool_orientation_right_hand;
    target.end_eff.poses[0].position.x += bendtool_in_base.pose.position.x+0.05;
    target.end_eff.poses[0].position.y += bendtool_in_base.pose.position.y-0.3;
    target.end_eff.poses[0].position.z += bendtool_in_base.pose.position.z - bendtool_offset_right_hand.z + BENDTOOL_RADIUS + 0.13;
    target.end_eff.poses[0].orientation.w = 0.0;
    target.end_eff.poses[0].orientation.x = 1.0;
    target.end_eff.poses[0].orientation.y = 0.0;
    target.end_eff.poses[0].orientation.z = 0.0;


    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Pre-positioning with fingers open FAILED!\n";
        return -1;
    }
    else
        cout << "Pre-positioning with fingers open DONE. \n";

    cout << "Press ENTER to continue" << endl;
    cin.ignore();

 


    //Close right hand
    //============================================
    int grasp_type = 2; // for bendtool
    ret = CloseFingers("right", grasp_type,1);

    if(ret ==0)
        cout << "Close left hand DONE. \n";
    else{
        cout << "Close left handFAILED! \n";
        return -1;
    }
    

    cout << "Press ENTER to continue" << endl;
    cin.ignore();
    
    // Lift tool
    //=============================================
    Eigen::Affine3d T_rotx, T_hand;
    geometry_msgs::Pose hand_pose_tmp;
    T_rotx = Eigen::Affine3d(Eigen::AngleAxisd(30.0/180.0*3.14159, Eigen::Vector3d(0, 0, 1)));
    tf::poseMsgToEigen(target.end_eff.poses[0], T_hand);
    T_hand = T_rotx*T_hand;
    tf::poseEigenToMsg(T_hand, hand_pose_tmp);
    target.end_eff.poses[0].orientation = hand_pose_tmp.orientation;
        
    target.end_eff.poses[0].position.z += 0.2;//LIFT_BENDTOOL_HEIGHT;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Lift tool FAILED!\n";
        return -1;
    }
    else{
        cout << "Lift tool DONE. \n";
        return 0;
    }

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::PickupToolRightArm(){

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "PickupToolRightArm" << std::endl;
    std::cout << "==========================" << std::endl;
    
    //Open fingers
    //============================================
    
    if(OpenFingers("right", 1)<0)
        return -1;
    else
        sleep(sleep_time);




    // Pre-positioning with fingers open
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);
    target.end_eff.poses[0].position = tool_offset_right_hand;
    target.end_eff.poses[0].orientation = tool_orientation_right_hand;
    target.end_eff.poses[0].position.x += tool_in_base.pose.position.x;
    target.end_eff.poses[0].position.y += tool_in_base.pose.position.y + 0.07;
    target.end_eff.poses[0].position.z += tool_in_base.pose.position.z + 0.04;

    cout << "tool_in_base.pose = " << tool_in_base.pose <<endl;
    cout << "target.end_eff.poses[0] = " << target.end_eff.poses[0] <<endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Pre-positioning with fingers open FAILED!\n";
        return -1;
    }
    else
        cout << "Pre-positioning with fingers open DONE. \n";


    // Pre-positioning with fingers open
    //=============================================
    Npts = 1;
    target.end_eff.poses.resize(Npts);
    target.end_eff.poses[0].position = tool_offset_right_hand;
    target.end_eff.poses[0].orientation = tool_orientation_right_hand;
    target.end_eff.poses[0].position.x += tool_in_base.pose.position.x;
    target.end_eff.poses[0].position.y += tool_in_base.pose.position.y + 0.07;
    target.end_eff.poses[0].position.z += tool_in_base.pose.position.z - 0.04;

    cout << "tool_in_base.pose = " << tool_in_base.pose <<endl;
    cout << "target.end_eff.poses[0] = " << target.end_eff.poses[0] <<endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Pre-positioning with fingers open FAILED!\n";
        return -1;
    }
    else
        cout << "Pre-positioning with fingers open DONE. \n";


    //Close fingers
    //============================================
    #ifdef USE_FINGER_ENC
        if(powerFingers("right", tool_diameter, 1) < 0)
            return -1;
        else
            sleep(sleep_time);
    #else
        if(CloseFingers("right", 1, 1)< 0)
            return -1;
        else
            sleep(sleep_time);
    #endif
   


    // Lift tool
    //=============================================
    target.end_eff.poses[0].position.z += LIFT_TOOL_HEIGHT+0.01;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Lift tool FAILED!\n";
        return -1;
    }
    else{
        cout << "Lift tool DONE. \n";
        return 1;
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//FindCalibration
bool Manipulate::ToolInHandCalibration(std::string hand){
    
    Speech("Starting multi view 3D point cloud registration ....");
    sleep(0.5);

    //rmb to turn head back before calib
    #ifdef USE_NECK_PAN
    NeckPan(0.0, 0.5);
    #endif

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    if(hand=="left")
        target.link_name = "l_ee";
    else if(hand=="right")
        target.link_name = "ee";

    int n_calib_poses = 3; //need at least 4 for calibration
    int n_calib_poses_meta = 2;
    geometry_msgs::PoseArray calib_hand_poses, calib_tool_poses, calib_wrist_poses;
    calib_hand_poses.poses.resize(n_calib_poses*n_calib_poses_meta);
    calib_tool_poses.poses.resize(n_calib_poses*n_calib_poses_meta);
    calib_tool_poses_live.poses.resize(n_calib_poses*n_calib_poses_meta);
    calib_wrist_poses.poses.resize(n_calib_poses*n_calib_poses_meta);

    int N_angle_steps = n_calib_poses-1;
    double angle_step = 0.25*3.14159265359/((double) N_angle_steps);//0.3
    int ind;

    //im-th meta pose, followed by wrist roll
    for(int im = 0; im < n_calib_poses_meta; im++){
        ind = im*n_calib_poses;

        if(im == 0){
            calib_hand_poses.poses[ind].position.x = 0.35+0.1;
            calib_hand_poses.poses[ind].position.y = -0.25;
            calib_hand_poses.poses[ind].position.z = TABLE_HEIGHT + 0.15;//0.2;
            calib_hand_poses.poses[ind].orientation.w = 0.5;
            calib_hand_poses.poses[ind].orientation.x = 0.5;
            calib_hand_poses.poses[ind].orientation.y = 0.5;
            calib_hand_poses.poses[ind].orientation.z = 0.5;
            
            Eigen::Affine3d T_hand0, T_rotz0;
            T_rotz0 = Eigen::Affine3d(Eigen::AngleAxisd(-0.0*3.14159265359, Eigen::Vector3d(0, 0, 1)));
            tf::poseMsgToEigen(calib_hand_poses.poses[ind], T_hand0);
            T_hand0 = T_hand0*T_rotz0;
            tf::poseEigenToMsg(T_hand0, calib_hand_poses.poses[ind]);


        }
        else if(im==1){
            calib_hand_poses.poses[ind].position.x = 0.37+0.05;
            calib_hand_poses.poses[ind].position.y = -0.4+0.05;
            calib_hand_poses.poses[ind].position.z = TABLE_HEIGHT + 0.18;
            calib_hand_poses.poses[ind].orientation.w = 0.2706;
            calib_hand_poses.poses[ind].orientation.x = 0.2706;
            calib_hand_poses.poses[ind].orientation.y = 0.65328;
            calib_hand_poses.poses[ind].orientation.z = 0.65328;

            Eigen::Affine3d T_hand0, T_rotz0;
            T_rotz0 = Eigen::Affine3d(Eigen::AngleAxisd(-0.0*3.14159265359, Eigen::Vector3d(0, 0, 1)));
            tf::poseMsgToEigen(calib_hand_poses.poses[ind], T_hand0);
            T_hand0 = T_hand0*T_rotz0;
            tf::poseEigenToMsg(T_hand0, calib_hand_poses.poses[ind]);
        }
        Eigen::Affine3d T_hand, T_rotz, T_rotz_waist_yaw;
        T_rotz_waist_yaw = Eigen::Affine3d(Eigen::AngleAxisd(waist_yaw+base_yaw, Eigen::Vector3d(0, 0, 1)));
        T_rotz = Eigen::Affine3d(Eigen::AngleAxisd(angle_step, Eigen::Vector3d(0, 0, 1)));
        tf::poseMsgToEigen(calib_hand_poses.poses[ind], T_hand);
        T_hand = T_rotz_waist_yaw*T_hand;
        tf::poseEigenToMsg(T_hand, calib_hand_poses.poses[ind]);


        Eigen::Affine3d T_wrist, T_hand2wrist;
        T_hand2wrist.translation() = Eigen::Vector3d(0.0, 0.0, -0.05-0.06004);
        T_hand2wrist.linear() = Eigen::Matrix3d::Identity();
        T_wrist = T_hand*T_hand2wrist;
        tf::poseEigenToMsg(T_wrist, calib_wrist_poses.poses[ind]);


        for (int i = ind+1; i < ind+n_calib_poses; i++){
            T_hand = T_hand*T_rotz;
            tf::poseEigenToMsg(T_hand, calib_hand_poses.poses[i]);
            T_wrist = T_hand*T_hand2wrist;
            tf::poseEigenToMsg(T_wrist, calib_wrist_poses.poses[i]);
        }
    }


    //moving to various poses for multi-view point cloud registration of in-hand tool
    tool_decision_received = false;
    calib_tool_poses_received = false;
    is_suitable_tool = 0;
    for(int i=0; i < n_calib_poses*n_calib_poses_meta; i++){

        calib_next = 0;
        
        target.end_eff.poses.resize(1);
        target.end_eff.poses[0] = calib_hand_poses.poses[i];

        cout << "Moving to calib pose" << i+1 << "/" << n_calib_poses*n_calib_poses_meta << "...\n";
        ac.sendGoal(target);
        ac.waitForResult(ros::Duration(20.0));
        result = ac.getResult();
        sleep(sleep_time);
        if((result->error_code == result->FAIL)){
            cout << "Moving to calib pose FAILED! Proceeding to the next pose.\n";
            cout << "faulty calib_hand_poses[" << i << "] = " << calib_hand_poses.poses[i] << endl;
                geometry_msgs::PoseStamped calib_wrist_cam_frame;
                if(i==0)
                    cout << "first calib pose cannot be reached. Unable to proceed!" << endl;
                else
                    calib_wrist_cam_frame = transformPose( calib_wrist_poses.poses[i-1], "world", "camera_rgb_optical_frame");
                tool_expt::calib calib_msg;
                calib_msg.pose = calib_wrist_cam_frame.pose;
                if(i == n_calib_poses*n_calib_poses_meta-1)
                    calib_msg.last_pose = true;
                else
                    calib_msg.last_pose = false;
                pub_calib.publish(calib_msg);
        }
        else{
            geometry_msgs::PoseStamped calib_wrist_cam_frame;
            calib_wrist_cam_frame = transformPose( calib_wrist_poses.poses[i], "world", "camera_rgb_optical_frame");
            tool_expt::calib calib_msg;
            calib_msg.pose = calib_wrist_cam_frame.pose;
            if(i == n_calib_poses*n_calib_poses_meta-1)
                calib_msg.last_pose = true;
            else
                calib_msg.last_pose = false;
            pub_calib.publish(calib_msg);
            if(i < n_calib_poses*n_calib_poses_meta-1){
                Speech("Processing point cloud...");
                cout << "Waiting for point cloud capture to be done..." << endl;
                while(ros::ok() && calib_next!=1){
                    usleep(100000);
                    ros::spinOnce();
                }
                Speech("Next pose...");
            }
        }
    }
    Speech("Registration done....");
    cout << "Multi-view point cloud registration of in-hand tool done." << endl;
    cout << "Waiting for tool suitability assessment." << endl;
    while(ros::ok() && !tool_decision_received){
        usleep(100000);
        ros::spinOnce();
    }

    if (is_suitable_tool == 1){
        Speech("Yes, this tool has the required functionality!");
        cout << "Tool is suitable. Waiting for calib tool poses ..." << endl;
        while(ros::ok() && !calib_tool_poses_received){
            usleep(100000);
            ros::spinOnce();
        }
        calib_tool_poses = calib_tool_poses_live;


        double obj2tool_yaw;
        if(functionality==1) //forward tool
        {
            object_to_tool_position_offset = pushforward_offset_right_tool;

            if( radian >= PI/4.0)
                obj2tool_yaw = radian - PI/2.0;
            else
                obj2tool_yaw = radian;
        }
        else if(functionality==2) //back tool
        {
            object_to_tool_position_offset = pullback_offset_right_tool;
            obj2tool_yaw = radian + PI/2.0;
        }
        else if(functionality==3) //side tool
        {            
            object_to_tool_position_offset = pushsideways_offset_right_tool;

            if(radian < (-PI/2.0)){ 
                obj2tool_yaw = radian + PI; //+ve
            }
            else if(radian > PI/2.0){
                obj2tool_yaw = radian - PI; //-ve
            }
            else{   //pushing east
                object_to_tool_position_offset.y = PUCK_RADIUS; //+ve
                obj2tool_yaw = radian;
            }
        }

        Eigen::MatrixXd Tool2HandTransform = ComputeTool2HandTransform(calib_hand_poses, calib_tool_poses);
        if(Tool2HandTransform==Eigen::MatrixXd::Zero(4,4)){
            std::cout << "Cannot compute tool-to-hand transform!" << std::endl;
            target.end_eff.poses.resize(1);
            target.end_eff.poses[0] = calib_hand_poses.poses[4];
            std::cout << "Moving back to calib pose[4]...\n";
            ac.sendGoal(target);
            ac.waitForResult(ros::Duration(20.0));
            result = ac.getResult();
            sleep(sleep_time);
            if(result->error_code == result->FAIL){
                std::cout << "Moving to calib pose[4] FAILED!\n";
            }
            return false;
        }
        else{
            Eigen::Affine3d tool2hand_eig, obj2tool_eig, obj2tool_eig_trans, obj2tool_rot_eig, obj2hand_eig, omnistraight_eig;

            cout << "==============================" << endl;
            //tool to hand
            tool2hand_eig.matrix() = Tool2HandTransform;
            cout << "tool2hand_eig = \n" << tool2hand_eig.matrix() << endl;
            
            //obj to tool translation
            obj2tool_eig_trans.translation() = Eigen::Vector3d(object_to_tool_position_offset.x, object_to_tool_position_offset.y, object_to_tool_position_offset.z);
            obj2tool_eig_trans.linear() = Eigen::Affine3d::Identity().linear();
            cout << "obj2tool_eig_trans = \n" << obj2tool_eig_trans.matrix() << endl;
            
            //obj to tool rotation
            obj2tool_rot_eig = Eigen::Affine3d(Eigen::AngleAxisd(obj2tool_yaw, Eigen::Vector3d(0, 0, 1)));
            cout << "obj2tool_rot_eig = \n" << obj2tool_rot_eig.matrix() << endl;
            cout << "++++++++" << endl;

            obj2tool_eig = obj2tool_rot_eig * obj2tool_eig_trans;
            cout << "obj2tool_eig = \n" << obj2tool_eig.matrix() << endl;

            //this here computes the tf
            obj2hand_eig = obj2tool_eig * tool2hand_eig;
            cout << "obj2hand_eig = \n" << obj2hand_eig.matrix() << endl;
            cout << "--------" << endl;

            tf::poseEigenToMsg(obj2hand_eig, obj2hand);
            std::cout << "obj2hand = \n" << obj2hand << std::endl;

            object_to_hand_position_offset = obj2hand.position;
            object_to_hand_orientation_offset = obj2hand.orientation;

            std::cout << "object_to_hand_position_offset = \n" << object_to_hand_position_offset << std::endl;
            std::cout << "object_to_hand_orientation_offset = \n" << object_to_hand_orientation_offset << std::endl;
            
            //wtout rotation
            omnistraight_eig =  obj2tool_eig_trans * tool2hand_eig;
            tf::poseEigenToMsg(omnistraight_eig, omnistraight);
            std::cout << "direct 90deg position = \n" << omnistraight.position << std::endl;
            std::cout << "direct 90deg orientation = \n" << omnistraight.orientation << std::endl;
            
            return true;
        }
    }
    else{
        Speech("No, this tool does not have ... the required functionality...");
        sleep(1.0);
        cout << "Tool is NOT suitable!" << endl;

        return false;
    }


}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


Eigen::MatrixXd Manipulate::ComputeTool2HandTransform(const geometry_msgs::PoseArray calib_hand_poses, const geometry_msgs::PoseArray calib_tool_poses){

    

    if(calib_hand_poses.poses.size()!=calib_tool_poses.poses.size()){
        cout << "ERROR: Number of hand poses != Number of tool poses !!!" << endl;
        cout << "calib_hand_poses.poses.size() = " << calib_hand_poses.poses.size() << endl;
        cout << "calib_tool_poses.poses.size() = " << calib_tool_poses.poses.size() << endl;
        return Eigen::MatrixXd::Zero(4,4);
    }

    Eigen::Matrix4d Tool2HandTransform;
    Eigen::Matrix3d Rth;
    Eigen::MatrixXd Rh, Rt, pleft, pth;
    Eigen::Affine3d calib_hand_poses_eig, calib_tool_poses_eig;

    tf::poseMsgToEigen(calib_hand_poses.poses[0],calib_hand_poses_eig);
    tf::poseMsgToEigen(calib_tool_poses.poses[0],calib_tool_poses_eig);

    Rh = calib_hand_poses_eig.matrix().block<3,3>(0,0);
    Rt = calib_tool_poses_eig.matrix().block<3,3>(0,0);
    pleft = calib_hand_poses_eig.matrix().block<3,1>(0,3) - calib_tool_poses_eig.matrix().block<3,1>(0,3);



    if(calib_hand_poses.poses.size()>1){
        for (int i = 1; i < calib_hand_poses.poses.size(); i++){
            Eigen::MatrixXd tmp((i+1)*3, 3);
            tf::poseMsgToEigen(calib_hand_poses.poses[i],calib_hand_poses_eig);
            tmp << Rh, calib_hand_poses_eig.matrix().block<3,3>(0,0); //concatenate recursively
            Rh = tmp;
            tf::poseMsgToEigen(calib_tool_poses.poses[i],calib_tool_poses_eig);
            tmp << Rt, calib_tool_poses_eig.matrix().block<3,3>(0,0); //concatenate recursively
            Rt = tmp;
//            cout << "Hand = " << Rh << endl;
//            cout << "Tool = " << Rt << endl;

            Eigen::MatrixXd tmp2((i+1)*3, 1);
            tmp2 << pleft, calib_hand_poses_eig.matrix().block<3,1>(0,3) - calib_tool_poses_eig.matrix().block<3,1>(0,3);
            pleft = tmp2;
//            cout << "phand-ptool = " << pleft << endl;
        }
    }
    Rth = pseudoInverse(Rt)*Rh;

    pth = pseudoInverse(Rt)*pleft;
    if(functionality==1)
        pth(0,0) += PUCK_RADIUS;



    Tool2HandTransform.block<3,3>(0,0) = Rth;
    Tool2HandTransform.block<3,1>(0,3) = pth;
    Eigen::MatrixXd tmp3(1,4);
    tmp3 << 0,0,0,1;
    Tool2HandTransform.block<1,4>(3,0) = tmp3;
    cout << "Tool-to-Hand transformation computed:" << endl;
    cout << Tool2HandTransform << endl;

    //Constrain to planar
    //===========================================================================
   if(pth(0,0) < -0.4)
        pth(0,0) = -0.4;
    else if(pth(0,0) > -0.32)
        pth(0,0) = -0.32;

    if(pth(1,0) < -0.04)
        pth(1,0) = -0.04;
    else if(pth(1,0) > 0.04)
        pth(1,0) = 0.04;

    if(pth(2,0) < 0.05)
        pth(2,0) = 0.05;
    else if(pth(2,0) > 0.08)
        pth(2,0) = 0.08;

    double max, other;
    if(Rth(1,0) > Rth(0,2)){
        max = Rth(1,0);
        other = signum(Rth(1,2))*sin(acos(max));
        Rth << -other, 0.0, max, max, 0.0, other, 0.0, 1.0, 0.0;
    }
    else{
        max = Rth(0,2);
        other = signum(Rth(0,0))*sin(acos(max));
        Rth << other, 0.0, max, max, 0.0, -other, 0.0, 1.0, 0.0;
    }


    if(max < 0.97)
        Rth << 0,0,1,1,0,0,0,1,0;

    Tool2HandTransform.block<3,3>(0,0) = Rth;
    Tool2HandTransform.block<3,1>(0,3) = pth;
    cout << "Tool-to-Hand transformation (with guard):" << endl;
    cout << Tool2HandTransform << endl;
    Speech("I have computed the transformation...");
    Speech(" from the tool ... to my hand...");
    sleep(1.0);


    return Tool2HandTransform;

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

double Manipulate::signum(const double num){
    double sgn;
    if(num > 0.0)
        sgn = 1.0;
    else if(num < 0.0)
        sgn = -1.0;
    else
        sgn = 0.0; 
    return sgn;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::CheckFakeToolTransform(){
    cout << "Waiting for fake tool transform..." << endl;
    while(ros::ok() && !fake_tool_transform_received){
        usleep(500000);
        ros::spinOnce();
    }
    if(ros::ok()){
        cout << "Fake tool transform received." << endl;
        fake_tool_transform_received = false;
        fake_tool_transform = fake_tool_transform_live;
        object_to_hand_position_offset = fake_tool_transform.position;
        object_to_hand_orientation_offset = fake_tool_transform.orientation;
    }

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionPullBackLeftArm(){


}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionPullBackRightArm(){

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac_task("task_single", true);
    ac_task.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target_task;
    m3_moveit::MoveitSingleResultConstPtr result_task;
    target_task.link_name = "ee";

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    int Npts;

    std::cout << "ActionPullBackRightArm" << std::endl;
    std::cout << "==========================" << std::endl;

    // Prepositioning
    //============================================
    Npts = 1;
    target_task.end_eff.poses.resize(Npts);
    target_task.end_eff.poses[0].position = object_to_hand_position_offset;
    //target_task.end_eff.poses[0].orientation = pushforward_orientation_right_hand;
    target_task.end_eff.poses[0].orientation.w = 0.2706;
    target_task.end_eff.poses[0].orientation.x = 0.2706;
    target_task.end_eff.poses[0].orientation.y = 0.65328;
    target_task.end_eff.poses[0].orientation.z = 0.65328;
    target_task.end_eff.poses[0].position.x += object_in_base.pose.position.x + 0.06;
    target_task.end_eff.poses[0].position.y += object_in_base.pose.position.y - 0.02;
    target_task.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.1 ;
    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << "target_task.end_eff.poses[0] = " << target_task.end_eff.poses[0] << std::endl;
    ac.sendGoal(target_task);
    ac.waitForResult(ros::Duration(20.0));
    result_task = ac.getResult();
    sleep(sleep_time);
    if(result_task->error_code == result_task->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }



    Npts = 1;
    target.end_eff.poses.resize(Npts);
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + 0.06;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y ;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z +0.05;
    std::cout << "target.end_eff.poses[0] = " << target.end_eff.poses[0] << std::endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }


    //cout << "Press Enter to Continue";
    //cin.ignore();

    // Pullback
    //============================================
    Npts = 1;
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(Npts);
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x ;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y ;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z+0.05;

    std::cout << "Pullback target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Pullback with right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Pullback with right hand DONE. \n";
    }
    

    //cout << "Press Enter to Continue";
    //cin.ignore();


    // Retract
    //============================================
    Npts = 1;
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(Npts);
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    //target.end_eff.poses[0].orientation = pushforward_orientation_right_hand;
    target.end_eff.poses[0].orientation.w = 0.2706;
    target.end_eff.poses[0].orientation.x = 0.2706;
    target.end_eff.poses[0].orientation.y = 0.65328;
    target.end_eff.poses[0].orientation.z = 0.65328;
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x + 0.02;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y - 0.02;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z + 0.1;

    cout << "Moving to last pose!!...\n";
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(30.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL) {
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        std::cout << "Moving to last pose FAILED!\n";
        return -1;
    }
    else{
        Speech("Done!");
        cout << "Moving to last pose DONE. \n";
    }

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::ActionPullBackBendTool(){
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "ActionPullBackBendTool" << std::endl;
    std::cout << "==========================" << std::endl;

    int Npts = 1;
    double shiftx1 = 0.07;
    double shifty1 = -0.07;
    double shiftx2 = 0.05;
    double shifty2 = -0.05;
    target.end_eff.poses.resize(Npts);

    // Pre-positioning
    //=============================================
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = bendtool_orientation_right_hand;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx1;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty1;
    target.end_eff.poses[0].position.z += 0.9;

    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }
    
    cout << "Press ENTER to continue" << endl;
    cin.ignore();

    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = bendtool_orientation_right_hand;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx1;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty1;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z+0.03 ;

    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }

    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = bendtool_orientation_right_hand;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x ;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y ;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z+0.03 ;

    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }




    // Pullback
    //============================================
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = bendtool_orientation_right_hand;
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x ;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y ;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z+0.01 ;

    std::cout << "Pullback target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }



    // via point to avoid disturbing the object during next move
    //==========================================================
    //MoveWaistFromCurrent(0.0);
    //MoveWaist(45.0);
    //return 0;

    target.end_eff.poses[0].position.x += 0.1;
    target.end_eff.poses[0].position.y += -0.1;
    target.end_eff.poses[0].position.z += 0.1;
    target.end_eff.poses[0].orientation.w = 0.5;
    target.end_eff.poses[0].orientation.x = 0.5;
    target.end_eff.poses[0].orientation.y = 0.5;
    target.end_eff.poses[0].orientation.z = 0.5;


    std::cout << "Going to via point...\n";
    std::cout << target.end_eff.poses[0] << std::endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        cout << "Going to via point FAILED!\n";
        return -1;
    }
    else{
        cout << "Going to via point DONE. \n";
        object_in_base.pose.position.x = target_in_base.pose.position.x;
        object_in_base.pose.position.z = target_in_base.pose.position.z;
        
    }
    
    Home();
    return 0;

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//FindSouth
int Manipulate::ActionPullBackRightArmTool(){
    
    
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "ActionPullBackRightArmTool" << std::endl;
    std::cout << "==========================" << std::endl;
    
    
    int Npts = 1;
    double shiftx1 = 0.1;
    double shifty1 = -0.1;
    double shiftx2 = 0.05;
    double shifty2 = -0.05;
    target.end_eff.poses.resize(Npts);

    // Pre-positioning
    //=============================================
    target.end_eff.poses[0].position = object_to_hand_position_offset;
//    target.end_eff.poses[0].orientation = pushsideways_orientation_right_hand;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx1;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty1;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.05;
    
    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }
    
    target.end_eff.poses[0].position = object_to_hand_position_offset;
//    target.end_eff.poses[0].orientation = pushsideways_orientation_right_hand;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx1;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty1;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z ;
    
    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }

    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + 0.025 ;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y ;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z ;

    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }
    
    //cout << "Press ENTER to continue" << endl;
    //cin.ignore();


    // Pullback
    //============================================
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x ;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y ;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z ;

    std::cout << "Pullback target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion DONE. \n";
    }
    


    // via point to avoid disturbing the object during next move
    //==========================================================

    target.end_eff.poses[0].position.x += shiftx2;
    target.end_eff.poses[0].position.y += shifty2;
    target.end_eff.poses[0].position.z = 0.9;


    std::cout << "Pulling back with right hand...\n";
    std::cout << target.end_eff.poses[0] << std::endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Pullback with right hand FAILED!\n";
        return -1;
    }
    else{
        Speech("Done!");
        cout << "Pullback with right hand DONE. \n";
        object_in_base.pose.position.x = target_in_base.pose.position.x;
        object_in_base.pose.position.z = target_in_base.pose.position.z;
        return 0;
    }
}








//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//grabbing and moving function

double shifty =0;
double shiftx =0;
double shifty2 =0;
double shiftx2 =0;
double handoffx =0;
double handoffy =0;
double finetune_x =0;
double finetune_y =0;
double finetune_x2 =0;
double finetune_y2 =0;

bool closerequired = false;

sc::result Manipulate::react( const EvTargetOmniObject & ){

    //reinitialise
    shifty =0;
    shiftx =0;
    shifty2 =0;
    shiftx2 =0;
    handoffx =0;
    handoffy =0;
    finetune_x =0;
    finetune_y =0;

    closerequired = false;

    //get current ee position
    tf::Quaternion ee_Quat;
    tf::Vector3 ee_Vec;
    getCurrentTransform("ee", ee_Quat, ee_Vec);

    cout << "================" << endl;
       cout << "ee xyz:"  << ee_Vec[0] << ", " << ee_Vec[1] << ", " << ee_Vec[2] << endl;
       cout << "ee quat:"  << ee_Quat[0] << ", " << ee_Quat[1] << ", " << ee_Quat[2] << ", " << ee_Quat[3] << endl;
           cout << "================" << endl;


    float roll = PI/2.0, pitch = PI/2.0, yaw = 0;                   //direct grab approach, pitch at 90degs 
        float roll2 = PI/2.0, pitch2 = radian, yaw2 = 0;            //end tarjectory hand pose, facing the direction of target
            float roll3 = PI/2.0, pitch3 = radian, yaw3 = 0;        //if direct grab fails, change approach angle

    if( (angle > 20.0) && (angle < 85.0) ){                             //1st quadrant (task 6)
        
        cout << "target in 1st quadrant" <<endl;
        pitch2 = radian;
        pitch3 = radian + PI/2.0;

        functionality = 1;//functionality-- 1:forw, 2:back , 3:side
    }
    else if( angle > 95.0 && angle < 160.0){                           //2nd quadrant (task 5)
    
        cout << "target in 2nd quadrant"<<endl;
        pitch2 = radian;
        pitch3 = radian;

        functionality = 1;
    }
    else if( angle < -95.0 && angle > -160.0){                         //3rd quadrant (task 8)
       
        cout << "target in 3rd quadrant"<<endl;
        pitch3 = radian + 3.0/2.0 * PI;

        functionality = 2;

        if(fabs(angle) > 150.0)
            pitch2 = radian;
        else
            pitch2 = pitch3;
    }
    else if( angle < -20.0 && angle > -85.0){                            //4th quadrant (task 7)
        
        cout << "target in 4th quadrant"<<endl;
        pitch3 = radian + PI;

        functionality = 2;

        if(fabs(angle) < 30.0)
            pitch2 = radian + PI/2.0;
        else{
            pitch2 = pitch3;
            closerequired = true;
        }
    }
    else if( angle >= 85.0 && angle <= 95.0){                           //North +-5 deg tolerance
    
        cout << "target is North"<<endl;
        pitch2 = radian;

        functionality = 1;

        double temp_radian;
        double temp_y_dif = fabs(target_in_base.pose.position.y - ee_Vec[1]);
        double temp_x_dif = fabs(target_in_base.pose.position.x - ee_Vec[0]);
        temp_radian = std::atan2(temp_y_dif,temp_x_dif);       //atan2 is -180 to 180 range; 
        
        pitch3 = temp_radian + PI/2.0;
    }
    else if( angle <= -85.0 && angle >= -95.0){                         //south +-5deg tolerance
    
        cout << "target is south" << endl;
        pitch2 = radian + PI;   

        functionality = 2;                                                

        double temp_radian;
        double temp_y_dif = fabs(target_in_base.pose.position.y - ee_Vec[1]);
        double temp_x_dif = fabs(target_in_base.pose.position.x - ee_Vec[0]);
        temp_radian = std::atan2(temp_y_dif,temp_x_dif);       //atan2 is -180 to 180 range; 
        
        pitch3 = temp_radian + PI/2.0;

        closerequired = true;
    }
    else if( fabs(angle) >= 160.0 ){                                          //West
    
        cout << "target is west" << endl;

        functionality = 3;

        double temp_radian;
        double temp_y_dif = fabs(target_in_base.pose.position.y - object_in_base.pose.position.y);
        double temp_x_dif = fabs(target_in_base.pose.position.x - ee_Vec[0]);
        temp_radian = std::atan2(temp_y_dif,temp_x_dif);       //atan2 is -180 to 180 range; 
        
        //cout << "x temp=" << temp_x_dif << ", y temp=" << temp_y_dif << endl; 

        pitch2 = temp_radian + PI/2.0;

        temp_y_dif = fabs(object_in_base.pose.position.y - ee_Vec[1]);
        temp_x_dif = fabs(object_in_base.pose.position.x - ee_Vec[0]);
        temp_radian = std::atan2(temp_y_dif,temp_x_dif);       //atan2 is -180 to 180 range; 

        pitch3 = temp_radian + PI/2.0;
    }
    else if( fabs(angle) <= 20.0){                                            //East
    
        cout << "target is east" << endl;

        functionality = 3;

        double temp_radian;
        double temp_y_dif = fabs(target_in_base.pose.position.y - object_in_base.pose.position.y);
        double temp_x_dif = fabs(target_in_base.pose.position.x - ee_Vec[0]);
        temp_radian = std::atan2(temp_y_dif,temp_x_dif);       //atan2 is -180 to 180 range; 

        pitch2 = PI/2.0 - temp_radian;

        temp_y_dif = fabs(object_in_base.pose.position.y - ee_Vec[1]);
        temp_x_dif = fabs(object_in_base.pose.position.x - ee_Vec[0]);
        temp_radian = std::atan2(temp_y_dif,temp_x_dif);       //atan2 is -180 to 180 range; 

        pitch3 = temp_radian + PI/2.0;
        closerequired = true;
    }
    else{
    
        cout << "xxxxxxxxxxxxxx unknown case! xxxxxxxxxxxxxxxxxxx" << endl;
        return transit< ReadyState >(); //quit
    }

    shiftx = -(objtohand);   //shiftx for adjusting grab position 
    shifty = 0;             //no offset for north direction

    handoffx = -(objtohand) * std::sin(pitch2);
    handoffy = (objtohand) * std::cos(pitch2);

    shiftx2 = -(objtohand) * std::sin(pitch3);
    shifty2 = (objtohand) * std::cos(pitch3);

    finetune_x = 0.025 * std::sin(pitch2);
    finetune_y = -0.025 * std::cos(pitch2);


    cout << "==========================================" << endl;
    cout << "re-adjust approach angle >>" << endl;
    cout << "direct grab at object:\n deg = 90" << endl;
    cout << "hand angle at end target:\n deg = " << pitch2 * (180.0/ PI) << " , radian = " << pitch2 << endl;
    cout << "reposition angle if required: deg = " << pitch3 * (180.0/ PI) << " , radian = " << pitch3 << endl;

    cout << "shiftx=" << shiftx <<endl;         //direct grab
    cout << "shifty=" << shifty <<endl;

    cout << "handoffx=" << handoffx <<endl;     //end effector
    cout << "handoffy=" << handoffy <<endl;
    
    cout << "shiftx2=" << shiftx2 <<endl;       //if direct fails
    cout << "shifty2=" << shifty2 <<endl;

    cout << "==========================================" << endl;

    Quaternionf q;  //for grabbing, directly face north to target
    q = AngleAxisf(roll, Vector3f::UnitX())
        * AngleAxisf(pitch, Vector3f::UnitY())
        * AngleAxisf(yaw, Vector3f::UnitZ());

    omni_orientation_right_hand.w = q.w();
    omni_orientation_right_hand.x = q.x();
    omni_orientation_right_hand.y = q.y();
    omni_orientation_right_hand.z = q.z();

    cout << "q:  w=" << q.w() << " x=" << q.x() << " y=" << q.y() << " z=" << q.z() << endl;

    Quaternionf q2;  //for target position
        q2 = AngleAxisf(roll2, Vector3f::UnitX())
        * AngleAxisf(pitch2, Vector3f::UnitY())
        * AngleAxisf(yaw2, Vector3f::UnitZ());
    
    omni_orientation_target.w = q2.w();
    omni_orientation_target.x = q2.x();
    omni_orientation_target.y = q2.y();
    omni_orientation_target.z = q2.z();
    
    cout << "q2:  w=" << q2.w() << " x=" << q2.x() << " y=" << q2.y() << " z=" << q2.z() << endl;

    Quaternionf q3;  //for reposition hand
        q3 = AngleAxisf(roll3, Vector3f::UnitX())
        * AngleAxisf(pitch3, Vector3f::UnitY())
        * AngleAxisf(yaw3, Vector3f::UnitZ());
    
    omni_orientation_right_hand2.w = q3.w();
    omni_orientation_right_hand2.x = q3.x();
    omni_orientation_right_hand2.y = q3.y();
    omni_orientation_right_hand2.z = q3.z();
    
    cout << "q3:  w=" << q3.w() << " x=" << q3.x() << " y=" << q3.y() << " z=" << q3.z() << endl;

    if(ActionOmniRight() < 0)
    {
        failFlag = true;
        failcount++;
    }
    else{
        failFlag = false;
    }

    cout << "++++++++++++++++++++++++++++++++" << endl;


    if(failFlag && failcount <= maxFail){
        cout << "re-attempt" <<endl;
          
        return transit< Manipulate>();
    }
    else{
        cout << "exit" << endl;

        //return transit< Startup >();
        return transit< ReadyState >();
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//planning stage
int Manipulate::ActionOmniRight(){
    int repoflag = false;
    bool object_within_reach = false;
    bool target_within_reach = false;

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "==========================" << std::endl;
    std::cout << "ActionOmniRight" << std::endl;
    std::cout << "==========================" << std::endl;

    int Npts = 2;
    target.end_eff.poses.resize(Npts);

    cout << ">>>>> checking if target can be reached <<<<<" << endl;
    object_to_hand_orientation_offset = omni_orientation_right_hand;

    //position directly above obj
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;        //direct approach 90deg N to target
    target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y = object_in_base.pose.position.y;
    target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;
    //go down to obj
    target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[1].position.y = object_in_base.pose.position.y;
    target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.02;
    

    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    if(result->error_code == result->FAIL){
        cout << "direct grab plan FAILED" << endl;
        cout << "----------------------------" << endl;

        cout << ">>>>> try reposition grabbing direction <<<<<" <<endl;
        Npts = 2;
        target.end_eff.poses.clear();
        target.end_eff.poses.resize(Npts);
        object_to_hand_orientation_offset = omni_orientation_right_hand2;

        //use hand orientation 2, position above obj
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;        //if direct approach fails, tilt hand to approach target
        target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx2;
        target.end_eff.poses[0].position.y = object_in_base.pose.position.y + shifty2;
        target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;
        //go down to obj
        target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx2;
        target.end_eff.poses[1].position.y = object_in_base.pose.position.y + shifty2;
        target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.02;

        target.plan_only = true;
        ac.sendGoal(target);
        target.plan_only = false;
        ac.waitForResult(ros::Duration(20.0));
        result = ac.getResult();

        if(result->error_code == result->FAIL){
            cout << "reposition plan FAILED" << endl;
            cout << "object CANNOT be reached" << endl;
            cout << "----------------------------" << endl;
            object_within_reach = false;
        }
        else{
            cout << "reposition plan SUCCESS" << endl;
            cout << "object CAN be reached" << endl;
            cout << "+++++++++++++++++++++++++++++" << endl;
            repoflag = true;
            object_within_reach = true;
        }  
    }
    else{
        cout << "reaching plan SUCCESS" << endl;
        cout << "object CAN be reached" << endl;
        cout << "+++++++++++++++++++++++++++++" << endl;
        object_within_reach = true;
    }
    sleep(sleep_time);

    cout << ">>>>> checking if target can be reached <<<<<" << endl;
  
    Npts = 1;
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(Npts);
    //to target
    target.end_eff.poses[0].orientation = omni_orientation_target;              //set to target orientation
    target.end_eff.poses[0].position.x = target_in_base.pose.position.x + handoffx + finetune_x;
    target.end_eff.poses[0].position.y = target_in_base.pose.position.y + handoffy + finetune_y;
    target.end_eff.poses[0].position.z = target_in_base.pose.position.z + 0.02;
    
    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    
    if(result->error_code == result->FAIL){
        cout << "target CANNOT be reached" << endl;
        cout << "----------------------------" << endl;
        target_within_reach = false;
    }
    else{
        cout << "target CAN be reached" << endl;
        cout << "+++++++++++++++++++++++++++++" << endl;
        target_within_reach = true;
    }  
                    
    if(object_within_reach && target_within_reach){
        cout << "BOTH obj and target within reached using hand!" << endl;
        if(ExOmni(repoflag) < 0){
            return -1;
        }
    }
    else{   //use tool
        if(ExOmniTool(object_within_reach, target_within_reach) < 0){
            return -1;
        }
    }


    return 1;
}


int Manipulate::ExOmni(bool repoflag){
    //have to consider opening and closing fingers inbtwn waypoint [0,1] and [2]...
    cout << "==============================================" << endl;
    cout << "Executing the plan" << endl;

    cout << "Press Enter to Continue >> executing ExOmni()";
    cin.ignore();

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));

    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    //open fingers
    if(OpenFingers("right", 0) < 0)
        return -1;
    else
        sleep(sleep_time);

    int Npts = 2;
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(Npts);

    cout << ">>>>> direct grab with right hand <<<<<" << endl;
    //position directly above obj
    if(!repoflag){
        //position directly above obj
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;        //direct approach 90deg N to target
        target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[0].position.y = object_in_base.pose.position.y;
        target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;
        //go down to obj
        target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[1].position.y = object_in_base.pose.position.y;
        target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.01;
    }
    else{
        //use hand orientation 2, position above obj
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;        //if direct approach fails, tilt hand to approach target
        target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx2;
        target.end_eff.poses[0].position.y = object_in_base.pose.position.y + shifty2;
        target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;
        //go down to obj
        target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx2;
        target.end_eff.poses[1].position.y = object_in_base.pose.position.y + shifty2;
        target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.01;
    }

    cout << "position = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation = \n" << target.end_eff.poses[0].orientation << endl;
        cout << "position = \n" << target.end_eff.poses[1].position << endl;
    cout << "orientation = \n" << target.end_eff.poses[1].orientation << endl;
    
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        cout << "Execution FAILED!\n";
        
        Home();

        CloseFingers("right",0,0);
        sleep(sleep_time);

        return -1;
    }
    else{
        cout << "waypoint 1 & 2 DONE. \n";
    }

    // Grasp
    //=============================================
    if(closerequired){
        cout << "grabbing (pinching)" << endl;
        #ifdef USE_FINGER_ENC
            if(pinchFingers("right", diameter_size, 0) <0)
                return -1;
            else
                sleep(sleep_time);
        #else
            MoveFingers("right",0.8, 0);
            sleep(sleep_time);
        #endif
    }

    //to target
    Npts = 1;
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(Npts);

    target.end_eff.poses[0].orientation = omni_orientation_target;              //set to target orientation
    target.end_eff.poses[0].position.x = target_in_base.pose.position.x + handoffx + finetune_x;
    target.end_eff.poses[0].position.y = target_in_base.pose.position.y + handoffy + finetune_y;
    target.end_eff.poses[0].position.z = target_in_base.pose.position.z + 0.01;

    cout << "position = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation = \n" << target.end_eff.poses[0].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        cout << "Execution FAILED!\n";
        
        Home();
        CloseFingers("right",0,0);
        sleep(sleep_time);
        
        return -1;
    }
    else{
        cout << "waypoint 3 DONE. \n";
    }

    // Release
    //=============================================
    if(closerequired){
        cout << "Releasing" << endl;
        if(OpenFingers("right", 0)<0)
            return -1;
        else
            sleep(sleep_time);
    }

    cout << "Execution DONE. \n";
    cout << "==============================================" << endl;

    //perform retraction of arm
    target.end_eff.poses.resize(1);
    target.end_eff.poses[0].orientation = omni_orientation_target;  
    target.end_eff.poses[0].position.x = target_in_base.pose.position.x + handoffx*1.3;
    target.end_eff.poses[0].position.y = target_in_base.pose.position.y + handoffy*1.3;
    target.end_eff.poses[0].position.z = target_in_base.pose.position.z + 0.10;

    cout << "position = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation = \n" << target.end_eff.poses[0].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        cout << "Retract FAILED!\n";

        return -1;
    }
    else{
        cout << "Retract DONE. \n";

        //if retraction failed, dun close fingers on obj
        if(CloseFingers("right",0,0)<0)
            return -1;
        else
            sleep(sleep_time);
        
        Home();
    }

    return 1;
}

int Manipulate::ExOmniTool(bool object_within_reach, bool target_within_reach){
   //functionality = 1;//functionality-- 1:forw, 2:back , 3:side
   bool actionFail = false;

   if(boundary_back.empty())
        boundary_back = load_boundary(functionality);

    min_augment = compute_min_aug(boundary_back, object_in_base.pose.position, target_in_base.pose.position, object_within_reach, target_within_reach);
    Speech("We need a tool with a functionality of pulling back...");
    sleep(1.0);
    Speech("I have computed the minimum augmentation needed to reach the object and the goal.");
    sleep(1.0);

    TurnHeadTiltPan(13.3, 0.0);

    Speech("Let's see over here...");
    MoveWaist(WAIST_TOOL);

    cout << "functionality = " << functionality << endl;

    int tool_type;
    n.getParam("/tool_type", tool_type);

    switch(tool_type){
        
        case 1:{

            cout << "Press Enter to Continue >> check tool location";
            cin.ignore();

            CheckToolLocation();
 
            if(sticklength >= min_augment){
                Speech("Yes, it satisfies the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m greater or equal to minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;32mAugmentation requirement satisfied!\033[0m" << endl;

                Speech("I will pick it up to get a better view ...");

                cout << "tool_in_base pose = " << tool_in_base.pose << endl;
                cout << "Press Enter to Continue >> pcik up tool";
                cin.ignore();

                if(PickupToolRightArm() < 0)
                    return -1;
                else
                    sleep(sleep_time);

                object_to_tool_position_offset = pushforward_offset_right_hand;

                if(!ToolInHandCalibration("right")){
                    Speech("Sorry, I cannot get complete the tool calibration...");
                    cout << "Tool-in-hand calibration cannot be completed! \n";
                }
                else{
                    Speech("I will use this tool to complete the task.");
                    MoveWaist(WAIST_INIT);
                    sleep(sleep_time);

                    TurnHeadToObjTarCenter();

                    if((ActionOmniRight_Tool()) < 0)
                        actionFail = true;
                    else
                        sleep(sleep_time); // wait for real robot motion to finish.
                
                    Home();
                }

                ActionReturnToolRightArm();


                if(actionFail)
                    return -1;
            }
            else{
                Speech("No, it does not satisfy the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m less than minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;31mAugmentation requirement NOT satisfied.\033[0m" << endl;
            }
            break;
        }//end case 1
        default:{
            cout << "invalid tool type" << endl;
            return -1;
            break;
        }//end default
    }//end switch

    return 1;
}

int Manipulate::ActionOmniRight_Tool(){
    bool repoflag = false;

    cout << "==============================================" << endl;
    ROS_WARN("Planning with tool"); 

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "ActionOmniRight with TOOL" << std::endl;
    std::cout << "==========================" << std::endl;

    handoffx = -(PUCK_RADIUS) * std::sin(radian);
    handoffy = (PUCK_RADIUS) * std::cos(radian);

        cout << "offsetx = " << handoffx << endl;
        cout << "offsety = " << handoffy << endl;

    //try omni orientation_right_hand
    shiftx = - PUCK_RADIUS;
    shifty = 0;

    //direct grabbing
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(3);
    
    target.end_eff.poses[0].position = omnistraight.position;
    target.end_eff.poses[0].orientation = omnistraight.orientation;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.05;

    cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

    //postion directly above obj
    target.end_eff.poses[1].position = omnistraight.position;
    target.end_eff.poses[1].orientation = omnistraight.orientation;
    target.end_eff.poses[1].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[1].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[1].position.z += object_in_base.pose.position.z;

    cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
    cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;

    //to target
    target.end_eff.poses[2].position = object_to_hand_position_offset;
    target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;              //set to target orientation
    target.end_eff.poses[2].position.x += target_in_base.pose.position.x;
    target.end_eff.poses[2].position.y += target_in_base.pose.position.y;
    target.end_eff.poses[2].position.z += target_in_base.pose.position.z;

    cout << "position 3 = \n" << target.end_eff.poses[2].position << endl;
    cout << "orientation 3 = \n" << target.end_eff.poses[2].orientation << endl;

    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");

        cout << ">>>>> try reposition grabbing direction <<<<<" <<endl;
        target.end_eff.poses.clear();
        target.end_eff.poses.resize(3);

        //postion directly above obj
        target.end_eff.poses[0].position = object_to_hand_position_offset;
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[0].position.x += object_in_base.pose.position.x + handoffx;
        target.end_eff.poses[0].position.y += object_in_base.pose.position.y + handoffy;
        target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.05;

        cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
        cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

        //go down to obj
        target.end_eff.poses[1].position = object_to_hand_position_offset;
        target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[1].position.x += object_in_base.pose.position.x + handoffx;
        target.end_eff.poses[1].position.y += object_in_base.pose.position.y + handoffy;
        target.end_eff.poses[1].position.z += object_in_base.pose.position.z;

        cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
        cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;

         //to target
        target.end_eff.poses[2].position = object_to_hand_position_offset;
        target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;              //set to target orientation
        target.end_eff.poses[2].position.x += target_in_base.pose.position.x;
        target.end_eff.poses[2].position.y += target_in_base.pose.position.y;
        target.end_eff.poses[2].position.z += target_in_base.pose.position.z;

        cout << "position 3 = \n" << target.end_eff.poses[2].position << endl;
        cout << "orientation 3 = \n" << target.end_eff.poses[2].orientation << endl;       

        target.plan_only = true;
        ac.sendGoal(target);
        target.plan_only = false;
        ac.waitForResult(ros::Duration(20.0));
        result = ac.getResult();
        sleep(sleep_time);

        if(result->error_code == result->FAIL){
            cout << "reposition plan FAILED" << endl;
            return -1;
        }
        else{
            cout << "reposition plan SUCCESS" << endl;
            cout << "planning DONE. \n";
            repoflag = true;
        }  

    }
    else{
        cout << "planning DONE. \n";
    }


    cout << "planning completed >>> OKAY" << endl;
    cout << "==============================================" << endl;
    ROS_WARN("Executing with tool"); 

    cout << "Press Enter to Continue >> executing ExOmni_Tool";
    cin.ignore();
    
    
    //direct grabbing
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
    if(!repoflag){
        //postion directly above obj
        target.end_eff.poses[0].position = omnistraight.position;
        target.end_eff.poses[0].orientation = omnistraight.orientation;
        target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
        target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.05;
    }
    else{
        //go down to obj
        target.end_eff.poses[0].position = object_to_hand_position_offset;
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[0].position.x += object_in_base.pose.position.x + handoffx;
        target.end_eff.poses[0].position.y += object_in_base.pose.position.y + handoffy;
        target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.05;
    }

    cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

    //target.plan_only = true;
    ac.sendGoal(target);
    //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 1 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 1 DONE. \n";
    }

    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
    if(!repoflag){
        //postion directly above obj
        target.end_eff.poses[0].position = omnistraight.position;
        target.end_eff.poses[0].orientation = omnistraight.orientation;
        target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
        target.end_eff.poses[0].position.z += object_in_base.pose.position.z;
    }
    else{
        //go down to obj
        target.end_eff.poses[0].position = object_to_hand_position_offset;
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[0].position.x += object_in_base.pose.position.x + handoffx;
        target.end_eff.poses[0].position.y += object_in_base.pose.position.y + handoffy;
        target.end_eff.poses[0].position.z += object_in_base.pose.position.z;
    }

    cout << "position 2 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 2 = \n" << target.end_eff.poses[0].orientation << endl;

   //target.plan_only = true;
    ac.sendGoal(target);
   //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 2 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 2 DONE. \n";
    }

    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
    //to target
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;              //set to target orientation
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x;
    target.end_eff.poses[0].position.y += target_in_base.pose.position.y;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z;

    cout << "position 3 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 3 = \n" << target.end_eff.poses[0].orientation << endl;

    //target.plan_only = true;
    ac.sendGoal(target);
    //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 3 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 3 DONE. \n";
    } 

    cout << "execution done" << endl;
    cout << "++++++++++++++++++++++++" << endl;

    //perform retraction of arm
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);

    target.end_eff.poses[0].position = object_to_hand_position_offset;  
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;  
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x + handoffx*1.3;
    target.end_eff.poses[0].position.y += target_in_base.pose.position.y + handoffy*1.3;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z + 0.05;

    std::cout << "Retracting back with right hand...\n";

    cout << "position 4 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 4 = \n" << target.end_eff.poses[0].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        cout << "retraction with right hand FAILED!\n";
        //return -1;
    }
    else{
        Speech("Done!");
        cout << "retraction with right hand DONE. \n";
        //return 1;
    }

    return 1;
}





//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//south fore/backhand motion

sc::result Manipulate::react( const EvTargetSofObject & ){

    //reinitialise
    shiftx = 0;
    shifty = 0;

    finetune_x =0;
    finetune_y =0;

    //get current ee postion
    tf::Quaternion ee_Quat;
    tf::Vector3 ee_Vec;
    getCurrentTransform("ee", ee_Quat, ee_Vec);

    cout << "================" << endl;
       cout << "ee xyz:"  << ee_Vec[0] << ", " << ee_Vec[1] << ", " << ee_Vec[2] << endl;
       cout << "ee quat:"  << ee_Quat[0] << ", " << ee_Quat[1] << ", " << ee_Quat[2] << ", " << ee_Quat[3] << endl;
           cout << "================" << endl;



    float roll = PI/2, pitch = radian + (3.0/2.0 * PI), yaw = 0;
        float roll2 = PI/2, pitch2 = radian + (PI/2.0), yaw2 = 0;

    functionality = 2; //functionality-- 1:forw, 2:back , 3:side

    if(angle >= -20.0){
        cout << "going east" << endl;
        functionality = 3;
    }
    else if(angle >= -85.0 && angle < -20.0){                                    //SE
        cout << "gone southeast" << endl;
    }
    else if(angle >= -95.0 && angle < -85.0){                                        //s
        cout << "gone south" << endl;
    }
    else if(angle >= -160.0 && angle < -95.0){                                                     //SW
        cout << "gone southwest" << endl;
    }
    else{
        cout << "gone west" << endl;
        functionality = 3;
    }
    


    cout << "==========================================" << endl;
    cout << "re-adjust approach angle >>" << endl;
    cout << "angle hand to target:\n deg = " << pitch * (180.0/ PI) << " , radian = " << pitch << endl;
    cout << "reposition angle if required:\n deg = " << pitch2 * (180.0/ PI) << " , radian = " << pitch2 << endl;

    shiftx = -(side_objtohand) * std::sin(radian);
    shifty = (side_objtohand) * std::cos(radian);

    finetune_x = forehand_finetune * std::sin(radian);
    finetune_y = -forehand_finetune * std::cos(radian);


    cout << "shiftx=" << shiftx <<endl;
    cout << "shifty=" << shifty <<endl;
    cout << "finetune_x=" << finetune_x <<endl;
    cout << "finetune_y=" << finetune_y <<endl;
    cout << "==========================================" << endl;

    Quaternionf q;
    q = AngleAxisf(roll, Vector3f::UnitX())
        * AngleAxisf(pitch, Vector3f::UnitY())
        * AngleAxisf(yaw, Vector3f::UnitZ());

    backward_orientation_right_hand.w = q.w();
    backward_orientation_right_hand.x = q.x();
    backward_orientation_right_hand.y = q.y();
    backward_orientation_right_hand.z = q.z();

    cout << "q:  w=" << q.w() << " x=" << q.x() << " y=" << q.y() << " z=" << q.z() << endl;
    
    Quaternionf q2;
    q2 = AngleAxisf(roll2, Vector3f::UnitX())
        * AngleAxisf(pitch2, Vector3f::UnitY())
        * AngleAxisf(yaw2, Vector3f::UnitZ());

    backward_orientation_2_right_hand.w = q2.w();
    backward_orientation_2_right_hand.x = q2.x();
    backward_orientation_2_right_hand.y = q2.y();
    backward_orientation_2_right_hand.z = q2.z();

    cout << "q2:  w=" << q2.w() << " x=" << q2.x() << " y=" << q2.y() << " z=" << q2.z() << endl;


    if(ActionPushbackS() < 0)
    {
        failFlag = true;
        failcount++;
    }
    else{
        failFlag = false;
    }

    Home();

    cout << "++++++++++++++++++++++++++++++++" << endl;


    if(failFlag && failcount <= maxFail){
        cout << "re-attempt" <<endl;
        return transit< Manipulate>();
    }
    else{
        cout << "exit" << endl;
        return transit< ReadyState >();
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionPushbackS(){      //right arm only
    bool repoflag = false;
    bool object_within_reach = false;
    bool target_within_reach = false;
    
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    //actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac2("cart_single", true);
    //ac2.waitForServer(ros::Duration(20.0));

    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "==========================" << std::endl;
    std::cout << "ActionPushbackS" << std::endl;
    std::cout << "==========================" << std::endl;

    int Npts = 2;
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(Npts);

    cout << ">>>>> checking if object can be reached with right hand <<<<<" << endl;
    object_to_hand_orientation_offset = backward_orientation_right_hand;

    //position directly above obj
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y = object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;
    
        cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;
    //go down to obj
    target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[1].position.y = object_in_base.pose.position.y + shifty;
    target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.02;

        cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
    cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;
    
    /*
    //to target
    target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[2].position.x = target_in_base.pose.position.x + shiftx + finetune_x;
    target.end_eff.poses[2].position.y = target_in_base.pose.position.y + shifty + finetune_y;
    target.end_eff.poses[2].position.z = target_in_base.pose.position.z + 0.02;
    */

    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    if(result->error_code == result->FAIL){

        cout << "plan FAILED" << endl;
        cout << "----------------------------" << endl;
        
        cout << ">>>>> try Re-positioning <<<<<" << endl;
        sleep(sleep_time);

        Npts = 2;
        target.end_eff.poses.clear();
        target.end_eff.poses.resize(Npts);
        object_to_hand_orientation_offset = backward_orientation_2_right_hand;

        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[0].position.y = object_in_base.pose.position.y + shifty;
        target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;
        
        cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
        cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;
        //go down to obj
        target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[1].position.y = object_in_base.pose.position.y + shifty;
        target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.02;
        
        cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
        cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;
        /*
        //backhand push to target
        target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[2].position.x = target_in_base.pose.position.x + shiftx + finetune_x;
        target.end_eff.poses[2].position.y = target_in_base.pose.position.y + shifty + finetune_y;
        target.end_eff.poses[2].position.z = target_in_base.pose.position.z + 0.02;
        */

        target.plan_only = true;
        ac.sendGoal(target);
        target.plan_only = false;
        ac.waitForResult(ros::Duration(20.0));
        result = ac.getResult();
        
        if(result->error_code == result->FAIL){
            cout << "reposition plan FAILED" << endl;
            cout << "object CANNOT be reached" << endl;
            cout << "----------------------------" << endl;
            object_within_reach = false;
        }
        else{
            cout << "reposition plan SUCCESS" << endl;
            cout << "object CAN be reached" << endl;
            cout << "+++++++++++++++++++++++++++++" << endl;
            repoflag = true;
            object_within_reach = true;
        }  

    }
    else{
        cout << "reaching plan SUCCESS" << endl;
        cout << "object CAN be reached" << endl;
        cout << "+++++++++++++++++++++++++++++" << endl;
        object_within_reach = true;
    }  
    sleep(sleep_time);

    cout << ">>>>> checking if target can be reached <<<<<" << endl;

    Npts = 1;
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(Npts);
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x = target_in_base.pose.position.x + shiftx + finetune_x;
    target.end_eff.poses[0].position.y = target_in_base.pose.position.y + shifty + finetune_y;
    target.end_eff.poses[0].position.z = target_in_base.pose.position.z + 0.02;

    cout << "position 3 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 3 = \n" << target.end_eff.poses[0].orientation << endl;

    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    
    if(result->error_code == result->FAIL){
        cout << "target CANNOT be reached" << endl;
        cout << "----------------------------" << endl;
        target_within_reach = false;
    }
    else{
        cout << "target CAN be reached" << endl;
        cout << "+++++++++++++++++++++++++++++" << endl;
        target_within_reach = true;
    }  
    

    //decide to use tool or not                
    if(object_within_reach && target_within_reach){
        cout << "BOTH obj and target within reach; using hand!" << endl;
        if(ExSouth() < 0){
            return -1;
        }
    }
    else{   //use tool

        if(ExSouthTool(object_within_reach, target_within_reach) < 0){
            return -1;
        }
    }

    cout << "finished" << endl;
    return 1;
}

int Manipulate::ExSouth(){ 
    
    cout << "==============================================" << endl;
    cout << "Executing the plan" << endl;

    cout << "Press Enter to Continue >> executing ExSouth()";
    cin.ignore();

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));

    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    int Npts = 3;
    target.end_eff.poses.resize(Npts);

    //position directly above obj
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y = object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;
    //go down to obj
    target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[1].position.y = object_in_base.pose.position.y + shifty;
    target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.02;

    //to target
    target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[2].position.x = target_in_base.pose.position.x + shiftx + finetune_x;
    target.end_eff.poses[2].position.y = target_in_base.pose.position.y + shifty + finetune_y;
    target.end_eff.poses[2].position.z = target_in_base.pose.position.z + 0.02;

        cout << "position 1= \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1= \n" << target.end_eff.poses[0].orientation << endl;
        cout << "position 2= \n" << target.end_eff.poses[1].position << endl;
    cout << "orientation 2= \n" << target.end_eff.poses[1].orientation << endl;
        cout << "position 3= \n" << target.end_eff.poses[2].position << endl;
    cout << "orientation 3= \n" << target.end_eff.poses[2].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    if(result->error_code == result->FAIL){
        cout << "Execution FAILED!\n";
        return -1;
    }
    else{
        cout << "Execution DONE. \n";
    }

    sleep(sleep_time);
    cout << "==============================================" << endl;

    //perform retraction of arm
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x = target_in_base.pose.position.x + shiftx + 0.01;
    target.end_eff.poses[0].position.y = target_in_base.pose.position.y + shifty + 0.01;
    target.end_eff.poses[0].position.z = target_in_base.pose.position.z + 0.08;

    cout << "position 4= \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 4= \n" << target.end_eff.poses[0].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    if(result->error_code == result->FAIL){
        cout << "Retract with right hand FAILED!\n";
    }
    else{
        cout << "Retract with right hand DONE. \n";
    }
    sleep(sleep_time);

    return 1;
}

int Manipulate::ExSouthTool(bool object_within_reach, bool target_within_reach){ 
    //functionality = 2;//functionality-- 1:forw, 2:back , 3:side
   bool actionFail = false;

   if(boundary_back.empty())
        boundary_back = load_boundary(functionality);

    min_augment = compute_min_aug(boundary_back, object_in_base.pose.position, target_in_base.pose.position, object_within_reach, target_within_reach);
    Speech("We need a tool with a functionality of pulling back...");
    sleep(1.0);
    Speech("I have computed the minimum augmentation needed to reach the object and the goal.");
    sleep(1.0);

    TurnHeadTiltPan(13.3, 0.0);

    Speech("Let's see over here...");
    MoveWaist(WAIST_TOOL);

    //MoveWaist(30.0);
    cout << "functionality = " << functionality << endl;

    int tool_type;
    n.getParam("/tool_type", tool_type);
    switch(tool_type){
        // 3D-printed tools with handles
        //----------------------------------
        case 1:{

            cout << "Press Enter to Continue >> check tool location";
            cin.ignore();

            CheckToolLocation();

            if(sticklength >= min_augment){
                Speech("Yes, it satisfies the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m greater or equal to minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;32mAugmentation requirement satisfied!\033[0m" << endl;

                Speech("I will pick it up to get a better view ...");

                cout << "tool_in_base pose = " << tool_in_base.pose << endl;
                cout << "Press Enter to Continue >> pickup tool";
                cin.ignore();

                if(PickupToolRightArm() < 0)
                    return -1;
                else
                    sleep(sleep_time);

                object_to_tool_position_offset = pullback_offset_right_hand;

                if(!ToolInHandCalibration("right")){
                    Speech("Sorry, I cannot get complete the tool calibration...");
                    cout << "Tool-in-hand calibration cannot be completed! \n";

                }
                else{
                    Speech("I will use this tool to complete the task.");
                    MoveWaist(WAIST_INIT);

                    sleep(sleep_time);

                    TurnHeadToObjTarCenter();

                    if(ActionPushbackS_Tool() < 0)
                        actionFail = true;
                    else
                        sleep(sleep_time); // wait for real robot motion to finish.
                    
                    Home();
                }


                ActionReturnToolRightArm();
                if(actionFail)
                    return -1;
            }
            else{
                Speech("No, it does not satisfy the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m less than minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;31mAugmentation requirement NOT satisfied.\033[0m" << endl;
                return -1;
            }
            break;
        }//end case 1
        default:{
            cout << "invalid tool type" << endl;
            return -1;
            break;
        }//end default
    }//end switch

    return 1;
}

int Manipulate::ActionPushbackS_Tool(){ 
    bool repoflag = false;

    cout << "==============================================" << endl;
    ROS_WARN("Planning with tool"); 

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "ActionPushbackS with TOOL" << std::endl;
    std::cout << "==========================" << std::endl;

    //Below is reference to toolinhandcalibration only, dun uncomment
    //object_to_hand_position_offset = obj2hand.position;
    //object_to_hand_orientation_offset = obj2hand.orientation;
    shiftx = -(PUCK_RADIUS) * std::sin(radian);
    shifty = (PUCK_RADIUS) * std::cos(radian);

        cout << "offsetx = " << shiftx << endl;
        cout << "offsety = " << shifty << endl;

    target.end_eff.poses.clear();
    target.end_eff.poses.resize(3);

    //position directly above obj
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;  //shiftx and y max is 0.12 away
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.07;
    
    cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

    //go down to obj
    target.end_eff.poses[1].position = object_to_hand_position_offset;
    target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[1].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[1].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[1].position.z += object_in_base.pose.position.z + 0.02;
 
    cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
    cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;

    //tool to target
    target.end_eff.poses[2].position = object_to_hand_position_offset;
    target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[2].position.x += target_in_base.pose.position.x;
    target.end_eff.poses[2].position.y += target_in_base.pose.position.y;
    target.end_eff.poses[2].position.z += target_in_base.pose.position.z + 0.02;

    cout << "position 3 = \n" << target.end_eff.poses[2].position << endl;
    cout << "orientation 3 = \n" << target.end_eff.poses[2].orientation << endl;

    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));

    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");

        cout << ">>>>> try reposition grabbing direction <<<<<" <<endl;
        target.end_eff.poses.clear();
        target.end_eff.poses.resize(3);

        //position directly above obj
        target.end_eff.poses[0].position = object_to_hand_position_offset;
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;  //shiftx and y max is 0.12 away
        target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
        target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.07;
        
        cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
        cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

        //go down to obj
        target.end_eff.poses[1].position = object_to_hand_position_offset;
        target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[1].position.x += object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[1].position.y += object_in_base.pose.position.y + shifty;
        target.end_eff.poses[1].position.z += object_in_base.pose.position.z + 0.02;
     
        cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
        cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;

        //tool to target
        target.end_eff.poses[2].position = omnistraight.position;
        target.end_eff.poses[2].orientation = omnistraight.orientation;
        target.end_eff.poses[2].position.x += target_in_base.pose.position.x;
        target.end_eff.poses[2].position.y += target_in_base.pose.position.y;
        target.end_eff.poses[2].position.z += target_in_base.pose.position.z + 0.02;

        cout << "position 3 = \n" << target.end_eff.poses[2].position << endl;
        cout << "orientation 3 = \n" << target.end_eff.poses[2].orientation << endl;   

        target.plan_only = true;
        ac.sendGoal(target);
        target.plan_only = false;
        ac.waitForResult(ros::Duration(20.0));
        result = ac.getResult();
        sleep(sleep_time);

        if(result->error_code == result->FAIL){
            cout << "reposition plan FAILED" << endl;
            return -1;
        }
        else{
            cout << "reposition plan SUCCESS" << endl;
            cout << "planning DONE. \n";
            repoflag = true;
        }  
    }
    else{
        cout << "Planning DONE. \n";
    }







    cout << "planning completed >>> OKAY" << endl;
    cout << "==============================================" << endl;
    ROS_WARN("Executing with tool"); 

    cout << "Press Enter to Continue >> executing ExSouth_Tool";
    cin.ignore();
    
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
        //postion directly above obj
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;  //shiftx and y max is 0.12 away
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.07;
    
    cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

    //target.plan_only = true;
    ac.sendGoal(target);
    //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 1 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 1 DONE. \n";
    }


    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
        //go down to obj
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.02;

    cout << "position 2 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 2 = \n" << target.end_eff.poses[0].orientation << endl;

    //target.plan_only = true;
    ac.sendGoal(target);
    //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 2 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 2 DONE. \n";
    }

    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
        //to target
    if(!repoflag){
        target.end_eff.poses[0].position = object_to_hand_position_offset;
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    }
    else{
        target.end_eff.poses[0].position = omnistraight.position;
        target.end_eff.poses[0].orientation = omnistraight.orientation;
    }
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x;
    target.end_eff.poses[0].position.y += target_in_base.pose.position.y;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z + 0.02;

    cout << "position 3 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 3 = \n" << target.end_eff.poses[0].orientation << endl;

    //target.plan_only = true;
    ac.sendGoal(target);
    //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 3 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 3 DONE. \n";
    } 

    cout << "execution done" << endl;
    cout << "++++++++++++++++++++++++" << endl;

    //retract and avoid
    // via point to avoid disturbing the object during next move
    //==========================================================
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);

    if(!repoflag){
        target.end_eff.poses[0].position = object_to_hand_position_offset;
        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    }
    else{
        target.end_eff.poses[0].position = omnistraight.position;
        target.end_eff.poses[0].orientation = omnistraight.orientation;
    }
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x + shiftx*1.3;  //shiftx and y max is 0.12 away
    target.end_eff.poses[0].position.y += target_in_base.pose.position.y + shifty*1.3;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z + 0.07;

    std::cout << "Retracting back with right hand...\n";
    //std::cout << target.end_eff.poses[0] << std::endl;

    cout << "position 4 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 4 = \n" << target.end_eff.poses[0].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        cout << "retraction with right hand FAILED!\n";
        //return -1;
    }
    else{
        Speech("Done!");
        cout << "retraction with right hand DONE. \n";
        //return 1;
    }
    return 1;
}






//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//north back/forehand motion event

sc::result Manipulate::react( const EvTargetNofObject & ){

    //reinitialise
    shiftx = 0;
    shifty = 0;

    finetune_x =0;
    finetune_y =0;
    finetune_x2 =0;
    finetune_y2 =0;

    //get current ee postion
    tf::Quaternion ee_Quat;
    tf::Vector3 ee_Vec;
    getCurrentTransform("ee", ee_Quat, ee_Vec);

    cout << "================" << endl;
       cout << "ee xyz:"  << ee_Vec[0] << ", " << ee_Vec[1] << ", " << ee_Vec[2] << endl;
       cout << "ee quat:"  << ee_Quat[0] << ", " << ee_Quat[1] << ", " << ee_Quat[2] << ", " << ee_Quat[3] << endl;
           cout << "================" << endl;

 
    float roll = PI/2, pitch = radian + (PI/2.0), yaw = 0;
        float roll2 = PI/2, pitch2 = radian - (PI/2.0), yaw2 = 0;

    functionality = 1;//functionality-- 1:forw, 2:back , 3:side

    if(angle <= 20.0){
        cout << "going east" << endl;
        functionality = 3;
    }
    else if(angle <= 85.0 && angle > 20.0){                                    //SE
        cout << "gone Northeast" << endl;
    }
    else if(angle <= 95.0 && angle > 85.0){                                        //s
        cout << "gone North" << endl;
    }
    else if(angle <= 160.0 && angle > 95.0){                                                     //SW
        cout << "gone Northwest" << endl;
    }
    else{
        cout << "gone west" << endl;
        functionality = 3;
    }

    cout << "==========================================" << endl;
    cout << "re-adjust approach angle >>" << endl;
    cout << "angle hand to target:\n deg = " << pitch * (180.0/ PI) << " , radian = " << pitch << endl;
    cout << "reposition angle if required:\n deg = " << pitch2 * (180.0/ PI) << " , radian = " << pitch2 << endl;

    shiftx = -(side_objtohand) * std::sin(radian);
    shifty = (side_objtohand) * std::cos(radian);

    finetune_x = backhand_finetune * std::sin(radian);
    finetune_y = -backhand_finetune * std::cos(radian);
    finetune_x2 = forehand_finetune * std::sin(radian);
    finetune_y2 = -forehand_finetune * std::cos(radian);



    cout << "shiftx=" << shiftx <<endl;
    cout << "shifty=" << shifty <<endl;
    cout << "finetune_x=" << finetune_x <<endl;
    cout << "finetune_y=" << finetune_y <<endl;
    cout << "==========================================" << endl;

    Quaternionf q;
    q = AngleAxisf(roll, Vector3f::UnitX())
        * AngleAxisf(pitch, Vector3f::UnitY())
        * AngleAxisf(yaw, Vector3f::UnitZ());

    forward_orientation_right_hand.w = q.w();
    forward_orientation_right_hand.x = q.x();
    forward_orientation_right_hand.y = q.y();
    forward_orientation_right_hand.z = q.z();

    cout << "q:  w=" << q.w() << " x=" << q.x() << " y=" << q.y() << " z=" << q.z() << endl;
        
    Quaternionf q2;
    q2 = AngleAxisf(roll2, Vector3f::UnitX())
        * AngleAxisf(pitch2, Vector3f::UnitY())
        * AngleAxisf(yaw2, Vector3f::UnitZ());

    forward_orientation_2_right_hand.w = q2.w();
    forward_orientation_2_right_hand.x = q2.x();
    forward_orientation_2_right_hand.y = q2.y();
    forward_orientation_2_right_hand.z = q2.z();

    cout << "q2:  w=" << q2.w() << " x=" << q2.x() << " y=" << q2.y() << " z=" << q2.z() << endl;

    if(ActionPushForwardN() < 0)
    {
        failFlag = true;
        failcount++;
    }
    else{
        failFlag = false;
    }
    Home();

    cout << "++++++++++++++++++++++++++++++++" << endl;


    if(failFlag && failcount <= maxFail){
        cout << "re-attempt" <<endl;
        return transit< Manipulate>();
    }
    else{
        cout << "exit" << endl;
        return transit< ReadyState >();
    }
}
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
int Manipulate::ActionPushForwardN(){ 
    bool repoflag = false;
    bool object_within_reach = false;
    bool target_within_reach = false;

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    //actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac2("task_single", true);
    //ac2.waitForServer(ros::Duration(20.0));

    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "==========================" << std::endl;
    std::cout << "ActionPushForwardN" << std::endl;
    std::cout << "==========================" << std::endl;

    int Npts = 2;
    target.end_eff.poses.resize(Npts);

    cout << ">>>>> checking if object can be reached with right hand <<<<<" << endl;
    object_to_hand_orientation_offset = forward_orientation_right_hand; //backhand orientation

    //position directly above obj
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y = object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;

    cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

    //go down to obj
    target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[1].position.y = object_in_base.pose.position.y + shifty;
    target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.02;

    cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
    cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;
    
    /*//backhand push to target
    target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[2].position.x = target_in_base.pose.position.x + shiftx + finetune_x2;
    target.end_eff.poses[2].position.y = target_in_base.pose.position.y + shifty + finetune_y2;
    target.end_eff.poses[2].position.z = target_in_base.pose.position.z + 0.02;
    */

    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    if(result->error_code == result->FAIL){

        cout << "plan FAILED" << endl;
        cout << "----------------------------" << endl;

        cout << ">>>>> try Re-positioning <<<<<" << endl;
        sleep(sleep_time);

        Npts = 2;
        target.end_eff.poses.clear();
        target.end_eff.poses.resize(Npts);
        object_to_hand_orientation_offset = forward_orientation_2_right_hand; //forehand orientation

        target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[0].position.y = object_in_base.pose.position.y + shifty;
        target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;

        cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
        cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

        //go down to obj
        target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx;
        target.end_eff.poses[1].position.y = object_in_base.pose.position.y + shifty;
        target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.02;

        cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
        cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;
        
        /*//forehand push to target
        target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;
        target.end_eff.poses[2].position.x = target_in_base.pose.position.x + shiftx + finetune_x;
        target.end_eff.poses[2].position.y = target_in_base.pose.position.y + shifty + finetune_y;
        target.end_eff.poses[2].position.z = target_in_base.pose.position.z + 0.02;
        */

        target.plan_only = true;
        ac.sendGoal(target);
        target.plan_only = false;
        ac.waitForResult(ros::Duration(20.0));
        result = ac.getResult();
        
        if(result->error_code == result->FAIL){
            cout << "reposition plan FAILED" << endl;
            cout << "object CANNOT be reached" << endl;
            cout << "----------------------------" << endl;
            object_within_reach = false;
        }
        else{
            cout << "reposition plan SUCCESS" << endl;
            cout << "object CAN be reached" << endl;
            cout << "+++++++++++++++++++++++++++++" << endl;
            repoflag = true;
            object_within_reach = true;
        }  

    }
    else{
        cout << "reaching plan SUCCESS" << endl;
        cout << "object CAN be reached" << endl;
        cout << "+++++++++++++++++++++++++++++" << endl;
        object_within_reach = true;
    }  
    sleep(sleep_time);

    cout << ">>>>> checking if target can be reached <<<<<" << endl;

    //forehand push to target
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x = target_in_base.pose.position.x + shiftx + finetune_x2;
    target.end_eff.poses[0].position.y = target_in_base.pose.position.y + shifty + finetune_y2;
    target.end_eff.poses[0].position.z = target_in_base.pose.position.z + 0.02;

    cout << "position 3 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 3 = \n" << target.end_eff.poses[0].orientation << endl;

    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    
    if(result->error_code == result->FAIL){
        cout << "target CANNOT be reached" << endl;
        cout << "----------------------------" << endl;
        target_within_reach = false;
    }
    else{
        cout << "target CAN be reached" << endl;
        cout << "+++++++++++++++++++++++++++++" << endl;
        target_within_reach = true;
    }  


    //decide to use tool or not                
    if(object_within_reach && target_within_reach){
        cout << "BOTH obj and target within reach; using hand!" << endl;
        if(ExNorth() < 0){
            return -1;
        }
    }
    else{   //use tool

        if(ExNorthTool(object_within_reach, target_within_reach) < 0){
            return -1;
        }
    }


    cout << "finished" << endl;
    return 1;
}

int Manipulate::ExNorth(){ 

    cout << "==============================================" << endl;
    cout << "Executing the plan" << endl;

    cout << "Press Enter to Continue >> executing ExNorth()";
    cin.ignore();
    
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    int Npts = 3;
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(Npts);

    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y = object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.10;
    //go down to obj
    target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[1].position.x = object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[1].position.y = object_in_base.pose.position.y + shifty;
    target.end_eff.poses[1].position.z = object_in_base.pose.position.z + 0.02;
    
    //forehand push to target
    target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[2].position.x = target_in_base.pose.position.x + shiftx + finetune_x;
    target.end_eff.poses[2].position.y = target_in_base.pose.position.y + shifty + finetune_y;
    target.end_eff.poses[2].position.z = target_in_base.pose.position.z + 0.02;

        cout << "position 1= \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1= \n" << target.end_eff.poses[0].orientation << endl;
        cout << "position 2= \n" << target.end_eff.poses[1].position << endl;
    cout << "orientation 2= \n" << target.end_eff.poses[1].orientation << endl;
        cout << "position 3= \n" << target.end_eff.poses[2].position << endl;
    cout << "orientation 3= \n" << target.end_eff.poses[2].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    if(result->error_code == result->FAIL){
        cout << "Execution FAILED!\n";
        return -1;
    }
    else{
        cout << "Execution DONE. \n";
    }

    sleep(sleep_time);
    cout << "==============================================" << endl;

    // via point to avoid disturbing the object during next move             //retract hand
    //==========================================================
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x = target_in_base.pose.position.x + shiftx - 0.01;
    target.end_eff.poses[0].position.y = target_in_base.pose.position.y + shifty - 0.01;
    target.end_eff.poses[0].position.z = target_in_base.pose.position.z + 0.08;

        cout << "position 4= \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 4= \n" << target.end_eff.poses[0].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    if(result->error_code == result->FAIL){
        cout << "Retract with right hand FAILED!\n";
    }
    else{
        cout << "Retract with right hand DONE. \n";
    }
    sleep(sleep_time);
    
    return 1;
}

int Manipulate::ExNorthTool(bool object_within_reach, bool target_within_reach){ 
    //functionality = 3;
    bool actionFail = false;

    if(boundary_side.empty())
        boundary_side = load_boundary(functionality);
    
    min_augment = compute_min_aug(boundary_side, object_in_base.pose.position, target_in_base.pose.position, object_within_reach, target_within_reach);
    Speech("We need a tool with a functionality of pushing sideways ...");
    sleep(1.0);
    Speech("I have computed the minimum augmentation needed to reach the object and the goal.");
    sleep(1.0);

    TurnHeadTiltPan(13.3, 0.0);

    Speech("Let's see over here...");
    MoveWaist(WAIST_TOOL);

    cout << "functionality = " << functionality << endl;

    int tool_type;
    n.getParam("/tool_type", tool_type);
    switch(tool_type){
        // 3D-printed tools with handles
        //----------------------------------
        case 1:{

            cout << "Press Enter to Continue >> check tool location";
            cin.ignore();

            CheckToolLocation();

            if(sticklength >= min_augment){
                Speech("Yes, it satisfies the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m greater or equal to minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;32mAugmentation requirement satisfied!\033[0m" << endl;
                
                Speech("I will pick it up to get a better view ...");

                cout << "tool_in_base pose = " << tool_in_base.pose << endl;
                cout << "Press Enter to Continue >> pcik up tool";
                cin.ignore();

                if(PickupToolRightArm() < 0)
                    return -1;
                else
                    sleep(sleep_time);

                object_to_tool_position_offset = pushsideways_offset_right_hand;

                if(!ToolInHandCalibration("right")){
                    Speech("Sorry, I cannot get complete the tool calibration...");
                    cout << "Tool-in-hand calibration cannot be completed! \n";
    //                cout << "Press Enter to Continue";
    //                cin.ignore();
                }
                else{
                    Speech("I will use this tool to complete the task.");
                    MoveWaist(WAIST_INIT);
    //                cout << "Press Enter to Continue";
    //                cin.ignore();
                    sleep(sleep_time);

                    TurnHeadToObjTarCenter();

                    if(ActionPushForwardN_Tool() < 0)
                        actionFail = true;
                    else
                        sleep(sleep_time); // wait for real robot motion to finish.
                    
                    Home();
                }

    //            CheckFakeToolTransform();
    //            MoveWaist(WAIST_INIT);
    //            cout << "Press Enter to Continue";
    //            cin.ignore();
    //            if(ActionPushSidewaysRightArm()==0)
    //                sleep(sleep_time); // wait for real robot motion to finish.



//                for(int i=1; i<=4; i++){
//                    cout << "iteration " << i+1 << endl;
//                    CheckObjectLocation();
//                    if(ActionPushSidewaysRightArm()==0)
//                        sleep(sleep_time);
//                }
    //            cout << "Press Enter to Release Tool!!";
    //            cin.ignore();
                ActionReturnToolRightArm();
                //post_event(EvPerceiveObject());
                if(actionFail)
                    return -1;
            }
            else{
                Speech("No, it does not satisfy the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m less than minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;31mAugmentation requirement NOT satisfied.\033[0m" << endl;
                return -1;
            }
            break;
        }//end case 1
        default:{
            cout << "invalid tool type" << endl;
            return -1;
            break;
        }//end default
    }//end switch
    
    return 1;
}

int Manipulate::ActionPushForwardN_Tool(){ 

    cout << "==============================================" << endl;
    ROS_WARN("Planning with tool"); 

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "ActionPushForwardN with TOOL" << std::endl;
    std::cout << "==========================" << std::endl;

    shiftx = -(PUCK_RADIUS) * std::sin(radian);
    shifty = (PUCK_RADIUS) * std::cos(radian);

    cout << "offsetx = " << shiftx << endl;
        cout << "offsety = " << shifty << endl;

    target.end_eff.poses.clear();
    target.end_eff.poses.resize(3);

     //position directly above obj
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.05;
    
    cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

    //go down to obj
    target.end_eff.poses[1].position = object_to_hand_position_offset;
    target.end_eff.poses[1].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[1].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[1].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[1].position.z += object_in_base.pose.position.z;

    cout << "position 2 = \n" << target.end_eff.poses[1].position << endl;
    cout << "orientation 2 = \n" << target.end_eff.poses[1].orientation << endl;

    //tool to target
    target.end_eff.poses[2].position = object_to_hand_position_offset;    
    target.end_eff.poses[2].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[2].position.x += target_in_base.pose.position.x;
    target.end_eff.poses[2].position.y += target_in_base.pose.position.y;
    target.end_eff.poses[2].position.z += target_in_base.pose.position.z;
 
     cout << "position 3 = \n" << target.end_eff.poses[2].position << endl;
    cout << "orientation 3 = \n" << target.end_eff.poses[2].orientation << endl;

    target.plan_only = true;
    ac.sendGoal(target);
    target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Planning FAILED!\n";
        return -1;
    }
    else{
        cout << "Planning DONE. \n";
    }

    cout << "planning completed >>> OKAY" << endl;
    cout << "==============================================" << endl;
    ROS_WARN("Executing with tool"); 
    
    cout << "Press Enter to Continue >> executing ExNorth_tool";
    cin.ignore();

    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
        //postion directly above obj
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.05;

    cout << "position 1 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 1 = \n" << target.end_eff.poses[0].orientation << endl;

    //target.plan_only = true;
    ac.sendGoal(target);
    //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 1 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 1 DONE. \n";
    }

    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
        //go down to obj
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z;

    cout << "position 2 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 2 = \n" << target.end_eff.poses[0].orientation << endl;

    //target.plan_only = true;
    ac.sendGoal(target);
    //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 2 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 2 DONE. \n";
    }

    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
        //to target
    target.end_eff.poses[0].position = object_to_hand_position_offset;    
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x;
    target.end_eff.poses[0].position.y += target_in_base.pose.position.y;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z;

    cout << "position 3 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 3 = \n" << target.end_eff.poses[0].orientation << endl;

    //target.plan_only = true;
    ac.sendGoal(target);
    //target.plan_only = false;
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Motion 3 FAILED!\n";
        return -1;
    }
    else{
        cout << "Motion 3 DONE. \n";
    } 

    cout << "execution done" << endl;
    cout << "++++++++++++++++++++++++" << endl;

    //retract and avoid
    // via point to avoid disturbing the object during next move             //retract hand
    //==========================================================
    target.end_eff.poses.clear();
    target.end_eff.poses.resize(1);
    
    target.end_eff.poses[0].position = object_to_hand_position_offset;    
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x + shiftx*1.3;
    target.end_eff.poses[0].position.y += target_in_base.pose.position.y + shifty*1.3;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z + 0.05;

    std::cout << "Retracting back with right hand...\n";
    //std::cout << target.end_eff.poses[0] << std::endl;

    cout << "position 4 = \n" << target.end_eff.poses[0].position << endl;
    cout << "orientation 4 = \n" << target.end_eff.poses[0].orientation << endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);

    if(result->error_code == result->FAIL){
        cout << "retraction with right hand FAILED!\n";
        //return -1;
    }
    else{
        Speech("Done!");
        cout << "retraction with right hand DONE. \n";
        //return 1;
    }
    return 1;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//findWest

int Manipulate::ActionPushSidewaysRightArm(){

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "ActionPushSidewaysRightArm" << std::endl;
    std::cout << "==========================" << std::endl;

    int tool_type;
    n.getParam("/tool_type", tool_type);
    if(tool_type == 4){
        object_to_hand_orientation_offset = bendtool_orientation_right_hand;
    }

    int Npts = 1;
    target.end_eff.poses.resize(Npts);
    double shifty = -0.025;

    // Pre-positioning
    //=============================================
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x ;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z + 0.1;
    //target.end_eff.poses[0].position.z += 1.0 ;

    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Pre-positioning with right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Pre-positioning with right hand DONE. \n";
    }
    
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x ;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z ;


    std::cout << "Pre-positioning target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Pre-positioning with right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Pre-positioning with right hand DONE. \n";
    }


    //cout << "Press Enter to Continue";
    //cin.ignore();

    // Push sideways
    //=============================================

    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x ;
    target.end_eff.poses[0].position.y += target_in_base.pose.position.y -0.03;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z ;

    std::cout << "Push sideways target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
      Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
      cout << "Push sideways with right hand FAILED!\n";
      return -1;
    }
    else{
      cout << "Push sideways with right hand DONE. \n";
    }


    //cout << "Press Enter to Continue";
    //cin.ignore();


    // via point to avoid disturbing the object during next move
    //==========================================================
    
    //double shiftz = 0.15;
    //target.end_eff.poses[0].position.z = 0.9;
//    target.end_eff.poses[0].position.y = object_in_base.pose.position.y - 0.05;
//    target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.05;
    target.end_eff.poses[0].position.y += -0.05;
    target.end_eff.poses[0].position.z += 0.1; //0.05
    target.end_eff.poses[0].orientation.w = 0.5;
    target.end_eff.poses[0].orientation.x = 0.5;
    target.end_eff.poses[0].orientation.y = 0.5;
    target.end_eff.poses[0].orientation.z = 0.5;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Retract with right hand FAILED!\n";
        return -1;
    }
    else{
        Speech("Done!");
        cout << "Retract with right hand DONE. \n";
        object_in_base.pose.position.y = target_in_base.pose.position.y;
        object_in_base.pose.position.z = target_in_base.pose.position.z;
        
    }
    Home();
    return 0;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionReturnToolRightArmSpecial(){
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    std::cout << "ActionReturnToolRightArm" << std::endl;
    std::cout << "==========================" << std::endl;


    // Moving tool to the side
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);

    target.end_eff.poses[0].position.x = 0.08;
    target.end_eff.poses[0].position.y = -0.45;
    target.end_eff.poses[0].position.z = TABLE_HEIGHT + TOOL_HANDLE_HEIGHT - HAND_THICKNESS/3.0 + 0.03;
    target.end_eff.poses[0].orientation.w = 0.5;
    target.end_eff.poses[0].orientation.x = 0.5;
    target.end_eff.poses[0].orientation.y = 0.5;
    target.end_eff.poses[0].orientation.z = 0.5;

    std::cout << "Moving tool to right side...\n";
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Moving tool to right side FAILED!\n";
        return -1;
    }
    else{
        cout << "Moving tool to right side DONE. \n";
    }

    //Open fingers
    //============================================
    target_fingers.trajectory.joint_names = fillJointNamesFingers("right");
    trajectory_msgs::JointTrajectoryPoint finger_joints;

    finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());
    finger_joints.positions[0] = 1.3;
    finger_joints.positions[1] = 1.59;
    finger_joints.positions[2] = 0.0;
    finger_joints.positions[3] = -finger_joints.positions[0];
    finger_joints.positions[4] = -finger_joints.positions[1];
    finger_joints.positions[5] = -finger_joints.positions[2];
    target_fingers.trajectory.points.push_back(finger_joints);

    std::cout << "Opening fingers in right hand... \n";
    for(int i; i<target_fingers.trajectory.joint_names.size(); i++)
        std::cout << target_fingers.trajectory.joint_names[i] << std::endl;

    ac_fingers.sendGoal(target_fingers);
    ac_fingers.waitForResult(ros::Duration(20.0));
    result_fingers = ac_fingers.getResult();
    if(result_fingers->error_code == result_fingers->FAIL){
        cout << "Opening fingers in right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Opening fingers in right hand DONE. \n";
    }

    // Moving hand away from tool
    //=============================================
    Npts=1;
    target.end_eff.poses.resize(Npts);

    target.end_eff.poses[0].position.x = 0.03;
    target.end_eff.poses[0].position.y = -0.45;
    target.end_eff.poses[0].position.z = 1.0;
    target.end_eff.poses[0].orientation.w = 0.5;
    target.end_eff.poses[0].orientation.x = 0.5;
    target.end_eff.poses[0].orientation.y = 0.5;
    target.end_eff.poses[0].orientation.z = 0.5;

    std::cout << "Moving hand away from tool...\n";
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "hand away from tool FAILED!\n";
        return -1;
    }
    else{
        cout << "hand away from tool DONE. \n";
    }

    //Close fingers
    //============================================
    finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());
    finger_joints.positions[0] = 0.0;
    finger_joints.positions[1] = 0.0;
    finger_joints.positions[2] = 0.0;
    finger_joints.positions[3] = -finger_joints.positions[0];
    finger_joints.positions[4] = -finger_joints.positions[1];
    finger_joints.positions[5] = -finger_joints.positions[2];
    target_fingers.trajectory.points.clear();
    target_fingers.trajectory.points.push_back(finger_joints);

    std::cout << "Closing fingers in right hand... \n";

    ac_fingers.sendGoal(target_fingers);
    ac_fingers.waitForResult(ros::Duration(20.0));
    result_fingers = ac_fingers.getResult();
    if(result_fingers->error_code == result_fingers->FAIL){
        cout << "Closing fingers in right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Closing fingers in right hand DONE. \n";
        return 0;
    }

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionReturnBendTool(){
    //Open right fingers
    //============================================
    int ret = OpenFingers("right",1);
    if(ret !=0){
        cout << "Open right fingers FAILED! \n";
    }
    else
        cout << "Open right fingers DONE. \n";

    //Close right hand
    //============================================
    int grasp_type = 2; // for bendtool
    ret = CloseFingers("right", grasp_type,0);

    if(ret ==0)
        cout << "Close left hand DONE. \n";
    else{
        cout << "Close left handFAILED! \n";
    }

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionReturnToolRightArm(){
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    Speech("Returning the tool ...");
    std::cout << "ActionReturnToolRightArm" << std::endl;
    std::cout << "==========================" << std::endl;


    MoveWaist(WAIST_TOOL);

    // Moving tool to drop position
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);

    target.end_eff.poses[0].position.x = -0.33;//0.33
    target.end_eff.poses[0].position.y = -0.45;//-0.4
    target.end_eff.poses[0].position.z = 0.83;//TABLE_HEIGHT + TOOL_HANDLE_HEIGHT - HAND_THICKNESS/3.0 + 0.03;
    target.end_eff.poses[0].orientation.w = 0.70711;//0.5
    target.end_eff.poses[0].orientation.x = sqrt(1.0-0.70711*0.70711);//0.5
    target.end_eff.poses[0].orientation.y = 0.0;//0.5
    target.end_eff.poses[0].orientation.z = 0.0;//0.5

    std::cout << "Moving tool to drop position...\n";
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Moving tool to drop position FAILED!\n";
        return -1;
    }
    else{
        cout << "Moving tool to drop position DONE. \n";
    }

    //Open fingers
    //============================================

    if(OpenFingers("right", 1)<0)
        return -1;
    else
        sleep(sleep_time);

  

    // Moving hand away from tool
    //=============================================
    Npts=1;
    target.end_eff.poses.resize(Npts);

    target.end_eff.poses[0].position.x = -0.33;
    target.end_eff.poses[0].position.y = -0.32;
    target.end_eff.poses[0].position.z = 0.83;
    target.end_eff.poses[0].orientation.w = 0.70711;
    target.end_eff.poses[0].orientation.x = sqrt(1.0-0.70711*0.70711);
    target.end_eff.poses[0].orientation.y = 0.0;
    target.end_eff.poses[0].orientation.z = 0.0;

    target.end_eff.poses[0].position.x += -0.03; //samuel: finger might hit tool, back abit more

    std::cout << "Moving hand away from tool...\n";
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "hand away from tool FAILED!\n";
        return -1;
    }
    else{
        Speech("Done!");
        cout << "hand away from tool DONE. \n";
    }

    cout << "press enter to continue >> return home" << endl;
    cin.ignore();

    //Close fingers
    //============================================
    if(CloseFingers("right",0,0)<0)
        return -1;
    else
        sleep(sleep_time);


    Home();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionPushForwardLeftArm(){


}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//FindNorth

int Manipulate::ActionPushForwardRightArm(bool using_tool){
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
//    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers_encoders", true);
    ac_fingers.waitForServer(ros::Duration(20.0));


    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    std::cout << "ActionPushForwardRightArm" << std::endl;
    std::cout << "==========================" << std::endl;

    double shiftx = -0.1;
    double shifty = 0.0;

    // Pre-positioning
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);

    //first point is a via point to avoid moving the object when pre-positioning
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z;

    cout << "Pre-positioning with right hand...\n";
    cout << target.end_eff.poses[0] << endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    sleep(sleep_time);
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Pre-positioning with right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Pre-positioning with with right hand DONE. \n";
    }

    trajectory_msgs::JointTrajectoryPoint finger_joints;
    target_fingers.trajectory.joint_names = fillJointNamesFingers("right");
//    target_fingers.trajectory.joint_names = fillJointNamesFingersEncoders("right");
    if(!using_tool){
        //Open fingers
        //============================================
        finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());

        // for encoders
//        finger_joints.positions[0] = 30.0;
//        finger_joints.positions[1] = 30.0;

        // for no encoders
        finger_joints.positions[0] = 1.3;
        finger_joints.positions[1] = 1.59;
        finger_joints.positions[2] = 0.0;
        finger_joints.positions[3] = -finger_joints.positions[0];
        finger_joints.positions[4] = -finger_joints.positions[1];
        finger_joints.positions[5] = -finger_joints.positions[2];

        target_fingers.trajectory.points.push_back(finger_joints);

        std::cout << "Opening fingers in right hand... \n";
        for(int i; i<target_fingers.trajectory.joint_names.size(); i++)
            std::cout << target_fingers.trajectory.joint_names[i] << std::endl;

        ac_fingers.sendGoal(target_fingers);
        ac_fingers.waitForResult(ros::Duration(20.0));
        result_fingers = ac_fingers.getResult();
        if(result_fingers->error_code == result_fingers->FAIL){
            cout << "Opening fingers in right hand FAILED!\n";
            return -1;
        }
        else{
            Speech("Done!");
            cout << "Opening fingers in right hand DONE. \n";
        }
    }


    // Push forward
    //=============================================

    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += target_in_base.pose.position.x ;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y  ;
    target.end_eff.poses[0].position.z += target_in_base.pose.position.z ;

    std::cout << "Push forward target.end_eff: \n";
    std::cout << target.end_eff.poses[0] << std::endl;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
      Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
      cout << "Push forward with right hand FAILED!\n";
      return -1;
    }
    else{
      cout << "Push forward with right hand DONE. \n";
    }


    // via point to avoid disturbing the object during next move
    //==========================================================

    target.end_eff.poses[0].position.x += shiftx;
    target.end_eff.poses[0].position.z = object_in_base.pose.position.z + 0.1;
    std::cout << "Retracting right hand...\n";
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        Speech("I am having a problem with the motion planning...");
        Speech("Cannot complete task. Abort.");
        cout << "Retracting right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Retracting right hand DONE. \n";
        object_in_base.pose.position.x = target_in_base.pose.position.x;
        object_in_base.pose.position.z = target_in_base.pose.position.z;
        if(using_tool)
            return 0;

    }

    if(!using_tool){
        //Close fingers
        //============================================
        finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());

        // for encoders
//        finger_joints.positions[0] = 0.0;
//        finger_joints.positions[1] = 0.0;

        // for no encoders
        finger_joints.positions[0] = 0.0;
        finger_joints.positions[1] = 0.0;
        finger_joints.positions[2] = 0.0;
        finger_joints.positions[3] = -finger_joints.positions[0];
        finger_joints.positions[4] = -finger_joints.positions[1];
        finger_joints.positions[5] = -finger_joints.positions[2];

        target_fingers.trajectory.points.clear();
        target_fingers.trajectory.points.push_back(finger_joints);

        std::cout << "Closing fingers in right hand... \n";

        ac_fingers.sendGoal(target_fingers);
        ac_fingers.waitForResult(ros::Duration(20.0));
        result_fingers = ac_fingers.getResult();
        if(result_fingers->error_code == result_fingers->FAIL){
            cout << "Closing fingers in right hand FAILED!\n";
            return -1;
        }
        else{
            cout << "Closing fingers in right hand DONE. \n";
            return 0;
        }
    }
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionPushForwardRightArmSpecial(){
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
    ac_fingers.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitFingersGoal target_fingers;
    m3_moveit::MoveitFingersResultConstPtr result_fingers;

    std::cout << "ActionPushForwardRightArmSpecial" << std::endl;
    std::cout << "================================" << std::endl;

    double shiftx = -0.12;
    double shifty = 0.0;

    // Pre-positioning
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);

    //first point is a via point to avoid moving the object when pre-positioning
    target.end_eff.poses[0].position = object_to_hand_position_offset;
    target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
    target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx;
    target.end_eff.poses[0].position.y += object_in_base.pose.position.y + shifty;
    target.end_eff.poses[0].position.z += object_in_base.pose.position.z;

    std::cout << "Pre-positioning with right hand...\n";
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Pre-positioning with right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Pre-positioning with with right hand DONE. \n";
    }

    target_fingers.trajectory.joint_names = fillJointNamesFingers("right");
    trajectory_msgs::JointTrajectoryPoint finger_joints;


    // Pushing with fingers closed
    //=============================================
    
    int N_points = floor(fabs(object_in_base.pose.position.x-target_in_base.pose.position.x + shiftx)/0.05);
    std::cout << "N_points = " << N_points << std::endl;
    
    for(int i=0; i<N_points; i++){
      target.end_eff.poses[0].position = object_to_hand_position_offset;
      target.end_eff.poses[0].orientation = object_to_hand_orientation_offset;
      target.end_eff.poses[0].position.x += object_in_base.pose.position.x + shiftx + 0.05*(i+1);
      target.end_eff.poses[0].position.y += object_in_base.pose.position.y  ;
      target.end_eff.poses[0].position.z += target_in_base.pose.position.z ;

      std::cout << "Push forward target.end_eff: \n";
      std::cout << target.end_eff.poses[0] << std::endl;
      ac.sendGoal(target);
      ac.waitForResult(ros::Duration(20.0));
      result = ac.getResult();
      if(result->error_code == result->FAIL){
          cout << "Push forward with right hand FAILED!\n";
          return -1;
      }
      else{
          cout << "Push forward with right hand DONE. \n";
      }
    }
    

    // via point to avoid disturbing the object during next move
    //==========================================================
    shiftx = -0.2;
    target.end_eff.poses[0].position.x += shiftx;
    target.end_eff.poses[0].position.z = 0.9;
    std::cout << "Retracting right hand...\n";
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        cout << "Retracting right hand FAILED!\n";
        return -1;
    }
    else{
        cout << "Retracting right hand DONE. \n";
        object_in_base.pose.position.x = target_in_base.pose.position.x;
        object_in_base.pose.position.z = target_in_base.pose.position.z;
        return 0;
    }


}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvObjectFrontOfTargetRightOfRobotTool & ) {

    Speech("Let's see over here...");
    MoveWaist(WAIST_TOOL);

    //MoveWaist(30.0);
    int tool_type;
    n.getParam("/tool_type", tool_type);
    switch(tool_type){
        // 3D-printed tools with handles
        //----------------------------------
        case 1:{
            CheckToolLocation();

            if(sticklength >= min_augment){
                Speech("Yes, it satisfies the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m greater or equal to minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;32mAugmentation requirement satisfied!\033[0m" << endl;

                Speech("I will pick it up to get a better view ...");

                PickupToolRightArm();

                object_to_tool_position_offset = pullback_offset_right_hand;

                if(!ToolInHandCalibration("right")){
                    Speech("Sorry, I cannot get complete the tool calibration...");
                    cout << "Tool-in-hand calibration cannot be completed! \n";

                }
                else{
                    Speech("I will use this tool to complete the task.");
                    MoveWaist(WAIST_INIT);

                    if(ActionPullBackRightArmTool()==0)
                        sleep(sleep_time); // wait for real robot motion to finish.

                }


                ActionReturnToolRightArm();
            }
            else{
                Speech("No, it does not satisfy the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m less than minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;31mAugmentation requirement NOT satisfied.\033[0m" << endl;
            }
            break;
        }
        // bendtool
        //----------------------------------
        case 2:{
            CheckBendToolLocation();
            PickupBendTool();
            CheckFakeToolTransform();

            MoveWaist(WAIST_INIT);
            cout << "Press Enter to Continue";
            cin.ignore();
        //    MoveWaistFromCurrent(0.0);
            BendTool(0.8, 0.3, 45.0);


            if(ActionPullBackBendTool()==0)
                sleep(sleep_time); // wait for real robot motion to finish.

            while(ros::ok()){
                CheckObjectLocation();
                if(ActionPullBackBendTool()==0)
                    sleep(sleep_time);
            }
            cout << "Press Enter to Release Tool!!";
            cin.ignore();
            ActionReturnBendTool();
            break;
        }
        // umbrella
        //----------------------------------
        case 3:{
            CheckBendToolLocation();
            PickupBendTool();
            CheckFakeToolTransform();

            MoveWaist(WAIST_INIT);
            cout << "Press Enter to Continue";
            cin.ignore();
            if(ActionPullBackBendTool()==0)
                sleep(sleep_time); // wait for real robot motion to finish.

            cout << "Press Enter to Release Tool!!";
            cin.ignore();
            ActionReturnBendTool();
            break;
        }

        default:{
            cout << "invalid tool type" << endl;
            break;
        }
    }

    cout << "Finished!  \n";
    return transit< ReadyState >();
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//findWest
sc::result Manipulate::react( const EvObjectRightOfTargetTool & ) {

    Speech("Let's see over here...");
    MoveWaist(WAIST_TOOL);
    int tool_type;
    n.getParam("/tool_type", tool_type);
    switch(tool_type){
        // 3D-printed tools with handles
        //----------------------------------
        case 1:{
            CheckToolLocation();

            if(sticklength >= min_augment){
                Speech("Yes, it satisfies the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m greater or equal to minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;32mAugmentation requirement satisfied!\033[0m" << endl;
                
                Speech("I will pick it up to get a better view ...");

                PickupToolRightArm();

                object_to_tool_position_offset = pushsideways_offset_right_hand;


                if(!ToolInHandCalibration("right")){
                    Speech("Sorry, I cannot get complete the tool calibration...");
                    cout << "Tool-in-hand calibration cannot be completed! \n";
                }
                else{
                    Speech("I will use this tool to complete the task.");
                    MoveWaist(WAIST_INIT);
                    if(ActionPushSidewaysRightArm()==0)
                        sleep(sleep_time); // wait for real robot motion to finish.
                }


                ActionReturnToolRightArm();
            }
            else{
                Speech("No, it does not satisfy the minimum augmentation...");
                cout << "Stick length of " << sticklength << "m less than minimum augmentation of " << min_augment << "m." << endl;
                cout << "\033[1;31mAugmentation requirement NOT satisfied.\033[0m" << endl;
            }
            break;
        }

        // bat
        //----------------------------------
        case 4:{
            CheckBendToolLocation();
            PickupBendTool();
            CheckFakeToolTransform();

            MoveWaist(WAIST_INIT);
            cout << "Press Enter to Continue";
            cin.ignore();
            if(ActionPushSidewaysRightArm()==0)
                sleep(sleep_time); // wait for real robot motion to finish.
            

            cout << "Press Enter to Release Tool!!";
            cin.ignore();
            ActionReturnBendTool();
            break;
        }

        default:{
            cout << "invalid tool type" << endl;
            break;
        }
    }

    cout << "Finished! \n";
    return transit< ReadyState >();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//FindNorth
sc::result Manipulate::react( const EvObjectBackOfTargetRightOfRobotTool & ) {

    Speech("Let's see over here...");
    MoveWaist(WAIST_TOOL);

    CheckToolLocation();

    if(sticklength >= min_augment){
        Speech("Yes, it satisfies the minimum augmentation...");
        cout << "Stick length of " << sticklength << "m greater or equal to minimum augmentation of " << min_augment << "m." << endl;
        cout << "\033[1;32mAugmentation requirement satisfied!\033[0m" << endl;
        
        Speech("I will pick it up to get a better view ...");

        PickupToolRightArm();

        object_to_tool_position_offset = pushforward_offset_right_hand;

        if(!ToolInHandCalibration("right")){
            Speech("Sorry, I cannot get complete the tool calibration...");
            cout << "Tool-in-hand calibration cannot be completed! \n";
        }
        else{
            Speech("I will use this tool to complete the task.");
            MoveWaist(WAIST_INIT);
            if(ActionPushForwardRightArm(using_tool)==0)
                sleep(sleep_time); // wait for real robot motion to finish.

        }

        ActionReturnToolRightArm();
    }
    else{
        Speech("No, it does not satisfy the minimum augmentation...");
        cout << "Stick length of " << sticklength << "m less than minimum augmentation of " << min_augment << "m." << endl;
        cout << "\033[1;31mAugmentation requirement NOT satisfied.\033[0m" << endl;
    }


    cout << "Finished! \n";
    return transit< ReadyState >();
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvObjectRightOfTarget & ) {
    object_to_hand_position_offset = pushsideways_offset_right_hand;
    object_to_hand_orientation_offset = pushsideways_orientation_right_hand;
    ActionPushSidewaysRightArm();
    //post_event(EvPerceiveObject());
    cout << "Finished!  \n";
    return transit< ReadyState >();
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvObjectFrontOfTargetRightOfRobot & ) {
    object_to_hand_position_offset = pullback_offset_right_hand;
    object_to_hand_orientation_offset = pullback_orientation_right_hand;
    ActionPullBackRightArm();

    //post_event(EvPerceiveObject());
    cout << "Finished!  \n";
    return transit< ReadyState >();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//FindNorth
sc::result Manipulate::react( const EvObjectBackOfTargetRightOfRobot & ) {
    
    if(object_in_base_live.pose.position.z == 99.0){
        //wide channel
        object_to_hand_position_offset = pushforward_offset_right_hand;
        object_to_hand_orientation_offset = pushforward_orientation_right_hand;
        object_to_hand_position_offset.x -= 0.0;
        ActionPushForwardRightArmSpecial();
    }
    else if(object_in_base_live.pose.position.z == 100.0){
        //narrow channel
        tool_handle_received = false;
        MoveWaist(-30.0);
        CheckToolLocation();
        PickupToolRightArm();
        MoveWaist(-10.0);
        CheckFakeToolTransform();
        object_to_hand_position_offset = fake_tool_transform.position;
        object_to_hand_orientation_offset = fake_tool_transform.orientation;
        if(ActionPushForwardRightArmSpecial()==0){
            sleep(sleep_time); // wait for real robot motion to finish.
            ActionReturnToolRightArmSpecial();
        }
    }else{
        //original
        object_to_hand_position_offset = pushforward_offset_right_hand;
        object_to_hand_orientation_offset = pushforward_orientation_right_hand;
        ActionPushForwardRightArm(using_tool);
    }

    //post_event(EvPerceiveObject());
    cout << "Finished!  \n";
    return transit< ReadyState >();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvReady & ) {
    cout << "Finished!  \n";
    return transit< ReadyState >();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

std::vector<std::string> Manipulate::fillJointNamesFingersEncoders(std::string hand){

    std::vector<std::string> joint_names;

    if(hand=="left"){
        joint_names.push_back("lf1");
        joint_names.push_back("lf2");
    }
    else{
        joint_names.push_back("rf1");
        joint_names.push_back("rf2");
    }

    return joint_names;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

std::vector<std::string> Manipulate::fillJointNamesFingers(std::string hand){

    std::vector<std::string> joint_names;

    if(hand=="left"){
        joint_names.push_back("l_index_finger_j0");
        joint_names.push_back("l_index_finger_j1");
        joint_names.push_back("l_index_finger_j2");
        joint_names.push_back("l_middle_finger_j0");
        joint_names.push_back("l_middle_finger_j1");
        joint_names.push_back("l_middle_finger_j2");
    }
    else{
        joint_names.push_back("index_finger_j0");
        joint_names.push_back("index_finger_j1");
        joint_names.push_back("index_finger_j2");
        joint_names.push_back("middle_finger_j0");
        joint_names.push_back("middle_finger_j1");
        joint_names.push_back("middle_finger_j2");
    }

    return joint_names;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


m3_moveit::MoveitWholeBodyGoal Manipulate::fillJointNamesFull(){

    m3_moveit::MoveitWholeBodyGoal target;

    target.torso_trajectory.joint_names.push_back("waist_yaw");
    target.torso_trajectory.joint_names.push_back("waist_pitch");
    target.torso_trajectory.joint_names.push_back("trunk_pitch");

    target.right_trajectory.joint_names.push_back("right_shoulder_pitch");
    target.right_trajectory.joint_names.push_back("right_shoulder_roll");
    target.right_trajectory.joint_names.push_back("right_shoulder_yaw");
    target.right_trajectory.joint_names.push_back("right_elbow");
    target.right_trajectory.joint_names.push_back("right_wrist_yaw");
    target.right_trajectory.joint_names.push_back("right_wrist_pitch");
    target.right_trajectory.joint_names.push_back("right_wrist_yaw2");

    target.left_trajectory.joint_names.push_back("left_shoulder_pitch");
    target.left_trajectory.joint_names.push_back("left_shoulder_roll");
    target.left_trajectory.joint_names.push_back("left_shoulder_yaw");
    target.left_trajectory.joint_names.push_back("left_elbow");
    target.left_trajectory.joint_names.push_back("left_wrist_yaw");
    target.left_trajectory.joint_names.push_back("left_wrist_pitch");
    target.left_trajectory.joint_names.push_back("left_wrist_yaw2");

    target.head_trajectory.joint_names.push_back("neck_tilt");


    return target;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


m3_moveit::MoveitWholeBodyGoal Manipulate::fillNamesFull(){

    m3_moveit::MoveitWholeBodyGoal target_mixed;

    target_mixed = fillJointNamesFull();

    target_mixed.left_link_name = "l_ee";
    target_mixed.right_link_name = "ee";

    return target_mixed;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::getCurrentTransforms(tf::Quaternion& tfQuat_left, tf::Quaternion& tfQuat_right,  tf::Vector3& tfVec_left, tf::Vector3& tfVec_right){
    tf::TransformListener listener;
    tf::StampedTransform transform_right, transform_left;

    try{
        listener.waitForTransform("world", "ee",
                                  ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("world", "ee",
                                 ros::Time(0), transform_right);
        listener.waitForTransform("world", "l_ee",
                                  ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("world", "l_ee",
                                 ros::Time(0), transform_left);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    tfQuat_left = transform_left.getRotation();
    tfQuat_right = transform_right.getRotation();
    tfVec_left = transform_left.getOrigin();
    tfVec_right = transform_right.getOrigin();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::getCurrentTransform(const std::string link, tf::Quaternion& tfQuat, tf::Vector3& tfVec){
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try{
        listener.waitForTransform("world", link,
                                  ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("world", link,
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    tfQuat = transform.getRotation();
    tfVec = transform.getOrigin();
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


bool Manipulate::getCurrentTransform(const std::string link, geometry_msgs::PoseStamped& output){
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try{
        listener.waitForTransform("world", link,
                                  ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform("world", link,
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
        return false;
    }

    tf::Quaternion tfQuat; 
    tf::Vector3 tfVec;

    tfQuat = transform.getRotation();
    tfVec = transform.getOrigin();
    output.pose.position.x = tfVec[0];
    output.pose.position.y = tfVec[1];
    output.pose.position.z = tfVec[2];

    output.pose.orientation.x = tfQuat.getX();
    output.pose.orientation.y = tfQuat.getY();
    output.pose.orientation.z = tfQuat.getZ();
    output.pose.orientation.w = tfQuat.getW();

    // tf::Stamped<tf::Pose> tf_stamp = transform;
    // tf::poseStampedTFToMsg( tf_stamp, output);
    return true;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::translateHandFromCurrent(const std::string link, const vector<double> moveX, const vector<double> moveY, const vector<double> moveZ, const int Npts, geometry_msgs::PoseArray& pose_array_target) {

    tf::Quaternion tfQuat;
    tf::Vector3 tfVec;
    getCurrentTransform(link, tfQuat, tfVec);

    //cout << "tfVec= " << tfVec << endl;

    geometry_msgs::Pose pose_target;
    pose_target.position.x = tfVec.getX();
    pose_target.position.y = tfVec.getY();
    pose_target.position.z = tfVec.getZ();

    if(link=="ee"){
        pose_target.orientation.w = 0.5;
        pose_target.orientation.x = 0.5;
        pose_target.orientation.y = 0.5;
        pose_target.orientation.z = 0.5;
    }
    else if(link=="l_ee"){
        pose_target.orientation.w = 0.5;
        pose_target.orientation.x = -0.5;
        pose_target.orientation.y = 0.5;
        pose_target.orientation.z = -0.5;
    }


    for (int i=0; i<Npts; i++){
        pose_target.position.x += moveX[i];
        pose_target.position.y += moveY[i];
        pose_target.position.z += moveZ[i];

        pose_array_target.poses.push_back(pose_target);
    }
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::fake_tool_transform_callBack(const geometry_msgs::PoseArray::Ptr& msg) {
    fake_tool_transform_received=true;
    fake_tool_transform_live = msg->poses[0];
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::tool_handle_position_callBack(const geometry_msgs::Pose::Ptr& msg) {
    tool_handle_received=true;
    cout << "Tool handle detected." << endl;
    geometry_msgs::Pose pose_in_cam = *msg;
    //geometry_msgs::Pose pose_in_cam = msg->poses[0];
    cout << "tool handle position in cam frame \n";
    cout << "x: " << pose_in_cam.position.x << "\t" << "y: " << pose_in_cam.position.y << "\t" <<  "z: " << pose_in_cam.position.z << endl;

    //for actual
    tool_in_base_live = transformPose( pose_in_cam, "camera_rgb_optical_frame", "world");

    //for ease of simulation without camera
//    tool_in_base_live = transformPose( pose_in_cam, "world", "world");

    cout << "tool handle position in base frame \n";
    cout << "x: " << tool_in_base_live.pose.position.x << "\t" << "y: " << tool_in_base_live.pose.position.y << "\t" <<  "z: " << tool_in_base_live.pose.position.z << endl;


}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::bendtool_position_callBack(const geometry_msgs::Pose::Ptr& msg) {
    bendtool_received=true;
    cout << "Bend tool detected." << endl;

    bendtool_in_base_live.pose = *msg;

    cout << "bend tool handle position (base frame) \n";
    cout << "x: " << bendtool_in_base_live.pose.position.x << "\t" << "y: " << bendtool_in_base_live.pose.position.y << "\t" <<  "z: " << bendtool_in_base_live.pose.position.z << endl;


}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


void Manipulate::fake_target_callBack(const geometry_msgs::PoseArray::Ptr& msg) {
    fake_target_received=true;
    geometry_msgs::Pose pose_in_cam = msg->poses[0];
    //target_in_base_live = transformPose( pose_in_cam, "camera_rgb_optical_frame", "world");//for actual
    target_in_base_live = transformPose( pose_in_cam, "world", "world");//for ease of simulation without camera
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::object_printout_callBack(const geometry_msgs::PoseArray::Ptr& msg) {

        geometry_msgs::PoseStamped object;
        geometry_msgs::Pose pose_in_cam0 = msg->poses[0];

        //for actual
        object = transformPose( pose_in_cam0, "camera_rgb_optical_frame", "world");
        //target_in_base_live = transformPose( pose_in_cam1, "camera_rgb_optical_frame", "world");

        //cout << "object position in base frame \n";
        cout <<  object.pose.position.x << "\t" << object.pose.position.y << endl;


}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::object_target_positions_callBack(const geometry_msgs::PoseArray::Ptr& msg) {

    if(msg->poses.size()>=2){
        object_target_received=true;
        obstacle_detected = false;

        cout << "Object and target detected." << endl;
        geometry_msgs::Pose pose_in_cam0 = msg->poses[0];
        geometry_msgs::Pose pose_in_cam1 = msg->poses[1];

        cout << "object position in cam frame \n";
        cout << "x: " << pose_in_cam0.position.x << "\t" << "y: " << pose_in_cam0.position.y << "\t" <<  "z: " << pose_in_cam0.position.z << endl;

        cout << "target position in cam frame \n";
        cout << "x: " << pose_in_cam1.position.x << "\t" << "y: " << pose_in_cam1.position.y << "\t" <<  "z: " << pose_in_cam1.position.z << endl;

        #ifdef USING_TABLE_CAM
            //for actual
            object_in_base_live = transformPose( pose_in_cam0, "camera_table_rgb_optical_frame", "world");
            target_in_base_live = transformPose( pose_in_cam1, "camera_table_rgb_optical_frame", "world");
        #else
             //for actual
            object_in_base_live = transformPose( pose_in_cam0, "camera_rgb_optical_frame", "world");
            target_in_base_live = transformPose( pose_in_cam1, "camera_rgb_optical_frame", "world");
        #endif

        geometry_msgs::Pose obj_tar;


        obj_tar.position.x = (object_in_base_live.pose.position.x + target_in_base_live.pose.position.x) / 2.0;
        obj_tar.position.y = (object_in_base_live.pose.position.y + target_in_base_live.pose.position.y) / 2.0;
        obj_tar.position.z = (object_in_base_live.pose.position.z + target_in_base_live.pose.position.z) / 2.0;
        obj_tar_center = transformPose( obj_tar, "world", "neckBase");



        cout << "object position in base frame!! \n";
        cout << "x: " << object_in_base_live.pose.position.x << "\t" << "y: " << object_in_base_live.pose.position.y << "\t" <<  "z: " << object_in_base_live.pose.position.z << endl;

        cout << "target position in base frame \n";
        cout << "x: " << target_in_base_live.pose.position.x << "\t" << "y: " << target_in_base_live.pose.position.y << "\t" <<  "z: " << target_in_base_live.pose.position.z << endl;

        cout << "object and target center to neck base\n";
        cout << "x: " << obj_tar_center.pose.position.x << "\t" << "y: " << obj_tar_center.pose.position.y << "\t" <<  "z: " << obj_tar_center.pose.position.z << endl;


        for( int i = 2; i < msg->poses.size(); i++)
        {
            // currently assume only 1 obstacle
            // take the first valid point!

            geometry_msgs::Pose pose_in_cam2 = msg->poses[i]; 
            geometry_msgs::PoseStamped temp_pose;

            #ifdef USING_TABLE_CAM
                temp_pose = transformPose( pose_in_cam2, "camera_table_rgb_optical_frame", "world");
            #else
                temp_pose = transformPose( pose_in_cam2, "camera_rgb_optical_frame", "world");
            #endif
            
            if(temp_pose.pose.position.z < 0.91)
            {
                obstacle_in_base_live = temp_pose;
                cout << "obstacle position in cam frame \n";
                cout << "x: " << pose_in_cam2.position.x << "\t" << "y: " << pose_in_cam2.position.y << "\t" <<  "z: " << pose_in_cam2.position.z << endl;

                cout << "obstacle position in base frame!! \n";
                cout << "x: " << obstacle_in_base_live.pose.position.x << "\t" << "y: " << obstacle_in_base_live.pose.position.y << "\t" <<  "z: " << obstacle_in_base_live.pose.position.z << endl;
                
                cout << "Obstacle detected." << endl;
                obstacle_detected = true;

                ROS_WARN_STREAM("using obst_" << i-2 << "\n");
                break;
            }
            else
            {
                cout << "obst_" << i-2 << " at " << temp_pose.pose.position.z << " too high rejected!" << endl;
            }
        }

        if(!obstacle_detected)
            cout << "NO obstacle_detected" << endl;
    }
    else
        object_target_received=false;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

geometry_msgs::PoseStamped Manipulate::transformPose(geometry_msgs::Pose pose_msg, std::string from_frame, std::string to_frame) {

    geometry_msgs::PoseStamped pose_out;
    geometry_msgs::PoseStamped pose_in;

    pose_in.header.stamp = ros::Time(0);
    pose_in.header.frame_id = from_frame;
    pose_in.pose = pose_msg;

    double norm = sqrt(pow(pose_in.pose.orientation.w, 2) + pow(pose_in.pose.orientation.x, 2) + pow(pose_in.pose.orientation.y, 2) + pow(pose_in.pose.orientation.z, 2));

    if(norm<1e-6){
        pose_in.pose.orientation.w = 1.0; //dummy values
        pose_in.pose.orientation.x = 0.0;
        pose_in.pose.orientation.y = 0.0;
        pose_in.pose.orientation.z = 0.0;
    }
    else{
        //normalize quarternion
        pose_in.pose.orientation.w = pose_in.pose.orientation.w/norm;
        pose_in.pose.orientation.x = pose_in.pose.orientation.x/norm;
        pose_in.pose.orientation.y = pose_in.pose.orientation.y/norm;
        pose_in.pose.orientation.z = pose_in.pose.orientation.z/norm;
    }

    tf_listener.waitForTransform(to_frame, from_frame, ros::Time(0), ros::Duration(7.0) );
    tf_listener.transformPose(to_frame, pose_in, pose_out);
    return pose_out;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::calib_tool_poses_callBack(const geometry_msgs::PoseArray::Ptr& msg){
    cout << "calib tool poses received" << endl;
    calib_tool_poses_received = true;
    for(int i=0; i<msg->poses.size(); i++){
        geometry_msgs::Pose pose_in_cam = msg->poses[i];
        //for actual
        calib_tool_poses_live.poses[i] = transformPose( pose_in_cam, "camera_rgb_optical_frame", "world").pose;

    }


}

void Manipulate::tool_decision_callBack(const std_msgs::Int32::Ptr& msg){
    cout << "tool decision received." << endl;
    tool_decision_received = true;

    is_suitable_tool = msg->data;
}

void Manipulate::calib_next_callBack(const std_msgs::Int32::Ptr& msg){
    calib_next = msg->data;
}

void Manipulate::joint_states_callBack(const sensor_msgs::JointState::Ptr& msg){

    joint_values.resize(10);    //clear joint_values
    joint_values_l.resize(10);
    for (int i = 0; i < 10; i++)
    {
        joint_values[i] = 0;
        joint_values_l[i] = 0;
    }

    #ifdef FIXED_WAIST
    // remove the waist pitch and trunk pitch
    joint_values[1] = -30.0 /180.0 * M_PI;
    joint_values_l[1] = -30.0 /180.0 * M_PI;
    joint_values[2] = 80.0 /180.0 * M_PI;
    joint_values_l[2] = 80.0 /180.0 * M_PI;
    #endif

    for (int i = 0; i < msg->name.size(); i++)
    {    
        //need to get the joint angles to initalise the moveit_ik solver
        if (msg->name.at(i) == "waist_yaw")
        {
            waist_yaw = msg->position.at(i);
            joint_values[0] = msg->position.at(i);
            joint_values_l[0] = msg->position.at(i);
        }
        if (msg->name.at(i) == "waist_pitch")
        {
            joint_values[1] = msg->position.at(i);
            joint_values_l[1] = msg->position.at(i);
        }
        if (msg->name.at(i) == "trunk_pitch")
        {
            joint_values[2] = msg->position.at(i);
            joint_values_l[2] = msg->position.at(i);
        }

        //right arm
        if (msg->name.at(i) == "right_shoulder_pitch")
        {
            joint_values[3] = msg->position.at(i);
        }
        if (msg->name.at(i) == "right_shoulder_roll")
        {
            joint_values[4] = msg->position.at(i);
        }
        if (msg->name.at(i) == "right_shoulder_yaw")
        {
            joint_values[5] = msg->position.at(i);
        }
        if (msg->name.at(i) == "right_elbow")
        {
            joint_values[6] = msg->position.at(i);
        }
        if (msg->name.at(i) == "right_wrist_yaw")
        {
            joint_values[7] = msg->position.at(i);
        }
        if (msg->name.at(i) == "right_wrist_pitch")
        {
            joint_values[8] = msg->position.at(i);
        }
        if (msg->name.at(i) == "right_wrist_yaw2")
        {
            joint_values[9] = msg->position.at(i);
        }

        //left arm        
        if (msg->name.at(i) == "left_shoulder_pitch")
        {
            joint_values_l[3] = msg->position.at(i);
        }
        if (msg->name.at(i) == "left_shoulder_roll")
        {
            joint_values_l[4] = msg->position.at(i);
        }
        if (msg->name.at(i) == "left_shoulder_yaw")
        {
            joint_values_l[5] = msg->position.at(i);
        }
        if (msg->name.at(i) == "left_elbow")
        {
            joint_values_l[6] = msg->position.at(i);
        }
        if (msg->name.at(i) == "left_wrist_yaw")
        {
            joint_values_l[7] = msg->position.at(i);
        }
        if (msg->name.at(i) == "left_wrist_pitch")
        {
            joint_values_l[8] = msg->position.at(i);
        }
        if (msg->name.at(i) == "left_wrist_yaw2")
        {
            joint_values_l[9] = msg->position.at(i);
        }
        
        
        //for neck
        if (msg->name.at(i) == "neck_tilt")
        {
            current_tilt_angle = msg->position.at(i) * 180.0/PI; // convert to deg
        }
        if (msg->name.at(i) == "neck_pan")
        {
            current_pan_angle = msg->position.at(i) * 180.0/PI; //convert deg
        }
    }

    //cout << "current_tilt_angle = " << current_tilt_angle << endl;
    joint_state = *msg;

    joint_updated = true;
//    std::cout << "right_wrist_pitch = " << joint_state.position.at(36) << std::endl;
}

void Manipulate::neck_pan_state_callBack(const sensor_msgs::JointState::Ptr& msg){
    //cout << "neck pan cb" << endl;
    for (int i=0; i<msg->name.size(); i++)
        if(msg->name.at(i)=="neck_pan")
            current_pan_angle = msg->position.at(i) * 180.0/PI; //convert deg
}

void Manipulate::sticklength_callBack(const std_msgs::Float64::Ptr& msg){
    cout << "Stick length received." << endl;
    sticklength_received = true;
    sticklength = msg->data;

}

sc::result Manipulate::react( const EvStart & ) {
	return transit< Startup >();
}
