#include <tool_expt/machine.h>
#include <tool_expt/manipulate.h>
#include <sensor_msgs/JointState.h>
#include <m3_moveit/MoveitWholeBodyAction.h>
#include <m3_moveit/MoveitFingersAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/CollisionObject.h>
#include <gazebo_msgs/SetModelState.h>

//#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/AttachedCollisionObject.h>


using namespace std;

//int go_arms123;


Startup::Startup(my_context ctx ) : my_base( ctx ) {
    ros::NodeHandle n;
//    context< Active >().gosee_pub = n.advertise<std_msgs::Int32>("/go_see",10);
//    context< Active >().gomove_pub = n.advertise<std_msgs::Int32>("/go_move",10);
    context< Active >().table_pub  = n.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);

}

Startup::~Startup() {}



sc::result Startup::react( const EvStart & ) {

//    cin.ignore();

    cout << "monte carlo machine sim" << endl;

    //Publish table as collision object
    //============================================
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";

//    /* The id of the object is used to identify it. */
    collision_object.id = "tabletop";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 1.5;
    primitive.dimensions[2] = 0.05;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.9;
    box_pose.position.y =  0.0;
    box_pose.position.z =  0.715;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    sleep(1); // IMPT!! this MUST come before publish, otherwise it will not publish. Possibly the preceding ADD operation needs time to finish?
    context< Active >().table_pub.publish(collision_object);
    ROS_INFO("Published collision object.");




//    //Close fingers
//    //============================================
//    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers", true);
////    actionlib::SimpleActionClient<m3_moveit::MoveitFingersAction> ac_fingers("fingers_encoders", true);
//    ac_fingers.waitForServer(ros::Duration(20.0));
//    m3_moveit::MoveitFingersGoal target_fingers;
//    m3_moveit::MoveitFingersResultConstPtr result_fingers;

//    target_fingers.trajectory.joint_names.push_back("index_finger_j0");
//    target_fingers.trajectory.joint_names.push_back("index_finger_j1");
//    target_fingers.trajectory.joint_names.push_back("index_finger_j2");
//    target_fingers.trajectory.joint_names.push_back("middle_finger_j0");
//    target_fingers.trajectory.joint_names.push_back("middle_finger_j1");
//    target_fingers.trajectory.joint_names.push_back("middle_finger_j2");

////    target_fingers.trajectory.joint_names.push_back("rf1");
////    target_fingers.trajectory.joint_names.push_back("rf2");

//    trajectory_msgs::JointTrajectoryPoint finger_joints;
//    finger_joints.positions.resize(target_fingers.trajectory.joint_names.size());
//    finger_joints.positions[0] = 0.0;
//    finger_joints.positions[1] = 0.0;
//    finger_joints.positions[2] = 0.0;
//    finger_joints.positions[3] = 0.0;
//    finger_joints.positions[4] = 0.0;
//    finger_joints.positions[5] = 0.0;
//    target_fingers.trajectory.points.push_back(finger_joints);

//    std::cout << "Closing fingers in right hand... \n";
//    std::cout << target_fingers.trajectory.joint_names[0] << std::endl;
//    std::cout << target_fingers.trajectory.joint_names[1] << std::endl;
//    std::cout << target_fingers.trajectory.joint_names[2] << std::endl;
//    std::cout << target_fingers.trajectory.joint_names[3] << std::endl;
//    std::cout << target_fingers.trajectory.joint_names[4] << std::endl;
//    std::cout << target_fingers.trajectory.joint_names[5] << std::endl;
//    ac_fingers.sendGoal(target_fingers);
//    ac_fingers.waitForResult(ros::Duration(20.0));
//    result_fingers = ac_fingers.getResult();
//    if(result_fingers->error_code == result_fingers->FAIL){
//        cout << "Closing fingers in right hand FAILED!\n";
//        return discard_event();
//    }
//    else{
//        cout << "Closing fingers in right hand DONE. \n";
//    }


    //Reset puck
    //============================================
//    geometry_msgs::Pose puck_pose;
//    puck_pose.position.x = 0.1;
//    puck_pose.position.y = -0.0;
//    puck_pose.position.z = 0.0;
//    puck_pose.orientation.x = 0.0;
//    puck_pose.orientation.y = 0.0;
//    puck_pose.orientation.z = 0.0;
//    puck_pose.orientation.w = 1.0;

//    geometry_msgs::Twist puck_twist;
//    puck_twist.linear.x = 0.0;
//    puck_twist.linear.y = 0.0;
//    puck_twist.linear.z = 0.0;
//    puck_twist.angular.x = 0.0;
//    puck_twist.angular.y = 0.0;
//    puck_twist.angular.z = 0.0;

//    gazebo_msgs::ModelState modelstate;
//    modelstate.model_name = (std::string) "puck";
//    modelstate.reference_frame = (std::string) "world";
//    modelstate.pose = puck_pose;
//    modelstate.twist = puck_twist;

//    ros::NodeHandle n;
//    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
//    gazebo_msgs::SetModelState setmodelstate;
//    setmodelstate.request.model_state = modelstate;
//    client.call(setmodelstate);




        return transit< Manipulate>();



}


