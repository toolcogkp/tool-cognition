#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>
#include "kaist_msgs/JointEncoderStatus.h"
#include "sensor_msgs/JointState.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <iostream>
#include <fstream>

void EncoderCallBack(const kaist_msgs::JointEncoderStatus::ConstPtr& msg);
void ObjectCallBack(const geometry_msgs::PoseArray::Ptr& msg) ;
void JointStatesCallBack(const sensor_msgs::JointState::ConstPtr& msg) ;

//geometry_msgs::PoseStamped transformPose(geometry_msgs::Pose pose_msg, std::string from_frame, std::string to_frame) ;

geometry_msgs::PoseArray object_in_cam0;
trajectory_msgs::JointTrajectory joint_trajectory;
bool received, first_time;
bool triggered = false;


void TimerCallback(const ros::TimerEvent& e);
void TimerCallbackS(const ros::TimerEvent& e);
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosbag_getxy");
    ros::NodeHandle nh;
    //ros::Subscriber sub_encoder = nh.subscribe("/JointEncoder", 2000, EncoderCallBack);
    //ros::Subscriber sub_drchubo_joint_states = nh.subscribe("/drchubo_joint_states", 2000, JointStatesCallBack);;
    ros::Subscriber sub_object = nh.subscribe("perception_mode/task_mode_cylinder_center", 2000, ObjectCallBack);;

    moveit::planning_interface::MoveGroup group("whole_body");
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(group.getCurrentState()->getRobotModel()));
    const Eigen::Affine3d &end_effector_state= kinematic_state->getGlobalLinkTransform("handmount_RIGHT");

    first_time=true;
    ros::Timer timer;

    while(ros::ok()){
        if(received && first_time){
            first_time=false;
            double duration = 45.0;
            cout << "Waiting " << duration <<  "s for rosbag to finish playing" << endl;
            timer = nh.createTimer(ros::Duration(duration), TimerCallback, true); //for object
            //timer = nh.createTimer(ros::Duration(90.0), TimerCallbackS, true); //for joint states
            //cout << "here" << endl;
        }
        if(triggered){
            ofstream myfile;
            myfile.open ("dat.txt");
            cout << "Writing to file...." << endl;
            for(int i=0; i<joint_trajectory.points.size(); i++){
                kinematic_state->setJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()) , joint_trajectory.points[i].positions);
                kinematic_state->update();
                Eigen::Vector3d vec = end_effector_state.translation();
                Eigen::AngleAxisd aa(end_effector_state.rotation());
                Eigen::Vector3d aa_axis = aa.axis();
                //cout <<  vec(0) << "\t" << vec(1) << "\t" << vec(2) << "\t" << aa.angle() << "\t" << aa_axis(0) << "\t" << aa_axis(1) << "\t" << aa_axis(2) << endl;
                myfile << vec(0) << "\t" << vec(1) << "\t" << vec(2) << "\t" << aa.angle() << "\t" << aa_axis(0) << "\t" << aa_axis(1) << "\t" << aa_axis(2) << endl;
            }
            cout << "Finished writing to file." << endl;
            triggered = false;
            myfile.close();
        }
        ros::spinOnce();
    }
    return 0;
}

void TimerCallbackS(const ros::TimerEvent& e){
    triggered = true;

}


void TimerCallback(const ros::TimerEvent& e){

    geometry_msgs::PoseStamped object;
    geometry_msgs::PoseStamped pose_in;
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("world", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(7.0) );

    for(int i=0; i<object_in_cam0.poses.size(); i++){
        pose_in.header.stamp = ros::Time(0);
        pose_in.header.frame_id = "camera_rgb_optical_frame";
        pose_in.pose.position.x = object_in_cam0.poses[i].position.x ;
        pose_in.pose.position.y = object_in_cam0.poses[i].position.y ;
        pose_in.pose.position.z = object_in_cam0.poses[i].position.z ;

        geometry_msgs::Quaternion quat_msg;
        quat_msg.w = 1.0; //dummy values
        quat_msg.x = 0.0;
        quat_msg.y = 0.0;
        quat_msg.z = 0.0;
        pose_in.pose.orientation = quat_msg;

        tf_listener.transformPose("world", pose_in, object);
        cout <<  object.pose.position.x << "\t" << object.pose.position.y << "\t" << object.pose.position.z << endl;
    }

}


void ObjectCallBack(const geometry_msgs::PoseArray::Ptr& msg) {

    received=true;
    object_in_cam0.poses.push_back(msg->poses[0]);
}

void EncoderCallBack(const kaist_msgs::JointEncoderStatus& msg) {


}

void JointStatesCallBack(const sensor_msgs::JointState::ConstPtr& msg) {

    received=true;
    trajectory_msgs::JointTrajectoryPoint joint_values;

    joint_values.positions.resize(18);
    joint_values.positions[0] = msg->position[17]; //waist yaw
    joint_values.positions[1] = msg->position[16]; //waist pitch
    joint_values.positions[2] = msg->position[15]; //torso pitch
    joint_values.positions[3] = msg->position[1]; //left shoulder pitch
    joint_values.positions[4] = msg->position[2]; //left shoulder roll
    joint_values.positions[5] = msg->position[3]; //left shoulder yaw
    joint_values.positions[6] = msg->position[0]; //left elbow
    joint_values.positions[7] = msg->position[6]; //left wrist yaw
    joint_values.positions[8] = msg->position[5]; // left wrist pitch
    joint_values.positions[9] = msg->position[4]; //left wrist yaw2
    joint_values.positions[10] = msg->position[7];// neck pitch
    joint_values.positions[11] = msg->position[9];//right shoulder pitch
    joint_values.positions[12] = msg->position[10];//right shoulder roll
    joint_values.positions[13] = msg->position[11];//right shoulder yaw
    joint_values.positions[14] = msg->position[8];//right elbow
    joint_values.positions[15] = msg->position[14];//right wrist yaw
    joint_values.positions[16] = msg->position[13];// right wrist pitch
    joint_values.positions[17] = msg->position[12];//right wrist yaw2

    joint_trajectory.points.push_back(joint_values);

    //cout << "received" <<endl;

}

