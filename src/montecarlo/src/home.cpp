#include <ros/ros.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <ros/service_client.h>
#include <iostream>

using namespace std;

double d2r(double d){
    double r;
    double pi = 3.14159265359;
    return r = d*pi/180.0;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "home_robot");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle n;
    ros::ServiceClient client_home;
    client_home = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    gazebo_msgs::SetModelConfiguration msg;
    msg.request.model_name = "drchubo";
    msg.request.urdf_param_name = "robot_description";

    msg.request.joint_names.push_back("right_shoulder_pitch");
    msg.request.joint_names.push_back("right_shoulder_roll");
    msg.request.joint_names.push_back("right_shoulder_yaw");
    msg.request.joint_names.push_back("right_elbow");
    msg.request.joint_names.push_back("right_wrist_yaw");
    msg.request.joint_names.push_back("right_wrist_pitch");
    msg.request.joint_names.push_back("right_wrist_yaw2");
    msg.request.joint_positions.push_back(d2r(-39.49));
    msg.request.joint_positions.push_back(d2r(27.88));
    msg.request.joint_positions.push_back(d2r(38.56));
    msg.request.joint_positions.push_back(d2r(144.10));
    msg.request.joint_positions.push_back(d2r(71.18));
    msg.request.joint_positions.push_back(d2r(8.96));
    msg.request.joint_positions.push_back(d2r(64.62));

    msg.request.joint_names.push_back("left_shoulder_pitch");
    msg.request.joint_names.push_back("left_shoulder_roll");
    msg.request.joint_names.push_back("left_shoulder_yaw");
    msg.request.joint_names.push_back("left_elbow");
    msg.request.joint_names.push_back("left_wrist_yaw");
    msg.request.joint_names.push_back("left_wrist_pitch");
    msg.request.joint_names.push_back("left_wrist_yaw2");
    msg.request.joint_positions.push_back(d2r(-39.49));
    msg.request.joint_positions.push_back(d2r(-27.88));
    msg.request.joint_positions.push_back(d2r(-38.56));
    msg.request.joint_positions.push_back(d2r(144.10));
    msg.request.joint_positions.push_back(d2r(-71.18));
    msg.request.joint_positions.push_back(d2r(8.96));
    msg.request.joint_positions.push_back(d2r(-64.62));

    msg.request.joint_names.push_back("waist_yaw");
    msg.request.joint_names.push_back("waist_pitch");
    msg.request.joint_names.push_back("trunk_pitch");
    msg.request.joint_positions.push_back(d2r(-10.0));
    msg.request.joint_positions.push_back(d2r(-16.98));
    msg.request.joint_positions.push_back(d2r(47.44));

    msg.request.joint_names.push_back("neck_tilt");
    msg.request.joint_positions.push_back(d2r(13.3));

    client_home.call(msg);
    cout << "call service status message = "<< msg.response.status_message<< endl;



}
