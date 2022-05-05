//dynamic tf broadcaster

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <math.h>

#include <tf/transform_broadcaster.h>

#include "monte_carlo/tf_switch.h" //srv generated

using namespace std;

string frame_id;
string child_frame_id;

tf::Transform set_tf;
tf::Vector3 translation;

tf::Quaternion q1;
tf::Quaternion q2;
tf::Quaternion q3;

tf::Quaternion q4;

bool switch_srv(monte_carlo::tf_switch::Request &req,
            monte_carlo::tf_switch::Response &res)
{
    int input = req.value;
    bool output = false;

    switch(input)
    {
        case 1:
        {
            set_tf.setOrigin( translation );
            set_tf.setRotation(q1);
            cout << "switch: 1 -> -6deg" << endl;
            output = true;
            break;
        }

        case 2:
        {
            set_tf.setOrigin( translation );
            set_tf.setRotation(q2);
            cout << "switch: 2 -> +6deg" << endl;
            output = true;
            break;
        }

        case 3:
        {
            set_tf.setOrigin( translation );
            set_tf.setRotation(q3);
            cout << "switch: 3 -> home2" << endl;
            output = true;
            break;
        }

        case 4:
        {
            set_tf.setOrigin( translation );
            set_tf.setRotation(q4);
            cout << "switch: 4 " << endl;
            output = true;
            break;
        }


        default:
        {
            ROS_WARN("unknown input -> ignored");
            output = false;
            break;
        }
    }

    res.okay = output;
    return true;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "camera_tf_broadcaster");
    ros::NodeHandle node;
    ros::ServiceServer service = node.advertiseService("tf_switch", switch_srv);
    
    frame_id = "head_frame";
    child_frame_id = "camera_rgb_optical_frame";
    
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    translation = tf::Vector3(-0.0837-0.01, 0.0903-0.01, 0.0189-0.02);    
    q1 = tf::Quaternion(0.52547, -0.47316, -0.52547, -0.47316);
    q2 = tf::Quaternion(-0.4731468, 0.5254827, 0.4731468, 0.5254827);
    q3 = tf::Quaternion(-0.4595201, 0.5210162, 0.4959087, 0.5251062);
	q4 = tf::Quaternion(-0.50507, 0.467901, 0.5541133, 0.467901);
    set_tf.setOrigin( translation );
    set_tf.setRotation(q1);

    cout << "start looping" << endl;
    cout << "switch: 1 -> -6deg" << endl;
    
    ros::Rate r(50); // in hz
    while( ros::ok() )
    {
        transform = set_tf;
        br.sendTransform(tf::StampedTransform( transform, ros::Time::now(), frame_id, child_frame_id));
        ros::spinOnce();
        r.sleep();
    }

    ROS_WARN_STREAM("dynamic broadcaster closed!");
    return 0;
};