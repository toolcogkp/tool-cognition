#include <tool_expt/machine.h>
#include <tool_expt/manipulate.h>
#include <sensor_msgs/JointState.h>
#include <m3_moveit/MoveitWholeBodyAction.h>
#include <m3_moveit/MoveitFingersAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/CollisionObject.h>
#include <gazebo_msgs/SetModelState.h>

using namespace std;

Startup::Startup(my_context ctx ) : my_base( ctx ) {
    ros::NodeHandle n;
    context< Active >().table_pub  = n.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);

}

Startup::~Startup() {}


sc::result Startup::react( const EvStart & ) {

    cout << "monte carlo machine sim" << endl;

    //Publish table as collision object
    //============================================
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";

	/* The id of the object is used to identify it. */
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
    sleep(1); 
    context< Active >().table_pub.publish(collision_object);
    ROS_INFO("Published collision object.");

    return transit< Manipulate>();

}


