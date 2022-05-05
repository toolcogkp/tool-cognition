#include <tool_expt/machine.h>
#include <tool_expt/manipulate.h>
#include <sensor_msgs/JointState.h>
#include <m3_moveit/MoveitWholeBodyAction.h>
#include <m3_moveit/MoveitFingersAction.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/CollisionObject.h>

using namespace std;

Startup::Startup(my_context ctx ) : my_base( ctx ) {
    ros::NodeHandle n;
    context< Active >().table_pub  = n.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
    sleep(2);
}

Startup::~Startup() {}

void dummyCallback(const sensor_msgs::JointState::ConstPtr& msg) {}

sc::result Startup::react( const EvStart & ) {
    cout << "monte carlo machine" << endl;

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/drchubo_joint_states", 1, dummyCallback);
    usleep(500000);
    if(sub.getNumPublishers() <= 0) {
        cout << "robot not ready\n";
        return discard_event();
    }

    return transit< Manipulate>();


}

