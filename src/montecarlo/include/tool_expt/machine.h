#ifndef MACHINE_HPP_INCLUDED
#define MACHINE_HPP_INCLUDED

#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

namespace sc = boost::statechart;


/*=============================================================== */
// declaring events
/*=============================================================== */
struct EvStart : sc::event< EvStart > {};
struct EvPerceiveObject : sc::event< EvPerceiveObject > {};
struct EvObjectRightOfTarget : sc::event< EvObjectRightOfTarget > {};
struct EvObjectFrontOfTargetRightOfRobot : sc::event< EvObjectFrontOfTargetRightOfRobot > {};
struct EvObjectBackOfTargetRightOfRobot : sc::event< EvObjectBackOfTargetRightOfRobot > {};
struct EvObjectFrontRightOfTargetTool : sc::event< EvObjectFrontRightOfTargetTool > {};
struct EvObjectBackRightOfTargetTool : sc::event< EvObjectBackRightOfTargetTool > {};
struct EvObjectFrontOfTargetRightOfRobotTool : sc::event< EvObjectFrontOfTargetRightOfRobotTool > {};
struct EvObjectFrontOfTargetLeftOfRobotTool : sc::event< EvObjectFrontOfTargetLeftOfRobotTool > {};
struct EvObjectRightOfTargetTool : sc::event< EvObjectRightOfTargetTool > {};
struct EvObjectBackOfTargetRightOfRobotTool : sc::event< EvObjectBackOfTargetRightOfRobotTool > {};
struct EvReady : sc::event< EvReady > {};

struct EvTargetNofObject : sc::event< EvTargetNofObject > {};
struct EvTargetSofObject : sc::event< EvTargetSofObject > {};
struct EvTargetOmniObject : sc::event< EvTargetOmniObject > {};

struct EvMonteCarloExpt : sc::event< EvMonteCarloExpt > {};
struct EvMonteCarloExpt2 : sc::event< EvMonteCarloExpt2 > {};
struct EvMonteCarloExpt2_left : sc::event< EvMonteCarloExpt2_left > {};

struct EvMonteCarloExpt3_left : sc::event< EvMonteCarloExpt3_left > {};


/* ============================================================== */
// sc::state_machine accepts at least two parameters
// 1st param: name of state machine
// 2nd param: state it has to enter when the machine is initiated
/* ============================================================== */
struct Active;
struct Machine : sc::state_machine< Machine, Active > {};

/* ======================================================================== */
// sc::simple_state class accepts up to four parameters
// 1st param: name of simple_state
// 2nd param: defines the state machine/context that this state belongs to
// 3rd param: inner initial state, if any
// 4th param: specifies whether and what kind of history is kept
/* ======================================================================== */
struct Startup;

struct Active : sc::simple_state< Active, Machine, Startup > {
	public:
		Active(){}
		~Active(){}

        ros::Publisher gosee_pub;
        ros::Publisher gomove_pub;
        ros::Publisher table_pub;
        int go_arm;
        void setGoArm(const int val) {go_arm = val;}

	private:


};

struct Startup : sc::state< Startup, Active > {
	public:
		typedef sc::custom_reaction< EvStart > reactions;

		Startup(my_context ctx);
		~Startup();

		sc::result react( const EvStart & );
};
#endif
