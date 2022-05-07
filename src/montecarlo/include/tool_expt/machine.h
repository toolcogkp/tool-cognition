/**
 * @file machine.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * state machine for olivia
 * @version 0.1
 * @date 2019-02-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */


#ifndef MACHINE_HPP_INCLUDED
#define MACHINE_HPP_INCLUDED

// boost
#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>

// ros
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

/**
 * @brief State machine Event structure: Startup sequence & initialization
 * 
 */
struct EvStart : sc::event< EvStart > {};

/**
 * @brief State machine Event structure: Send request to perception module to detect Object 
 * 
 */
struct EvPerceiveObject : sc::event< EvPerceiveObject > {};

/**
 * @brief State machine Event structure: Sequence when object is on the right side of target goal
 * 
 */
struct EvObjectRightOfTarget : sc::event< EvObjectRightOfTarget > {};

/**
 * @brief State machine Event structure: Right Body Sequence when object in front of target goal
 * 
 */
struct EvObjectFrontOfTargetRightOfRobot : sc::event< EvObjectFrontOfTargetRightOfRobot > {};

/**
 * @brief State machine Event structure: Right Body Sequence when object is behind target goal
 * 
 */
struct EvObjectBackOfTargetRightOfRobot : sc::event< EvObjectBackOfTargetRightOfRobot > {};

/**
 * @brief State machine Event structure: Sequence when object in front right of tool
 * 
 */
struct EvObjectFrontRightOfTargetTool : sc::event< EvObjectFrontRightOfTargetTool > {};

/**
 * @brief State machine Event structure: Sequence when object is on back right of tool
 * 
 */
struct EvObjectBackRightOfTargetTool : sc::event< EvObjectBackRightOfTargetTool > {};

/**
 * @brief State machine Event structure: Sequence when object is on the front right of tool
 * 
 */
struct EvObjectFrontOfTargetRightOfRobotTool : sc::event< EvObjectFrontOfTargetRightOfRobotTool > {};

/**
 * @brief State machine Event structure: Sequence when object is on the front left of tool
 * 
 */
struct EvObjectFrontOfTargetLeftOfRobotTool : sc::event< EvObjectFrontOfTargetLeftOfRobotTool > {};

/**
 * @brief State machine Event structure: Sequence when object is on the right side of tool
 * 
 */
struct EvObjectRightOfTargetTool : sc::event< EvObjectRightOfTargetTool > {};

/**
 * @brief State machine Event structure: Sequence when object is behind and right of tool
 * 
 */
struct EvObjectBackOfTargetRightOfRobotTool : sc::event< EvObjectBackOfTargetRightOfRobotTool > {};

/**
 * @brief State machine Event structure: Ready Sequence
 * 
 */
struct EvReady : sc::event< EvReady > {};

/**
 * @brief State machine Event structure: Sequence when target goal is North of object
 * 
 */
struct EvTargetNofObject : sc::event< EvTargetNofObject > {};

/**
 * @brief State machine Event structure: Sequence when target goal is South of object
 * 
 */
struct EvTargetSofObject : sc::event< EvTargetSofObject > {};

/**
 * @brief State machine Event structure: Based on object position with target, transition into selective event sequences
 * 
 */
struct EvTargetOmniObject : sc::event< EvTargetOmniObject > {};

/**
 * @brief State machine Event structure: Monte Carlo search experiment
 * 
 */
struct EvMonteCarloExpt : sc::event< EvMonteCarloExpt > {};

/**
 * @brief State machine Event structure: Monte Carlo search experiment 2
 * 
 */
struct EvMonteCarloExpt2 : sc::event< EvMonteCarloExpt2 > {};

/**
 * @brief State machine Event structure: Monte Carlo search experiment 2 with Left Arm of Olivia
 * 
 */
struct EvMonteCarloExpt2_left : sc::event< EvMonteCarloExpt2_left > {};

/**
 * @brief State machine Event structure: Monte Carlo search experiment 3 with Left Arm of Olivia
 * 
 */
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

/**
 * @brief structure for Active state
 * 
 */
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

/**
 * @brief structure for Startup state -> transit to Active state upon startup
 * 
 */
struct Startup : sc::state< Startup, Active > {
	public:
		typedef sc::custom_reaction< EvStart > reactions;

		Startup(my_context ctx);
		~Startup();

		sc::result react( const EvStart & );
};
#endif
