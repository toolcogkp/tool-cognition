#ifndef MOVEIT_IK_HPP_INCLUDED
#define MOVEIT_IK_HPP_INCLUDED

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <math.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_model/joint_model.h>
#include <moveit_msgs/JointLimits.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

#define ATTEMPTS 2
#define UPDATE_RATE 50.0

//ik solver using moveit for node.m, samuel was here
class MOVEIT_IK
{
public:
	MOVEIT_IK(){};
	~MOVEIT_IK(){};

    void init(bool right=true)
    {
        //=======================================================//
        //moveit
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        kinematic_model = robot_model_loader.getModel();
        // sleep(0.5);

        // planning_scene_monitor::LockedPlanningSceneRW ps(m_planningSceneMonitor);
        // ps->getCurrentStateNonConst().update();

        // planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model)); 
        planning_scene::PlanningScene planning_scene(kinematic_model);
        robot_state::RobotStatePtr current_state(new robot_state::RobotState(planning_scene.getCurrentStateNonConst()));
        kinematic_state = current_state;
        // kinematic_state->setToDefaultValues();
        // kinematic_state->update();
        
        // robot_state::RobotStatePtr temp_state(new robot_state::RobotState(kinematic_model));
        // kinematic_state = temp_state;
        // kinematic_state->setToDefaultValues();
        // kinematic_state->update();
        // sleep(0.5);

        if(right)
        {           
            arm_model_group = kinematic_model->getJointModelGroup("right_arm");
            arm_joint_names = arm_model_group->getVariableNames();
            sleep(0.5);

            joint_model_group = kinematic_model->getJointModelGroup("torso_and_right_arm");
            mv_joint_names = joint_model_group->getVariableNames();
            sleep(0.5);
        }
        else
        {
            ROS_ERROR_STREAM("MOVEIT IK DOING LEFT!");
            arm_model_group = kinematic_model->getJointModelGroup("left_arm");
            arm_joint_names = arm_model_group->getVariableNames();
            sleep(0.5);

            #ifdef FIXED_WAIST
            joint_model_group = kinematic_model->getJointModelGroup("waist_and_left_arm");
            #else
            joint_model_group = kinematic_model->getJointModelGroup("torso_and_left_arm");
            #endif
            mv_joint_names = joint_model_group->getVariableNames();
            sleep(0.5);    
        }
        cout << "moveit_ik initalised\n" << endl;
    };

    void init_home( vector<double> joint_values )
    {
        for(int i = 3; i<10; i++)
            home_arm.push_back(joint_values[i]);
     
        #ifdef FIXED_WAIST
            //just take the 8 values instead of all 10
            home.push_back(joint_values[0]);   //waist_yaw
            home.push_back(joint_values[2]);   //trunk_pitch
            for(int i = 3; i<10; i++)
                home.push_back(joint_values[i]);   
        #else
            home = joint_values;
        #endif

        sleep(0.5);
        reset();
        //sleep(0.5);
    };

    bool ik_solve(Affine3d target_KS)
    {
        bool ik_moveit = false;

        desired_arm_values.resize(7);
        for(int i =0; i<7; i++)
            desired_arm_values[i] = 0;

        int size_of_chain = 10;
        #ifdef FIXED_WAIST
        size_of_chain = 9;
        #endif

        desired_joint_values.resize(size_of_chain);
        for(int i =0; i<size_of_chain; i++)
            desired_joint_values[i] = 0;

        std::size_t attempts = ATTEMPTS;
        double update_rate_ = UPDATE_RATE;
        double timeout = 1.0/update_rate_; //timeout for each attempts
        ik_moveit = kinematic_state->setFromIK(arm_model_group, target_KS, attempts, timeout);

        //NOTE! if ik is solved, set kinematics model to the state, use reset to clear();
        if (!ik_moveit)
        {
            //ROS_ERROR_STREAM("\nMOVEIT Did not find IK solution");
            ik_moveit = false;
        }
        else{
            //ROS_INFO_STREAM("\nMOVEIT IK Okay");
            // if(!checkJointLimits())
            // {
            //     ROS_ERROR("press anykey to continue");
            //     cin.ignore();
            // }

            if(!kinematic_state->satisfiesBounds())
            {
                kinematic_state->enforceBounds();
            }

            kinematic_state->copyJointGroupPositions(arm_model_group, desired_arm_values);
            kinematic_state->copyJointGroupPositions(joint_model_group, desired_joint_values);
            
            // cout << "moveit jv: \n[";
            // for(int i = 0; i < desired_arm_values.size(); i++){
            //     cout << " " << desired_arm_values[i];
            // } cout << " ]" << endl;

            // update_arm(desired_arm_values);
            update(desired_joint_values);
            //kinematic_state->update();
            // sleep(0.5);

            ik_moveit = true;    
        }

        return ik_moveit;
    };

    //arm only
    void update_arm( vector<double> arm_joint_values ) //set only arm to next joint position
    {
        // cout << "av: \n[";
        // for(int i = 0; i < arm_joint_values.size(); i++){
        //     cout << " " << arm_joint_values[i];
        // } cout << " ]" << endl;

        kinematic_state->setJointGroupPositions(arm_model_group, arm_joint_values);
        //kinematic_state->update();
        //cout << "moveit kinematic_state updated\n" << endl;
    };

    //arm + torso
    void update( vector<double> joint_values )  //set model to next joint position
    {
        // cout << "moveit jv: \n[";
        // for(int i = 0; i < joint_values.size(); i++){
        //     cout << " " << joint_values[i];
        // } cout << " ]" << endl;

        kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
        //kinematic_state->update();
        //cout << "moveit kinematic_state updated\n" << endl;
    };

    void reset() //set model to home joint position
    {
        // cout << "reset to home!" << endl;
        update_arm(home_arm);
        update(home);
        // kinematic_state->update();
        // sleep(0.5);

        // cout << "moveit kinematic_state reset to home\n" << endl;
    }

    MatrixXd jacobian_solve()
    {
        Vector3d reference_point_position(0.0,  0.0,  0.0);
        MatrixXd jacobian;
        kinematic_state->getJacobian(arm_model_group, kinematic_state->getLinkModel(arm_model_group->getLinkModelNames().back()),
                                    reference_point_position,
                                    jacobian);
        // ROS_INFO_STREAM("Jacobian: " << jacobian);
        return jacobian;
    }

    void getJointLimits()
    {
        const std::vector<const moveit::core::JointModel*>& joints = arm_model_group->getActiveJointModels();
        
        max_limit.clear();
        min_limit.clear();

        for (int i = 0; i < joints.size(); ++i)
        {
            // Ignore joints with more than one variable
            if (joints[i]->getVariableCount() > 1)
                continue;
        
            //double current_value = getVariablePosition(joints[i]->getName());
        
            // check if joint is beyond limits
            //bool out_of_bounds = !satisfiesBounds(joints[i]);
        
            const moveit::core::VariableBounds& bound = joints[i]->getVariableBounds()[0];

            max_limit.push_back(bound.max_position_);
            min_limit.push_back(bound.min_position_);
        }//end for;
    }

    bool checkJointLimits()
    {
        cout << "checking joint limits" << endl;

        if(desired_arm_values.size() != max_limit.size())
        {
            ROS_WARN_STREAM("joint missing when checking limits");
            return false;
        }

        for( int i = 0; i < desired_arm_values.size(); i++)
        {
            double ik_val = desired_arm_values.at(i);
            double max_val = max_limit.at(i);
            double min_val = min_limit.at(i);

            if( ik_val > max_val )
            {
                ROS_ERROR_STREAM("ik value[" << i << "](" << ik_val  << ") larger than max_limit (" << max_val << ")");
                return false;
            }
            if( ik_val < min_val )
            {
                ROS_ERROR_STREAM("ik value[" << i << "](" << ik_val  << ") smaller than min_limit (" << min_val << ")");
                return false;
            }
        }

        return true;
    }

    vector< double > max_limit;
    vector< double > min_limit;
    vector< double > desired_arm_values;
    vector< double > desired_joint_values;

    //for moveit ik solver
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

private:

    planning_scene::PlanningScenePtr planning_scene_;

    std::vector<std::string> mv_joint_names;
    robot_state::JointModelGroup *joint_model_group;
    std::vector<std::string> arm_joint_names;
    robot_state::JointModelGroup *arm_model_group;

    std::vector<double> home_arm;
    std::vector<double> home;

    vector< moveit_msgs::JointLimits >  *jvl;
};

#endif //MOVEIT_IK_HPP_INCLUDED