/**
 * @file kdl_ik.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * class to use KDL library to solve Inverse Kinematics(IK)
 * @version 0.1
 * @date 2019-03-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef KDL_IK_HPP_INCLUDED
#define KDL_IK_HPP_INCLUDED

// ros
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// eigen
#include <Eigen/Core>
#include <Eigen/SVD>

// conversions
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

// math
#include <math.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>

#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <kdl/frames.hpp>
#include <eigen_conversions/eigen_kdl.h>

// definition
#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;
using namespace KDL;

/// Degree to radian
#define D2R M_PI/180.0 

/// radian to degree
#define R2D 180.0/M_PI

/**
 * @brief ik solver using KDL for node.m
 * ROS library usage, KDL setup class to quickly solve IK
 */
class KDL_IK
{
public:

    /**
     * @brief Construct a new kdl ik object
     * 
     */
	KDL_IK(){};

    /**
     * @brief Destroy the kdl ik object
     * 
     */
	~KDL_IK(){};

    /**
     * @brief initialise kdl ik object
     * KDL requires reading the urdf of robot to form a KDL tree.
     * Unable to handle tree with more than one branches
     * Hence, separated with a boolean flag for left or right arm branches of the tree.
     * Solves IK fast with mathematics
     * 
     * @param right true = right arm of olivia, false = left arm of olivia
     */
    void init(bool right=true)
    {
        ros::NodeHandle nh("~");

        //=======================================================//
        //KDL construct tree
        std::string robot_desc_string;
        nh.getParam(std::string("/robot_description"), robot_desc_string);

        if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
            ROS_FATAL_STREAM("Failed to construct kdl tree");
            return;
        }
        else
            ROS_INFO_STREAM("SUCCESS: kdl tree constructed");

        //=======================================================//
        string base_link("world");
        string end_link, arm_link;

        if(right)
        {
            end_link = "ee";
            arm_link = "upper_torso_right";
        }
        else
        {
            ROS_ERROR_STREAM("KDL_IK DOING LEFT!");
            end_link = "l_ee";
            arm_link = "upper_torso_left";
        }

        //construct chain -- RS to ee
        if(!my_tree.getChain(arm_link, end_link, arm_chain)){
            ROS_ERROR_STREAM("Failed to get chain from " << arm_link << " to " << end_link);
            return;
        }
        else
            ROS_INFO_STREAM("SUCCESS: kdl get chain from " << arm_link << " to " << end_link);
        sleep(0.5);

        //construct chain -- world to RS
        if(!my_tree.getChain(base_link, arm_link, RS_chain)){
            ROS_ERROR_STREAM("Failed to get chain from " << base_link << " to " << arm_link);
            return;
        }
        else
            ROS_INFO_STREAM("SUCCESS: kdl get chain from " << base_link << " to " << arm_link);
        sleep(0.5);

        arm_no = arm_chain.getNrOfJoints();
        rs_no = RS_chain.getNrOfJoints();

        cout << "arm no: " << arm_no << " ; rs_no: " << rs_no << endl;
        cout << "KDL_ik initalised\n" << endl;
    };

    /**
     * @brief Function to solve the IK for the given target pose
     * 
     * @param target_KS Affine matrix for the target
     * @return true IK solved successfully
     * @return false error encountered 
     */
    bool ik_solve(Affine3d target_KS)
    {
        bool ikFlag = false;

        armAngles_d.resize(7); //desired jnt angles for arm
        for(int i =0; i<7; i++)
            armAngles_d(i) = 0;

        // convert target frame reference from world to RS
        Eigen::Affine3d target_RS = RS_KS.inverse() * target_KS; 
        
        //=======================================================//
        //KDL IK solver for desired jnt angles from target cartesian frame
        ChainFkSolverPos_recursive fksolver(arm_chain);
        ChainIkSolverVel_pinv iksolver_v(arm_chain);
        
        //include joint limits into IK
        ChainIkSolverPos_NR_JL iksolver(arm_chain, minJnt, maxJnt, fksolver, iksolver_v, 5000);
        
        Frame F_desired;
        tf::transformEigenToKDL(target_RS, F_desired); // from eigen affine3d to kdl frame 

        int ret = iksolver.CartToJnt(armAngles, F_desired, armAngles_d); //input and output in radian!

        //in KDL: enum  { E_DEGRADED = +1, E_NOERROR = 0, E_NO_CONVERGE = -1, E_UNDEFINED = -2 }
        if(ret == 0)
        {
            ikFlag = true;
            for(int i = 0; i < 7; i++){
                if( fabs( armAngles_d(i) ) > M_PI)  //check if KDL solution hits singularity
                    ikFlag = false;
            }
        }

        if(ikFlag)
        {

            ROS_WARN_STREAM("KDL IK sol: ");
            cout << " [";
            for(int i = 0; i < 7; i++){
                cout << " " << armAngles_d(i);
            } cout << " ]" << endl;

            update(armAngles_d);
        }
        else
        {
            ROS_ERROR_STREAM("IK failed");
        }
        
        
        return ikFlag;
    };

    /**
     * @brief Get the Desired joint values in an array format
     * 
     * @return JntArray KDL joint array data format
     */
    JntArray getDesired()
    {
        ROS_WARN_STREAM("KDL IK sol: ");
            cout << " [";
            for(int i = 0; i < 7; i++){
                cout << " " << armAngles_d(i);
            } cout << " ]" << endl;

        return armAngles_d;
    }

    /**
     * @brief update the arm values to given joint array values
     * 
     * @param joint_values KDL joint array data format
     */
    void update( JntArray joint_values )
    {
        armAngles = joint_values;
    };

    /**
     * @brief reset and reinitialise the joint values of the arm to home position
     * this function is used to reset position of KDL tree
     */
    void reset()
    {
        //reset to init arm_values
        armAngles = armAngles_Home;
    };

    /**
     * @brief perform forward kinematics (FK) and store the position of given joint values as the home position 
     * 
     * @param joint_values the given joint values
     * @param max Maximum value of each joints
     * @param min Minimum value of each joints
     */
    void init_home( vector<double> joint_values, JntArray max, JntArray min )
    {
        //update to current position using the FK solver
        
        cout << "jv: [";
        for(int i = 0; i < joint_values.size(); i++){
            cout << " " << joint_values[i];
        } cout << " ]" << endl;

        maxJnt = max;
        minJnt = min;

        int size_of_torso = 3;
        #ifdef FIXED_WAIST
        size_of_torso = 2;
        #endif

        armAngles_Home.resize(7);
        armAngles.resize(7);
        RSAngles.resize(size_of_torso);

        for(int i =0; i < joint_values.size(); i++){ //i from 0 to 9
            if(i < size_of_torso)
                RSAngles(i) = joint_values[i];
            else
                armAngles(i-size_of_torso) = joint_values[i];
        }

        armAngles_Home = armAngles;

        //fk solve for world_to_RS frame
        ChainFkSolverPos_recursive FKSolver = ChainFkSolverPos_recursive(RS_chain);
        Frame RS_frame;
        FKSolver.JntToCart(RSAngles, RS_frame);
        tf::transformKDLToEigen (RS_frame, RS_KS);

        cout << "RS_KS: \n" << RS_KS.matrix() << endl;
        cout << "kdl kinematic_state updated\n" << endl;
    };

    /**
     * @brief solve the jacobian matrix of the KDL tree
     * 
     * @return MatrixXd jacobian matrix
     */
    MatrixXd jacobian_solve()
    {
        //RS to ee
        MatrixXd jacobian;
        KDL::Jacobian jacob;    //KDL jacobian format
        jacob.resize(arm_no);
        ChainJntToJacSolver jacSolver(arm_chain);
        jacSolver.JntToJac(armAngles, jacob);
        jacobian = jacob.data;

        return jacobian;
    }

private:

    /// store the KDL tree
    KDL::Tree my_tree; 

    /// store the kdl chain up to the arm
    KDL::Chain arm_chain;

    /// store the kdl chain up to the right shoulder (RS)
    KDL::Chain RS_chain;

    /// store the joint values
    std::vector<double> joint_values;
    
    /// store the arm values for 'Home' position
    JntArray armAngles_Home;

    /// store the current arm joint values
    JntArray armAngles;

    /// store the current right shoulder(RS) joint values
    JntArray RSAngles;

    /// store the delta (difference) of each joint in the arm
    JntArray armAngles_d;

    /// store the Right shoulder (RS) kinematics solution (KS)
    Affine3d RS_KS;

    /// the arm's number 
    int arm_no;

    /// the right shoulder number
    int rs_no;

    /// store the maximum and minimum values of each joint
    JntArray maxJnt, minJnt;
};

#endif //end KDL_IK_HPP_INCLUDED