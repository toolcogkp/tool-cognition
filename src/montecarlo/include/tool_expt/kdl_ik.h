#ifndef KDL_IK_HPP_INCLUDED
#define KDL_IK_HPP_INCLUDED

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

//KDL
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

#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;
using namespace KDL;

#define D2R M_PI/180.0 
#define R2D 180.0/M_PI

//ik solver using moveit for node.m, samuel was here
class KDL_IK
{
public:
	KDL_IK(){};
	~KDL_IK(){};

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

    JntArray getDesired()
    {
        ROS_WARN_STREAM("KDL IK sol: ");
            cout << " [";
            for(int i = 0; i < 7; i++){
                cout << " " << armAngles_d(i);
            } cout << " ]" << endl;

        return armAngles_d;
    }

    //update arm
    void update( JntArray joint_values )
    {
        armAngles = joint_values;
    };

    //reset to init arm_values
    void reset()
    {
        armAngles = armAngles_Home;
    };

    //update to current position using the FK solver
    void init_home( vector<double> joint_values, JntArray max, JntArray min )
    {
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

    KDL::Tree my_tree; 
    KDL::Chain arm_chain;
    KDL::Chain RS_chain;

    std::vector<double> joint_values;
    
    JntArray armAngles_Home;

    JntArray armAngles;
    JntArray RSAngles;

    JntArray armAngles_d;
    Affine3d RS_KS;

    int arm_no;
    int rs_no;

    JntArray maxJnt, minJnt;
};

#endif //KDL_IK_HPP_INCLUDED