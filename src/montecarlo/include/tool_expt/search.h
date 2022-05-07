/**
 * @file search.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief
 * class to perform monte carlo tree search
 * @version 0.1
 * @date 2019-03-12
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef SEARCH_HPP_INCLUDED
#define SEARCH_HPP_INCLUDED

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

// tool experiment
#include <tool_expt/create_tool.h>
#include <tool_expt/moveit_ik.h>
#include <tool_expt/kdl_ik.h>

#include <tool_expt/node.h>
#include <tool_expt/gjk.h>

// logger
#include <tool_expt/recorder.h>

// math
#include <math.h>

// definitions
#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

/**
 * @brief Class object to perform monte carlo tree search algorithm
 * 
 */
class MCT_Search
{
public:
    /**
     * @brief Construct a new mct search object
     * 
     */
    MCT_Search();
    /**
     * @brief Destroy the mct search object
     * 
     */
    ~MCT_Search(){};

    /**
     * @brief initialise class object
     * 
     * @param create_tool pointer to create_tool class object
     * @param moveit_ik pointer to moveit IK solver object
     * @param kdl_ik pointer to kdl IK solver object
     */
    void init(Create_Tool *create_tool, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik=NULL);
    /**
     * @brief initialise other parameters for MCTS consideration 
     * 
     * @param num_grasp number of grasping pose to interpolate
     * @param num_atk_angle the attack angle of the tool
     * @param plays number of search iterations
     */
    void init_params(int num_grasp, int num_atk_angle, int plays = 500);

    /**
     * @brief start the MCTS search
     * 
     */
    void search();

    //params obtained from tool_expt
    int N_via_pts, N_intrapath_pts, N_path_pts, max_iter;
    vector< Affine3d > T_world_obj;
    vector< Affine3d > T_obj_func;
    Affine3d T_obj_task;

    Shape subtool1, subtool2, object;

    // vector< Vector3d > grasp_loci;
    vector< Affine3d > grasp_loci;

    bool perceptFlag;
    vector< Shape > tool_pieces;

    //===================results====================
    double score;
    double value;
    /**
     * @brief Get the Resulting best Path to follow to pull/push object
     * 
     * @return vector< Affine3d > best path
     */
    vector< Affine3d > getResult(){ return result_T_real_grasp; };

    /**
     * @brief Get the Resulting best Grab pose
     * 
     * @return Affine3d grab pose
     */
    Affine3d getGrab(){ return result_grab_pos; };

    /**
     * @brief Get the Resulting best attach angle of the tool to object
     * 
     * @return double angle
     */
    double getAtkAngle(){return result_atk_angle; };

    /**
     * @brief Get the Score of the best solution
     * 
     * @return double score
     */
    double getScore(){return max_score; };

private:

    int Nplays;

    /**
     * @brief Construct the MCTS tree
     * 
     */
    void monte_carlo_tree();
    /**
     * @brief Perform the MCTS search through the tree
     * 
     */
    void monte_carlo_search();
    
    bool paramFlag;
    Create_Tool *m_create_tool;
    MOVEIT_IK *m_moveit_ik;
    KDL_IK *m_kdl_ik;
    GJK m_gjk;

    //===================search====================
    int N_grasp, N_atk_angle;

    double atk_angle_step;
    VectorXd atk_angle;

    VectorXd N_pts;
    int N_seg;

    vector< Vector3d > nvec_seg;
    VectorXd angle_seg;
    vector< vector< Vector3d > > tool_seg;

    //====================tree======================

    /// 4 dimension cell: N_grasp -> N_atk_angle -> N_seg -> N_pts 
    vector< vector< vector< vector< MCT_NODE >>>> P_tree;
    /// 3 dimension cell: N_grasp -> N_atk_angle -> N_seg
    vector< vector< vector< MCT_NODE >>> S_tree;
    /// 2 dimension cell: N_grasp -> N_atk_angle
    vector< vector< MCT_NODE >> A_tree;
    /// 1 dimension
    vector< MCT_NODE > G_tree;

    /// root node
    MCT_NODE root;

    ///  3 dimension cell -> N_seg -> N_pts -> N_path_pts
    vector< vector< vector< Affine3d >>> TWT;
    ///  5 dimension cell -> N_grasp -> N_atk_angle -> N_seg -> N_pts -> N_path_pts 
    vector< vector< vector< vector< vector< Affine3d >>>>> TWG;
    
    /**
     * @brief Create a affine object
     * 
     * @param roll angle in radian
     * @param pitch angle in radian
     * @param yaw angle in radian
     * @param x translation in x
     * @param y translation in y
     * @param z translation in z
     * @return Affine3d eigen affine matrix
     */
    Affine3d create_affine(double roll, double pitch, double yaw, double x, double y, double z);
    
    //====================result======================
    double max_score;
    vector< int > best;
    vector< Affine3d > result_T_real_grasp;
    Affine3d result_grab_pos;
    double result_atk_angle;

    /// logger
    Recorder rec;
};

#endif //SEARCH_HPP_INCLUDED