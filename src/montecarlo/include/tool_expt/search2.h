/**
 * @file search2.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * class to perform monte carlo tree search using recursive function
 * version 2.0
 * @version 0.1
 * @date 2019-04-10
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef SEARCH2_HPP_INCLUDED
#define SEARCH2_HPP_INCLUDED

// ros
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// eigen
#include <Eigen/Core>
#include <Eigen/SVD>

// conversion
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

// tool experiment
#include <tool_expt/create_tool.h>
#include <tool_expt/moveit_ik.h>
#include <tool_expt/kdl_ik.h>

#include <tool_expt/node2.h>
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
 * recursive function, optimisation and memory save
 * version 2.0
 */
class MCT_Search2
{
public:
    /**
     * @brief Construct a new mct search2 object
     * 
     */
	MCT_Search2();
    /**
     * @brief Destroy the mct search2 object
     * 
     */
	~MCT_Search2(){};

    /**
     * @brief initialise class object
     * 
     * @param create_tool pointer to create_tool class object
     * @param moveit_ik pointer to moveit IK solver object
     * @param kdl_ik pointer to kdl IK solver object
     * @param right which arm, true=right, false=left
     */
    void init(Create_Tool *create_tool, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik=NULL, bool right=true);
    /**
     * @brief initialise other parameters for MCTS consideration 
     * 
     * @param num_grasp number of grasping pose to interpolate
     * @param num_atk_angle the attack angle of the tool
     * @param plays number of search iterations
     */
    void init_params(int num_grasp, int num_atk_angle, int plays = 500);
    /**
     * @brief Start the MCTS search
     * 
     */
    void search();

    /**
     * @brief Set the node data
     * 
     * @param node_idx node index 
     * @param cflag is there a collision object?
     * @param sflag is it a small segment?
     * @return NODE_DATA 
     */
    NODE_DATA set_ndata(int node_idx[7], bool cflag=false, bool sflag=false);

    //-----------------------------------------------------------------------//
    //params obtained from tool_expt
	int N_via_pts, N_intrapath_pts, N_path_pts, max_iter, N_via_candidates;

	Shape subtool1, subtool2, object, obstacle;
    bool perceptFlag;
    vector< Shape > tool_pieces;
    bool obstruct_flag;

    Affine3d T_obj_task;

    // for each via pts candidates
	vector< vector< Affine3d > > all_T_world_obj_bb;	//size depends on number of via pts
	vector< vector< Affine3d > > all_T_world_obst_bb;	//size depends on number of via pts
	vector< vector< Affine3d > > all_T_world_obj;		//size depends on number of via pts
	vector< vector< Vector3d > > all_move_vec;			//move vec for each via pt 
	vector< vector< Affine3d > > all_T_world_func;		//function with w.r.t. world 

    vector< vector< Affine3d > > all_T_obj_func;      //function with w.r.t. object 

    //-----------------------------------------------------------------------//
    //params obtained from create_tool
	vector< Affine3d > grasp_loci;

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
    /**
     * @brief Get the Resulting best Via Point index 
     * 
     * @return int index of the best via point
     */
    int getViaPt(){ return result_via_pt_index; };
    /**
     * @brief Compute the adjacent segment indices
     * 
     */
    void computeAdjacentSeg();
    vector< vector< int > > adj_indices; //for each seg, the next closest segs, indexes
    
private:

    bool m_right;

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

    /// the root node of the recursive mcts tree
    MCT_NODE2 root;

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
    int result_via_pt_index;

    /// logger
    Recorder rec;
};

#endif //SEARCH2_HPP_INCLUDED