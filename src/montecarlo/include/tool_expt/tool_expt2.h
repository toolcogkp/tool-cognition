/**
 * @file tool_expt2.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * class for tool experiment 2
 * variable for experiment can be stored here
 * version 2 update, using monte carlo search 2 (recursive) and obstacle check
 * @version 0.1
 * @date 2019-04-14
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef TOOL_EXPT_2_HPP_INCLUDED
#define TOOL_EXPT_2_HPP_INCLUDED

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
#include <tool_expt/gjk.h>
#include <tool_expt/search2.h>
#include <tool_expt/tool_expt.h>

// math
#include <math.h>

// definition
#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

/**
 * @brief Tool experiment class object
 * save all the data and variables used for the tool experiment
 * computes and create task,
 * Handles and directs the MCTS experiment
 * version 2.0, using the recursive search function + check for obstacles
 */
class Tool_Expt_2	: public Tool_Expt
{
public:
	/**
	 * @brief Construct a new Tool_Expt_2 object
	 * 
	 */
	Tool_Expt_2();
	/**
	 * @brief Destroy the Tool_Expt_2 object
	 * 
	 */
	~Tool_Expt_2(){};

	/**
	 * @brief tool experiment initialization
	 * 
	 * @param create_tool pointer to create_tool obj
	 * @param mct_search pointer to mct_search obj
	 * @param obj_pose object pose
	 * @param tar_pose target pose
	 */
	void init(Create_Tool *create_tool, MCT_Search2 *mct_search2, geometry_msgs::Pose obj_pose, geometry_msgs::Pose tar_pose);
	/**
	 * @brief initialise other parameters used in the experiment
	 * 
	 * @param obj_dia object diameter
	 * @param obst_dia obstacle diameter
	 * @param num_candidates number of via point candidates
	 * @param obst_pose obstacle pose
	 * @param max_iteration number of iterations for mcts
	 */
	void init_params(double obj_dia, double obst_dia, double num_candidates, geometry_msgs::Pose obst_pose, int max_iteration);
	/**
	 * @brief standalone main function to test experiment outside state machine
	 * 
	 */
	void main();

	/**
	 * @brief Create a obstacle object
	 * and draw the bounding box for thee obstacle
	 */
	void create_obstacle();
	/**
	 * @brief check if object to target is being blocked by obstacle
	 * 
	 */
	void obstruction_check();
	int N_via_candidates;
	
	/**
	 * @brief Create all the via points between target and object
	 * if obstructed by obstacle, use candidate via points to avoid
	 */
	void create_path();

	/**
	 * @brief create the task function matrix and object_to_functionality tf matrix.
	 * 
	 */
	void functionality();

	/**
	 * @brief send all the data over to search class obj and perform MCTS tree search
	 * 
	 */
	void search();

	bool obstruct_flag;

	double m_obst_dia;
	geometry_msgs::Pose m_obst_pose;

	Shape obstacle;
	Affine3d obst_affine;
	Vector3d obst_pos;

	//===================path====================
	vector< Vector3d > via_pts;		//via_pt coordinates
	
	Vector3d obj_via;				//each via point keeps same displacement
	double angle_obj_via;
	Affine3d T_obj_via;				//single displacement

	Vector3d via_tar;
	double angle_via_tar;
	Affine3d T_via_tar;				//angle change from via to target

	Affine3d T_task_func_wt_gap;
	Affine3d T_task_func_wt_gap_1;

	// for each via pts candidates
	vector< vector< Affine3d > > all_T_world_obj_bb;	//size depends on number of via pts
	vector< vector< Affine3d > > all_T_world_obst_bb;	//size depends on number of via pts
	vector< vector< Affine3d > > all_T_world_obj;		//size depends on number of via pts
	vector< vector< Vector3d > > all_move_vec;			//move vec for each via pt 
	vector< vector< Affine3d > > all_T_world_func;		//function with w.r.t. world 

	vector< vector< Affine3d > > all_T_obj_func;		//function with w.r.t. obj 

	//===================search====================
	MCT_Search2 *m_mct_search2;

	//===================results====================
    /**
     * @brief Get the Resulting best Path to follow to pull/push object
     * 
     * @return vector< Affine3d > best path
     */
	vector< Affine3d > getResult(){return result;};
    /**
     * @brief Get the Resulting best Grab pose
     * 
     * @return Affine3d grab pose
     */
	Affine3d getGrab(){return result_grab;};
    /**
     * @brief Get the Resulting best attach angle of the tool to object
     * 
     * @return double angle
     */
    double getAtkAngle(){return result_atk; };
    /**
     * @brief Get the Score of the best solution
     * 
     * @return double score
     */
	double getScore(){return max_score;};
	/**
	 * @brief Get the Via Pt to use from the best solution to avoid obstacle 
	 * 
	 * @return Vector3d viapoint translation
	 */
	Vector3d getViaPt(){return result_via_pt;};

	/**
	 * @brief Get the Obstruction. Check if obstructed
	 * 
	 * @return true obstructed
	 * @return false not obstructed
	 */
	bool getObstruct(){return obstruct_flag;};

private:
    vector< Affine3d > result;  //_T_real_grasp
	Vector3d result_via_pt;
	Affine3d result_grab;
	double result_atk;
	double max_score;
};

#endif //TOOL_EXPT_2_HPP_INCLUDED