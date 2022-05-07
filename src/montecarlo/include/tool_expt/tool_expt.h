/**
 * @file tool_expt.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * class for tool experiment
 * variable for experiment can be stored here
 * @version 0.1
 * @date 2019-03-04
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef TOOL_EXPT_HPP_INCLUDED
#define TOOL_EXPT_HPP_INCLUDED

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
#include <tool_expt/search.h>
#include <tool_expt/gjk.h>

// math
#include <math.h>

// definitions
#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

/**
 * @brief Structure of the function template
 * 
 */
struct Func_Templ
{
	Vector3d left;
	Vector3d right;
	Vector3d normal;
	Vector3d objdir;
	Vector3d objpos;
};

/**
 * @brief Tool experiment class object
 * save all the data and variables used for the tool experiment
 * computes and create task,
 * Handles and directs the MCTS experiment
 */
class Tool_Expt
{
public:
	/**
	 * @brief Construct a new Tool_Expt object
	 * 
	 */
	Tool_Expt();
	/**
	 * @brief Destroy the Tool_Expt object
	 * 
	 */
	~Tool_Expt(){};

	/**
	 * @brief tool experiment initialization
	 * 
	 * @param create_tool pointer to create_tool obj
	 * @param mct_search pointer to mct_search obj
	 * @param obj_pose object pose
	 * @param tar_pose target pose
	 */
	void init(Create_Tool *create_tool, MCT_Search *mct_search, geometry_msgs::Pose obj_pose, geometry_msgs::Pose tar_pose);
	/**
	 * @brief initialise other parameters used in the experiment
	 * 
	 * @param obj_dia object diameter
	 * @param num_via_pts number of via points to required
	 * @param num_intra_pts number of intra path points
	 * @param max_iteration number of iterations for mcts
	 */
	void init_params(double obj_dia, int num_via_pts, int num_intra_pts, int max_iteration);
	/**
	 * @brief initialise vision data obtain from perception module
	 * 
	 * @param v_data vision data used for this experiment
	 * @param percept is data obtained from perception module?
	 */
	void init_vision(vision_each v_data, bool percept=true);
	/**
	 * @brief standalone main function to test experiment outside of state machine
	 * 
	 */
	void main();

	/**
	 * @brief Create an eigen affine
	 * 
	 * @param roll 
	 * @param pitch 
	 * @param yaw 
	 * @param x 
	 * @param y 
	 * @param z 
	 * @return Affine3d eigen
	 */
	Affine3d create_affine(double roll, double pitch, double yaw, double x, double y, double z);
	/**
	 * @brief Create a eigen affine 
	 * 
	 * @param roll 
	 * @param pitch 
	 * @param yaw 
	 * @param xyz 
	 * @return Affine3d 
	 */
	Affine3d create_affine(double roll, double pitch, double yaw, Vector3d xyz);

	bool perceptFlag;
	/**
	 * @brief Create the task.
	 * set up object, target and object_to_target vector and poses.
	 */
	void create_task();
	
	/**
	 * @brief Create a tool.
	 * compute and set up tool vector and poses.
	 */
	void create_tool();
	/**
	 * @brief Create a object.
	 * generate the object vertices and get ready for gjk collision check.
	 * set up object bounding box
	 */
	void create_object();

	/**
	 * @brief perform GJK collision check
	 * 
	 * @return true collided
	 * @return false no collision
	 */
	bool collision_check();
	
	/**
	 * @brief Create all the via points between target and object
	 * 
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

	bool paramFlag;
	double m_obj_dia;

	//===============create_task===============
	int N_via_pts, N_intrapath_pts, N_path_pts;

	geometry_msgs::Pose m_obj_pose;
	geometry_msgs::Pose m_tar_pose;
	// geometry_msgs::Pose m_tool_pose;

	Affine3d obj_affine, tar_affine;
	// , tool_affine;
	Vector3d obj_pos, tar_pos, tool_pos;

	Vector3d via_pt;				//via_pt
	vector< Affine3d > T_world_obj;	//size depends on number of via pts

	Vector3d obj_task;				//task == target
	double angle_obj_task;
	Affine3d T_obj_task;			//obj to task displacement


	//================create_path==================
	// Vector3d obj_via;				//each via point keeps same displacement
	// double angle_obj_via;
	// Affine3d T_obj_via;				//single displacement

	// Vector3d via_tar;
	// double angle_via_tar;
	// Affine3d T_via_tar;				//angle change from via to target

	vector< Vector3d > path_pts;		//all the via pts coordinates
	vector< Affine3d > T_all_path_pts; 	//Tf with angle between viapts and targets 

	//================create_tool==================
	Create_Tool *m_create_tool;
	Shape subtool1, subtool2, object;

	//for Lijun 
	vision_each vdata;
	vector< Shape > tool_pieces;
	/**
	 * @brief Create a percept tool object.
	 * This function creates tool object from perception data.
	 * compute and set up tool vector and poses.
	 * 
	 */
	void create_percept_tool();

	//==================collision==================
	GJK m_gjk;
	int max_iter;

	//================functionality================
	Func_Templ m_func_templ;
	double angle_task_func;
	Affine3d T_task_func;
	vector< Affine3d > T_obj_func;

	//===================search====================
	MCT_Search *m_mct_search;

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

private:
    vector< Affine3d > result;  //_T_real_grasp
	Affine3d result_grab;
	double result_atk;
	double max_score;
};

#endif //TOOL_EXPT_HPP_INCLUDED