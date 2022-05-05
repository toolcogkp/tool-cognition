#ifndef TOOL_EXPT_HPP_INCLUDED
#define TOOL_EXPT_HPP_INCLUDED

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

#include <tool_expt/create_tool.h>
#include <tool_expt/search.h>
#include <tool_expt/gjk.h>
#include <math.h>


#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

struct Func_Templ
{
	Vector3d left;
	Vector3d right;
	Vector3d normal;
	Vector3d objdir;
	Vector3d objpos;
};

class Tool_Expt
{
public:
	Tool_Expt();
	~Tool_Expt(){};

	void init(Create_Tool *create_tool, MCT_Search *mct_search, geometry_msgs::Pose obj_pose, geometry_msgs::Pose tar_pose/*, geometry_msgs::Pose tool_pose*/);
	void init_params(double obj_dia, int num_via_pts, int num_intra_pts, int max_iteration);
	void init_vision(vision_each v_data, bool percept=true);
	void main();

	Affine3d create_affine(double roll, double pitch, double yaw, double x, double y, double z);
	Affine3d create_affine(double roll, double pitch, double yaw, Vector3d xyz);

	bool perceptFlag;
	void create_task();
	
	//create bounding boxes
	void create_tool();
	void create_object();

	bool collision_check();
	
	void create_path();
	void functionality();
	
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

	//return results
	vector< Affine3d > getResult(){return result;};
	Affine3d getGrab(){return result_grab;};
    double getAtkAngle(){return result_atk; };
	double getScore(){return max_score;};

private:
    vector< Affine3d > result;  //_T_real_grasp
	Affine3d result_grab;
	double result_atk;
	double max_score;
};

#endif //TOOL_EXPT_HPP_INCLUDED