#ifndef TOOL_EXPT_2_HPP_INCLUDED
#define TOOL_EXPT_2_HPP_INCLUDED

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
#include <tool_expt/gjk.h>
 
#include <tool_expt/search2.h>
#include <math.h>

#include <tool_expt/tool_expt.h>

#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

class Tool_Expt_2	: public Tool_Expt
{
public:
	Tool_Expt_2();
	~Tool_Expt_2(){};

	void init(Create_Tool *create_tool, MCT_Search2 *mct_search2, geometry_msgs::Pose obj_pose, geometry_msgs::Pose tar_pose/*, geometry_msgs::Pose tool_pose*/);
	void init_params(double obj_dia, double obst_dia, double num_candidates, geometry_msgs::Pose obst_pose, int max_iteration);
	void main();

	//create bounding boxes
	void create_obstacle();

	void obstruction_check();
	int N_via_candidates;
	
	void create_path();
	void functionality();
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

	//return results
	vector< Affine3d > getResult(){return result;};
	Affine3d getGrab(){return result_grab;};
    double getAtkAngle(){return result_atk; };
	double getScore(){return max_score;};
	Vector3d getViaPt(){return result_via_pt;};

	bool getObstruct(){return obstruct_flag;};

private:
    vector< Affine3d > result;  //_T_real_grasp
	Vector3d result_via_pt;
	Affine3d result_grab;
	double result_atk;
	double max_score;
};

#endif //TOOL_EXPT_2_HPP_INCLUDED