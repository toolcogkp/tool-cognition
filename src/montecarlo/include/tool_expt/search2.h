#ifndef SEARCH2_HPP_INCLUDED
#define SEARCH2_HPP_INCLUDED

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
#include <tool_expt/moveit_ik.h>
#include <tool_expt/kdl_ik.h>

#include <tool_expt/node2.h>
#include <tool_expt/gjk.h>

#include <tool_expt/recorder.h>

#include <math.h>

#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

//monte carlo tree search samuel was here
class MCT_Search2
{
public:
	MCT_Search2();
	~MCT_Search2(){};

    void init(Create_Tool *create_tool, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik=NULL, bool right=true);
    void init_params(int num_grasp, int num_atk_angle, int plays = 500);
    void search();

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
    vector< Affine3d > getResult(){ return result_T_real_grasp; };
    Affine3d getGrab(){ return result_grab_pos; };
    double getAtkAngle(){return result_atk_angle; };
    double getScore(){return max_score; };
    int getViaPt(){ return result_via_pt_index; };

    void computeAdjacentSeg();
    vector< vector< int > > adj_indices; //for each seg, the next closest segs, indexes
    
private:

    bool m_right;

    int Nplays;
    void monte_carlo_tree();
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

    MCT_NODE2 root;
	Affine3d create_affine(double roll, double pitch, double yaw, double x, double y, double z);
    
    //====================result======================
    double max_score;
    vector< int > best;
    vector< Affine3d > result_T_real_grasp;
    Affine3d result_grab_pos;
    double result_atk_angle;
    int result_via_pt_index;

    Recorder rec;
};

#endif //SEARCH2_HPP_INCLUDED