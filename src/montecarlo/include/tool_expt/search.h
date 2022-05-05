#ifndef SEARCH_HPP_INCLUDED
#define SEARCH_HPP_INCLUDED

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

#include <tool_expt/node.h>
#include <tool_expt/gjk.h>

#include <tool_expt/recorder.h>

#include <math.h>

#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

//monte carlo tree search samuel was here
class MCT_Search
{
public:
    MCT_Search();
    ~MCT_Search(){};

    void init(Create_Tool *create_tool, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik=NULL);
    void init_params(int num_grasp, int num_atk_angle, int plays = 500);
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
    vector< Affine3d > getResult(){ return result_T_real_grasp; };
    Affine3d getGrab(){ return result_grab_pos; };
    double getAtkAngle(){return result_atk_angle; };
    double getScore(){return max_score; };

private:

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

    //4 dimension cell: N_grasp -> N_atk_angle -> N_seg -> N_pts 
    vector< vector< vector< vector< MCT_NODE >>>> P_tree;
    //3 dimension cell: N_grasp -> N_atk_angle -> N_seg
    vector< vector< vector< MCT_NODE >>> S_tree;
    //2 dimension cell: N_grasp -> N_atk_angle
    vector< vector< MCT_NODE >> A_tree;
    //1 dimension
    vector< MCT_NODE > G_tree;

    MCT_NODE root;

    // 3 dimension cell -> N_seg -> N_pts -> N_path_pts
    vector< vector< vector< Affine3d >>> TWT;
    // 5 dimension cell -> N_grasp -> N_atk_angle -> N_seg -> N_pts -> N_path_pts 
    vector< vector< vector< vector< vector< Affine3d >>>>> TWG;
    
    Affine3d create_affine(double roll, double pitch, double yaw, double x, double y, double z);
    
    //====================result======================
    double max_score;
    vector< int > best;
    vector< Affine3d > result_T_real_grasp;
    Affine3d result_grab_pos;
    double result_atk_angle;

    Recorder rec;
};

#endif //SEARCH_HPP_INCLUDED