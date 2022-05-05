#ifndef NODE_HPP_INCLUDED
#define NODE_HPP_INCLUDED

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <Eigen/Core>
#include <Eigen/SVD>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <math.h>

#include <tool_expt/moveit_ik.h>
#include <tool_expt/kdl_ik.h>
#include <tool_expt/recorder.h>

#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

//monte carlo tree search, samuel was here
class MCT_NODE
{
public:
	MCT_NODE(){init();};
    ~MCT_NODE(){};

    MCT_NODE(string str, Recorder *recorder, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik=NULL)
    { 
        init(); 
        label = str;

        m_recorder = recorder;
        m_moveit_ik = moveit_ik;
        m_kdl_ik = kdl_ik;
    };

    // void initalise(string str, MOVEIT_IK *moveit_ik)
    // {
    //     init(); 
    //     label = str;
    //     m_moveit_ik = moveit_ik;
    // };

    string label;
    vector< Affine3d > state;
    vector< Affine3d > state2;
    vector< Affine3d > state3;
    Affine3d T_tg;
    Affine3d T_tf;
    
    bool small_seg;
    double score;
    int num_visit;
    double node_value;

    vector< MCT_NODE > childs;
    int num_childs;
    string type;
    Vector3d arm_len;
    bool collision_obj;

    void init();
    double update();

    void set_node_idx(int g, int a = -1, int s = -1, int p = -1)
    {
        idx_G = g;
        idx_A = a;
        idx_S = s;
        idx_P = p;
    }

    void write_dataset() //for recorder
    {
        data.label = label;
        
        int idx_temp[] = { idx_G, idx_A, idx_S, idx_P, -1, -1, -1};
        // data.index.push_back(idx_G);
        // data.index.push_back(idx_A);
        // data.index.push_back(idx_S);
        // data.index.push_back(idx_P);

        for(int i=0; i<7; i++)
            data.index[i] = idx_temp[i];

        data.type = type;
        data.colFlag = collision_obj? 1 : 0;
        data.smallFlag = small_seg? 1 : 0;
        data.score = score;
        data.num_visit = num_visit;
    }


private:
    void update_score(double val);
    double calculate_value();
    bool all_visited();

    Recorder *m_recorder;
    MOVEIT_IK *m_moveit_ik;
    KDL_IK *m_kdl_ik;
    // Vector3d invKin3(Vector3d length, Vector3d X, double psi);

    int idx_G; //grasp 
    int idx_A; //Atk Angle
    int idx_S; //segment
    int idx_P; //point/position

    dataset data;
};

#endif //NODE_HPP_INCLUDED