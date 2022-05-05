#ifndef NODE2_HPP_INCLUDED
#define NODE2_HPP_INCLUDED

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
#include <memory>

using namespace std;
using namespace Eigen;

// template<typename T, typename... Args>
// std::unique_ptr<T> make_unique(Args&&... args) {
//     return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
// }

struct NODE_DATA
{
    vector< Affine3d > state;
    vector< Affine3d > state2;
    vector< Affine3d > state3;
    Affine3d T_tg;
    Affine3d T_tf;

    bool collision_obj;
    bool small_seg;

    int idx[7];
};

//monte carlo tree search, samuel was here
class MCT_NODE2
{
public:
	MCT_NODE2(){init();};
    ~MCT_NODE2(){};

    MCT_NODE2(string str, NODE_DATA ndata, Recorder *recorder, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik=NULL)
    { 
        init(); 

        label = str;
        m_ndata = ndata;

        m_recorder = recorder;
        
        m_kdl_ik = kdl_ik;
        m_moveit_ik = moveit_ik;

        best_score = 0;

        best_idx.clear();
        for(int i = 0; i < 7; i++)
            best_idx.push_back(m_ndata.idx[i]);
    };

    MCT_NODE2( MCT_NODE2 *parent, string str, NODE_DATA ndata, Recorder *recorder, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik=NULL)
    { 
        init(); 
        parentNode = parent;

        label = str;
        m_ndata = ndata;

        m_recorder = recorder;

        m_kdl_ik = kdl_ik;
        m_moveit_ik = moveit_ik;

        // m_recorder->showAddress();
    };

    MCT_NODE2* parentNode;
    vector< MCT_NODE2* > childs;
    void create_child( string str, NODE_DATA ndata );

    string label;
    
    double score;
    double node_value;

    int num_visit;
    int num_childs;
    string type;
    Vector3d arm_len;

    void init();
    double update();

    void write_dataset() //for recorder
    {
        cout << "write_dataset" << endl;
        data.label = label;        
        // cout << "a" << endl;

        // data.index = m_ndata.idx;
        for(int i=0; i<7; i++)
            data.index[i] = m_ndata.idx[i];

        data.type = type;
        data.colFlag = (int) m_ndata.collision_obj? 1 : 0;
        data.smallFlag = (int) m_ndata.small_seg? 1 : 0;
       
        // cout << "b" << endl;
        data.score = score;
        data.num_visit = num_visit;

        cout << "done" << endl;
    }
    
    NODE_DATA m_ndata;

    //best stuff
    double best_score;
    vector<int> best_idx;
    vector< Affine3d > best_state;

private:
    void update_score(double val);
    double calculate_value();
    bool all_visited();

    Recorder *m_recorder;
    MOVEIT_IK *m_moveit_ik;
    KDL_IK *m_kdl_ik;
    // Vector3d invKin3(Vector3d length, Vector3d X, double psi);

    dataset data;
};

#endif //NODE2_HPP_INCLUDED