/**
 * @file node2.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * Classs object for a mcts node, version 2
 * @version 0.1
 * @date 2019-03-07
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef NODE2_HPP_INCLUDED
#define NODE2_HPP_INCLUDED

// ros
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

// eigen
#include <Eigen/Core>
#include <Eigen/SVD>

// conversion
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

// math
#include <math.h>

// tool experiment
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

/**
 * @brief structure for a mcts node
 * 
 */
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

/**
 * @brief class object for monte carlo tree search
 * This is class for single node in the tree
 * version 2.0, setup node to perform as a recursive function
 */
class MCT_NODE2
{
public:
    /**
     * @brief Construct a new mct node2 object
     * 
     */
	MCT_NODE2(){init();};
    /**
     * @brief Destroy the mct node2 object
     * 
     */
    ~MCT_NODE2(){};

    /**
     * @brief Construct a new mct node2 object
     * 
     * @param str name 
     * @param ndata node data 
     * @param recorder pointer to logging class obj
     * @param moveit_ik pointer to moveit IK solver class obj
     * @param kdl_ik pointer to kdl IK solver class obj
     */
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

    /**
     * @brief Construct a new mct node2 object
     * 
     * @param parent pointer to parent node
     * @param str name
     * @param ndata node data
     * @param recorder pointer to logging class obj
     * @param moveit_ik pointer to moveit IK solver class obj
     * @param kdl_ik pointer to kdl IK solver class obj
     */
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

    /**
     * @brief Create a child object
     * 
     * @param str name 
     * @param ndata node data
     */
    void create_child( string str, NODE_DATA ndata );

    string label;
    
    double score;
    double node_value;

    int num_visit;
    int num_childs;
    string type;
    Vector3d arm_len;

    /**
     * @brief initialisation of node
     * 
     */
    void init();
    /**
     * @brief update node score
     * 
     * @return double score
     */
    double update();

    /**
     * @brief Function to export node data to logger
     * 
     */
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
    /**
     * @brief update the score of the node
     * 
     * @param val 
     */
    void update_score(double val);
    /**
     * @brief calculate the score of the node
     * check if IK solution is possible,
     * then score depends of the errors, delta of movement,
     * the size of the segment, the grasping position, 
     * the attack angle of the tool to object and etc.
     * @return double score value
     */
    double calculate_value();

    /**
     * @brief check if this branch & child are all visited
     * 
     * @return true visited
     * @return false not finished
     */
    bool all_visited();

    Recorder *m_recorder;
    MOVEIT_IK *m_moveit_ik;
    KDL_IK *m_kdl_ik;
    // Vector3d invKin3(Vector3d length, Vector3d X, double psi);

    dataset data;
};

#endif //NODE2_HPP_INCLUDED