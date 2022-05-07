/**
 * @file node.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * Classs object for a mcts node
 * @version 0.1
 * @date 2019-02-08
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef NODE_HPP_INCLUDED
#define NODE_HPP_INCLUDED

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

// conversions
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

// maths
#include <math.h>

// tool experiment
#include <tool_expt/moveit_ik.h>
#include <tool_expt/kdl_ik.h>
#include <tool_expt/recorder.h>

// definitions
#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;

/**
 * @brief class object for monte carlo tree search
 * This is class for single node in the tree
 */
class MCT_NODE
{
public:
    /**
     * @brief Construct a new mct node object
     * 
     */
	MCT_NODE(){init();};
    /**
     * @brief Destroy the mct node object
     * 
     */
    ~MCT_NODE(){};

    /**
     * @brief Construct a new mct node object
     * 
     * @param str name
     * @param recorder pointer to logging class 
     * @param moveit_ik pointer to moveit IK solver class
     * @param kdl_ik pointer to kdl IK solver class
     */
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
     * @brief Set the node idx object
     * 
     * @param g 
     * @param a 
     * @param s 
     * @param p 
     */
    void set_node_idx(int g, int a = -1, int s = -1, int p = -1)
    {
        idx_G = g;
        idx_A = a;
        idx_S = s;
        idx_P = p;
    }

    /**
     * @brief Function to export node data to logger 
     * 
     */
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
    /**
     * @brief update the score of the node
     * 
     * @param val 
     */
    void update_score(double val);
    /**
     * @brief calculate the score of the node
     * check if IK solution is possible,
     * then score depends of the errors, delta of movement and etc.
     * @return double score value
     */
    double calculate_value();
    /**
     * @brief check if this branch is all visited
     * 
     * @return true visited
     * @return false not finished
     */
    bool all_visited();

    Recorder *m_recorder;
    MOVEIT_IK *m_moveit_ik;
    KDL_IK *m_kdl_ik;
    // Vector3d invKin3(Vector3d length, Vector3d X, double psi);

    /// grasp idx 
    int idx_G; //grasp 
    /// Attack angle of tool to object
    int idx_A; //Atk Angle
    /// Segment index (is the segment wide or small?)
    int idx_S; //segment
    /// position index (which point of the tool does this node targets?)
    int idx_P; //point/position

    dataset data;
};

#endif //NODE_HPP_INCLUDED