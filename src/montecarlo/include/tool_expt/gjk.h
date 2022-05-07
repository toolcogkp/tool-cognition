/**
 * @file gjk.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * Gilbert–Johnson–Keerthi (GJK) algorithm for object collision check
 * GJK Header structure and functions
 * @version 0.1
 * @date 2019-03-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef GJK_HPP_INCLUDED    
#define GJK_HPP_INCLUDED

// ros 
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// eigen
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

// math
#include <math.h>

using namespace std;
using namespace Eigen;

/**
 * @brief shape for GJK object
 * Contains verticles of object, eg. A CUBE have 8 vertices
 * Using Eigen Vector3d format for 3 values coordinates (x,y,z) in type double.
 */
struct Shape
{
    vector< Vector3d > vertices;
};

Shape genSweptVolObj( Shape tool, Vector3d move_vec );

/**
 * @brief GJK algothrim wrapper
 * Use this class for fast collision check between 2 objects
 */
class GJK
{
public:
	/**
	 * @brief Construct a new GJK object
	 * 
	 */
    GJK(){};

    /**
     * @brief Destroy the GJK object
     * 
     */
	~GJK(){};
	
    /**
     * @brief collision checking function
     * 
     * @param s1 Shape of object 1
     * @param s2 Shape of object 2
     * @param iterations number of iterations
     * @return true = collision detected
     * @return false = no collision
     */
    bool collision_check(Shape s1, Shape s2, int iterations);

private:

    /**
     * @brief number of iteration used in for the GJK check
     * 
     */
    int iteration;

    /**
     * @brief Weights used for GJK algorithm
     * 
     */
    Vector3d vec;
    
    /**
     * @brief compute line check
     * 
     * @param s1 shape of object 1
     * @param s2 shape of object 2
     * @param v weight
     * @param a 
     * @param b 
     */
    void pickLine( Shape s1, Shape s2, Vector3d v, Vector3d &a, Vector3d &b );

    /**
     * @brief Compute Triangle check
     * 
     * @param s1 shape of object 1
     * @param s2 shape of object 2
     * @param a 
     * @param b 
     * @param c 
     * @return true = collision detected
     * @return false = no collision
     */
    bool pickTriangle( Shape s1, Shape s2, Vector3d &a, Vector3d &b, Vector3d &c );

    /**
     * @brief Compute tetrahedron check
     * 
     * @param s1 shape of object 1
     * @param s2 shape of object 2
     * @param a 
     * @param b 
     * @param c 
     * @param d 
     * @return true = collision detected
     * @return false = no collision
     */
    bool pickTetrahedron( Shape s1, Shape s2, Vector3d &a, Vector3d &b, Vector3d &c, Vector3d &d );

    /**
     * @brief Compute support vector
     * 
     * @param s1 shape of object 1
     * @param s2 shape of object 2
     * @param v weight vector
     * @return Vector3d support vector
     */
    Vector3d support(Shape s1, Shape s2, Vector3d v);

    /**
     * @brief Get the FurthestlnDir object
     * 
     * @param s1 shape of object
     * @param v weight
     * @return Vector3d FurthestLnDir vector
     */
    Vector3d getFurthestlnDir(Shape s1, Vector3d v);
};

#endif //GJK_HPP_INCLUDED