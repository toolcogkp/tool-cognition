#ifndef GJK_HPP_INCLUDED    
#define GJK_HPP_INCLUDED

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>

#include <math.h>

using namespace std;
using namespace Eigen;

struct Shape
{
    vector< Vector3d > vertices;
};

Shape genSweptVolObj( Shape tool, Vector3d move_vec );

class GJK
{
public:
	GJK(){};
	~GJK(){};
	
    //GOT collision = return true
    bool collision_check(Shape s1, Shape s2, int iterations);

private:

    int iteration;
    Vector3d vec;
    
    void pickLine( Shape s1, Shape s2, Vector3d v, Vector3d &a, Vector3d &b );
    bool pickTriangle( Shape s1, Shape s2, Vector3d &a, Vector3d &b, Vector3d &c );
    bool pickTetrahedron( Shape s1, Shape s2, Vector3d &a, Vector3d &b, Vector3d &c, Vector3d &d );

    Vector3d support(Shape s1, Shape s2, Vector3d v);
    Vector3d getFurthestlnDir(Shape s1, Vector3d v);
};

#endif //TOOL_EXPT_HPP_INCLUDED