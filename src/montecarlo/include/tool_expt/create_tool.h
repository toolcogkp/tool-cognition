/**
 * @file create_tool.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * Class wrapper to create a Tool object
 * Contains required information of 'Tool' object for monte carlo search 
 * @version 0.1
 * @date 2019-03-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef CREATE_TOOL_HPP_INCLUDED
#define CREATE_TOOL_HPP_INCLUDED

// ros 
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

// eigen 
#include <Eigen/Core>
#include <Eigen/SVD>

// conversions
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

// math
#include <math.h>

// pcl library for perception
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// vision data class
#include <tool_expt/vision_data.h>

// logging class 
#include <tool_expt/recorder.h>

// common definition used
#include <tool_expt/definitions.h>

// namespaces
using namespace std;
using namespace Eigen;
using namespace pcl;

/**
 * @brief Create Tool class object
 * Contains required information for a detect Tool object
 */
class Create_Tool
{
public:
	/**
	 * @brief Construct a new Create_Tool object
	 * 
 	 */
	Create_Tool(){initFlag = false;};
	
	/**
	 * @brief Destroy the Create_Tool object
	 * 
	 */
	~Create_Tool(){};

	/**
	 * @brief initialise function to set default parameters/stucture objects
	 * 1) initialise step size required for monte carlo computation
	 * 2) initialise empty tool_pose and tool_affine
	 * 3) initialise a default 'Tool' stucture 
	 * 4) initialise perception Flag
	 */
	void init();

	/**
	 * @brief function to create 3D Tool object
	 * Compute vertices of tested 'Tool' object
	 * Fill 3D rotation matrix based on object
	 * Generate 3D segment of given object
	 * 
	 * WHEN USING PERCEPTION MODULE, USE create_perceptTool FUNCTION instead and provide visual data.
	 *
	 */
	void create();

	/**
	 * @brief printing for debug check
	 * 
	 */
	void displayDetails(void);

	/**
	 * @brief Get the Vertices object
	 * return sets of vertices in 3d coordinates frame
	 * @return vector< Vector3d > sets of verticles from created 'Tool' object
	 */
	vector< Vector3d > getVertices(){ return tool_vertices_3d;};	
	
	/**
	 * @brief Store the number of point for each segement of each 'Tool' object
	 * the number of points in each segment (exclude the edges/extreme_pts)
	 * @return VectorXd 
	 */
	VectorXd get_n_pts();						

	/**
	 * @brief Get the number of segment for this 'Tool' object
	 * 
	 * @return int number of segments
	 */
	int get_N_seg(){ return N_seg;};

	/**
	 * @brief Return the normal vector for each segment of this 'Tool' object
	 * 
	 * @return vector< Vector3d > sets of normal_vector for each segment
	 */
	vector< Vector3d > get_nvec_seg(){ return nvec_seg_3d;};

	/**
	 * @brief Get the tool seg object
	 * 
	 * @return vector< vector<Vector3d> > 3d coordinates for each point in each segment
	 */
	vector< vector<Vector3d> > get_tool_seg(){ return tool_seg_3d;};

	/**
	 * @brief Get the rotational angles for each segment of the 'Tool' object
	 * 
	 * @return VectorXd 
	 */
	VectorXd get_angle_seg(){ return angle_seg; };

	/**
	 * @brief Create a 'Tool' object from perception data
	 * Refer to vision_data.h for perceived object data structure and format
	 * 
	 * @param vdat perception data 
	 * @return int 
	 */
	int create_perceptTool(vision_each vdat);

	/**
	 * @brief Get every vertices in every segments of this 'Tool' object
	 * Each vertice contains the 3d coordinates in (x,y,z) and double data_type.
	 * @return vector< vector<Vector3d> > 3d sets of verticles in every segment of this tool
	 */
	vector< vector<Vector3d> > get_seg_vertices(){ return seg_vertices_3d;};
	
	/**
	 * @brief Compute a set of possible grasping location for Olivia to hold 
	 * 
	 * @return int 
	 */
	int findGraspLoci();

	/**
	 * @brief Store the set of Affine Matrix of possible grasping location in this Tool frame 
	 * 
	 */
	vector< Affine3d > grasp_loci;

	/**
	 * @brief logging function
	 * To record and export all of this 'Tool' object information
	 * @return int 
	 */
	int write2file();

private:

	/// flag to track if this is a default(coded) or a percieved 'Tool' object
	bool perceptFlag;
	
	/// flag to check if data is properly initialised before computation
	bool initFlag;
	
	/// number of segments in this 'Tool'
	int N_seg;				

	/// store previous size of segment
	int prev_size;

	/// store step size of each segment (segment length)
	double len_step;
	
	/// store displacement between mesh axis and tool axis
	double wid_seg1;

	/// store the yawing(rotation) angle of tool
	double tool_yaw;

	/// store the length of each segment
	VectorXd len_seg;

	/// store the number of points in each segment
	VectorXd n_pts_seg;

	/// store the angle of each segment
	VectorXd angle_seg;

	/// store the delta (difference) angle for each segment
	VectorXd delta_angle;

	//---2D------------------------------------------------------//					
	/*Vector2d tool_xy;	
	vector <Matrix2d > R;
	vector< vector<Vector2d> > tool_seg;
	vector< Vector2d > nvec_seg;
	vector< Vector2d > tool_vertices;
	*/

	//--3D-------------------------------------------------------//
	/// store the previous size of segment
	int prev_size_2;

	/// store the rotational matrix for each segment
	vector< Matrix3d > R3;

	/// store the pose of the tool in eigen affine matrix
	Affine3d tool_affine;

	/// store the translational vector of the tool
	Vector3d tool_trans;

	/// store the set of vertices that are contained in each segment of the 'Tool' object
	vector< vector<Vector3d> > tool_seg_3d;

	/// store the normal vector for each segment of the 'Tool' object
	vector< Vector3d > nvec_seg_3d;

	/// store the 3D vertices of the 'Tool' object
	vector< Vector3d > tool_vertices_3d;

	//void fill2DRotation(int idx);
	//void fill2DSegment(int idx);

	/**
	 * @brief function to compute and fill the rotational matrix of the selected segment of this 'Tool' object
	 * 
	 * @param idx index of the segment
	 */
	void fill3DRotation(int idx);

	/**
	 * @brief function to compute and fill the segmention of the selected segment of this 'Tool' object
	 * 
	 * @param idx index of the segment
	 */
	void fill3DSegment(int idx);

	vector< float > objheight2table; //decomissioned; only used on testing concept phase

	/// for each number of 'Tool', store the 3d coordinates of tool vertices	
	vector< PointCloud<PointXYZ> > hullClustersOnAxis;

	/// for each percieved 'Tool', store the Transform(TF) of the tool projection from the table plane 		
	vector< Matrix4f > proj2TableTFs;

	/// for each number of 'Tool', and for each number of segments in each 'Tool', store the convex pointcloud points
	vector< vector< PointCloud<PointXYZ> > > obj_convex;

	/// for each number of 'Tool', and for each number of segments in each 'Tool', store the 3d coordinates of the vertices
	vector< vector<Vector3d> > seg_vertices_3d;

	/// store the tool height from the table plane
	double tool_height;

	/// how many pieces segmented using PCL
	int N_pieces; 
	
	/**
	 * @brief Function to compute the euclidean_distance between 2 points
	 * 
	 * @param pt1 xyz coordinate of point 1
	 * @param pt2 xyz coordinate of point 2
	 * @return double distance between points
	 */
	double euclidean_distance(Vector3d pt1, Vector3d pt2);

	/**
	 * @brief Compute linear spacing between a given length 
	 * 
	 * @param start_in start coordinate
	 * @param end_in end coordinate
	 * @param num_in number of spacing
	 * @return std::vector<double> vector containing the coordinate of each spacing
	 */
	std::vector<double> linspace(double start_in, double end_in, int num_in);

	/**
	 * @brief compute the angle between 3 given points 
	 * angle is computed between vector of point12 and point23,
	 * join pt1-pt2-pt3, find angle formed at pt2 between pt1 and pt3.
	 * @param pt1 point 1
	 * @param pt2 point 2
	 * @param pt3 point 3
	 * @return double angle in radian
	 */
	double find_angle(Vector3d pt1, Vector3d pt2, Vector3d pt3);

	/**
	 * @brief Compute the angle in a triangle formed by 3 vectors
	 * 	assume a triangle and find angle theta at coordinate b (middle argument)
	 * 			     a
	 *				  /\
	 *  		     /  \
	 *			    /    \
	 *	 angle@ b  /_)____\ c
	 * 
	 * @param va vector 1
	 * @param vb vector 2
	 * @param vc vector 3
	 * @return double angle in radian
	 */
	double findAngleTriangle( Vector3d va, Vector3d vb, Vector3d vc);
};

#endif //CREATE_TOOL_HPP_INCLUDED