#ifndef CREATE_TOOL_HPP_INCLUDED
#define CREATE_TOOL_HPP_INCLUDED

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

#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <tool_expt/vision_data.h>

#include <tool_expt/recorder.h>

#include <tool_expt/definitions.h>

using namespace std;
using namespace Eigen;
using namespace pcl;


class Create_Tool
{
public:
	Create_Tool(){initFlag = false;};
	~Create_Tool(){};

	void init();
	void create();
	void displayDetails(void);

	vector< Vector3d > getVertices(){ return tool_vertices_3d;};		//sets of vertices 3d coordinates
	VectorXd get_n_pts();												//number of points in each segment (exclude the edges/extreme_pts)
	int get_N_seg(){ return N_seg;};									//number of segments
	vector< Vector3d > get_nvec_seg(){ return nvec_seg_3d;};			//sets of normal_vector for each segment
	vector< vector<Vector3d> > get_tool_seg(){ return tool_seg_3d;};			//3d coordinates for each point in each segment

	VectorXd get_angle_seg(){ return angle_seg; };
	int create_perceptTool(vision_each vdat);
	vector< vector<Vector3d> > get_seg_vertices(){ return seg_vertices_3d;};	//3d coordinates for each vertices in each segment
	
	//grasp
	int findGraspLoci();
	vector< Affine3d > grasp_loci;

	int write2file();

private:

	bool perceptFlag;
	
	bool initFlag;
	int N_seg;				//no of segment
	int prev_size;
	double len_step;		//each segment length
	double wid_seg1;
	double tool_yaw;

	VectorXd len_seg;
	VectorXd n_pts_seg;
	VectorXd angle_seg;
	VectorXd delta_angle;

	//---2D					
	/*Vector2d tool_xy;	
	vector <Matrix2d > R;
	vector< vector<Vector2d> > tool_seg;
	vector< Vector2d > nvec_seg;
	vector< Vector2d > tool_vertices;
	*/

	//--3D
	int prev_size_2;
	vector< Matrix3d > R3;
	Affine3d tool_affine;
	Vector3d tool_trans;

	vector< vector<Vector3d> > tool_seg_3d;
	vector< Vector3d > nvec_seg_3d;
	vector< Vector3d > tool_vertices_3d;

	//void fill2DRotation(int idx);
	//void fill2DSegment(int idx);

	void fill3DRotation(int idx);
	void fill3DSegment(int idx);

	vector< float > objheight2table;
	vector< PointCloud<PointXYZ> > hullClustersOnAxis;		//number of tools -> 3d coordinates of tool vertices	
	vector< Matrix4f > proj2TableTFs;
	vector< vector< PointCloud<PointXYZ> > > obj_convex;	//number of tools -> number of segments -> 3d coordinates of segment vertices
	vector< vector<Vector3d> > seg_vertices_3d;

	double tool_height;
	int N_pieces; //how many pcl pieces
	double euclidean_distance(Vector3d pt1, Vector3d pt2);
	std::vector<double> linspace(double start_in, double end_in, int num_in);

	double find_angle(Vector3d pt1, Vector3d pt2, Vector3d pt3);
	double findAngleTriangle( Vector3d va, Vector3d vb, Vector3d vc);
};

#endif //TOOL_TEST_HPP