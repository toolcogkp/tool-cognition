#include <tool_expt/create_tool.h>

using namespace std;
using namespace Eigen;


//------------------------------------------------------------
/*
void Create_Tool::fill2DRotation(int idx)
{
	//2x2 2D rotation matrix
	Matrix2d temp;
	temp << cos(angle_seg[idx]), -sin(angle_seg[idx]),
			sin(angle_seg[idx]),  cos(angle_seg[idx]);
	R.push_back(temp);
}

void Create_Tool::fill2DSegment(int idx)
{   
	vector<Vector2d> coordinates;

	Matrix2d rot;
	rot = R[idx];

	Vector2d temp_1;
	temp_1 << 0, wid_seg1/2.0;

	vector<double> ls = linspace(0, len_seg[idx], n_pts_seg[idx]);

	int size = n_pts_seg[idx];

	for(int i = 0; i < size; i++)
	{
		Vector2d temp_2;
		temp_2 <<  ls[i], 0;

		Vector2d coord;
		if(idx == 0){
			//mathlab: tool_seg{i} = tool_pos + R(:,:,i)*[0;wid_seg1/2] + R(:,:,i)*[linspace(0,len_seg(i),n_pts_seg(i)) ; zeros(1,n_pts_seg(i))];
			coord = tool_xy + rot*temp_1 + rot*temp_2;
		}
		else{
			//mathlab: tool_seg{i} = tool_seg{i-1}(:,end) + R(:,:,i)*[linspace(0,len_seg(i),n_pts_seg(i)) ; zeros(1,n_pts_seg(i))];
			coord = tool_seg.at(idx-1).at(prev_size) + rot*temp_2;
		}
		coordinates.push_back(coord);
	}//end for

	prev_size = size-1;
	tool_seg.push_back(coordinates);
}*/


std::vector<double> Create_Tool::linspace(double start_in, double end_in, int num_in)
{

  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  int num = static_cast<int>(num_in);

  if (num == 0) { return linspaced; }
  if (num == 1) 
    {
      linspaced.push_back(start);
      return linspaced;
    }

  double delta = (end - start) / (num - 1);

  for(int i=0; i < num-1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
  linspaced.push_back(end); // I want to ensure that start and end
                            // are exactly the same as the input
  return linspaced;
}

//-------------------------------------------------------------

void Create_Tool::fill3DRotation(int idx)
{
	//4x4 3D rotation matrix
	double ang = angle_seg(idx);							//along z axis

	Quaterniond temp_q;
	temp_q = AngleAxisd(0, Vector3d::UnitX())					//roll, axis_x
			* AngleAxisd(0, Vector3d::UnitY())					//pitch, axis_y
			* AngleAxisd(ang, Vector3d::UnitZ());			//yaw, axis_z
	Matrix3d temp_r1 = temp_q.normalized().toRotationMatrix();
						
	//Quaterniond temp_q2(tool_affine.linear());
	//Matrix3d temp_r2 = temp_q2.normalized().toRotationMatrix();
	
	//Matrix3d temp;
	//temp = temp_r2 * temp_r1;

	R3.push_back(temp_r1);
}

void Create_Tool::fill3DSegment(int idx)
{   
	vector<Vector3d> coordinates;

	Matrix3d rot;
	rot = R3.at(idx);

	//translate 1.5cm from tool axis
	Vector3d temp_1( 0, wid_seg1/2.0, 0);

	//segmentation translation
	int segs = n_pts_seg[idx];
	if( segs < 2)
		segs = 2; //make sure ending is inside => min 2
	vector<double> ls = linspace(0, len_seg[idx], segs);
 
	// int size = n_pts_seg[idx];
	int size = ls.size();
	// cout << "ls size " << size << endl;

	for(int i = 0; i < size; i++)
	{
		// cout << "ls: " << ls[i] << endl;
		//4x1 seg translation
		Vector3d temp_2( ls[i], 0, 0 );

		Vector3d coord;
		if(idx == 0){
			//3x1 + 3x3 * 3x1 + 3x3 * 3x1			
			coord = tool_trans + rot*temp_1 + rot*temp_2;
		}
		else{
			coord = tool_seg_3d.at(idx-1).at(prev_size_2) + rot*temp_2;
		}
		coordinates.push_back(coord);
	}//end for

	prev_size_2 = size-1;
	tool_seg_3d.push_back(coordinates);
}

void Create_Tool::init()
{
	// --- the tool info should be obtain from perception next time

	len_step = STEP_SIZE;	//step size->affects computational speed
	
	geometry_msgs::Pose tool_pose;	//with respect to the tool_world frame
	tool_pose.position.x = tool_pose.position.y = tool_pose.position.z = 0;
	tool_pose.orientation.x = tool_pose.orientation.y = tool_pose.orientation.z = 0;
	tool_pose.orientation.w = 1;

	// --- 3D
	tf::poseMsgToEigen(tool_pose, tool_affine);		//4x4
	tool_trans = tool_affine.translation(); 	//3x1 vector

	Quaterniond quat(tool_affine.linear());
	Vector3d tool_euler = quat.toRotationMatrix().eulerAngles(0,1,2);
	tool_yaw = tool_euler(2); //pi/2*0;	
	cout << "tool yaw: " << tool_yaw << endl;

	N_seg = 6;
	len_seg.resize(N_seg);
	delta_angle.resize(N_seg);
	n_pts_seg.resize(N_seg);
	angle_seg.resize(N_seg);

	delta_angle << 0, M_PI/4.0, -M_PI/2.0, -M_PI/2.0, -M_PI/4.0, -M_PI/2.0;	
	wid_seg1 = 0.03; 
	len_seg[0] = 0.30;
	len_seg[1] = 0.15;
	len_seg[2] = 0.03;
	len_seg[3] = len_seg[1] + wid_seg1*sin(0.5*delta_angle[1]);
	len_seg[4] = len_seg[0] + wid_seg1*sin(0.5*delta_angle[1]);
	len_seg[5] = 0.03;
	
	initFlag = true;
	perceptFlag = false;
	cout << "tool initialised" << endl;
	// --- end tool info
}

void Create_Tool::create()
{
	if(!initFlag){
		cout << "tool not initialised!" << endl;
		return;
	}
	
	//rmb to clear in case run create() multiple times
	nvec_seg_3d.clear();
	tool_vertices_3d.clear();
	R3.clear();
	tool_seg_3d.clear();

	//figure out number of segs per len step
	for(int i = 0; i < N_seg; i++)
	{
		if(len_seg[i] < LEN_SMALL)
			len_step = STEP_SIZE_SMALL;
		else
			len_step = STEP_SIZE;

		n_pts_seg[i] = floor(len_seg[i]/len_step) + 1; // round down nearest int

		if(i == 0){
			angle_seg(i) = tool_yaw + delta_angle(i);
		}
		else{
			angle_seg(i) = angle_seg(i-1) + delta_angle(i);
		}

		// --- 2D
		//fill2DRotation(i);
		//fillSegment(i);
		// Vector2d temp_3(0,1);
		// temp_3 = R[i]*temp_3;
		// nvec_seg.push_back(temp_3);
		// tool_vertices.push_back(tool_seg.at(i).at(0));

		// --- 3D
		fill3DRotation(i);
		fill3DSegment(i);

		Vector3d temp_4(0,1,0);
		temp_4 = R3.at(i)*temp_4;
		nvec_seg_3d.push_back(temp_4);
		tool_vertices_3d.push_back(tool_seg_3d.at(i).at(0));
	}  
}

void Create_Tool::displayDetails(void)
{
	cout << "*******************************" << endl;

	cout << "N_seg: " << N_seg << endl;
	cout << "len_step: " << len_step << endl;

	cout << "len_seg: \n" << len_seg.transpose() << endl;
	cout << "delta_angle : \n" << 180/M_PI * delta_angle.transpose() << endl;
	cout << "angle_seg: \n" << 180/M_PI * angle_seg.transpose() << endl;
	
	cout << "n_pts_seg: \n" << n_pts_seg.transpose() << endl;

	// --- 2D
	/*cout << "rotational: \n";
	for(int i=0; i<N_seg; i++)
	{
		cout << "idx " << i << ": \n";
		cout << R[i] << endl << endl;
	}cout << "\n";
	
	cout << "nvec_seg: \n";
	for(int i=0; i<N_seg; i++)
	{
		cout << nvec_seg[i].transpose() << "\n";
	}cout << "\n";

	cout << "tool_vertices: \n";
	for(int i=0; i<N_seg; i++)
	{
		cout << tool_vertices[i].transpose() << "\n";
	}cout << "\n";

	cout << "tool_seg: \n";
	for(int i=0; i<N_seg; i++)
	{
		cout << "index " << i << ": \n";
		for(int j = 0; j < tool_seg.at(i).size(); j++)
		{
			cout << tool_seg.at(i).at(j).transpose() << "\n";
		}cout << "\n";
	}cout << "\n";*/
	
	// --- 3D
	cout << "rotational_3d: \n";
	for(int i=0; i<N_seg; i++)
	{
		cout << "idx " << i << ": \n";
		cout << R3.at(i).matrix() << endl << endl;
	}cout << "\n";

	cout << "nvec_seg_3d: \n";
	for(int i=0; i<N_seg; i++)
	{
		cout << nvec_seg_3d[i].transpose() << "\n";
	}cout << "\n";

	cout << "tool_vertices_3d: \n";
	for(int i=0; i<N_seg; i++)
	{
		cout << tool_vertices_3d[i].transpose() << "\n";
	}cout << "\n";

	cout << "tool_seg_3d: \n";
	for(int i=0; i<N_seg; i++)
	{
		cout << "index " << i << ": \n";
		for(int j = 0; j < tool_seg_3d.at(i).size(); j++)
		{
			cout << tool_seg_3d.at(i).at(j).transpose() << "\n";
		}cout << "\n";
	}cout << "\n";

	// --- 3D percept
	if(perceptFlag)
	{
		cout << "seg_vertices_3d: \n";
		for( int i =0; i < seg_vertices_3d.size(); i++)
		{
			vector< Vector3d > segment = seg_vertices_3d.at(i);
			for(int z =0; z < segment.size(); z++)
				cout<< "segment["<< z << "]: " << segment.at(z).transpose() << endl;
		}

		cout << "grasp_loci: \n";
		for(int n = 0; n < grasp_loci.size(); n++)
			cout << "grasp_loci[" << n << "]: \n" << grasp_loci.at(n).matrix() << endl;
	}

	cout << "*******************************\n" << endl;

}

//return num of pts for use in monte carlo search
VectorXd Create_Tool::get_n_pts()
{
	VectorXd n_pts;
	n_pts.resize(N_seg);
	for( int i = 0; i < N_seg; i++)
	{
		if(n_pts_seg(i) > 2)
		{
			n_pts(i) = n_pts_seg(i) -2; //exclude extreme points
		}
		else
		{
			n_pts(i) = 1;
		}
	}
	return n_pts;
}

int Create_Tool::create_perceptTool(vision_each vdat)
{
	perceptFlag = true;
	wid_seg1 = 0; //no displacement between mesh axis and tool axis

	// for each tool only
	// multiple tool can use multiple create_tool class later on
	PointCloud<PointXYZ> outerVertices = vdat.hullClustersOnAxis;	//red
	N_seg = outerVertices.size();

	tool_height = vdat.objhight2table;
	//-------------------------------------------------------------------//

	//for monte carlo
	tool_vertices_3d.clear();	//vector< Vector3d >
	for(int i = 0; i < outerVertices.size(); i++)
	{
		PointXYZ this_pt = outerVertices.at(i);
		Vector3d v_temp( this_pt.x, this_pt.y, this_pt.z );	//z is zero here
		tool_vertices_3d.push_back(v_temp);
	}
    cout << "tool vertices okay!" << endl;

	len_seg.resize(N_seg);
	delta_angle.resize(N_seg);
	for(int i = 0; i < N_seg; i++)
	{
		// cout << "i -" << i << endl;
		double length = 0;
		if(i == N_seg-1)	//last one find wt respect to first vertice
			length = euclidean_distance(tool_vertices_3d.at(0), tool_vertices_3d.at(i));	
		else			//find wt respect to previous vertice
			length = euclidean_distance(tool_vertices_3d.at(i+1), tool_vertices_3d.at(i));	
		len_seg(i) = length;
		// cout << "length okay" << endl;

		double ang = 0;
		if(i == 0)
		{
			// ang = find_angle(tool_vertices_3d.at(5), tool_vertices_3d.at(i), tool_vertices_3d.at(i+1)); //4, 5, 0 -> angle at 5
			Vector3d v_temp = tool_vertices_3d.at(1) - tool_vertices_3d.at(0);
			ang = atan2( v_temp(1), v_temp(0) );
		} 
		else if(i == N_seg-1)
			ang = find_angle(tool_vertices_3d.at(i-1), tool_vertices_3d.at(i), tool_vertices_3d.at(0)); //4, 5, 0 -> angle at 5
		else
			ang = find_angle(tool_vertices_3d.at(i-1), tool_vertices_3d.at(i), tool_vertices_3d.at(i+1)); //0, 1, 2 -> angle at 1
		delta_angle(i) = ang;
		// cout << "ang okay" << endl;
	}
	
    cout << "delta angles and length segment okay!" << endl;

	//TODO divide the vertices into points
	tool_seg_3d.clear();		//vector< vector<Vector3d> >
	n_pts_seg.resize(N_seg);	//number of points in each seg VectorXd
	angle_seg.resize(N_seg);
	
	for(int a = 0; a < N_seg; a++)
	{	
		if(len_seg[a] < LEN_SMALL)
			len_step = STEP_SIZE_SMALL;
		else
			len_step = STEP_SIZE;

		//for each seg
		n_pts_seg(a) = floor( len_seg[a] / len_step ) + 1; 

		if(a == 0){
			angle_seg(a) = delta_angle(a);
		}
		else{
			angle_seg(a) = angle_seg(a-1) + delta_angle(a);
		}

		fill3DRotation(a);
    	// cout << "fill rotation okay!" << endl;

		//segmentation translation
		fill3DSegment(a);
    	// cout << "fill segment okay!" << endl;

		Vector3d temp_4(0,1,0);
		temp_4 = R3.at(a)*temp_4;
		nvec_seg_3d.push_back(temp_4);
		// tool_vertices_3d.push_back(tool_seg_3d.at(i).at(0));
	}//end for
    cout << "fill rotation and segment okay!" << endl;

	//-------------------------------------------------------------------//

	//for each tool only
	vector< PointCloud<PointXYZ> > pc_segments = vdat.toolConvexHulls;
	N_pieces = pc_segments.size(); // get number of segments

	// ONLY for gjk only 
	// -> !warning dun convert to 'points' using these 'segments' 
	seg_vertices_3d.clear();	//vector< vector<Vector3d> >
	for(int j = 0; j < N_pieces; j++)
	{	
		//for each seg
		PointCloud<PointXYZ> each_seg = pc_segments.at(j);
		vector< Vector3d > seg_temp;
		for(int k = 0; k < each_seg.size(); k++)
		{
			PointXYZ this_pt = each_seg.at(k);
			Vector3d v_temp( this_pt.x, this_pt.y, this_pt.z );
			seg_temp.push_back(v_temp);
		}
		seg_vertices_3d.push_back(seg_temp);
	}
    cout << "gjk seg_vertices okay!" << endl;

	findGraspLoci();
	cout << "find grasp loci okay!" << endl;

	#ifdef RECORD_START
	write2file();
	cout << "write to file done" << endl;
	#endif
}

double Create_Tool::euclidean_distance(Vector3d pt1, Vector3d pt2)
{
	Vector3d diff = pt1 - pt2;
	return diff.norm(); 
}

double Create_Tool::find_angle(Vector3d pt0, Vector3d pt1, Vector3d pt2)
{
	Vector3d v01 = pt1 - pt0;
	Vector3d v12 = pt2 - pt1;

	//angle2 - angle1
	double theta =  atan2(v12(1), v12(0)) - atan2(v01(1), v01(0));
	
	if(theta > M_PI)
		theta = theta - 2*M_PI;
	
	if(theta < -M_PI)
		theta = theta + 2*M_PI;

	return  theta;
}

double Create_Tool::findAngleTriangle( Vector3d va, Vector3d vb, Vector3d vc)
{
	//assume a triangle and find angle theta at coordinate b (middle argument)
	// 			     a
	//				  /\
	//  		     /  \
	//			    /    \
	//	 angle@ b  /_)____\ c

	Vector3d v_ba = va - vb;
	Vector3d v_bc = vc - vb;

	//cos theta = a . b / ||a||*||b||
	double theta = v_ba.dot(v_bc) / ( v_ba.norm() * v_bc.norm() );
	return theta; 
}

int Create_Tool::findGraspLoci()
{
	// using the toolConvexHulls segments (green ones, also for gjk)
	// already saved into: 	vector< vector<Vector3d> > seg_vertices_3d;
	// each segment each points coordinates

	cout << "finding grasp loci" << endl;
	cout << "++++++++++++++++++++++++++++" << endl;
	grasp_loci.clear();

	//for each segment
	for( int i =0; i < seg_vertices_3d.size(); i++)
	{
		vector< Vector3d > segment = seg_vertices_3d.at(i);
		int t_size = segment.size();
		
		cout << "checking segment " << i << endl;

 		//ignore triangle segments (assume cannot grab triangle)
		if(t_size > 6 && (t_size%2 == 0) ) //must be even too by right
		{
			int h_size = t_size / 2; //for the first half (since the back is repeated with addition of z)
			Vector3d temp = segment.at(h_size);
			double height = temp(2); //z

			// cout << "checking midpoint " << endl;
			
			//find the midpoint between each vertices
			vector< Vector3d > midpoints;
			for(int j =0; j < h_size; j++)
			{
				Vector3d point;
				if(j == h_size-1)
					point = 0.5 * ( segment.at(j) + segment.at(0) );
				else
					point = 0.5 * ( segment.at(j) + segment.at(j+1) );
				midpoints.push_back(point);
				// cout << "checking point: " << point.transpose() << endl;
			}
			// cout << "midpoints okay " << endl;

			//find largest length between midpoints
			vector< Vector3d > index;
			double largest_len = -1; 
			for(int k =0; k < midpoints.size(); k++)
			{
				int idx = k;
				for( int l =idx+1; l < midpoints.size(); l++)
				{
					double length_temp = euclidean_distance( midpoints.at(idx), midpoints.at(l) );
					if( length_temp > largest_len)
					{
						largest_len = length_temp;

						index.clear();
						index.push_back(midpoints.at(idx));
						index.push_back(midpoints.at(l));
					}
				}
			}

			if(index.size() == 2)
			{
		
				cout << "largest len: " << largest_len << endl;
				cout << "@index: \n" << index.at(0).transpose() 
							<< "\n" << index.at(1).transpose() << endl;

				//assume hand length to be 0.11m or 11cm
				//  hand ____________
				// 		| ____|/|  __\	^
				// 		|   |///|  ___\	|
				// 		|   |///|  ___/	| 11cm
				// 		|___|///|___/	v
				//	   		tool

				//if long enough to fit hand
				if(largest_len > GRAB_ALLOWED_LENGTH)
				{
				
					int division = floor( (largest_len - GRAB_ALLOWED_LENGTH) / SPACING ) + 2;
					vector<double> ls = linspace( GRAB_ALLOWED_LENGTH/2.0, largest_len - GRAB_ALLOWED_LENGTH/2.0, division); //linspace of 11cm
					
					//find gradient rotation matrix of the line
					Vector3d vec_temp = index.at(1) - index.at(0);	// coordinates that give largest length
					double ang = atan2( vec_temp(1), vec_temp(0) ); // y/x
					Quaterniond temp_q;
					temp_q = AngleAxisd(0, Vector3d::UnitX())					//roll, axis_x
							* AngleAxisd(0, Vector3d::UnitY())					//pitch, axis_y
							* AngleAxisd(ang, Vector3d::UnitZ());				//yaw, axis_z
					Matrix3d rot_temp = temp_q.normalized().toRotationMatrix();	//get rotation of vector
										

					for(int m = 1; m < ls.size()-1; m++)	//remove first and last and 2nd last also
					{
						Vector3d ls_temp( ls[m], 0, 0 );
						Vector3d loci_trans;
						loci_trans = index.at(0) + rot_temp * ls_temp; // get x, y, z=0

						Affine3d temp_t(Translation3d(Vector3d(loci_trans(0), loci_trans(1), loci_trans(2))));
						Matrix4d temp_r;
						temp_r.setIdentity();
						temp_r.block<3, 3>(0, 0) = rot_temp;

						Affine3d loci;
						loci = (temp_t * temp_r).matrix();
						grasp_loci.push_back(loci);
					}
				}// check length > GRAB_ALLOWED_LENGTH
			}// check if index size correct
			else
			{
				cout << "index not size 2" << endl;
			}
			
		}// end if triangle
		else
		{
			cout << "triangle/odd segment reject" << endl;
		}
		cout << "-----------------------------------------------" << endl;
	}// end for each seg

	return 1;
}

//--------------------------------------------------------------------------------------//

int Create_Tool::write2file()
{
	ofstream myfile;

	std::string out_string;
    std::stringstream ss;
    // sftp://olivia@192.168.1.103/home/olivia/Documents
    ss << "/home/olivia/Documents/tool_file.txt";
    out_string = ss.str();

    myfile.open(out_string.c_str(), ios::out);

    if(!myfile.is_open())
    {
        cout<< "CREATE_TOOL: FAILED TO OPEN FILE @: " << out_string << endl;
        return -1;
    }

	myfile << "vertices:  \n\n";
	for(int i=0; i < tool_vertices_3d.size(); i++)
	{
		myfile << "vertice[" << i << "]:\n";
		myfile << tool_vertices_3d.at(i).transpose() << "\n";
	}myfile << "\n";

	myfile << "tool_segment: \n\n";
	for(int i=0; i<N_seg; i++)
	{
		myfile << "segment[" << i << "]: \n";
		for(int j = 0; j < tool_seg_3d.at(i).size(); j++)
		{
			myfile << "point[" << j << "]: \n";
			myfile << tool_seg_3d.at(i).at(j).transpose() << "\n";
		}myfile << "\n";
	}myfile << "\n";

	myfile << "grasp_location:  \n\n";
	for(int j=0; j < grasp_loci.size(); j++)
	{
		myfile << "grasp[" << j << "]:\n";
		myfile << grasp_loci.at(j).matrix() << "\n\n";
	}

	myfile << "done!\n";
    myfile.close();

    cout << "DONE! records in Documents as txt" << endl;
    return 0;
}