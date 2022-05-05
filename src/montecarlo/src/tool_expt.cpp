#include <tool_expt/tool_expt.h>

using namespace std;
using namespace Eigen;

Tool_Expt::Tool_Expt()
{
    m_create_tool = NULL;
    paramFlag = false;
    perceptFlag = false;
}

void Tool_Expt::init(Create_Tool *create_tool, MCT_Search *mct_search, geometry_msgs::Pose obj_pose, geometry_msgs::Pose tar_pose/*, geometry_msgs::Pose tool_pose*/)
{
    m_create_tool = create_tool;
    m_mct_search = mct_search;

    m_obj_pose = obj_pose;
    m_tar_pose = tar_pose;
    // m_tool_pose = tool_pose;
}

void Tool_Expt::init_params(double obj_dia, int num_via_pts, int num_intra_pts, int max_iterations)
{
    perceptFlag = false;

    m_obj_dia = obj_dia;
    
    N_via_pts = num_via_pts;
    N_intrapath_pts = num_intra_pts;
    N_path_pts = (N_via_pts+1) * (N_intrapath_pts+2);

    max_iter = max_iterations;
    
    result.clear();
    Matrix4d temp_r;
    temp_r.setIdentity();
    result_grab = temp_r.matrix();

    paramFlag = true;
    //display
	cout << "param initialised: " << endl;
    cout << "N_via_pts=" << N_via_pts 
         << ", N_intra_pts=" << N_intrapath_pts 
         << ", N_path_pts=" << N_path_pts
         << endl;
}

void Tool_Expt::init_vision(vision_each v_data, bool percept)
{
    perceptFlag = percept;
    vdata = v_data;
    cout << "vision data acquired" << endl;
}

Affine3d Tool_Expt::create_affine(double roll, double pitch, double yaw, double x, double y, double z) 
{
    Affine3d temp_t(  Translation3d( Vector3d(x,y,z) )  );

    Quaterniond temp_q;
	temp_q =  AngleAxisd(roll, Vector3d::UnitX())					//roll, axis_x
			* AngleAxisd(pitch, Vector3d::UnitY())					//pitch, axis_y
			* AngleAxisd(yaw, Vector3d::UnitZ());			        //yaw, axis_z
    Matrix3d temp_z = temp_q.normalized().toRotationMatrix();
    Matrix4d temp_r;
    temp_r.setIdentity();
    temp_r.block<3,3>(0,0) = temp_z;

    Affine3d temp;
    temp = (temp_t * temp_r).matrix();
    return temp;
}

Affine3d Tool_Expt::create_affine(double roll, double pitch, double yaw, Vector3d xyz) 
{
    Affine3d temp_t(  Translation3d( Vector3d(xyz(0),xyz(1),xyz(2)) )  );

    Quaterniond temp_q;
	temp_q =  AngleAxisd(roll, Vector3d::UnitX())					//roll, axis_x
			* AngleAxisd(pitch, Vector3d::UnitY())					//pitch, axis_y
			* AngleAxisd(yaw, Vector3d::UnitZ());			        //yaw, axis_z
    Matrix3d temp_z = temp_q.normalized().toRotationMatrix();
    Matrix4d temp_r;
    temp_r.setIdentity();
    temp_r.block<3,3>(0,0) = temp_z;

    Affine3d temp;
    temp = (temp_t * temp_r).matrix();
    return temp;
}

void Tool_Expt::create_tool()
{
    cout << "creating tool bounding box..." << endl;
    m_create_tool->create();
    m_create_tool->displayDetails();

    vector< Vector3d > all_vertices;
    all_vertices = m_create_tool->getVertices();
 
    subtool1.vertices.push_back(all_vertices.at(0));
    subtool1.vertices.push_back(all_vertices.at(1));
    subtool1.vertices.push_back(all_vertices.at(4));
    subtool1.vertices.push_back(all_vertices.at(5));
   

    subtool2.vertices.push_back(all_vertices.at(1));
    subtool2.vertices.push_back(all_vertices.at(2));
    subtool2.vertices.push_back(all_vertices.at(3));
    subtool2.vertices.push_back(all_vertices.at(4));
        
    cout << "tool creation done \n" << endl;
}

void Tool_Expt::create_object()
{
    cout << "creating object bounding box..." << endl;
    Vector3d shift( 0.0, 0.0, 0.0);
    double obj_dia_shr = m_obj_dia*0.99;
    object.vertices.push_back(0.5*Vector3d( -obj_dia_shr, -obj_dia_shr,  0.01 ) + shift);
    object.vertices.push_back(0.5*Vector3d(  obj_dia_shr, -obj_dia_shr,  0.01 ) + shift);
    object.vertices.push_back(0.5*Vector3d(  obj_dia_shr,  obj_dia_shr,  0.01 ) + shift);
    object.vertices.push_back(0.5*Vector3d( -obj_dia_shr,  obj_dia_shr,  0.01 ) + shift);
    object.vertices.push_back(0.5*Vector3d( -obj_dia_shr, -obj_dia_shr, -0.01 ) + shift);
    object.vertices.push_back(0.5*Vector3d(  obj_dia_shr, -obj_dia_shr, -0.01 ) + shift);
    object.vertices.push_back(0.5*Vector3d(  obj_dia_shr,  obj_dia_shr, -0.01 ) + shift);
    object.vertices.push_back(0.5*Vector3d( -obj_dia_shr,  obj_dia_shr, -0.01 ) + shift);

    cout << "object creation done \n" << endl;
}

void Tool_Expt::create_percept_tool()
{
    cout << "creating percept tool..." << endl;

    m_create_tool->create_perceptTool(vdata);
    m_create_tool->displayDetails();
    cout << "create percept tool okay!" << endl;

    //create tool shape for gjk
    vector< vector< Vector3d > > all_vertices;
    all_vertices = m_create_tool->get_seg_vertices();
    tool_pieces.clear();
    for(int i = 0; i < all_vertices.size(); i++)
    {
        Shape subtool;
        subtool.vertices = all_vertices.at(i);
        tool_pieces.push_back(subtool);
    }    
    cout << "perception_tool creation done \n" << endl;
}

bool Tool_Expt::collision_check()
{
    bool collide_flag = false;
    
    if( !perceptFlag)
    {
        collide_flag = m_gjk.collision_check(subtool1, object, max_iter) ||
                        m_gjk.collision_check(subtool2, object, max_iter) ;
    }
    else
    {
        cout << "percept seg collision testing" << endl;
        for(int i = 0; i < tool_pieces.size(); i++)
        {
            if(  m_gjk.collision_check(tool_pieces.at(i), object, max_iter) )
            {
                collide_flag = true;
                break;
            }
        }
    }
    
    if(collide_flag)
    {
        //ROS_ERROR_STREAM("\nCollision DETECTED!");
        cout << "collision = 1" << endl;
    }
    else
    {
        //ROS_WARN_STREAM("\nNO collision detected");
        cout << "collision = 0" << endl; 
    }
    return collide_flag;
}

void Tool_Expt::create_task()
{
    //get all transform between world and task and obj and tool
    //transform from matlab to real world
    T_world_obj.clear();

    //obj
    tf::poseMsgToEigen(m_obj_pose, obj_affine);		    //4x4
	obj_pos = obj_affine.translation(); 	            //3x1 vector
    cout << "obj_affine: \n" << obj_affine.matrix() << endl;

    //target
    tf::poseMsgToEigen(m_tar_pose, tar_affine);		    //4x4
	tar_pos = tar_affine.translation(); 	            //3x1 vector
    cout << "tar_affine: \n" << tar_affine.matrix() << endl;

    //T_obj_task (direction)
    obj_task = tar_pos - obj_pos;   //task_dir
    angle_obj_task = atan2( obj_task(1), obj_task(0) ); //y/x
    T_obj_task = create_affine(0, 0, angle_obj_task,0,0,0);  
    cout << "T_obj_task: \n" << T_obj_task.matrix() << endl;

    //------------------------------//     
    cout << "task creation done \n" << endl;
}

void Tool_Expt::create_path()
{
    
    //------------------------------//
    //currently ignore orentation of object
    T_world_obj.push_back( create_affine(0,0,0,obj_pos(0),obj_pos(1),obj_pos(2))); //first obj position
    path_pts.push_back(obj_pos);
    
    //for via pts inbtwn
    for(int i=0; i < N_via_pts; i++)
    {
        Vector3d temp_p;    //spawn via_pts between start and end
        temp_p = obj_pos + obj_task * (i+1)/(N_via_pts+1);
        T_world_obj.push_back( create_affine(0,0,0,temp_p(0),temp_p(1),temp_p(2))); //pushback 2 of same
        T_world_obj.push_back( create_affine(0,0,0,temp_p(0),temp_p(1),temp_p(2))); //sec one for orientation correction
        path_pts.push_back( temp_p );
    }
    T_world_obj.push_back( create_affine(0,0,0,tar_pos(0),tar_pos(1),tar_pos(2))); //last target position
    path_pts.push_back(tar_pos);

    for(int j=1; j < path_pts.size(); j++)  //minimum 2
    {
        Vector3d temp_v = path_pts.at(j) - path_pts.at(j-1);
        double angle_v = atan2( temp_v(1), temp_v(0) );
        Affine3d temp_a = create_affine(0,0,angle_v, 0,0,0);
        T_all_path_pts.push_back(temp_a);
        T_all_path_pts.push_back(temp_a);
    }

    //------------------------------//     
    cout << "path creation done \n" << endl;
}

void Tool_Expt::functionality()
{
    //old code
    m_func_templ.left   = Vector3d(0,  0.01, 0);
    m_func_templ.right  = Vector3d(0, -0.01, 0);
    m_func_templ.normal = Vector3d(1.0, 0, 0);
    
    m_func_templ.objdir = Vector3d(1.0, 0.0, 0);
    m_func_templ.objpos = Vector3d(0.5*m_obj_dia, 0, 0);

    //T_task_func
    angle_task_func = atan2( m_func_templ.objdir(1), m_func_templ.objdir(0) );
    T_task_func = create_affine(0, 0, angle_task_func, -m_func_templ.objpos(0), -m_func_templ.objpos(1), m_func_templ.objpos(2));
        cout << "T_task_func: \n" << T_task_func.matrix() << endl;

    Affine3d T_task_fun_one = create_affine(0, 0, angle_task_func, -m_func_templ.objpos(0)-0.05, -m_func_templ.objpos(1), m_func_templ.objpos(2));

    T_obj_func.clear();
  
    for(int a = 0; a < T_all_path_pts.size(); a++)
    {
        if(a == 0)  //first point move back abit -> allow more room to manuveur
            T_obj_func.push_back( T_all_path_pts.at(0) * T_task_fun_one );
        else
            T_obj_func.push_back( T_all_path_pts.at(a) * T_task_func );        
    }

    for(int i = 0; i< T_obj_func.size(); i++){
        cout << "T_obj_func[" << i << "]: \n" << T_obj_func.at(i).matrix() << endl; 
    }

    cout << "functionality done \n" << endl;
}

void Tool_Expt::search()
{
    //pass the params first!
    m_mct_search->N_via_pts = N_via_pts;                
    m_mct_search->N_intrapath_pts = N_intrapath_pts;    
    m_mct_search->N_path_pts = N_path_pts;
    m_mct_search->max_iter = max_iter;

    m_mct_search->T_world_obj = T_world_obj;
    m_mct_search->T_obj_func = T_obj_func;

    m_mct_search->subtool1 = subtool1;
    m_mct_search->subtool2 = subtool2;
    m_mct_search->object = object;

    m_mct_search->tool_pieces = tool_pieces;
    m_mct_search->perceptFlag = perceptFlag;

    m_mct_search->T_obj_task = T_obj_task;

    cout << "values used: " << endl;
    cout << "  N_via_pts:=" << N_via_pts;
    cout << " N_intrapath_pts:=" << N_intrapath_pts;
    cout << " N_path_pts:=" << N_path_pts << endl;

    for(int i = 0; i < T_world_obj.size() ; i++)
    {
        cout << "  T_world_obj[" << i << "]:\n" << T_world_obj.at(i).matrix() << endl;
    } cout << endl;

    for(int i = 0; i < T_obj_func.size() ; i++)
    {
        cout << "  T_obj_func[" << i << "]:\n" << T_obj_func.at(i).matrix() << endl;
    } cout << endl;

    m_mct_search->search();
    result = m_mct_search->getResult();
    result_grab = m_mct_search->getGrab();
    result_atk = m_mct_search->getAtkAngle();
    max_score = m_mct_search->getScore();
}

void Tool_Expt::main()
{
    if(m_create_tool == NULL || m_mct_search == NULL || !paramFlag){
        cout << "run init and init_param first" << endl;
        return;
    }
    
    ROS_WARN_STREAM("CREATE TASK");
    cout << "=================================create task==================================" << endl;
    cout << ">>> create_task" << endl;
    create_task();

    ROS_WARN_STREAM("CREATE BOUNDING BOXES");
    cout << "=============================create bounding boxes==============================" << endl;
    cout << ">>> create_object" << endl;
    create_object();
    
    cout << ">>> create_tool" << endl;
    if(!perceptFlag)
        create_tool();
    else
        create_percept_tool();
    
    ROS_WARN_STREAM("CREATE PATH");
    cout << "=================================create path==================================" << endl;
    cout << ">>> pathfinding" << endl;
    create_path();
    
    ROS_WARN_STREAM("CREATE FUNCTIONALITY");
    cout << "=================================functionality==================================" << endl;
    cout << ">>> functionality" << endl;
    functionality();

    ROS_WARN_STREAM("COLLISION CHECK");
    cout << "=================================collision check==================================" << endl;
    cout << ">>> collision" << endl;
    collision_check();

    //stop!
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore(); 

    ROS_WARN_STREAM("START SEARCH");
    cout << "===================================search====================================" << endl;
    cout << ">>> search" << endl;
    search();
    
    cout << "experiment done" << endl;
}