#include <tool_expt/tool_expt2.h>

using namespace std;
using namespace Eigen;

Tool_Expt_2::Tool_Expt_2()
{
    m_create_tool = NULL;
    paramFlag = false;
    perceptFlag = false;
    obstruct_flag = false;
}
 
void Tool_Expt_2::init(Create_Tool *create_tool, MCT_Search2 *mct_search2, geometry_msgs::Pose obj_pose, geometry_msgs::Pose tar_pose/*, geometry_msgs::Pose tool_pose*/)
{
    m_create_tool = create_tool;
    m_mct_search2 = mct_search2;

    m_obj_pose = obj_pose;
    m_tar_pose = tar_pose;
    // m_tool_pose = tool_pose;
}

void Tool_Expt_2::init_params(double obj_dia, double obst_dia, double num_candidates, geometry_msgs::Pose obst_pose, int max_iterations)
{
    perceptFlag = false;

    m_obj_dia = obj_dia;
    m_obst_dia = obst_dia;

    m_obst_pose = obst_pose;

    N_via_pts = 0;
    N_intrapath_pts = 0;
    N_path_pts = (N_via_pts+1) * (N_intrapath_pts+2);   //2 if no obstacles

    max_iter = max_iterations;

    N_via_candidates = num_candidates;
    N_via_candidates /= 2;
    N_via_candidates *= 2;  //make sure even

    result.clear();
    Matrix4d temp_r;
    temp_r.setIdentity();
    result_grab = temp_r.matrix();

    paramFlag = true;
    //display
	cout << "param initialised. obstacle detected. " << endl;
}

void Tool_Expt_2::create_obstacle()
{
    cout << "creating obstacle bounding box..." << endl;
    Vector3d shift( 0.0, 0.0, 0.0);
    double obst_dia_shr = m_obst_dia*0.99;
    obstacle.vertices.push_back(0.5*Vector3d( -obst_dia_shr, -obst_dia_shr,  0.03 ) + shift);
    obstacle.vertices.push_back(0.5*Vector3d(  obst_dia_shr, -obst_dia_shr,  0.03 ) + shift);
    obstacle.vertices.push_back(0.5*Vector3d(  obst_dia_shr,  obst_dia_shr,  0.03 ) + shift);
    obstacle.vertices.push_back(0.5*Vector3d( -obst_dia_shr,  obst_dia_shr,  0.03 ) + shift);
    obstacle.vertices.push_back(0.5*Vector3d( -obst_dia_shr, -obst_dia_shr, -0.03 ) + shift);
    obstacle.vertices.push_back(0.5*Vector3d(  obst_dia_shr, -obst_dia_shr, -0.03 ) + shift);
    obstacle.vertices.push_back(0.5*Vector3d(  obst_dia_shr,  obst_dia_shr, -0.03 ) + shift);
    obstacle.vertices.push_back(0.5*Vector3d( -obst_dia_shr,  obst_dia_shr, -0.03 ) + shift);

    cout << "obstacle creation done \n" << endl;
}

void Tool_Expt_2::obstruction_check()
{
    //obstacle
    tf::poseMsgToEigen(m_obst_pose, obst_affine);		    //4x4
	obst_pos = obst_affine.translation(); 	            //3x1 vector
    cout << "obst_affine: \n" << obst_affine.matrix() << endl;

    cout << "T_obj_task: \n" << T_obj_task.matrix() << endl;

    Affine3d temp_z = create_affine(0, 0, M_PI/2.0, 0, 0, 0);  
    Vector3d trans_dir = (temp_z.rotation() * obj_task);
    double val = (1.0/obj_task.norm()) ;
    trans_dir = val * trans_dir;

    cout << "trans_dir: \n" << trans_dir.transpose() << endl;

    Vector3d sv1 = obj_pos +  0.5 * m_obj_dia * trans_dir;
    Vector3d sv2 = obj_pos -  0.5 * m_obj_dia * trans_dir;
    Vector3d sv3 = tar_pos +  0.5 * m_obj_dia * trans_dir;
    Vector3d sv4 = tar_pos -  0.5 * m_obj_dia * trans_dir;

    //compute SV
    Shape SV_obj;
    Vector3d shift( 0.0, 0.0, 0.0);
    SV_obj.vertices.push_back( sv1 + shift);
    SV_obj.vertices.push_back( sv2 + shift);
    SV_obj.vertices.push_back( sv3 + shift);
    SV_obj.vertices.push_back( sv4 + shift);

    //display vertices
    for(int r = 0; r < SV_obj.vertices.size(); r++)
    {
        cout << "\n SV_obj vertices[" << r << "]: " << endl;
        cout << "\t" << SV_obj.vertices.at(r).transpose() << endl;
    }cout << endl;

    //convert obst bounding box to world frame
    Shape world_obst = obstacle;
    Matrix3d obst_rot = obst_affine.rotation();
    for(int m = 0; m < world_obst.vertices.size(); m++)
        world_obst.vertices.at(m) = ( obst_rot * obstacle.vertices.at(m) ) + obst_pos;

    //display vertices
    for(int r = 0; r < world_obst.vertices.size(); r++)
    {
        cout << "\n world_obst vertices[" << r << "]: " << endl;
        cout << "\t" << world_obst.vertices.at(r).transpose() << endl;
    }cout << endl;

    obstruct_flag = m_gjk.collision_check(SV_obj, world_obst, max_iter);
    
    if(!obstruct_flag)
    {
        cout << "obstruction = 0" << endl;

        N_via_candidates = 1;
        N_via_pts = 0;
    }
    else
    {
        cout << "obstruction = 1" << endl; 
    
        N_via_pts = 1;  

        for(int u=0; u<N_via_candidates/2; u++)
        {
            via_pts.push_back( obst_pos + ( (-VIA_PT_SCALING)*0.5*(m_obj_dia+m_obst_dia) - u*(0.05) )*trans_dir );
            via_pts.push_back( obst_pos +  ( (VIA_PT_SCALING)*0.5*(m_obj_dia+m_obst_dia) + u*(0.05) )*trans_dir );
        }

        for(int j=0; j<N_via_candidates; j++)
            cout << "via_pts[" << j << "]: " << via_pts.at(j).transpose() << endl;
    }

    N_path_pts = (N_via_pts+1)*(N_intrapath_pts+2); // %intermediate points + start and end points

    return;
}

void Tool_Expt_2::functionality()
{
    m_func_templ.left   = Vector3d(0,  0.01, 0);
    m_func_templ.right  = Vector3d(0, -0.01, 0);
    m_func_templ.normal = Vector3d(1.0, 0, 0);
    
    m_func_templ.objdir = Vector3d(1.0, 0.0, 0);
    m_func_templ.objpos = Vector3d(0.5*m_obj_dia, 0, 0);

    //T_task_func
    angle_task_func = atan2( m_func_templ.objdir(1), m_func_templ.objdir(0) );
    T_task_func = create_affine(0, 0, angle_task_func, -m_func_templ.objpos(0), -m_func_templ.objpos(1), m_func_templ.objpos(2));
        cout << "T_task_func: \n" << T_task_func.matrix() << endl;

    T_task_func_wt_gap = create_affine(0, 0, angle_task_func, -m_func_templ.objpos(0) - 0.06, -m_func_templ.objpos(1), m_func_templ.objpos(2));

    T_task_func_wt_gap_1 = create_affine(0, 0, angle_task_func, -m_func_templ.objpos(0) - 0.02, -m_func_templ.objpos(1), m_func_templ.objpos(2));

    cout << "functionality 2 done \n" << endl;
}

void Tool_Expt_2::create_path()
{
    //clear all
    all_T_world_obj_bb.clear();
    all_T_world_obst_bb.clear();
    all_T_world_obj.clear();
    all_move_vec.clear();
    all_T_world_func.clear();
    
    all_T_obj_func.clear();

    if(obstruct_flag)
    {   
        cout << "compute TF -> HAVE OBSTRUCTION" << endl;

        //do for each via pts
        for(int i = 0; i < N_via_candidates; i++)
        {
            Vector3d via_pos = via_pts.at(i);

            // get T_obj_via
            obj_via = via_pos - obj_pos;
            angle_obj_via = atan2( obj_via(1), obj_via(0) );
            T_obj_via = create_affine(0,0,angle_obj_via, 0, 0, 0);
            cout << "T_obj_via: \n" << T_obj_via.matrix() << endl;

            // get T_via_tar
            via_tar = tar_pos - via_pos;
            angle_via_tar = atan2( via_tar(1), via_tar(0) );
            T_via_tar = create_affine(0,0,angle_via_tar, 0, 0, 0);
            cout << "T_via_tar: \n" << T_via_tar.matrix() << endl;
        
            //-----------------------------------------------------------------------//
            vector< Affine3d > this_T_world_obj_bb;            
            this_T_world_obj_bb.push_back( create_affine(0,0,angle_obj_via, obj_pos) );
            this_T_world_obj_bb.push_back( create_affine(0,0,angle_obj_via, via_pos) );
            this_T_world_obj_bb.push_back( create_affine(0,0,angle_via_tar, via_pos) );
            this_T_world_obj_bb.push_back( create_affine(0,0,angle_via_tar, tar_pos) );
            all_T_world_obj_bb.push_back(this_T_world_obj_bb);      //1 Tf Path for each via
            
            //-----------------------------------------------------------------------//
            vector< Affine3d > this_T_world_obst_bb;
            this_T_world_obst_bb.push_back( create_affine(0,0,angle_obj_via, obst_pos) );
            this_T_world_obst_bb.push_back( create_affine(0,0,angle_obj_via, obst_pos) );
            this_T_world_obst_bb.push_back( create_affine(0,0,angle_via_tar, obst_pos) );
            this_T_world_obst_bb.push_back( create_affine(0,0,angle_via_tar, obst_pos) );
            all_T_world_obst_bb.push_back(this_T_world_obst_bb);    //1 Tf Path for each via
            
            //-----------------------------------------------------------------------//
            vector< Affine3d > this_T_world_obj;
            this_T_world_obj.push_back( create_affine(0,0,0, obj_pos) );
            this_T_world_obj.push_back( create_affine(0,0,0, via_pos) );
            this_T_world_obj.push_back( create_affine(0,0,0, via_pos) );
            this_T_world_obj.push_back( create_affine(0,0,0, tar_pos) );
            all_T_world_obj.push_back(this_T_world_obj);            //1 Tf Path for each via
        
            //-----------------------------------------------------------------------//
            vector< Vector3d > this_move_vec;
            this_move_vec.push_back( obj_via );
            this_move_vec.push_back( -obj_via );
            this_move_vec.push_back( via_tar );
            this_move_vec.push_back( -via_tar );
            all_move_vec.push_back(this_move_vec);

            //-----------------------------------------------------------------------//
            vector< Affine3d > this_T_world_func;
            vector< Affine3d > this_T_obj_func;
            
            Affine3d T_obj_func_1 = T_obj_via * T_task_func;
            Affine3d T_obj_func_2 = T_via_tar * T_task_func;            
            
            Affine3d T_obj_func_1_wt_gap = T_obj_via * T_task_func_wt_gap;
            Affine3d T_obj_func_2_wt_gap = T_via_tar * T_task_func_wt_gap_1;            
            
            this_T_obj_func.push_back( T_obj_func_1_wt_gap );
            this_T_obj_func.push_back( T_obj_func_1 );
            this_T_obj_func.push_back( T_obj_func_2_wt_gap );
            this_T_obj_func.push_back( T_obj_func_2 );

            this_T_world_func.push_back( this_T_world_obj.at(0) * T_obj_func_1_wt_gap );
            this_T_world_func.push_back( this_T_world_obj.at(1) * T_obj_func_1 );
            this_T_world_func.push_back( this_T_world_obj.at(2) * T_obj_func_2_wt_gap );
            this_T_world_func.push_back( this_T_world_obj.at(3) * T_obj_func_2 );
            
            all_T_obj_func.push_back( this_T_obj_func );
            all_T_world_func.push_back( this_T_world_func );

            //leave the obj_vertices to be done inside tree creation
        }//end for loop each via candidate
    }
    else    //if no obstruction, just use all_xxx.at(0)
    {
        cout << "compute TF -> NO OBSTRUCTION" << endl;
    
        //zero via points
        //-----------------------------------------------------------------------//
        vector< Affine3d > this_T_world_obj_bb;            
        this_T_world_obj_bb.push_back( create_affine(0,0,angle_obj_task, obj_pos) );
        this_T_world_obj_bb.push_back( create_affine(0,0,angle_obj_task, tar_pos) );
        all_T_world_obj_bb.push_back(this_T_world_obj_bb);      //1 Tf Path for each via
        
        //-----------------------------------------------------------------------//
        vector< Affine3d > this_T_world_obst_bb;
        this_T_world_obst_bb.push_back( create_affine(0,0,angle_obj_task, obst_pos) );
        this_T_world_obst_bb.push_back( create_affine(0,0,angle_obj_task, obst_pos) );
        all_T_world_obst_bb.push_back(this_T_world_obst_bb);    //1 Tf Path for each via
        
        //-----------------------------------------------------------------------//
        vector< Affine3d > this_T_world_obj;
        this_T_world_obj.push_back( create_affine(0,0,0, obj_pos) );
        this_T_world_obj.push_back( create_affine(0,0,0, tar_pos) );
        all_T_world_obj.push_back(this_T_world_obj);            //1 Tf Path for each via
    
        //-----------------------------------------------------------------------//
        vector< Vector3d > this_move_vec;
        this_move_vec.push_back( obj_task );
        this_move_vec.push_back( obj_task );
        all_move_vec.push_back(this_move_vec);

        //-----------------------------------------------------------------------//
        vector< Affine3d > this_T_world_func;
        vector< Affine3d > this_T_obj_func;
        
        Affine3d T_obj_func_1 = T_obj_task * T_task_func;

        Affine3d T_obj_func_1_wt_gap = T_obj_task * T_task_func_wt_gap;

        this_T_obj_func.push_back( T_obj_func_1_wt_gap );
        this_T_obj_func.push_back( T_obj_func_1 );
        all_T_obj_func.push_back( this_T_obj_func );

        this_T_world_func.push_back( this_T_world_obj.at(0) * T_obj_func_1_wt_gap );
        this_T_world_func.push_back( this_T_world_obj.at(1) * T_obj_func_1 );
        all_T_world_func.push_back( this_T_world_func );

        //leave the obj_vertices to be done inside tree creation
    }//end 

    cout << "path creation 2 done \n" << endl;
}

void Tool_Expt_2::search()
{
    //pass the params first!  Samuel was here!            
    m_mct_search2->N_via_pts = N_via_pts;                
    m_mct_search2->N_intrapath_pts = N_intrapath_pts;    
    m_mct_search2->N_path_pts = N_path_pts;
    m_mct_search2->max_iter = max_iter;

    m_mct_search2->N_via_candidates = N_via_candidates;  

    m_mct_search2->T_obj_task = T_obj_task;
   
    m_mct_search2->all_T_world_obj_bb = all_T_world_obj_bb;
    m_mct_search2->all_T_world_obst_bb = all_T_world_obst_bb;
    m_mct_search2->all_T_world_obj = all_T_world_obj;
    m_mct_search2->all_move_vec = all_move_vec;
    m_mct_search2->all_T_world_func = all_T_world_func;

    m_mct_search2->all_T_obj_func = all_T_obj_func;

    m_mct_search2->subtool1 = subtool1;
    m_mct_search2->subtool2 = subtool2;
    m_mct_search2->object = object;
    m_mct_search2->obstacle = obstacle;

    m_mct_search2->tool_pieces = tool_pieces;
    m_mct_search2->perceptFlag = perceptFlag;
    m_mct_search2->obstruct_flag = obstruct_flag;

    cout << "values used: " << endl;
    cout << "  N_via_pts:=" << N_via_pts;
    cout << " N_intrapath_pts:=" << N_intrapath_pts;
    cout << " N_path_pts:=" << N_path_pts << endl;

    //-----------------------------------------------------------------------//
    m_mct_search2->search();

    //results
    cout << ">> passing results" << endl;
    result = m_mct_search2->getResult();
    if(obstruct_flag)
        result_via_pt = via_pts.at( m_mct_search2->getViaPt() );
    result_grab = m_mct_search2->getGrab();
    result_atk = m_mct_search2->getAtkAngle();
    max_score = m_mct_search2->getScore();
}

void Tool_Expt_2::main()
{
    if(m_create_tool == NULL || m_mct_search2 == NULL || !paramFlag){
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

    cout << ">>> create_obstacle" << endl;
    create_obstacle();

    ROS_WARN_STREAM("OBSTRUCTION CHECK");
    cout << "=================================obstruction check==================================" << endl;
    cout << ">>> obstruction" << endl;
    obstruction_check();

    //stop!
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore(); 
    
    ROS_WARN_STREAM("CREATE FUNCTIONALITY");
    cout << "=================================functionality==================================" << endl;
    cout << ">>> functionality" << endl;
    functionality();

    ROS_WARN_STREAM("CREATE PATH");
    cout << "=================================create path==================================" << endl;
    cout << ">>> pathfinding" << endl;
    create_path();
    
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