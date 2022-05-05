#include <tool_expt/manipulate.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace Eigen;


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::display_poses_matrix2()
{    
    cout << "---------------------------------------------" << endl;
 
    tf::poseMsgToEigen(object_in_base.pose, obj_TF); //4x4
    cout << "obj_TF: \n"
         << obj_TF.matrix() << "\n\n"
         << endl;

    AddMarker(obj_TF, "object");

    tf::poseMsgToEigen(target_in_base.pose, tar_TF); //4x4
    cout << "tar_TF: \n"
         << tar_TF.matrix() << "\n\n"
         << endl;

    AddMarker(tar_TF, "target");

    tf::poseMsgToEigen(obstacle_in_base.pose, obst_TF); //4x4
    cout << "obst_TF: \n"
         << obst_TF.matrix() << "\n\n"
         << endl;

    AddMarker(obst_TF, "obstacle");

    publishMarker();
    return 1;
}

int Manipulate::display_tool_matrix()
{
    cout << "---------------------------------------------" << endl;

    tf::poseMsgToEigen(tool_in_base.pose, tool_TF); //4x4
    cout << "tool_TF: \n"
         << tool_TF.matrix() << "\n\n"
         << endl;
         
    AddMarker(tool_TF, "tool");    
    publishMarker();
    return 1;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::mcts_demo_2()
{

    ROS_INFO_STREAM("testing mcts vision with obstacle");

    cout << "\nhoming" << endl;

    Home();
    sleep(sleep_time);

    addTableCollision();
    addPuckCollision();
    addObstacleCollision();

    cout << "\nMove Waist" << endl;
    MoveWaistFromCurrent(WAIST_TOOL);
    sleep(sleep_time);

    cout << "\nneck pan" << endl;

    ROS_INFO_STREAM("switch tf to 2...");
    ActionSwitchTF(2);

    //spin for vision data
    cout << "waiting for /vision/data and joint_state to be published" << endl;
    detected = false;
    ros::Rate r(100); // in hz
    while( ros::ok() && ( !detected || joint_values.size() <= 0 ) )
    {
        ros::spinOnce(); //get vision_data from subscriber
        r.sleep();
    }
    
    cout << "objhight2table " << endl;
    for(int i = 0; i < v_data.objhight2table.size(); i++)
        ROS_INFO_STREAM("objhight2table[" << i << "]:\n" << v_data.objhight2table.at(i));

    cout << "pro2tableTFs " << endl;
    for(int i = 0; i < v_data.pro2tableTFs.size(); i++)
        ROS_INFO_STREAM("pro2tableTFs[" << i << "]:\n" << v_data.pro2tableTFs.at(i));

    cout << "hullClustersOnAxis " << endl;
    for(int i = 0; i < v_data.hullClustersOnAxis.size(); i++)
        ROS_INFO_STREAM("hullClustersOnAxis[" << i << "]:\n" << v_data.hullClustersOnAxis.at(i));

    for(int i = 0; i <  v_data.toolConvexHulls.size(); i++)
    {
        vector< PointCloud<PointXYZ> > objsConvexHull_temp =  v_data.toolConvexHulls.at(i);
        for(int j =0; j < objsConvexHull_temp.size(); j++)
        {
            PointCloud<PointXYZ> this_cloud = objsConvexHull_temp.at(j);
            ROS_INFO_STREAM("toolConvexHulls[" << i << "][" << j << "]:\n");
            for(int k =0; k < this_cloud.size(); k++)
            {
                PointXYZ this_point = this_cloud.at(k);
                cout << "\tpt" << k << "=[" << this_point.x << "," << this_point.y << "," << this_point.z << "]" << endl;
            }
        }
    }

    cout << "===============================================" << endl;

    tf::Quaternion cam_quat;
    tf::Vector3 cam_vec;
    getCurrentTransform("camera_rgb_optical_frame", cam_quat, cam_vec);
    
    Quaterniond temp_q(cam_quat[3], cam_quat[0], cam_quat[1], cam_quat[2]); //eigen quaternion w is first, tf quaternion w is last 
    Matrix3d temp_z = temp_q.normalized().toRotationMatrix();
    
    Matrix4d TFworld2axis;
    TFworld2axis.setIdentity();
    TFworld2axis.block<3, 3>(0, 0) = temp_z;

    TFworld2axis(0,3) = cam_vec[0];
    TFworld2axis(1,3) = cam_vec[1];
    TFworld2axis(2,3) = cam_vec[2];

    // test transformation
    Matrix4d TFaxis2Obj = v_data.pro2tableTFs.at(0);
    cout << "TF: \n" << TFaxis2Obj << endl;

    Matrix4d TFworld2obj = TFworld2axis * TFaxis2Obj;
    Affine3d obj_pose;
    obj_pose.matrix() = TFworld2obj;
    Vector3d obj_trans = obj_pose.translation();
    Quaterniond obj_quat(obj_pose.rotation());

    vtool_in_base.pose.position.x = obj_trans(0);
    vtool_in_base.pose.position.y = obj_trans(1);
    vtool_in_base.pose.position.z = obj_trans(2);

    vtool_in_base.pose.orientation.x = obj_quat.x();
    vtool_in_base.pose.orientation.y = obj_quat.y();
    vtool_in_base.pose.orientation.z = obj_quat.z();
    vtool_in_base.pose.orientation.w = obj_quat.w();

    tool_in_base.pose.position = vtool_in_base.pose.position;
    tool_in_base.pose.orientation = vtool_in_base.pose.orientation;

    ResetPuck();
    ResetTool();
    ResetObstacle();

    Affine3d vtool_TF;
    tf::poseMsgToEigen(vtool_in_base.pose, vtool_TF); //4x4
    cout << "vtool_TF: \n"
         << vtool_TF.matrix() << "\n\n"
         << endl;

    addVisualToolCollision(toolMeshes.at(0));

    ROS_INFO_STREAM("switch tf to 1...");
    ActionSwitchTF(1);

    //----------------------------------------------------------------------------//
    //----------------------------------------------------------------------------//
    
    clearMarker();
    display_poses_matrix2();

    cout << "joint_values: [ ";
    for (int i = 0; i < joint_values.size(); i++)
        cout << joint_values[i] << " ";
    cout << "] " << endl;

    joint_values[0] = WAIST_INIT / 180 * 3.14159265;


   //moveit_ik_solver
    m_moveit_ik.init();
    m_moveit_ik.init_home(joint_values);
    m_moveit_ik.getJointLimits();
    sleep(1.0);

    cout << "max joint limits: [ ";
    for (int i = 0; i < m_moveit_ik.max_limit.size(); i++)
    {
        cout << m_moveit_ik.max_limit.at(i) << " ";
    }
    cout << "] " << endl;

    cout << "min joint limits: [ ";
    for (int i = 0; i < m_moveit_ik.min_limit.size(); i++)
    {
        cout << m_moveit_ik.min_limit.at(i) << " ";
    }
    cout << "] " << endl;

    vector< double > maxl;
    maxl = m_moveit_ik.max_limit;
    vector< double > minl;
    minl = m_moveit_ik.min_limit;

    int size_no = maxl.size();
     KDL::JntArray maxJnt, minJnt;
     maxJnt.resize(size_no);
     minJnt.resize(size_no);
     for(int i=0; i < size_no; i++)
     {
         maxJnt(i) = maxl[i];
         minJnt(i) = minl[i];
     }

    //KDL_ik_solver
    m_kdl_ik.init();
    m_kdl_ik.init_home(joint_values, maxJnt, minJnt);
    sleep(1.0);

    ROS_INFO_STREAM("\nSTART TOOL Planning");
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore(); 

    //create_tool
    m_create_tool.init(); 
    sleep(sleep_time);

    //monte carlo tree search
    m_mct_search2.init(&m_create_tool, &m_moveit_ik, &m_kdl_ik);
    m_mct_search2.init_params(N_GRASP, N_ATK_ANGLE, N_PLAYS);
    sleep(sleep_time);

    //main experiment
    m_tool_expt2.init(&m_create_tool, &m_mct_search2, object_in_base.pose, target_in_base.pose/*, tool_in_base.pose*/);
    m_tool_expt2.init_params(PUCK_RADIUS*2.0, OBST_RADIUS*2.0, N_VIA_CANDIDATES, obstacle_in_base.pose, MAX_GJK_ITER);
    m_tool_expt2.init_vision( v_list.at(0) ); 
    sleep(sleep_time);

    double tool_width = v_list.at(0).objhight2table;    //use different index for multiple tool

    //execute
    m_tool_expt2.main();

    //results
    m_result = m_tool_expt2.getResult();
    m_result_grab = m_tool_expt2.getGrab();
    m_result_atk = m_tool_expt2.getAtkAngle();
    max_score = m_tool_expt2.getScore();
    m_result_via_pt = m_tool_expt2.getViaPt();

    cout << "grab loci: \n" << m_result_grab.matrix() << endl;
    cout << "atk angle: " << m_result_atk << endl;

    cout << "via pt: " << m_result_via_pt.transpose() << endl;

    cout << "done planning~!" << endl;
    cout << "-------------------------------------------------------" << endl;

    if(max_score <= 0)
    {
        ROS_ERROR("\nMCTS FAILED TO FIND SOLUTION");
        cout << "quitting..." << endl;
        return -1;
    }    

    Affine3d via_pt_af = create_affine( 0, 0, 0, m_result_via_pt);
    AddMarker(via_pt_af, "result_via_pt");
    publishMarker();
    
    //----------------------------------------------------------------------------//
    //----------------------------------------------------------------------------//

    ROS_INFO_STREAM("\nSTART TOOL EXPERIMENT");
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore(); 

    double rad_init = WAIST_INIT / 180 * 3.14159265;
    Affine3d int_waist = create_affine(0,0,rad_init, 0,0,0);

    Affine3d tf_hand_atk = create_affine( 0, 0, m_result_atk,  0,0,0);
    Affine3d h_p = create_affine( M_PI/2.0, M_PI/2.0, 0, 0.1782, -0.4509, 1.0168 );
    h_p = h_p * tf_hand_atk;    
    home_pose = int_waist * h_p;
    cout << "home_pose pos: \n" << home_pose.matrix() << endl;

    double rad_waist = WAIST_TOOL / 180 * 3.14159265;
    Affine3d tool_waist = create_affine(0,0,rad_waist, 0,0,0);
    home_Tool = tool_waist * h_p;
    cout << "home_Tool pos: \n" << home_Tool.matrix() << endl;

    cout << "open fingers right" << endl;
    OpenFingers("right", 0);
    sleep(sleep_time);

    Affine3d grab_pose, return_pose, grab_pose0, grab_pose1, tf_grab_hand, tf_grab_hand1, tf_return_hand;
    
    //note in the tool frame
    double grab_dist = 0.04 + 0.01 + tool_width;
    tf_grab_hand = create_affine( M_PI, 0, -M_PI/2.0, 0.0, 0.0, grab_dist);  
    grab_pose = vtool_TF * m_result_grab * tf_grab_hand * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation
    
    grab_pose0 = grab_pose;
    Vector3d temp_gg = grab_pose0.translation();
    Vector3d temp_ht = home_Tool.translation();
    temp_gg(0) = (temp_ht(0) + temp_gg(0)) * 1.0/2.0;
    temp_gg(1) = (temp_ht(1) + temp_gg(1)) * 1.0/2.0; 
    temp_gg(2) = (temp_ht(2) + temp_gg(2)) * 1.0/2.0;
    grab_pose0.translation() = temp_gg;
    
    double grab_dist1 = 0.04 + 0.10 + tool_width;
    tf_grab_hand1 = create_affine( M_PI, 0, -M_PI/2.0, 0.0, 0.0, grab_dist1);  //0.04 + 0.02 + tool_protrusion
    grab_pose1 = vtool_TF * m_result_grab * tf_grab_hand1 * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation
    
    double return_dist = 0.04 + tool_width; //nearer when returning tool
    tf_return_hand = create_affine( M_PI, 0, -M_PI/2.0, 0.0, 0.0, return_dist);  //0.04 + 0.02 + tool_protrusion
    return_pose = vtool_TF * m_result_grab * tf_return_hand * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation
    
    grabposes.push_back(grab_pose1);
    grabposes.push_back(grab_pose);

    returnposes.push_back(grab_pose0);
    returnposes.push_back(grab_pose1);
    returnposes.push_back(return_pose);

    AddMarker(grab_pose0, "grab_pose_0");
    AddMarker(grab_pose1, "grab_pose_1");
    AddMarker(return_pose, "grab_pose");
    publishMarker();
    sleep(sleep_time);

    cout << "go to grab tool" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    sleep(1);
    if( ActionGoToPos(grabposes, false, /*"cart_single"*/"task_single", true) < 1 )
    {
        ROS_ERROR_STREAM("Failed");
        cout << "close fingers right" << endl;
        CloseFingers("right", 0, 0);
        sleep(sleep_time);

        finish_v(false, false, true);
        return -1;
    }
    sleep(sleep_time);

    ROS_WARN_STREAM("check if position correct, if not ctrl+c here!"); //cihan+jennifer pkg
    cout << "attach tool planner" << endl; //cihan+jennifer pkg
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    ActionAttachObj("right", "vtool", tool_collision);

    cout << "close fingers right" << endl;
    MoveFingers("right", 0.60, 0);
    sleep(sleep_time);
    
    home_tool_poses.push_back(grab_pose1);
    home_tool_poses.push_back(home_Tool);

    cout << "homing" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    if( ActionGoToPos( home_tool_poses, false, "task_single", true ) < 1 )
    {
        ROS_ERROR_STREAM("failed to go to home");
        sleep(sleep_time);

        cout << "homing" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        if( Home(WAIST_TOOL) < 0)
        {
            ROS_ERROR_STREAM("failed to go to home");
            finish_v(true, false, true);
            return -1;
        }
    }
    sleep(sleep_time);


    //----------------------------------------------------------------//

    cout << "\nMove Waist" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    MoveWaistFromCurrent(WAIST_INIT);
    sleep(sleep_time);

    //----------------------------------------------------------------//

    for(int i =0; i < m_result.size(); i++)
        AddMarker(m_result.at(i), "result_pose_"+to_string(i));
    publishMarker();

    vector< Affine3d >prepos;
    Affine3d first = m_result.at(0);
    Vector3d temp_tran = first.translation();
    Vector3d temp_hh = home_pose.translation();
    temp_tran(0) = temp_hh(0) + (temp_tran(0) - temp_hh(0)) * 1.0/3.0;
    temp_tran(1) = temp_hh(1) + (temp_tran(1) - temp_hh(1)) * 1.0/3.0; 
    temp_tran(2) = temp_hh(2) + (temp_tran(2) - temp_hh(2)) * 1.0/3.0;
    first.translation() = temp_tran;
    prepos.push_back(first);
    prepos.push_back(m_result.at(0)); //sec
    cout << "going to pre-positioning..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    cout << "pre-positioning:" << endl;
    if (ActionGoToPos(prepos, false, "task_single", true ) >= 0)
    {
        ROS_INFO_STREAM("pre-positioning okay~!\n");
    }
    else
    {
        ROS_ERROR_STREAM("failed to go to pre-position\n");
        finish_v(true, false);
        return -1;
    }
    sleep(sleep_time);

    cout << "\nremove puck collision..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    if (ActionRmObj("puck") < 0)
    {
        ROS_ERROR_STREAM("Cant remove puck collision -> quitting\n");
        cout << "homing" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
    }
    else
    {
        ROS_INFO_STREAM("puck removed okay~!\n");
    }
    sleep(sleep_time);

    bool traj_okay = false;
    vector< Affine3d > result_go;
    
    result_go.push_back(m_result.at(1));
    cout << "going to position..." << endl;
    if (ActionGoToPos(result_go, false, "cart_single", true ) >= 0)
    {
        traj_okay = true;
        ROS_INFO_STREAM("movement okay~!\n");
    }
    else
    {
        traj_okay = false;
        ROS_ERROR_STREAM("failed to go to position");
    }
    sleep(sleep_time);

    object_in_base.pose.position.x = m_result_via_pt(0);
    object_in_base.pose.position.y = m_result_via_pt(1);
    object_in_base.pose.position.z = m_result_via_pt(2);
    addPuckCollision();
    sleep(5.0);    

    result_go.clear();
    result_go.push_back(m_result.at(2));
    cout << "going to position..." << endl;
    cin.ignore();
    if (ActionGoToPos(result_go, false, "task_single", true ) >= 0)
    {
        traj_okay = true;
        ROS_INFO_STREAM("movement okay~!\n");
    }
    else
    {
        traj_okay = false;
        ROS_ERROR_STREAM("failed to go to position");
    }
    sleep(sleep_time);

    cout << "\nremove puck collision..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    if (ActionRmObj("puck") < 0)
    {
        ROS_ERROR_STREAM("Cant remove puck collision -> quitting\n");
        cout << "homing" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
    }
    else
    {
        ROS_INFO_STREAM("puck removed okay~!\n");
    }
    sleep(sleep_time);

    result_go.clear();
    result_go.push_back(m_result.at(3));
    cout << "going to position..." << endl;
    sleep(sleep_time);
    if (ActionGoToPos(result_go, false, "cart_single", true ) >= 0)
    {
        traj_okay = true;
        ROS_INFO_STREAM("movement okay~!\n");
    }
    else
    {
        ROS_ERROR_STREAM("failed to go to position");
    }
    sleep(sleep_time);

    cout << "going to return position..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    Affine3d last;
    if(traj_okay)
        last = m_result.at(m_result.size() - 1);
    else
        last = m_result.at(0);
    temp_tran = last.translation();
    temp_tran(2) += 0.10; 
    last.translation() = temp_tran;
    cout << "return position:" << endl;
    if (ActionGoToPos(last) >= 0)
    {
        ROS_INFO_STREAM("position okay~!\n");
    }
    else
    {
        ROS_ERROR_STREAM("failed to go to position");
        finish_v(true, false);
        return -1;
    }
    sleep(sleep_time);

    finish_v(true);

    ROS_INFO_STREAM("FINISHED");
    return 1;
}
