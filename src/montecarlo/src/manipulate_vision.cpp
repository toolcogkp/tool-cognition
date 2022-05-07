#include <tool_expt/manipulate.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
// #include <moveit/move_group_interface/move_group_interface.h>

using namespace std;
using namespace Eigen;
// using namespace cv;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::display_poses_matrix()
{    
    cout << "---------------------------------------------" << endl;
    tf::poseMsgToEigen(tool_in_base.pose, tool_TF); 
    cout << "tool_TF: \n"
         << tool_TF.matrix() << "\n\n"
         << endl;
         
    AddMarker(tool_TF, "tool");

    tf::poseMsgToEigen(object_in_base.pose, obj_TF); 
    cout << "obj_TF: \n"
         << obj_TF.matrix() << "\n\n"
         << endl;

    AddMarker(obj_TF, "object");

    tf::poseMsgToEigen(target_in_base.pose, tar_TF); 
    cout << "tar_TF: \n"
         << tar_TF.matrix() << "\n\n"
         << endl;

    AddMarker(tar_TF, "target");

    publishMarker();
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
void Manipulate::vision_data_callBack(const findtoolontalbe::vision_msgs::Ptr &msg)
{
    ROS_INFO_STREAM("vision callBack received!");

    if(!detected)
    {
        //process msg
        tool_num = msg->tool_num;
        toolMeshes = msg->objMeshes;

        std_msgs::Float32MultiArray objhight2table_msg = msg->objhight2table;
        v_data.objhight2table = objhight2table_msg.data;

        for(int i=0; i < msg->hullClustersOnAxis.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ> this_cluster;
            pcl::fromROSMsg( msg->hullClustersOnAxis.at(i), this_cluster);
            v_data.hullClustersOnAxis.push_back(this_cluster);
        }

        for(int i=0; i < msg->pro2tableTFs.size(); i++)
        {
            Affine3d this_pose;
            tf::poseMsgToEigen( msg->pro2tableTFs.at(i), this_pose);
            v_data.pro2tableTFs.push_back(this_pose.matrix());
        }

        for(int i=0; i < msg->objsConvexHulls_vec.size(); i++)  //tool size
        {
            std::vector<pcl::PointCloud<pcl::PointXYZ> > toolConvex;
            findtoolontalbe::pcl_vector this_vec = msg->objsConvexHulls_vec.at(i);

            for(int j=0; j < this_vec.objsConvexHulls.size(); j++)  //segments
            {
                pcl::PointCloud<pcl::PointXYZ> this_cluster;
                pcl::fromROSMsg( this_vec.objsConvexHulls.at(j), this_cluster);
                toolConvex.push_back(this_cluster);
            }
            v_data.toolConvexHulls.push_back(toolConvex);
        }

        //number of tool
        for(int i = 0; i < tool_num; i++)
        {
            vision_each v_each;
            v_each.objhight2table = v_data.objhight2table.at(i);
            v_each.hullClustersOnAxis = v_data.hullClustersOnAxis.at(i);
            v_each.pro2tableTFs = v_data.pro2tableTFs.at(i);
            v_each.toolConvexHulls = v_data.toolConvexHulls.at(i);
            v_list.push_back(v_each);
        }

        detected = true;
    }
    else
    {
        ROS_WARN_STREAM("dropping vision data");
    }
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::vision_cb_test()
{
    // //testing code for request and replan

    ROS_INFO_STREAM("testing li jun vision");

    cout << "\nhoming" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    Home();
    sleep(sleep_time);

    cout << "\nMove Waist" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    MoveWaistFromCurrent(WAIST_TOOL);
    sleep(sleep_time);

    //spin for vision data
    detected = false;
    ros::Rate r(100); // in hz
    while( ros::ok() && ( !detected || joint_values.size() <= 0 ) )
    {
        cout << "waiting for /vision/data and joint_state to be published" << endl;
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
    ResetPuck();
    ResetTool();

    display_poses_matrix();
    Affine3d vtool_TF;
    tf::poseMsgToEigen(vtool_in_base.pose, vtool_TF); 
    cout << "vtool_TF: \n"
         << vtool_TF.matrix() << "\n\n"
         << endl;

    //----------------------------------------------------------------------------//
    //----------------------------------------------------------------------------//

    cout << "joint_values: [ ";
    for (int i = 0; i < joint_values.size(); i++)
        cout << joint_values[i] << " ";
    cout << "] " << endl;

    joint_values[0] = WAIST_INIT / 180 * 3.14159265;

   //moveit_ik_solver
    m_moveit_ik.init();
    //sleep(1.0);
    m_moveit_ik.init_home(joint_values);
    //sleep(1.0);
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

    addVisualToolCollision(toolMeshes.at(0));
    addTableCollision();
    addPuckCollision();

    ROS_INFO_STREAM("\nSTART TOOL Planning");
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore(); 

    //create_tool
    m_create_tool.init();
    sleep(sleep_time);

    //monte carlo tree search
    m_mct_search.init(&m_create_tool, &m_moveit_ik);
    m_mct_search.init_params(3, 1, 1500);     // N_grasp, N_atk_angle, Nplays
    sleep(sleep_time);

    //main experiment
    // Create_Tool, Search, obj pose, tar pose, tool_pose -> use vtool_pose;
    m_tool_expt.init(&m_create_tool, &m_mct_search, object_in_base.pose, target_in_base.pose/*, vtool_in_base.pose*/);
    // obj_diameter, via_pts, intra_pts, max_gjk_iterations, percept;
    m_tool_expt.init_params(PUCK_RADIUS*2.0, 0, 0, 6);
    m_tool_expt.init_vision( v_list.at(0) );  //use vision module for first tool!
    sleep(sleep_time);

    double tool_width = v_list.at(0).objhight2table;    //use different index for multiple tool

    //execute
    m_tool_expt.main();

    //results
    m_result = m_tool_expt.getResult();
    m_result_grab = m_tool_expt.getGrab();
    m_result_atk = m_tool_expt.getAtkAngle();
    max_score = m_tool_expt.getScore();

    cout << "grab loci: \n" << m_result_grab.matrix() << endl;
    cout << "atk angle: " << m_result_atk << endl;

    cout << "done planning~!" << endl;
    cout << "-------------------------------------------------------" << endl;

    if(max_score < 1)
    {
        ROS_ERROR("\nMCTS FAILED TO FIND SOLUTION");
        cout << "quitting..." << endl;
        return -1;
    }    

    //----------------------------------------------------------------------------//
    //----------------------------------------------------------------------------//

    ROS_INFO_STREAM("\nSTART TOOL EXPERIMENT");
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore(); 

    Affine3d tf_hand_atk = create_affine( 0, 0, m_result_atk,  0,0,0);
    home_pose = create_affine( M_PI/2.0, M_PI/2.0, 0, 0.1782, -0.4509, 1.0168 );
    home_pose = home_pose * tf_hand_atk;

    double rad_waist = WAIST_TOOL / 180 * 3.14159265;
    Affine3d tool_waist = create_affine(0,0,rad_waist, 0,0,0);
    home_Tool = tool_waist * home_pose;

    cout << "open fingers right" << endl;;
    OpenFingers("right", 0);
    sleep(sleep_time);

    vector< Affine3d > grabposes;
    Affine3d grab_pose, grab_pose0, tf_grab_hand;
    //note in the tool frame
    double grab_dist = 0.04 + 0.02 + tool_width;
    tf_grab_hand = create_affine( M_PI, 0, -M_PI/2.0, 0.0, 0.0, grab_dist);  //tool_protrusion
    grab_pose = vtool_TF * m_result_grab * tf_grab_hand * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation
    
    grab_pose0 = grab_pose;
    Vector3d temp_gg = grab_pose0.translation();
    Vector3d temp_ht = home_Tool.translation();
    temp_gg(0) = (temp_ht(0) + temp_gg(0)) /2.0;
    temp_gg(1) = (temp_ht(1) + temp_gg(1)) /2.0; 
    temp_gg(2) = 1.00;
    grab_pose0.translation() = temp_gg;
    
    grabposes.push_back(grab_pose0);
    grabposes.push_back(grab_pose);

    cout << "go to grab tool" << endl;

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

    ROS_WARN_STREAM("check if position correct, if not ctrl+c here!"); 
    cout << "attach tool planner" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    ActionAttachObj("right", "vtool", tool_collision);

    cout << "close fingers right" << endl;

    MoveFingers("right", 0.60, 0);
    sleep(sleep_time);

    cout << "homing" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    if( ActionGoToPos( home_Tool, false, "task_single", true ) < 1 )
    {
        ROS_ERROR_STREAM("failed to go to home");
        finish_v(false, false, true);
        return -1;
    }
    sleep(sleep_time);

    //----------------------------------------------------------------//


    cout << "\nMove Waist" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    MoveWaistFromCurrent(WAIST_INIT);
    sleep(sleep_time);

    //----------------------------------------------------------------//

    vector< Affine3d >prepos;
    Affine3d first = m_result.at(0);
    Vector3d temp_tran = first.translation();
    Vector3d temp_hh = home_pose.translation();
    temp_tran(0) = (temp_hh(0) + temp_tran(0)) /2.0;
    temp_tran(1) = (temp_hh(1) + temp_tran(1)) /2.0; 
    temp_tran(2) = 1.10;
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
        ActionGoToPos( home_pose, false, "task_single", true );
        finish_v(true, false);
        return -1;
    }
    else
    {
        ROS_INFO_STREAM("puck removed okay~!\n");
    }
    sleep(sleep_time);

    bool traj_okay = false;
    vector< Affine3d > result_go;
    for(int i =1; i < m_result.size(); i++) 
        result_go.push_back(m_result.at(i));
    cout << "going to position..." << endl;
    if (ActionGoToPos(result_go, false, "task_single", true ) >= 0)
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

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::addVisualToolCollision(shape_msgs::Mesh co_mesh)
{
    vtool_collision.header.frame_id = "world";
    vtool_collision.id = "vtool";

    vtool_collision.mesh_poses.resize(1);
    vtool_collision.mesh_poses[0].position.x =  vtool_in_base.pose.position.x; 
    vtool_collision.mesh_poses[0].position.y =  vtool_in_base.pose.position.y; 
    vtool_collision.mesh_poses[0].position.z =  vtool_in_base.pose.position.z; 

    vtool_collision.mesh_poses[0].orientation.x = vtool_in_base.pose.orientation.x; 
    vtool_collision.mesh_poses[0].orientation.y = vtool_in_base.pose.orientation.y; 
    vtool_collision.mesh_poses[0].orientation.z = vtool_in_base.pose.orientation.z; 
    vtool_collision.mesh_poses[0].orientation.w = vtool_in_base.pose.orientation.w; 

    vtool_collision.meshes.push_back(co_mesh);
    vtool_collision.operation = vtool_collision.ADD;

    if (ActionAddObj("vtool", vtool_collision) < 0)
    {
        ROS_ERROR_STREAM("failed to add tool collision!");
        return -1;
    }
    return 1;
}
