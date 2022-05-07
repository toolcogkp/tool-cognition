#include <tool_expt/manipulate.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace Eigen;



//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvMonteCarloExpt & ) 
{
    WAIST_INIT = -30.0;
    WAIST_TOOL = 60.0;

    cout << "homing! press enter to continue:_" << endl; cin.ignore();
    Home();
    sleep(sleep_time);

    //--- main here ---
    if( findObjectTargetLocation() > 0 )
    {
        cout << "continue? _"; cin.ignore();
        if( findToolTargetLocation() > 0 )
        {
            cout << "continue? _"; cin.ignore();
            if( mct_tool_experiment() > 0)
            {
                cout << "continue? _"; cin.ignore();
                perform_experiment();
            }
            else
            {
                cout << "returning waist to init position: _"; cin.ignore();
                MoveWaistFromCurrent(WAIST_INIT);
                sleep(sleep_time);

                cout << "returning head to init position: _"; cin.ignore();      
                TurnHeadTiltPan(13.3, 0.0);
                sleep(sleep_time);                
            }
        }    
    }

    return transit< ReadyState >();

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

Affine3d Manipulate::create_affine(double roll, double pitch, double yaw, double x, double y, double z)
{
    Affine3d temp_t(Translation3d(Vector3d(x, y, z)));

    Quaterniond temp_q;
    temp_q = AngleAxisd(roll, Vector3d::UnitX())    //roll, axis_x
             * AngleAxisd(pitch, Vector3d::UnitY()) //pitch, axis_y
             * AngleAxisd(yaw, Vector3d::UnitZ());  //yaw, axis_z
    Matrix3d temp_z = temp_q.normalized().toRotationMatrix();
    Matrix4d temp_r;
    temp_r.setIdentity();
    temp_r.block<3, 3>(0, 0) = temp_z;

    Affine3d temp;
    temp = (temp_t * temp_r).matrix();
    return temp;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

Affine3d Manipulate::create_affine(double roll, double pitch, double yaw, Vector3d xyz)
{
    double x = xyz(0);
    double y = xyz(1);
    double z = xyz(2);

    Affine3d temp_t(Translation3d(Vector3d(x, y, z)));

    Quaterniond temp_q;
    temp_q = AngleAxisd(roll, Vector3d::UnitX())    //roll, axis_x
             * AngleAxisd(pitch, Vector3d::UnitY()) //pitch, axis_y
             * AngleAxisd(yaw, Vector3d::UnitZ());  //yaw, axis_z
    Matrix3d temp_z = temp_q.normalized().toRotationMatrix();
    Matrix4d temp_r;
    temp_r.setIdentity();
    temp_r.block<3, 3>(0, 0) = temp_z;

    Affine3d temp;
    temp = (temp_t * temp_r).matrix();
    return temp;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::findObjectTargetLocation()
{
    //get object and target location!

    std_msgs::Int32 req_object_target_msg;
    req_object_target_msg.data = 1;
    
    Speech("Please place the object and the goal...");
    sleep(0.5);
    Speech("And let me know when you are ready...");
    cout << "Place object and target. When ready, press Enter to Continue";
    cin.ignore();
    
    Speech("I am detecting the object... and the goal ...");
    pub_req_object_target.publish(req_object_target_msg);
    cout << "Waiting for object and target to be detected..." << endl;

    double increment = -5.0;
    while(ros::ok() && !object_target_received){
        
        //check rosmaster for topics
        for(int i =0; i < 10; i++){
            usleep(100000);
            ros::spinOnce();
        }

    }
    
    object_target_received = false;

    if(ros::ok()){

        Speech("Found them...");
        cout << "Object and target detected ." << endl;
        
        object_in_base = object_in_base_live;
        target_in_base = target_in_base_live;
        
        //for safety with respect to table top at ~0.74m
        object_in_base.pose.position.z = 0.81;          
        target_in_base.pose.position.z = 0.81;     

        double ex = fabs(object_in_base.pose.position.x - target_in_base.pose.position.x);
        double ey = fabs(object_in_base.pose.position.y - target_in_base.pose.position.y);
        double emin = ex;
        if(ey < emin)
            emin = ey;
        if(object_in_base.pose.position.z>1.0 || target_in_base.pose.position.z<0.72){
            Speech("Sorry");
            Speech("The object... or the goal... is not properly placed...");
            sleep(1.0);
            Speech( "Please align them ... along the X ... or Y ... direction.");
            sleep(1.5);
            cout << "Invalid object goal configuration." << endl;
            post_event(EvMonteCarloExpt());
            return -1;
        }


        
        bool sim_robot = false;
        n.getParam("/sim_robot",sim_robot);
        if(sim_robot)
            ResetPuck();


        float target_tol = 0.05;
        std::cout << "==========================" << std::endl;
        std::cout << "object location: "<< object_in_base.pose.position.x 
                    << ", " << object_in_base.pose.position.y << ", " 
                    << object_in_base.pose.position.z << std::endl;

        std::cout << "target location: "<< target_in_base.pose.position.x 
                    << ", " << target_in_base.pose.position.y << ", " 
                    << target_in_base.pose.position.z << std::endl;

        cout << "==========================================" << endl;
        double y_dif = -(target_in_base.pose.position.y - object_in_base.pose.position.y);
        double x_dif = (target_in_base.pose.position.x - object_in_base.pose.position.x);
        cout << "x dif=" << x_dif << ", y_dif=" << y_dif << endl; 
        cout << "==========================================" << endl;

        if(fabs(y_dif) < target_tol && fabs(x_dif) < target_tol)
        {
            std::cout << "Object already at target!!!" <<std::endl;
            post_event(EvReady());
            return -1;
        }
        else{
            cout << ">>>>> cal angle of obj to target <<<<<" << endl;
            //calculate the angle of object to target
            //south angle is negative
            radian = std::atan2(x_dif,y_dif);     
            angle = round(radian * 180.0 / PI); 
            ROS_WARN("\nangle obj2tar: deg= %f , radian= %f", angle, radian);
            cout << "==========================================" << endl;
                
            cout << "adding moveit collision..." << endl;
            addTableCollision();
            addPuckCollision();
            cout << "Collsion added..." << endl << endl;

            return 1;
        }
    }
    else
        return -1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::findToolTargetLocation()
{
    ROS_INFO_STREAM("testing li jun vision");
    
    Speech("Let's see over here...");
    ROS_WARN("NEXT -> FINDING TOOL!");

    cout << "---------------------------------------------" << endl;

    cout << "rotating body CCW! beware of left side! continue? _"; cin.ignore();
    MoveWaistFromCurrent(WAIST_TOOL);
    sleep(sleep_time);

    cout << "tilting head upwards to see! continue? _"; cin.ignore();
    TurnHeadTiltPan(-13.0, 0.0);
    sleep(sleep_time);

    cout << "switch camera frame transform to 2. continue? _"; cin.ignore(); 
    ActionSwitchTF(2);
    sleep(sleep_time);

    ROS_INFO_STREAM("go for tool detection... Press anykey to continue...");
    cin.ignore();

    detected = false;
    joint_updated = false;
    cout << "Waiting for tool to be detected..." << endl;
    ros::Rate r(100); // in hz
    do
    {
        ros::spinOnce(); //get vision_data from subscriber
        r.sleep();
    }while( ros::ok() && (!detected || !joint_updated) );

    cout << "data obtained" << endl;
    cout << "===============================================" << endl;

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


    //transform for vtool
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
    
    cout << "--------------------------------" << endl;
    cout << "obj pose: \n" << vtool_in_base.pose << endl;

    Vector3d euler = obj_quat.toRotationMatrix().eulerAngles(0, 1, 2);
    euler = 180.0/M_PI * euler;
    cout << "Euler: " << euler.transpose() << endl;

    tool_in_base = vtool_in_base;

    cout << "adding moveit collision..." << endl;
    addVisualToolCollision(toolMeshes.at(0));

    cout << "Collsion added..." << endl << endl;

    cout << "switch camera frame transform to 1. continue? _"; cin.ignore(); 
    ActionSwitchTF(1);
    sleep(sleep_time);

    cout << "tool detection done" << endl;
    cout << "===============================================" << endl;

    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::mct_tool_experiment()
{
    //at this point, data obtained are
    //1) object
    //2) target
    //3) TOOL

    ROS_INFO_STREAM("testing mct search");

    cout << "---------------------------------------------------" << endl;

    ROS_INFO_STREAM("\nSTART INITALISATION");
    cout << "press enter to continue_"; cin.ignore();

    display_poses_matrix();

    
    if(joint_values.size() <= 0)
    {
        ROS_ERROR_STREAM("joint_states appears to be not filled");
        return -1;
    }

    cout << "joint_values: [ ";
    for (int i = 0; i < joint_values.size(); i++)
        cout << joint_values[i] << " ";
    cout << "] " << endl;

    joint_values[0] =  WAIST_INIT/180 * 3.14159265;  
    cout << "fake set waist yaw to be zero for ik state planning ->  no movement" << endl;

    cout << "---------------------------------------------------" << endl;

    ROS_INFO_STREAM("\nSTART SEARCHING EXPERIMENT");
    cout << "press enter to continue_"; cin.ignore();


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

    //create_tool
    m_create_tool.init();
    sleep(sleep_time);

    //monte carlo tree search
    m_mct_search.init(&m_create_tool, &m_moveit_ik);
    m_mct_search.init_params(3, N_ATK_ANGLE, N_PLAYS);     // N_grasp, N_atk_angle, Nplays
    sleep(sleep_time);

    //main experiment
    // Create_Tool, Search, obj pose, tar pose, tool_pose -> use vtool_pose;
    m_tool_expt.init(&m_create_tool, &m_mct_search, object_in_base.pose, target_in_base.pose);
    // obj_diameter, N_via_pts, intra_pts, max_gjk_iterations, percept;
    m_tool_expt.init_params(PUCK_RADIUS*2.0, 0, 0, 6);
    m_tool_expt.init_vision( v_list.at(0) );  //use vision module for first tool!
    sleep(sleep_time);

    current_tool_width = v_list.at(0).objhight2table;    //use different index for multiple tool

    sleep(1.0);

    //execute
    m_tool_expt.main();
    sleep(1.0);

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

    ROS_INFO_STREAM("MCST planning okay");
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::perform_experiment()
{
    ROS_INFO_STREAM("\nSTART TOOL EXPERIMENT");
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();

    Affine3d tf_hand_atk = create_affine( 0, 0, m_result_atk,  0,0,0);
    Affine3d h_p = create_affine( M_PI/2.0, M_PI/2.0, 0, 0.1782, -0.4509, 1.0168 );
    h_p = h_p * tf_hand_atk;


    double rad_init = WAIST_INIT / 180 * M_PI;
    Affine3d init_waist = create_affine(0,0, rad_init, 0, 0, 0);
    home_pose = init_waist * h_p;

    double rad_waist = WAIST_TOOL / 180 * M_PI;
    Affine3d tool_waist = create_affine(0,0, rad_waist, 0, 0, 0);
    home_Tool = tool_waist * h_p;

    cout << "open fingers right" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    OpenFingers("right", 0);
    sleep(sleep_time);

    Affine3d grab_pose, tf_grab_hand;
    Affine3d return_pose, tf_return_hand;
    Affine3d grab_pose0;
    Affine3d grab_pose1,  tf_grab_hand1;
    
    Affine3d grab_pose2, grab_pose3;

    //note in the tool frame
    double grab_dist = 0.04 + 0.005 + current_tool_width;  //tool_protrusion
    tf_grab_hand = create_affine( M_PI, 0, -M_PI/2.0, 0.0, 0.0, grab_dist); //move back from center of robot ee to tool center
    grab_pose = tool_TF * m_result_grab * tf_grab_hand * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation * hand orient to atk
    
    grab_pose0 = grab_pose;
    Vector3d temp_gg = grab_pose0.translation();
    Vector3d temp_ht = home_Tool.translation();
    cout << "temp gg " << temp_gg.transpose() << endl;
    cout << "temp_ht " << temp_ht.transpose() << endl;
    temp_gg(0) = (temp_ht(0) + temp_gg(0) ) * 1.0/2.0;
    temp_gg(1) = (temp_ht(1) + temp_gg(1) ) * 1.0/2.0; 
    temp_gg(2) = (temp_ht(2) + temp_gg(2) ) * 1.0/2.0;
    grab_pose0.translation() = temp_gg;

    grab_pose2 = grab_pose;
    temp_gg(0) = (temp_ht(0) + temp_gg(0) ) * 2.0/3.0;
    temp_gg(1) = (temp_ht(1) + temp_gg(1) ) * 2.0/3.0; 
    temp_gg(2) = 1.05;
    grab_pose2.translation() = temp_gg;
    
    grab_pose3 = grab_pose;
    temp_gg(0) = (temp_ht(0) + temp_gg(0) ) * 2.0/5.0;
    temp_gg(1) = (temp_ht(1) + temp_gg(1) ) * 2.0/5.0; 
    temp_gg(2) = 1.05;
    grab_pose3.translation() = temp_gg;

    double grab_dist1 = 0.04 + 0.10 + current_tool_width;  //tool_protrusion 
    tf_grab_hand1 = create_affine( M_PI, 0, -M_PI/2.0, 0.0, 0.0, grab_dist1); //move back from center of robot ee to tool center
    grab_pose1 = tool_TF * m_result_grab * tf_grab_hand1 * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation * hand orient to atk

    double return_dist = 0.04 + current_tool_width;  //owards the rack!
    tf_return_hand = create_affine( M_PI, 0, -M_PI/2.0, 0.0, 0.0, return_dist); //move back from center of robot ee to tool center
    return_pose = tool_TF * m_result_grab * tf_return_hand * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation * hand orient to atk

    #ifdef HARDCODE
        Vector3d temp_offset = grab_pose.translation();
        temp_offset(2) = temp_offset(2) + 0.025;
        grab_pose.translation() = temp_offset;
    #endif

    grabposes.push_back(grab_pose0);
    grabposes.push_back(grab_pose1);
    grabposes.push_back(grab_pose);

    returnposes.push_back(grab_pose0);
    returnposes.push_back(grab_pose1);
    returnposes.push_back(return_pose);  

    AddMarker(grab_pose0, "grab_pose_0");
    AddMarker(grab_pose1, "grab_pose_1");
    AddMarker(return_pose, "grab_pose");
    publishMarker();
    sleep(1);

    cout << "go to grab tool" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    sleep(1);
    if( ActionGoToPos(grabposes, false, /*"cart_single"*/"task_single_right", true) < 1 )
    {
        sleep(sleep_time);

        ROS_ERROR_STREAM("Failed");
        cout << "close fingers right" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
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
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();

    #ifdef USE_FINGER_ENC
        powerFingers("right", 3.0, 1); //side, diameter, mode
        sleep(sleep_time);
    #else
        MoveFingers("right", 0.60, 0); //side, %tage, mode
        sleep(sleep_time);
    #endif


    Affine3d half_home_pose;
    half_home_pose = grab_pose;
    Vector3d temp_hhp = half_home_pose.translation();
    temp_ht = home_Tool.translation();
    temp_hhp(0) = (temp_ht(0) + temp_hhp(0) ) * 1.0/2.0;
    temp_hhp(1) = (temp_ht(1) + temp_hhp(1) ) * 1.0/2.0; 
    temp_hhp(2) = (temp_ht(2) + temp_hhp(2) ) * 1.0/2.0;
    half_home_pose.translation() = temp_hhp;
    
    home_path_poses.push_back(grab_pose1);
    #ifndef HARDCODE
        home_path_poses.push_back(home_Tool);
    #endif

    if( ActionGoToPos( home_path_poses, false, "task_single_right", true ) < 1 )
    {
        ROS_ERROR_STREAM("failed to go to home");
        finish_v(true, false, true);
        return -1;
    }

    #ifdef HARDCODE
    cout << "homing to home tool" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    Home(WAIST_TOOL);
    #endif

    sleep(sleep_time);


    //-------------------------------------------------------------

    cout << "returning waist to init position: _"; cin.ignore();
    MoveWaistFromCurrent(WAIST_INIT);
    sleep(sleep_time);

    cout << "turning head to look at obj and target: _"; cin.ignore();      
    TurnHeadToObjTarCenter();
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
    sleep(1);
    if (ActionGoToPos(prepos, false, "task_single_right", true ) >= 0)
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
        ActionGoToPos( home_pose, false, "task_single_right", true );
        finish_v(true, false);
        return -1;
    }
    else
    {
        ROS_INFO_STREAM("puck removed okay~!\n");
    }
    sleep(sleep_time*2);

    bool traj_okay = false;
    vector< Affine3d > result_go;
    for(int i =1; i < m_result.size(); i++) //skip the first place that was already done by preposition
        result_go.push_back(m_result.at(i));
    cout << "going to position..." << endl;
    ROS_WARN_STREAM("press enter to continue...");
    cin.ignore();
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
    if (ActionGoToPos(last, false, "task_single_right", true) >= 0)
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

void Manipulate::finish_v(bool tool_in_hand, bool okay, bool wflag)
{
    if(okay)
    {
        Affine3d last = m_result.at(m_result.size() - 1);
        Vector3d temp_tran = last.translation();
        Vector3d temp_hh = home_pose.translation();
        double pos_half_x = temp_hh(0) + (temp_tran(0) - temp_hh(0))* 3.0/4.0;
        double pos_half_y = temp_hh(1) + (temp_tran(1) - temp_hh(1))* 3.0/4.0; 
        double pos_half_z = (temp_hh(2) + temp_tran(2)) * 1.0/2.0;

        Affine3d home_af = home_pose;
        home_af.translation() = Vector3d(pos_half_x, pos_half_y, pos_half_z);
        cout << "going halfway to home via point" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();

        if (ActionGoToPos(home_af, false, "task_single_right", true ) >= 0)
        {
            ROS_INFO_STREAM("halfway point position okay~!\n");
        }
        else
        {
            ROS_WARN_STREAM("failed to go halfway home from here...\n");
        }
    }
    sleep(sleep_time);
    sleep(1);

    if(!wflag)
    {
        cout << "Return to home_pose" << endl;
        ROS_INFO_STREAM("Press anykey to return to home...");
        cin.ignore();
        ActionGoToPos( home_pose, false, "task_single_right", true );
        sleep(sleep_time);
    }
    else    
    {
        cout << "Return to home_Tool" << endl;
        ROS_INFO_STREAM("Press anykey to return to home...");
        cin.ignore();
        ActionGoToPos( home_Tool, false, "task_single_right", true );
        sleep(sleep_time);
    }


    #ifdef HARDCODE
    cout << "homing to home" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    Home();
    #endif

    if(tool_in_hand)
    {
        cout << "open fingers right" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        OpenFingers("right", 0);
        sleep(sleep_time);

        cout << "detach tool planner" << endl; 
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        ActionDetachObj("right", "vtool");

        cout << "\nremove tool collision..." << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        if (ActionRmObj("vtool") < 0)
        {
            cout << "gg quit" << endl;
            cin.ignore();
            return;
        }
        
        tool_in_hand = false;
    }

    cout << "close fingers right" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    CloseFingers("right", 0, 0);
    sleep(sleep_time);

    if(wflag)   
    {
        cout << "Move waist back to init" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        MoveWaistFromCurrent(WAIST_INIT);
    }

    cout << "Return to home" << endl;
    ROS_INFO_STREAM("Press anykey to return to home...");
    cin.ignore();
    Home();

    cout << "Finish expt~!" << endl;
    cout << "-------------------------------------------------------" << endl;

    return;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::returnTool()
{
    ROS_WARN("-> RETURN TOOL!");
    cout << "---------------------------------------------" << endl;

    cout << "rotating body CCW! beware of left side! continue? _"; cin.ignore();
    MoveWaistFromCurrent(WAIST_TOOL);
    sleep(sleep_time);

    cout << "go to return tool" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    if( ActionGoToPos(returnposes, false, /*"cart_single"*/"task_single_right", true) < 1 )
    {
        ROS_ERROR_STREAM("Failed GG");
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        sleep(sleep_time);
        return -1;
    }
    sleep(sleep_time);

    cout << "open fingers right" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    OpenFingers("right", 0);
    sleep(sleep_time);

    cout << "detach tool planner" << endl; 
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    ActionDetachObj("right", "vtool");


    return 1;
}
