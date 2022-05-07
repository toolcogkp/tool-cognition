#include <tool_expt/manipulate.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <tool_expt/recorder.h>

using namespace std;
using namespace Eigen;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvMonteCarloExpt2 & ) 
{

    WAIST_INIT = 0.0;
    WAIST_TOOL = 60.0;

    OBST_RADIUS = 0.04;

    #ifdef USE_LOWER_TORSO
        init_x = 0.27;
        init_y = -0.45;
        init_z = 0.95;
    #else
        init_x = 0.1782;
        init_y =  -0.4509;
        init_z = 1.0168;
    #endif

    #ifdef UMBRELLA
        OBST_RADIUS = 0.03;
        PUCK_RADIUS = 0.02; 
    #endif



    cout << "homing! press enter to continue:_" << endl; cin.ignore();
    Home();
    sleep(sleep_time);

    //--- main here ---
    if( findObjectTargetLocation2() > 0 )
    {
        cout << "adding moveit collision..." << endl;    
        addTableCollision();
        addPuckCollision(PUCK_RADIUS);
        addObstacleCollision(OBST_RADIUS);
        cout << "Collsion added..." << endl << endl;

        cout << "continue? _"; cin.ignore();
        
        std_msgs::Int32 sw;
        cout << "switch camera_olivia_node to mode 0. continue? _"; cin.ignore(); 
        sw.data = 0;
        camera_swap_publisher.publish(sw);
        sleep(sleep_time);

        cout << "---------------------------------------------" << endl;
        cout << "rotating body CCW! beware of left side! continue? _"; cin.ignore();
        MoveWaistFromCurrent(WAIST_TOOL);
        sleep(sleep_time);
        TurnHeadTiltPan(-13.0, 0.0);
        sleep(sleep_time);

        if( findToolTargetLocation2() > 0 )
        {
            cout << "continue? _"; cin.ignore();
            cout << "returning waist to init position: _"; cin.ignore();
            MoveWaistFromCurrent(WAIST_INIT);
            sleep(sleep_time);
            TurnHeadTiltPan(13.3, 0.0);
            sleep(sleep_time);                
        }    
    }

    return transit< ReadyState >();

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::findObjectTargetLocation2()
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
        
        obstacle_detected = true;

        object_in_base = object_in_base_live; //from callback function
        target_in_base = target_in_base_live;
        if(obstacle_detected)
            obstacle_in_base = obstacle_in_base_live;
        else
        {
            cout << "Obstacle NOT detected ." << endl;
            //put far away
            obstacle_in_base.pose.position.x = 5.0;
            obstacle_in_base.pose.position.y = 5.0;
        }

        object_in_base.pose.position.z = 0.81;                 
        target_in_base.pose.position.z = 0.74;       //table top is 0.74     
        obstacle_in_base.pose.position.z = 0.83;     

        object_in_base.pose.orientation.x = object_in_base.pose.orientation.y = object_in_base.pose.orientation.z = 0;
        object_in_base.pose.orientation.w = 1;
        target_in_base.pose.orientation.x = target_in_base.pose.orientation.y = target_in_base.pose.orientation.z = 0;
        target_in_base.pose.orientation.w = 1;
        obstacle_in_base.pose.orientation.x = obstacle_in_base.pose.orientation.y = obstacle_in_base.pose.orientation.z = 0;
        obstacle_in_base.pose.orientation.w = 1;

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
        {
            ResetPuck();
            ResetObstacle();
        }


        float target_tol = 0.05;
        std::cout << "==========================" << std::endl;
        std::cout << "object location: "<< object_in_base.pose.position.x 
                    << ", " << object_in_base.pose.position.y << ", " 
                    << object_in_base.pose.position.z << std::endl;

        std::cout << "target location: "<< target_in_base.pose.position.x 
                    << ", " << target_in_base.pose.position.y << ", " 
                    << target_in_base.pose.position.z << std::endl;

        std::cout << "obstacle location: "<< obstacle_in_base.pose.position.x 
                    << ", " << obstacle_in_base.pose.position.y << ", " 
                    << obstacle_in_base.pose.position.z << std::endl;



        cout << "==========================================" << endl;
        double y_dif = -(target_in_base.pose.position.y - object_in_base.pose.position.y);
        double x_dif = (target_in_base.pose.position.x - object_in_base.pose.position.x);
        cout << "x dif=" << x_dif << ", y_dif=" << y_dif << endl; 

        if(fabs(y_dif) < target_tol && fabs(x_dif) < target_tol)
        {
            std::cout << "Object already at target!!!" <<std::endl;
            post_event(EvReady());
            return -1;
        }
        else
        {
            
            cout << ">>>>> cal angle of obj to target <<<<<" << endl;
            //calculate the angle of object to target
            //south angle is negative
            radian = std::atan2(x_dif,y_dif);      
            angle = round(radian * 180.0 / PI); 
            ROS_WARN("\nangle obj2tar: deg= %f , radian= %f", angle, radian);
            cout << "==========================================" << endl;
                
            return 1;
        }
    }
    else
        return -1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::findToolTargetLocation2()
{
    std_msgs::Int32 sw;
    ROS_INFO_STREAM("using li jun vision");
    
    Speech("Let's see over here...");
    ROS_WARN("NEXT -> FINDING TOOL!");

    cout << "switch camera frame transform to 2. continue? _\n"; 
    ActionSwitchTF(2);
    sleep(sleep_time);

    cout << "switch camera_olivia_node to mode 2. continue? _\n"; 
    sw.data = 2;
    camera_swap_publisher.publish(sw);
    sleep(sleep_time);

    ROS_INFO_STREAM("go for tool detection... Press anykey to continue...");
    cin.ignore();

    detected = false;
    joint_updated = false;
    cout << "Waiting for tool to be detected..." << endl;

    ros::Rate r(100); // in hz
    do
    {
        // cout << "waiting for /vision/data to be published" << endl;
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

    #ifdef USING_RACK_CAM
        getCurrentTransform("camera_rack_rgb_optical_frame", cam_quat, cam_vec);
    #else
        getCurrentTransform("camera_rgb_optical_frame", cam_quat, cam_vec);
    #endif

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

    cout << "tool detection done" << endl;
    cout << "===============================================" << endl;
    cout << "press enter to continue_"; cin.ignore();

    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::mct_tool_experiment2()
{
    //at this point, data obtained are
    //1) object
    //2) target
    //3) TOOL

    ROS_INFO_STREAM("testing mct search with obstacle");

    cout << "---------------------------------------------------" << endl;

    ROS_INFO_STREAM("\nSTART INITALISATION");
    cout << "press enter to continue_"; cin.ignore();

    clearMarker();
    display_poses_matrix2();
    display_tool_matrix();
    
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

    ROS_INFO_STREAM("\nSTART SEARCHING EXPERIMENT");
    cout << "press enter to continue_"; cin.ignore();

    //create_tool
    m_create_tool.init();
    sleep(sleep_time);

    //monte carlo tree search
    m_mct_search2.init(&m_create_tool, &m_moveit_ik, &m_kdl_ik);
    m_mct_search2.init_params(N_GRASP, N_ATK_ANGLE, N_PLAYS);
    sleep(sleep_time);

    //main experiment
    m_tool_expt2.init(&m_create_tool, &m_mct_search2, object_in_base.pose, target_in_base.pose);
    m_tool_expt2.init_params(PUCK_RADIUS*2.0, OBST_RADIUS*2.0, N_VIA_CANDIDATES, obstacle_in_base.pose, MAX_GJK_ITER);
    m_tool_expt2.init_vision( v_list.at(0) );  //use vision module for first tool!
    sleep(sleep_time);

    current_tool_width = v_list.at(0).objhight2table;    //use different index for multiple tool
    cout << "get tool width" << endl;    
    sleep(1.0);

    //execute
    m_tool_expt2.main();
    sleep(1.0);

    //results
    max_score = m_tool_expt2.getScore();
    if(max_score <= 0)
    {
        ROS_ERROR("\nMCTS FAILED TO FIND SOLUTION");
        cout << "quitting..." << endl;
        return -1;
    }    

    m_obstruct_flag = m_tool_expt2.getObstruct();

    m_result = m_tool_expt2.getResult();
    m_result_grab = m_tool_expt2.getGrab();
    m_result_atk = m_tool_expt2.getAtkAngle();

    cout << "grab loci: \n" << m_result_grab.matrix() << endl;
    cout << "atk angle: " << m_result_atk << endl;

    if(m_obstruct_flag)
    {
        m_result_via_pt = m_tool_expt2.getViaPt();
        cout << "via pt: " << m_result_via_pt.transpose() << endl;
        
        Affine3d via_pt_af = create_affine( 0, 0, 0, m_result_via_pt);
        AddMarker(via_pt_af, "result_via_pt");
        publishMarker();
    }

    cout << "done planning~!" << endl;
    cout << "-------------------------------------------------------" << endl;

    //----------------------------------------------------------------------------//
    //----------------------------------------------------------------------------//

    ROS_INFO_STREAM("MCST planning okay");
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::perform_experiment2()
{
    ROS_INFO_STREAM("\nSTART TOOL EXPERIMENT");
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();

    //-------------------------------------------------------------------//

    Affine3d tf_hand_atk = create_affine( 0, 0, m_result_atk,  0,0,0);
    Affine3d h_p = create_affine( M_PI/2.0, M_PI/2.0, 0, init_x, init_y, init_z ); 
    h_p = h_p * tf_hand_atk;

    double rad_init = WAIST_INIT / 180 * M_PI;
    Affine3d init_waist = create_affine(0,0, rad_init, 0, 0, 0);
    home_pose = init_waist * h_p;

    double rad_waist = WAIST_TOOL / 180 * M_PI;
    Affine3d tool_waist = create_affine(0,0, rad_waist, 0, 0, 0);
    home_Tool = tool_waist * h_p;

    #ifdef UMBRELLA
        Affine3d um_rot = create_affine(0,0, -M_PI/2.0, 0,0,0);
        home_pose = home_pose * um_rot;
        home_Tool = home_Tool * um_rot;
    #endif

    Affine3d grab_pose, tf_grab_hand;
    Affine3d return_pose, tf_return_hand;
    Affine3d grab_pose0;
    Affine3d grab_pose1,  tf_grab_hand1;
    
    Affine3d grab_pose2, grab_pose3;

    //note in the tool frame
    double grab_dist = 0.04 + GRAB_SPACE + current_tool_width; 
    tf_grab_hand = create_affine( M_PI, 0, -M_PI/2.0, 0, 0, grab_dist); 
    grab_pose = tool_TF * m_result_grab * tf_grab_hand * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation * hand orient to atk
    
    #ifdef UMBRELLA
        Vector3d temp_watever = grab_pose.translation();
        temp_watever(2) += OFFSET;
        grab_pose.translation() = temp_watever;
    #endif

    #ifdef SHOW_ALL_GRASP_LOCI
        vector< Affine3d > all_grasp_loci = m_create_tool.grasp_loci;
        for(int g = 0; g < all_grasp_loci.size(); g++)
        {
            Affine3d this_loci_pose = tool_TF * all_grasp_loci.at(g); //world to tool * tool to grab * grab to hand orientation * hand orient to atk
            AddMarker( this_loci_pose, "grasp_pt_"+to_string(g)) ;
        }
        publishMarker();
    #endif

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

    double grab_dist1 = 0.04 + 0.12 + current_tool_width;  
    tf_grab_hand1 = create_affine( M_PI, 0, -M_PI/2.0, 0.0, 0, grab_dist1); 
    grab_pose1 = tool_TF * m_result_grab * tf_grab_hand1 * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation * hand orient to atk

    double return_dist = 0.04 + current_tool_width;  //towards the rack!
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

    #ifdef RECORD_START
    write2file();
    cout << "write to file done" << endl;
    #endif

    //----------------------------------------------------------------------------

    cout << "open fingers right" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    OpenFingers("right", 0);
    sleep(sleep_time);

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
        #ifdef UMBRELLA
            powerFingers("right", 2.0, 1); //side, diameter, mode
        #else
            powerFingers("right", 3.0, 1); //side, diameter, mode
        #endif
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
    home_path_poses.push_back(home_Tool);
    
    cout << "homing" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    if( ActionGoToPos( home_path_poses, false, "task_single_right", true ) < 1 )
    {
        ROS_ERROR_STREAM("failed to go to home");
        sleep(sleep_time);

        cout << "homing again" << endl;
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


    //-------------------------------------------------------------
    cout << "switch camera frame transform to 1. continue? _"; cin.ignore(); 
    ActionSwitchTF(1);
    sleep(sleep_time);

    std_msgs::Int32 sw;
    cout << "switch camera_olivia_node back to mode 0. continue? _"; cin.ignore(); 
    sw.data = 0;
    camera_swap_publisher.publish(sw);
    sleep(sleep_time);
    
    cout << "returning waist to init position: _"; cin.ignore();
    MoveWaistFromCurrent(WAIST_INIT);
    sleep(sleep_time);

    TurnHeadTiltPan(-13.3, 0.0);
    sleep(sleep_time);    

    cout << "switch camera_olivia_node back to mode 1. continue? _"; cin.ignore(); 
    sw.data = 0;
    camera_swap_publisher.publish(sw);
    sleep(sleep_time);
    
    //----------------------------------------------------------------//

    for(int i =0; i < m_result.size(); i++)
        AddMarker(m_result.at(i), "result_pose_"+to_string(i));
    publishMarker();
    sleep(sleep_time);    
    
    vector< Affine3d >prepos;
    Affine3d first = m_result.at(0);
    Vector3d temp_tran = first.translation();
    Vector3d temp_hh = home_pose.translation();
    temp_tran(0) = temp_hh(0) + (temp_tran(0) - temp_hh(0)) * 1.0/3.0;
    temp_tran(1) = temp_hh(1) + (temp_tran(1) - temp_hh(1)) * 1.0/3.0; 
    temp_tran(2) = temp_hh(2) + (temp_tran(2) - temp_hh(2)) * 1.0/3.0;
    first.translation() = temp_tran;
    sleep(sleep_time);

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
    sleep(sleep_time);

    //--------------------------------------------------------------------------//

    int return_idx = 0;
    vector< Affine3d > result_go;
    if(!m_obstruct_flag)
    {
        //-----------do all----------
        for(int i =1; i < m_result.size(); i++) //skip the first place that was already done by preposition
            result_go.push_back(m_result.at(i));
        cout << "going to position..." << endl;
        ROS_WARN_STREAM("press enter to continue...");
        cin.ignore();
        if (ActionGoToPos(result_go, false, "cart_single_right", true ) >= 0)
        {   
            return_idx = m_result.size() - 1;
            ROS_INFO_STREAM("movement okay~!\n");
        }
        else
        {
            ROS_ERROR_STREAM("failed to go to position");
        }
        sleep(sleep_time);
    }
    else //obstructed
    {
        //-----------do one by one-----------------
        result_go.push_back(m_result.at(1));
        cout << "going to position 2..." << endl;
        ROS_WARN_STREAM("press enter to continue...");
        cin.ignore();
        if (ActionGoToPos(result_go, false, "cart_single_right", true ) >= 0)
        {
            return_idx = 1;
            ROS_INFO_STREAM("movement okay~!\n");
        }
        else
        {
            ROS_ERROR_STREAM("failed to go to position");
        }
        sleep(sleep_time);

        //--------------------
        // add collision back for second leg
        object_in_base.pose.position.x = m_result_via_pt(0);
        object_in_base.pose.position.y = m_result_via_pt(1);
        object_in_base.pose.position.z = m_result_via_pt(2);
        addPuckCollision(PUCK_RADIUS);
        sleep(5.0);    //seems necessary to fix bug where collision not recognised!

        result_go.clear();
        result_go.push_back(m_result.at(2));
        cout << "going to position 3..." << endl;
        ROS_WARN_STREAM("press enter to continue...");
        cin.ignore();
        if (ActionGoToPos(result_go, false, "task_single_right", true ) >= 0)
        {
            return_idx = 2;
            ROS_INFO_STREAM("movement okay~!\n");
        }
        else
        {
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
        if (ActionGoToPos(result_go, false, "cart_single_right", true ) >= 0)
        {
            return_idx = 3;
            ROS_INFO_STREAM("movement okay~!\n");
        }
        else
        {
            ROS_ERROR_STREAM("failed to go to position");
        }
        sleep(sleep_time);
    }
    //----------------------------------------------------------

    cout << "going to return position..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    Affine3d last;
    last = m_result.at(return_idx);
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


//--------------------------------------------------------------------------------------//

int Manipulate::write2file()
{
    ofstream myfile;

    std::string out_string;
    std::stringstream ss;
    ss << "/home/olivia/Documents/pose_results_file.txt";
    out_string = ss.str();

    myfile.open(out_string.c_str(), ios::out);

    if(!myfile.is_open())
    {
        cout<< "MANIPULATE: FAILED TO OPEN FILE @: " << out_string << endl;
        return -1;
    }

    myfile << "obj_TF:\n";
    myfile << obj_TF.matrix() << "\n";
    myfile << "\n";

    myfile << "tar_TF:\n";
    myfile << tar_TF.matrix() << "\n";
    myfile << "\n";

    myfile << "tool_TF:\n";
    myfile << tool_TF.matrix() << "\n";
    myfile << "\n";

    myfile << "obst_TF:\n";
    myfile << obst_TF.matrix() << "\n";
    myfile << "\n";

    myfile << "\n-------------------------------\n\n";

    myfile << "grabposes: \n\n";
    for(int i=0; i<grabposes.size(); i++)
    {
        myfile << "grabposes[" << i << "]: \n";
        myfile << grabposes.at(i).matrix() << "\n";
    }myfile << "\n";

    myfile << "m_result: \n\n";
    for(int i=0; i<m_result.size(); i++)
    {
        myfile << "m_result[" << i << "]: \n";
        myfile << m_result.at(i).matrix() << "\n";
    }myfile << "\n";

    myfile << "done!\n";
    myfile.close();

    cout << "DONE! records in Documents as txt" << endl;
    return 0;
}
