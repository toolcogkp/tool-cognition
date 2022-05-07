#include <tool_expt/manipulate.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <tool_expt/recorder.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace Eigen;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvMonteCarloExpt3_left & ) 
{
    ROS_ERROR_STREAM("DOING LEFT!");

    WAIST_INIT = -30.0;
    WAIST_TOOL = 60.0;
    OBST_RADIUS = 0.03;
    PUCK_RADIUS = 0.045;

    #ifdef USE_LOWER_TORSO
        init_x = 0.27;
        init_y = 0.45;
        init_z = 0.95;
    #else
        init_x = 0.3217;
        init_y = 0.36278;
        init_z = 1.0168;
    #endif

    std_msgs::Int32 sw;
    cout << "switch camera_olivia_node to mode 0. continue? _"; 

    sw.data = 0;
    camera_swap_publisher.publish(sw);
    sleep(sleep_time);

    #ifdef USE_LOWER_TORSO
        ActionSwitchTF(3);
    #else
        ActionSwitchTF(4);
    #endif
    sleep(sleep_time);

    cout << "homing! press enter to continue:_" << endl; cin.ignore();
    #ifdef USE_LOWER_TORSO
        Home2(WAIST_INIT);
    #else
        Home(WAIST_INIT);
    #endif
    sleep(sleep_time);

    cout << "switch camera_olivia_node to mode 4. continue? _"; 

    sw.data = 1;
    camera_swap_publisher.publish(sw);
    sleep(sleep_time);

    //--- main here ---
    if( findObjectTargetLocation2_left() > 0 )
    {
        addTableCollision();
        addPuckCollision(PUCK_RADIUS);
        if(obstacle_detected)
            addObstacleCollision(OBST_RADIUS);
        cout << "Collsion added..." << endl << endl;

        cout << "continue? _"; cin.ignore();

        if( findToolTargetLocation3() > 0 )
        {                   
            cout << "continue? _"; cin.ignore();
		    cout << "switch camera_olivia_node to mode 0.\n"; 
		    sw.data = 0;
		    camera_swap_publisher.publish(sw);
		    sleep(sleep_time);

		    cout << "---------------------------------------------" << endl;
		    cout << "rotating body CCW! beware of left side! continue? _"; cin.ignore();

		    #ifdef USE_LOWER_TORSO
		        neck_and_waist_trunk( -13.3, 0.4, 0.05, WAIST_TOOL, 80.0);
		    #else
		        neck_and_waist_trunk( 0.0, 0.2, 0.05, WAIST_TOOL, 47.44);
		    #endif
		    sleep(sleep_time);
		
		    cout << "switch camera frame transform to 2. continue? _\n"; 
		    ActionSwitchTF(2);
		    sleep(sleep_time);

		    cout << "switch camera_olivia_node to mode 2. continue? _\n"; 
		    sw.data = 2;
		    camera_swap_publisher.publish(sw);
		    sleep(sleep_time);

		    cout << "get TF here! continue?_" << endl; cin.ignore();
		    findToolLoca();
		    display_tool_matrix(); 

        }    
    }

    return transit< ReadyState >();

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
// neck in radian
// waist and trunk in degree
int Manipulate::neck_and_waist_trunk(double tilt_angle, double pan_angle, double pan_speed, double angle_deg, double angle_deg_2)
{
    cout << "moving waist trunk and neck" << endl;

    actionlib::SimpleActionClient<m3_moveit::MoveitWholeBodyAction> moveit_j_ac("joint_whole_body", true);
    moveit_j_ac.waitForServer(ros::Duration(5.0));

    actionlib::SimpleActionClient<neck_pan::NeckPanAction> neck_ac("neck_pan", true);
    neck_ac.waitForServer(ros::Duration(20.0));

    m3_moveit::MoveitWholeBodyGoal target_joint;
    m3_moveit::MoveitWholeBodyResultConstPtr result_joint;

    target_joint = SetJointTargetsToCurrent();

    if(pan_angle > 1.22 || pan_angle < -1.22){
    cout << "neck PAN angle outside of 1.22 to -1.22 rad range >>> saturates" << endl;

   if(pan_angle > 1.22)
     pan_angle = 1.22;
    else if(pan_angle < -1.22)
        pan_angle = -1.22;
    else
        return -1; 
    }

    if(pan_speed > 1.0)
        pan_speed = 1.0;
    else if(pan_speed < 0)
        pan_speed = 0;

    if(tilt_angle > 13.3 || tilt_angle < -13.3){
        cout << "neck tilt angle outside of 13.3 to -13.3deg range >>> saturate" << endl;
        if(tilt_angle > 13.3)
            tilt_angle = 13.3;
        else if(tilt_angle < -13.3)
            tilt_angle = -13.3;
        else
            return -1;
    }

    neck_pan::NeckPanGoal goal;
    goal.angle = pan_angle;
    goal.speed = pan_speed;

    if(!joint_state.name.empty())
    {
        target_joint.torso_trajectory.points[0].positions[0] = (angle_deg/180.0*3.14159265);
        target_joint.torso_trajectory.points[0].positions[2] = (angle_deg_2/180.0*3.14159265);
        target_joint.head_trajectory.points[0].positions[0] =  tilt_angle/180*3.14159265; 

        cout << "Moving to neck pan angle = "<< pan_angle << "rads\n";
        cout << "Moving to waist yaw angle from current...\n";
        moveit_j_ac.sendGoal(target_joint);
        neck_ac.sendGoal(goal);


        moveit_j_ac.waitForResult(ros::Duration(30.0));
        neck_ac.waitForResult(ros::Duration(10.0));
        
        sleep(sleep_time);
        
        neck_pan::NeckPanResultConstPtr result  = neck_ac.getResult();
        result_joint = moveit_j_ac.getResult();
    
        if (result->result == neck_pan::NeckPanResult::K_RESULT_OK)
        {
            cout << "neck PANNING done!" << endl;
            return 1;
        }
        else
        {
            cout << "neck PANNING failed" << endl;
            return -1;
        }

        if(result_joint->error_code == result_joint->FAIL) 
        {
            std::cout << "Unable to reach waist yaw angle!\n";
        }
        else
        {
            cout << "Reached waist yaw angle.\n";
            return -1;
        }
    }

    cout << "done!" << endl;
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::findToolTargetLocation3()
{
    std_msgs::Int32 sw;
    ROS_INFO_STREAM("using vision");
    
    Speech("Let's see over here...");
    ROS_WARN("NEXT -> FINDING TOOL!");

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
    cout << "press enter to continue_"; cin.ignore();

    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::findToolLoca()
{
    tf::Quaternion cam_quat;
    tf::Vector3 cam_vec;

    //check rosmaster for topics
    for(int i =0; i < 10; i++){
        usleep(100000);
        ros::spinOnce();
    }

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
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::mct_tool_experiment3_left()
{
    ROS_ERROR_STREAM("DOING LEFT 3!");

    //at this point, data obtained are
    //1) object
    //2) target

    ROS_INFO_STREAM("testing mct search with obstacle");

    cout << "---------------------------------------------------" << endl;

    ROS_INFO_STREAM("\nSTART INITALISATION");
    cout << "press enter to continue_"; cin.ignore();

    clearMarker();
    display_poses_matrix2();
    
    if(joint_values_l.size() <= 0)
    {
        ROS_ERROR_STREAM("joint_states LEFT appears to be not filled");
        return -1;
    }

    cout << "joint_values_l: [ ";
    for (int i = 0; i < joint_values_l.size(); i++)
        cout << joint_values_l[i] << " ";
    cout << "] " << endl;

    joint_values_l[0] =  WAIST_INIT/180 * 3.14159265;  
    cout << "fake set waist yaw to be zero for ik state planning ->  no movement" << endl;

    cout << "---------------------------------------------------" << endl;

    //moveit_ik_solver
    m_moveit_ik.init(false);
    m_moveit_ik.init_home(joint_values_l);
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
    m_kdl_ik.init(false);
    m_kdl_ik.init_home(joint_values_l, maxJnt, minJnt);
    sleep(1.0);

    ROS_INFO_STREAM("\nSTART SEARCHING EXPERIMENT");
    cout << "press enter to continue_"; cin.ignore();

    //create_tool
    m_create_tool.init();
    sleep(sleep_time);

    //monte carlo tree search
    m_mct_search2.init(&m_create_tool, &m_moveit_ik, &m_kdl_ik, false);
    m_mct_search2.init_params(N_GRASP, N_ATK_ANGLE, N_PLAYS);
    sleep(sleep_time);

    //inflated diameter
    double OBST_RADIUS_INFLATED = OBST_RADIUS+0.01;

    //main experiment
    m_tool_expt2.init(&m_create_tool, &m_mct_search2, object_in_base.pose, target_in_base.pose/*, tool_in_base.pose*/);
    m_tool_expt2.init_params(PUCK_RADIUS*2.0, OBST_RADIUS_INFLATED*2.0, N_VIA_CANDIDATES, obstacle_in_base.pose, MAX_GJK_ITER);
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
        
        //put via pt markers
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
