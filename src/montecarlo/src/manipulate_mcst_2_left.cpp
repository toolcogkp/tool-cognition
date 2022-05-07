#include <tool_expt/manipulate.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <tool_expt/recorder.h>

using namespace std;
using namespace Eigen;



double euclidean_distance(Vector3d pt1, Vector3d pt2)
{
    Vector3d diff = pt1 - pt2;
    return diff.norm(); 
}

std::vector<double> linspace(double start_in, double end_in, int num_in)
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
  linspaced.push_back(end); 
                            
  return linspaced;
}


vector<Affine3d> linspace_this( Affine3d start, Affine3d end, double spacing)
{
    cout << "linspace_this" << endl;
    vector<Affine3d> done;

    Affine3d aff_a = start;
    Vector3d pos_a = aff_a.translation();
    Affine3d aff_b = end;
    Vector3d pos_b = aff_b.translation();
    
    //distance spacing
    double vec_dist = euclidean_distance(pos_a, pos_b);
    cout << "dist: " << vec_dist << endl;
    int space_idx = floor( vec_dist/ spacing ) + 1;
    vector<double> ls = linspace( 0, vec_dist, space_idx);
    cout << "ls: \n[ ";
    for(int l=0; l < ls.size(); l++)  
        cout << ls.at(l) << " ";
    cout << "]\n" << endl;

    //angle
    Vector3d diff = pos_b - pos_a;
    double angle = atan2( diff(1), diff(0) );
    Quaterniond temp_q;
    temp_q = AngleAxisd(0, Vector3d::UnitX())                   //roll, axis_x
            * AngleAxisd(0, Vector3d::UnitY())                  //pitch, axis_y
            * AngleAxisd(angle, Vector3d::UnitZ());           //yaw, axis_z
    Matrix3d rot = temp_q.normalized().toRotationMatrix();

    // done.push_back(start);
    for(int i = 1; i < ls.size(); i++)
    {
        Vector3d temp( ls[i], 0, 0 );
        Vector3d vec = pos_a + rot *  temp;
        Affine3d pose = start;
        pose.translation() = vec;
        done.push_back(pose);
    }

    return done;
}

Affine3d Manipulate::EstimatePose(double fraction, Affine3d start, Affine3d end)
{
    cout << "estimate puck position based on fraction of path done" << endl;
    
    Affine3d aff_a = start;
    Vector3d pos_a = aff_a.translation();
    Affine3d aff_b = end;
    Vector3d pos_b = aff_b.translation();

    //distance 
    double vec_dist = euclidean_distance(pos_a, pos_b);
    double est_dist = vec_dist * fraction;
    est_dist += 0.01; 

    //angle
    Vector3d diff = pos_b - pos_a;
    double angle = atan2( diff(1), diff(0) );
    Quaterniond temp_q;
    temp_q = AngleAxisd(0, Vector3d::UnitX())                   //roll, axis_x
            * AngleAxisd(0, Vector3d::UnitY())                  //pitch, axis_y
            * AngleAxisd(angle, Vector3d::UnitZ());           //yaw, axis_z
    Matrix3d rot = temp_q.normalized().toRotationMatrix();

    Vector3d temp( est_dist, 0, 0 );
    Vector3d vec = pos_a + rot *  temp;

    Affine3d est_pose = start;
    est_pose.translation() = vec;
    return est_pose;    
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

sc::result Manipulate::react( const EvMonteCarloExpt2_left & ) 
{
    ROS_ERROR_STREAM("DOING LEFT!");


    WAIST_INIT = -30.0;
    WAIST_TOOL = 60.0;

    OBST_RADIUS = 0.03;
    PUCK_RADIUS = 0.03;

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
    // cin.ignore(); 
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

    cout << "switch camera_olivia_node to mode 1. continue? _"; 
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
        
        cout << "switch camera_olivia_node to mode 0. continue? _"; 

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

        if( findToolTargetLocation2() > 0 )
        {
            cout << "continue? _"; cin.ignore();
            if( mct_tool_experiment2_left() > 0)
            {
                cout << "continue? _"; cin.ignore();
                perform_experiment2_left();
            }
            else
            {        
                cout << "switch camera_olivia_node to mode 0. continue? _"; 
                sw.data = 0;
                camera_swap_publisher.publish(sw);
                sleep(sleep_time);

                cout << "returning waist to init position: _"; cin.ignore();
  
                #ifdef USE_LOWER_TORSO
                   neck_and_waist_trunk( -9.0, 0.0, 0.05, WAIST_INIT, 65.0);
                #else
                    neck_and_waist_trunk( 13.3, 0.0, 0.05, WAIST_INIT, 47.44);
                #endif
                sleep(sleep_time);                
            }
        }    
    }

    return transit< ReadyState >();

}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::findObjectTargetLocation2_left()
{
    ROS_ERROR_STREAM("DOING LEFT!");

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

        if(obstacle_detected)
            obstacle_in_base = obstacle_in_base_live;
        else
        {
            ROS_INFO_STREAM("Obstacle NOT detected!!! \n");

            obstacle_in_base.pose.position.x = 5.0;    //put somewhere away
            obstacle_in_base.pose.position.y = 5.0;
        }

        object_in_base.pose.position.z = 0.88 - 0.07;          
        target_in_base.pose.position.z = 0.88 - 0.07;   //table top is 0.74     
        obstacle_in_base.pose.position.z = 0.88 - 0.07;     

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

int Manipulate::mct_tool_experiment2_left()
{
    ROS_ERROR_STREAM("DOING LEFT!");

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

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::perform_experiment2_left()
{
    ROS_ERROR_STREAM("DOING LEFT!");
 
    ROS_INFO_STREAM("\nSTART TOOL EXPERIMENT");
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();

    //-------------------------------------------------------------------//

    Affine3d tf_hand_atk = create_affine( 0, 0, m_result_atk,  0,0,0);
    Affine3d h_p = create_affine( -M_PI/2.0, M_PI/2.0, 0, init_x, init_y, init_z );     
    h_p = h_p * tf_hand_atk;

    double rad_init = WAIST_INIT / 180 * M_PI;
    Affine3d init_waist = create_affine(0,0, rad_init, 0, 0, 0);
    home_pose = init_waist * h_p;

    double rad_waist = WAIST_TOOL / 180 * M_PI;
    Affine3d tool_waist = create_affine(0,0, rad_waist, 0, 0, 0);
    home_Tool = tool_waist * h_p;

    // #ifdef UMBRELLA
        Affine3d um_rot = create_affine(0,0, M_PI/2.0, 0,0,0);
        home_pose = home_pose * um_rot;
        home_Tool = home_Tool * um_rot;
    
        //raise higher by 5cm
        Vector3d temp_watever = home_Tool.translation();
        temp_watever(2) += 0.05; 
        home_Tool.translation() = temp_watever;
        temp_watever = home_pose.translation();
        temp_watever(2) += 0.05; 
        home_pose.translation() = temp_watever;
    // #endif

    Affine3d grab_pose, tf_grab_hand;
    Affine3d return_pose, tf_return_hand;
    Affine3d grab_pose0;
    Affine3d grab_pose1,  tf_grab_hand1;
    
    Affine3d grab_pose2, grab_pose3;

    //note in the tool frame
    double grab_dist = 0.04 + GRAB_SPACE + current_tool_width;  
    tf_grab_hand = create_affine( M_PI, 0, M_PI/2.0, 0.0, 0, grab_dist); 
    grab_pose = tool_TF * m_result_grab * tf_grab_hand * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation * hand orient to atk
    
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

    double grab_dist1 = 0.04 + 0.12 + current_tool_width + 0.02;  
    tf_grab_hand1 = create_affine( M_PI, 0, M_PI/2.0, 0.0, 0, grab_dist1); //move back from center of robot ee to tool center
    grab_pose1 = tool_TF * m_result_grab * tf_grab_hand1 * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation * hand orient to atk

    double return_dist = 0.04 + current_tool_width;  //towards the rack!
    tf_return_hand = create_affine( M_PI, 0, M_PI/2.0, 0.0, 0.0, return_dist); //move back from center of robot ee to tool center
    return_pose = tool_TF * m_result_grab * tf_return_hand * tf_hand_atk; //world to tool * tool to grab * grab to hand orientation * hand orient to atk

    temp_watever = grab_pose1.translation();
    temp_watever(2) += OFFSET;  //push up z by offset
    grab_pose1.translation() = temp_watever;

    temp_watever = grab_pose.translation();
    temp_watever(2) += OFFSET; 
    grab_pose.translation() = temp_watever;

    grabposes.push_back(grab_pose0);
    grabposes.push_back(grab_pose1);
    grabposes.push_back(grab_pose);

    returnposes.push_back(grab_pose0);
    returnposes.push_back(grab_pose1);
    returnposes.push_back(return_pose);  

    AddMarker(grab_pose0, "grab_pose_0");
    AddMarker(grab_pose1, "grab_pose_1");
    AddMarker(grab_pose, "grab_pose");
    publishMarker();
    sleep(1);

    #ifdef RECORD_START
    write2file();
    cout << "write to file done" << endl;
    #endif

    //----------------------------------------------------------------------------

    cout << "open fingers left" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    OpenFingers("left", 0);
    sleep(sleep_time);

    // cout << "tool grabbing pos: \n" << grab_pose.matrix() << endl;
    cout << "go to grab tool" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    sleep(1);
    if( ActionGoToPos(grabposes, false, /*"cart_single"*/"task_single_right", true, "l_ee") < 1 )
    {
        sleep(sleep_time);

        ROS_ERROR_STREAM("Failed");
        cout << "close fingers left" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        CloseFingers("left", 0, 0);
        sleep(sleep_time);

        finish_v_l(false, false, true);
        return -1;
    }
    sleep(sleep_time);

    ROS_WARN_STREAM("check if position correct, if not ctrl+c here!"); 
    cout << "attach tool planner" << endl; 
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    ActionAttachObj("left", "vtool", tool_collision);

    cout << "close fingers left" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();

    #ifdef USE_FINGER_ENC
        #ifdef UMBRELLA
            powerFingers("left", 2.0, 1); //side, diameter, mode
        #else
            powerFingers("left", 3.0, 1); //side, diameter, mode
        #endif
        sleep(sleep_time);
    #else
        MoveFingers("left", 0.60, 0); //side, %tage, mode
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
    if( ActionGoToPos( home_path_poses, false, "task_single_right", true, "l_ee") < 1 )
    {
        ROS_ERROR_STREAM("failed to go to home");
        sleep(sleep_time);

        cout << "homing again" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        #ifdef USE_LOWER_TORSO
            if( Home2(WAIST_INIT) < 0)
            {
                ROS_ERROR_STREAM("failed to go to home");
                finish_v_l(true, false, true);
                return -1;
            }
        #else
            if( Home(WAIST_TOOL) < 0)
            {
                ROS_ERROR_STREAM("failed to go to home");
                finish_v_l(true, false, true);
                return -1;
            }
        #endif
    }
    sleep(sleep_time);


    //-------------------------------------------------------------

    cout << "switch camera frame transform to 1. continue? _\n"; 
    #ifdef USE_LOWER_TORSO
        ActionSwitchTF(3);
    #else
        ActionSwitchTF(4);
    #endif
    sleep(sleep_time);

    std_msgs::Int32 sw;
    cout << "switch camera_olivia_node back to mode 0. continue? _\n"; 

    sw.data = 0;
    camera_swap_publisher.publish(sw);
    sleep(sleep_time);

    cout << "returning waist to init position: _"; 
    cin.ignore();
    #ifdef USE_LOWER_TORSO
       neck_and_waist_trunk( -9.0, 0.0, 0.1, WAIST_INIT, 65.0);
    #else
        neck_and_waist_trunk( 13.3, 0.0, 0.1, WAIST_INIT, 47.44);
    #endif
    sleep(sleep_time);    

    cout << "switch camera_olivia_node back to mode 1. continue? _"; 
    sw.data = 1;
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
    temp_tran(2) = temp_hh(2) + (temp_tran(2) - temp_hh(2)) * 1.0/3.0 + 0.10;   
    first.translation() = temp_tran;
    sleep(sleep_time);

    
    prepos.push_back(m_result.at(0)); //sec
    cout << "going to pre-positioning..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    cout << "pre-positioning:" << endl;
    sleep(1);
    if (ActionGoToPos(prepos, false, "task_single_right", true, "l_ee") >= 0)
    {
        ROS_INFO_STREAM("pre-positioning okay~!\n");
    }
    else
    {
        ROS_ERROR_STREAM("failed to go to pre-position\n");
        finish_v_l(true, false);
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
        ActionGoToPos( home_pose, false, "task_single_right", true, "l_ee");
        finish_v_l(true, false);
        return -1;
    }
    else
    {
        ROS_INFO_STREAM("puck removed okay~!\n");
    }
    ROS_INFO_STREAM("sleeping");
    sleep(5.0);

    //--------------------------------------------------------------------------//

    int return_idx = 0;
    vector< Affine3d > result_go;


    if(!m_obstruct_flag)
    {
        ROS_WARN_STREAM("DO ALL");
        //-----------do all----------
        for(int i =1; i < m_result.size(); i++) //skip the first place that was already done by preposition
            result_go.push_back(m_result.at(i));
        cout << "going to position..." << endl;
        ROS_WARN_STREAM("press enter to continue...");
        cin.ignore();
        if (ActionGoToPos(result_go, false, "cart_single_right", true, "l_ee" ) >= 0)
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
        ROS_WARN_STREAM("DO ONE BY ONE");
        //-----------do one by one-----------------
        bool failed = false;
        double fraction = 0.0;

        result_go.push_back(m_result.at(1));
        cout << "going to position 2..." << endl;
        ROS_WARN_STREAM("press enter to continue...");
        cin.ignore();

        if (ActionGoToPos(result_go, &fraction, false, "cart_single_right", true, "l_ee") >= 0)
        {
        
                Affine3d puck_TF = obj_TF;
                Affine3d via_pt_TF = create_affine( 0, 0, 0, m_result_via_pt);  
                int idx = 1;

                failed = !ReAdjustment(idx, result_go, fraction, puck_TF, via_pt_TF);
                if(!failed)
                {
                    return_idx = 1;
                    ROS_INFO_STREAM("movement okay~!\n");
                } 
        }
        else
        {
            ROS_ERROR_STREAM("failed to go to position 2");
            failed = true;
        }
        sleep(sleep_time);


        if(!failed)
        {
            // --------------------
            // add collision back for second leg
            object_in_base.pose.position.x = m_result_via_pt(0);
            object_in_base.pose.position.y = m_result_via_pt(1);
            object_in_base.pose.position.z = m_result_via_pt(2);
            addPuckCollision(PUCK_RADIUS);
            sleep(5.0);    

            result_go.clear();
            result_go.push_back(m_result.at(2));
            cout << "going to position 3..." << endl;
            ROS_WARN_STREAM("press enter to continue...");
            cin.ignore();
            if (ActionGoToPos(result_go, false, "task_single_right", true, "l_ee" ) >= 0)
            {
                return_idx = 2;
                ROS_INFO_STREAM("movement okay~!\n");
            }
            else
            {
                ROS_ERROR_STREAM("failed to go to position 3");
                failed = true;
            }
            sleep(sleep_time);
        }

        if(!failed)
        {
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
            ROS_INFO_STREAM("sleeping");
            sleep(5.0);
            
            fraction = 0.0;
            result_go.clear();
            result_go.push_back(m_result.at(3));
            cout << "going to position 4..." << endl;
            sleep(sleep_time);
            if (ActionGoToPos(result_go, &fraction, false, "cart_single_right", true, "l_ee" ) >= 0)
            {
                    Affine3d puck_TF = create_affine( 0, 0, 0, m_result_via_pt);    //start at via_pt
                    Affine3d finish_TF = tar_TF;  
                    int idx = 1;

                    failed = !ReAdjustment(idx, result_go, fraction, puck_TF, finish_TF);
                    if(!failed)
                    {
                        return_idx = 3;
                        ROS_INFO_STREAM("movement okay~!\n");
                    } 
            }
            else
            {
                ROS_ERROR_STREAM("failed to go to position 4");
                failed = true;
            }
            sleep(sleep_time);
        }

    }
    //----------------------------------------------------------

    cout << "going to return position..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    Affine3d last;
    last = m_result.at(return_idx);
    temp_tran = last.translation();
    temp_tran(2) += 0.12; 
    last.translation() = temp_tran;
    cout << "return position:" << endl;
    if (ActionGoToPos(last, false, "task_single_right", true, "l_ee") >= 0)
    {
        ROS_INFO_STREAM("position okay~!\n");
    }
    else
    {
        ROS_ERROR_STREAM("failed to go to position");
        finish_v_l(true, false);
        return -1;
    }
    sleep(sleep_time);

    finish_v_l(true);

    ROS_INFO_STREAM("FINISHED");
    return 1;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::finish_v_l(bool tool_in_hand, bool okay, bool wflag)
{
    if(!wflag)
    {
        cout << "Return to home_pose" << endl;
        ROS_INFO_STREAM("Press anykey to return to home...");
        cin.ignore();

        ActionGoToPos( home_pose, false, "task_single_right", true, "l_ee" );
        sleep(sleep_time);
    }
    else    
    {
        cout << "Return to home_Tool" << endl;
        ROS_INFO_STREAM("Press anykey to return to home...");
        cin.ignore();
        ActionGoToPos( home_Tool, false, "task_single_right", true, "l_ee" );
        sleep(sleep_time);
    }

    if(tool_in_hand)
    {
        cout << "open fingers left" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        OpenFingers("left", 0);
        sleep(sleep_time);

        cout << "detach tool planner" << endl; 
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        ActionDetachObj("left", "vtool");

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

    cout << "close fingers left" << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    CloseFingers("left", 0, 0);
    sleep(sleep_time);

    if(wflag)   
    {
        cout << "Move waist back to init" << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        cin.ignore();
        MoveWaistFromCurrent(WAIST_INIT);
        sleep(sleep_time);
    }

    cout << "Return to home" << endl;
    ROS_INFO_STREAM("Press anykey to return to home...");
    cin.ignore();
    #ifdef USE_LOWER_TORSO
        Home2(WAIST_INIT);
    #else
        Home(WAIST_INIT);
    #endif
    sleep(sleep_time);

    cout << "Finish expt~!" << endl;
    cout << "-------------------------------------------------------" << endl;

    return;
}


//-----------------------------------------------------------------
//start_TF := start TF of PUCK!
//end_TF := end TF of PUCK!

bool Manipulate::ReAdjustment(int idx, vector< Affine3d > gohere, double fraction, Affine3d start_TF, Affine3d end_TF)
{
    if(fraction <= 0)
    {
        ROS_ERROR_STREAM("FAILED!");
        return false;
    }
    else if(fraction < PATH_THRESHOLD)
    {
        string func_name = "RE" + std::to_string(idx) + ": "; 
        ROS_WARN_STREAM("fraction = " << fraction << "; NOT " << PATH_THRESHOLD);
        // testing reattempt
        cout << "going for readjustment " << idx << "..." << endl;

        Affine3d estimated_TF = EstimatePose( fraction, start_TF, end_TF);
        Vector3d estimated_pos = estimated_TF.translation();
        // --------------------
        // add collision back 
        object_in_base.pose.position.x = estimated_pos(0);
        object_in_base.pose.position.y = estimated_pos(1);
        object_in_base.pose.position.z = estimated_pos(2);
        tf::poseMsgToEigen(object_in_base.pose, start_TF); 
        
        cout << func_name << "Add puck collision" << endl;
        ROS_WARN_STREAM("press enter to continue...");
        cin.ignore();
        addPuckCollision(PUCK_RADIUS);
        sleep(5.0);   

        cout << func_name << "get TF for l_ee... DONT MOVE" << endl;
        ROS_WARN_STREAM("press enter to continue...");
        cin.ignore();

        Affine3d l_ee_affine;
        geometry_msgs::PoseStamped l_ee_pose;
        getCurrentTransform("l_ee", l_ee_pose);
        tf::poseMsgToEigen(l_ee_pose.pose, l_ee_affine);

        cout << func_name << "Return to home_pose" << endl;
        ROS_INFO_STREAM("Press anykey to return to home...");
        cin.ignore();
        if( ActionGoToPos( home_pose, false, "task_single_right", true, "l_ee" ) < 0)
        {
            ROS_ERROR_STREAM("FAILED!");
            return false;
        }
        sleep(sleep_time);

        cout << func_name << "Return to prev_pose" << endl;
        ROS_INFO_STREAM("Press anykey to return to home...");
        cin.ignore();
        if( ActionGoToPos( l_ee_affine, false, "task_single_right", true, "l_ee" ) < 0)
        {
            ROS_ERROR_STREAM("FAILED!");
            return false;
        }
        sleep(sleep_time);

        cout << func_name << "Remove puck collision..." << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        ActionRmObj("puck");
        sleep(5.0);

        cout << func_name << "Go to position..." << endl;
        ROS_INFO_STREAM("Press anykey to continue...");
        if (ActionGoToPos(gohere, &fraction, false, "cart_single_right", true, "l_ee") < 0)
        {  
            ROS_ERROR_STREAM("FAILED!");
            return false;
        }
        else
        {
            idx++;
            return ReAdjustment(idx, gohere, fraction, start_TF, end_TF);
        }
    }

    //100%
    ROS_INFO_STREAM("movement okay~!\n");
    return true;
}
