#include <tool_expt/manipulate.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

using namespace std;
using namespace Eigen;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

#include <sys/ioctl.h>
#include <ncurses.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

bool Manipulate::kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
bool resFlag;
bool inputFlag;

void Manipulate::doneCb(const actionlib::SimpleClientGoalState& state,
                        const m3_moveit::MoveitSingleResultConstPtr& result)
{
    ROS_INFO("result receieved");
    resFlag = true;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::activeCb()
{
    ROS_INFO("Goal just went active");
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::feedbackCb(const m3_moveit::MoveitSingleFeedbackConstPtr& feedback)
{
    ROS_INFO("feedback receieved");

    switch(feedback->state)
    {
        case -1:
        {
            ROS_INFO("Timeout for checking attached items");
            break;
        }
        case 0:
        {
            ROS_INFO("Start planning");
            break;
        }
        case 3:
        {
            ROS_INFO("Planning only stage started");
            break;    
        }
        case 4:
        {
            ROS_WARN("server asking for input: _ ");
            inputFlag = true;
            break;
        }
        default:
        {
            ROS_WARN("action server in unknown state");
            break;
        }
    }
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionAttachObj(string arm_name, string obj_name, moveit_msgs::CollisionObject cobj)
{
    std::cout << "ActionAttachObject" << std::endl;
    std::cout << "==========================" << std::endl;

    actionlib::SimpleActionClient<m3_moveit::MoveitPickPlaceAction> moveit_attach_ac("attach_object", true);
    moveit_attach_ac.waitForServer(ros::Duration(10.0));

    m3_moveit::MoveitPickPlaceGoal target;
    m3_moveit::MoveitPickPlaceResultConstPtr result;

    target.arm_name = arm_name; //"right" or "left"
    target.object_name = obj_name;
    target.collision_object = cobj;

    moveit_attach_ac.sendGoal(target);
    moveit_attach_ac.waitForResult(ros::Duration(20.0));
    result = moveit_attach_ac.getResult();

    sleep(sleep_time);

    if (result->error_code == result->FAIL)
    {
        cout << "attach FAILED!\n";
        return -1;
    }
    else
    {
        cout << "attach DONE. \n";
    }

    std::cout << "==========================" << std::endl;
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionDetachObj(string arm_name, string obj_name)
{
    std::cout << "ActionDetachObject" << std::endl;
    std::cout << "==========================" << std::endl;

    actionlib::SimpleActionClient<m3_moveit::MoveitPickPlaceAction> moveit_detach_ac("detach_object", true);
    moveit_detach_ac.waitForServer(ros::Duration(10.0));

    m3_moveit::MoveitPickPlaceGoal target;
    m3_moveit::MoveitPickPlaceResultConstPtr result;

    target.arm_name = arm_name; // "right" or "left"
    target.object_name = obj_name;

    moveit_detach_ac.sendGoal(target);
    moveit_detach_ac.waitForResult(ros::Duration(20.0));
    result = moveit_detach_ac.getResult();

    sleep(sleep_time);

    if (result->error_code == result->FAIL)
    {
        cout << "detach FAILED!\n";
        return -1;
    }
    else
    {
        cout << "detach DONE. \n";
    }

    std::cout << "==========================" << std::endl;
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionRmObj(string obj_id)
{

    actionlib::SimpleActionClient<m3_moveit::MoveitCollideAction> ac("remove_collision", true);
    ac.waitForServer(ros::Duration(10.0));
    m3_moveit::MoveitCollideGoal target;
    m3_moveit::MoveitCollideResultConstPtr result;
    target.object_name = obj_id;

    std::cout << "ActionRmObj" << std::endl;
    std::cout << "==========================" << std::endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    sleep(sleep_time);

    if (result->error_code == result->FAIL)
    {
        cout << "removal FAILED!\n";
        return -1;
    }
    else
    {
        cout << "removal DONE. \n";
    }

    std::cout << "==========================" << std::endl;
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionAddObj(string obj_id, moveit_msgs::CollisionObject cobj)
{

    actionlib::SimpleActionClient<m3_moveit::MoveitCollideAction> ac("add_collision", true);
    ac.waitForServer(ros::Duration(10.0));
    m3_moveit::MoveitCollideGoal target;
    m3_moveit::MoveitCollideResultConstPtr result;

    target.object_name = obj_id;
    target.collision_object = cobj;

    std::cout << "ActionAddObj" << std::endl;
    std::cout << "==========================" << std::endl;

    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(20.0));
    result = ac.getResult();

    sleep(sleep_time);

    if (result->error_code == result->FAIL)
    {
        cout << "addition FAILED!\n";
        return -1;
    }
    else
    {
        cout << "addition DONE. \n";
        sleep(2.0); //give time for planning scene to update
    }

    std::cout << "==========================" << std::endl;
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::checkUserInput()
{ 
    resFlag = false;
    inputFlag = false;

    ros::Rate r(20); // in hz
    ROS_INFO("waiting for server response...");
    while (ros::ok() && !resFlag)
    {
        if(inputFlag)
        {
            if( kbhit() )   //key pressed
            {
                int ch = getchar();
                printf("\tYou pressed '%c'!\n", ch);
                
                std_msgs::Int32 ff;
                ff.data = -1;
                if(ch == 'r' | ch == 'R')
                {
                    printf("user input R - replan");
                    ff.data = 1;
                }
                else if(ch == 'q' | ch == 'Q')
                {
                    printf("user input Q - quit");
                    ff.data = 2;
                }
                else if(ch == 'e' | ch == 'E')
                {
                    printf("user input e - execute");
                    ff.data = 3;
                }

                if(ff.data > 0)
                {
                    input_pub.publish(ff);
                    inputFlag = false;
                }
            }
        }
        r.sleep();
        ros::spinOnce();
    } cout << endl;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionGoToPos(Vector3d action_pos, Quaterniond action_quat, bool plan_only, string action_name, bool replan, string link_name)
{ //right arm only

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac(action_name, true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = link_name;
    target.plan_only = plan_only;
    target.replan = replan;

    std::cout << "ActionGoToPos" << std::endl;
    std::cout << "==========================" << std::endl;
    sleep(sleep_time);
    
    // Pre-positioning
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);

    target.end_eff.poses[0].orientation.w = action_quat.w();
    target.end_eff.poses[0].orientation.x = action_quat.x();
    target.end_eff.poses[0].orientation.y = action_quat.y();
    target.end_eff.poses[0].orientation.z = action_quat.z();

    target.end_eff.poses[0].position.x = action_pos(0);
    target.end_eff.poses[0].position.y = action_pos(1);
    target.end_eff.poses[0].position.z = action_pos(2);

    if(!replan)
    {
        ac.sendGoal(target);
        ac.waitForResult(ros::Duration(20.0));
    }
    else
    {    
        ac.sendGoal(target, 
                    boost::bind(&Manipulate::doneCb, this, _1, _2),
                    boost::bind(&Manipulate::activeCb, this),
                    boost::bind(&Manipulate::feedbackCb, this, _1)
                    );
        checkUserInput();
    }
    result = ac.getResult();
    sleep(sleep_time);

    if (result->error_code == result->FAIL)
    {
        cout << "Motion FAILED!\n";
        return -1;
    }
    else
    {
        cout << "Motion DONE. \n";
    }

    std::cout << "==========================\n"
              << std::endl;
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionGoToPos(Affine3d this_pose, bool plan_only, string action_name, bool replan, string link_name)
{ //right arm only

    std::cout << "ActionGoToPos" << std::endl;
    std::cout << "==========================" << std::endl;

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac(action_name, true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = link_name;
    target.plan_only = plan_only;
    target.replan = replan;

    geometry_msgs::Pose my_pose;
    tf::poseEigenToMsg(this_pose, my_pose);

    cout << this_pose.matrix() << endl;

    // Pre-positioning
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);

    target.end_eff.poses[0].orientation.w = my_pose.orientation.w;
    target.end_eff.poses[0].orientation.x = my_pose.orientation.x;
    target.end_eff.poses[0].orientation.y = my_pose.orientation.y;
    target.end_eff.poses[0].orientation.z = my_pose.orientation.z;

    target.end_eff.poses[0].position.x = my_pose.position.x;
    target.end_eff.poses[0].position.y = my_pose.position.y;
    target.end_eff.poses[0].position.z = my_pose.position.z;
    
    if(!replan)
    {
        ac.sendGoal(target);
        ac.waitForResult(ros::Duration(20.0));
    }
    else
    {    
        ac.sendGoal(target, 
                    boost::bind(&Manipulate::doneCb, this, _1, _2),
                    boost::bind(&Manipulate::activeCb, this),
                    boost::bind(&Manipulate::feedbackCb, this, _1)
                    );
        checkUserInput();
    }
    result = ac.getResult();
    sleep(sleep_time);

    if (result->error_code == result->FAIL)
    {
        cout << "Motion FAILED!\n";
        return -1;
    }
    else
    {
        cout << "Motion DONE. \n";
    }

    std::cout << "==========================\n"
              << std::endl;
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionGoToPos(std::vector<Affine3d> these_poses, bool plan_only, string action_name, bool replan, string link_name)
{ 
    //right arm only
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac(action_name, true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = link_name;
    target.replan = replan;

    std::cout << "ActionGoToPos" << std::endl;
    std::cout << "==========================" << std::endl;
    
    // Pre-positioning
    //=============================================
    int Npts = these_poses.size();
    target.end_eff.poses.resize(Npts);
    target.plan_only = plan_only;

    for (int i = 0; i < Npts; i++)
    {
        Affine3d this_pose = these_poses.at(i);
        geometry_msgs::Pose my_pose;
        tf::poseEigenToMsg(this_pose, my_pose);
        cout << "pose [" << i << "]:" <<endl;
        cout << this_pose.matrix() << endl;

        target.end_eff.poses[i].orientation.w = my_pose.orientation.w;
        target.end_eff.poses[i].orientation.x = my_pose.orientation.x;
        target.end_eff.poses[i].orientation.y = my_pose.orientation.y;
        target.end_eff.poses[i].orientation.z = my_pose.orientation.z;

        target.end_eff.poses[i].position.x = my_pose.position.x;
        target.end_eff.poses[i].position.y = my_pose.position.y;
        target.end_eff.poses[i].position.z = my_pose.position.z;
    }
    
    if(!replan)
    {
        ac.sendGoal(target);
        ac.waitForResult(ros::Duration(20.0));
    }
    else
    {    
        ac.sendGoal(target, 
                    boost::bind(&Manipulate::doneCb, this, _1, _2),
                    boost::bind(&Manipulate::activeCb, this),
                    boost::bind(&Manipulate::feedbackCb, this, _1)
                    );
        checkUserInput();
    }
    result = ac.getResult();
    sleep(sleep_time);

    if (result->error_code == result->FAIL)
    {
        cout << "Motion FAILED!\n";
        return -1;
    }
    else
    {
        cout << "Motion DONE. \n";
    }

    std::cout << "==========================\n"
              << std::endl;
    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::ActionGoToPos(std::vector<Affine3d> these_poses, double* fraction, bool plan_only, string action_name, bool replan, string link_name)
{ 
    //right arm only
    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac(action_name, true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = link_name;
    target.replan = replan;

    std::cout << "ActionGoToPos" << std::endl;
    std::cout << "==========================" << std::endl;
    
    // Pre-positioning
    //=============================================
    int Npts = these_poses.size();
    target.end_eff.poses.resize(Npts);
    target.plan_only = plan_only;

    for (int i = 0; i < Npts; i++)
    {
        Affine3d this_pose = these_poses.at(i);
        geometry_msgs::Pose my_pose;
        tf::poseEigenToMsg(this_pose, my_pose);
        cout << "pose [" << i << "]:" <<endl;
        cout << this_pose.matrix() << endl;

        target.end_eff.poses[i].orientation.w = my_pose.orientation.w;
        target.end_eff.poses[i].orientation.x = my_pose.orientation.x;
        target.end_eff.poses[i].orientation.y = my_pose.orientation.y;
        target.end_eff.poses[i].orientation.z = my_pose.orientation.z;

        target.end_eff.poses[i].position.x = my_pose.position.x;
        target.end_eff.poses[i].position.y = my_pose.position.y;
        target.end_eff.poses[i].position.z = my_pose.position.z;
    }
    
    if(!replan)
    {
        ac.sendGoal(target);
        ac.waitForResult(ros::Duration(20.0));
    }
    else
    {    
        ac.sendGoal(target, 
                    boost::bind(&Manipulate::doneCb, this, _1, _2),
                    boost::bind(&Manipulate::activeCb, this),
                    boost::bind(&Manipulate::feedbackCb, this, _1)
                    );
        checkUserInput();
    }
    result = ac.getResult();
    sleep(sleep_time);

    *fraction = result->fraction; //return fraction

    if (result->error_code == result->FAIL)
    {
        cout << "Motion FAILED!\n";
        return -1;
    }
    else
    {
        cout << "Motion DONE. \n";
    }

    std::cout << "==========================\n"
              << std::endl;
    return 1;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//this is for testing going to single hardcoded position only

int Manipulate::ActionGoToPos()
{ //right arm only

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("task_single", true);
    ac.waitForServer(ros::Duration(20.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;
    target.link_name = "ee";

    std::cout << "ActionGoToPos" << std::endl;
    std::cout << "==========================" << std::endl;

        //moveit_ik_solver
        m_moveit_ik.init();

        for(int i = 0; i < 20; i++)
            ros::spinOnce(); //get joint_states from subscriber
        cout << "joint_values: [ ";
        for (int i = 0; i < joint_values.size(); i++)
            cout << joint_values[i] << " ";
        cout << "] " << endl;

        m_moveit_ik.init_home(joint_values);
        m_moveit_ik.getJointLimits();
        
    Matrix4d pos;
    Affine3d aff;
    pos <<  0, 1,  0,  0.80,
            1, 0,  0, -0.18,
            0, 0, -1,  0.79,
            0, 0,  0,  1;
    aff.matrix() = pos;

    Matrix4d pos2;
    Affine3d aff2;
    pos2 <<  0, 1,  0,  0.75,
            1, 0,  0, -0.18,
            0, 0, -1,  0.79,
            0, 0,  0,  1;
    aff2.matrix() = pos2;

    Matrix4d pos3;
    Affine3d aff3;
    pos3 <<  0, 1,  0,  0.25,
            1, 0,  0, -0.25,
            0, 0, -1,  0.79,
            0, 0,  0,  1;
    aff3.matrix() = pos3;

    Matrix4d pos4;
    Affine3d aff4;
    pos4 <<  0, 1,  0,  0.15,
            1, 0,  0, -0.18,
            0, 0, -1,  0.79,
            0, 0,  0,  1;
    aff4.matrix() = pos4;

        moveit_msgs::RobotState rs;
        robotStateToRobotStateMsg( *m_moveit_ik.kinematic_state, rs);
        drs.state = rs;
        display_rs_publisher.publish(drs);
        ros::spinOnce();
        cin.ignore();

        cout << "-----AFF1:" << endl;
        if( m_moveit_ik.ik_solve(aff) )
        {
            ROS_INFO("\nIK1 OKAY!");
            //m_moveit_ik.reset();
        }
        else
        {
            ROS_ERROR("\nIK1 NOT RIGHT\n");
        }

        robotStateToRobotStateMsg( *m_moveit_ik.kinematic_state, rs);
        drs.state = rs;
        display_rs_publisher.publish(drs);
        ros::spinOnce();
        cin.ignore();

        cout << "------AFF2:" << endl;
        if( m_moveit_ik.ik_solve(aff2) )
        {
            ROS_INFO("\nIK2 OKAY!");
            //m_moveit_ik.reset();
        }
        else
        {
            ROS_ERROR("\nIK2 NOT RIGHT\n");
        }

        robotStateToRobotStateMsg( *m_moveit_ik.kinematic_state, rs);
        drs.state = rs;
        display_rs_publisher.publish(drs);
        ros::spinOnce();
        cin.ignore();

        cout << "-----AFF3:" << endl;
        if( m_moveit_ik.ik_solve(aff3) )
        {
            ROS_INFO("\nIK3 OKAY!");
            //m_moveit_ik.reset();
        }
        else
        {
            ROS_ERROR("\nIK3 NOT RIGHT\n");
        }

        robotStateToRobotStateMsg( *m_moveit_ik.kinematic_state, rs);
        drs.state = rs;
        display_rs_publisher.publish(drs);
        ros::spinOnce();
        cin.ignore();

        cout << "----AFF4:" << endl;
        if( m_moveit_ik.ik_solve(aff4) )
        {
            ROS_INFO("\nIK4 OKAY!");
            //m_moveit_ik.reset();
        }
        else
        {
            ROS_ERROR("\nIK4 NOT RIGHT\n");
        }

        robotStateToRobotStateMsg( *m_moveit_ik.kinematic_state, rs);
        drs.state = rs;
        display_rs_publisher.publish(drs);
        ros::spinOnce();
        
    //=============================================
    int Npts = 1;
    target.end_eff.poses.resize(Npts);

    Quaterniond Q(aff.linear());
    Vector3d trans_temp = aff.translation();

    target.end_eff.poses[0].orientation.w = Q.w();
    target.end_eff.poses[0].orientation.x = Q.x();
    target.end_eff.poses[0].orientation.y = Q.y();
    target.end_eff.poses[0].orientation.z = Q.z();

    target.end_eff.poses[0].position.x = trans_temp(0); //ready
    target.end_eff.poses[0].position.y = trans_temp(1);
    target.end_eff.poses[0].position.z = trans_temp(2); //- 0.306 - 0.123

    std::cout << "==========================\n" << std::endl;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::addTableCollision()
{
    table_collision.primitives.clear();
    table_collision.primitive_poses.clear();

    //Publish table as collision object
    //============================================
    table_collision.header.frame_id = "world";

    //    /* The id of the object is used to identify it. */
    table_collision.id = "table";

    //table top
    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.2;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 0.05;
    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.7;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.715;
    table_collision.primitives.push_back(primitive);
    table_collision.primitive_poses.push_back(box_pose);

    //1 leg front left
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.04;
    primitive.dimensions[2] = 0.69;
    //geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.95;
    box_pose.position.y =  0.45;
    box_pose.position.z =  0.345;
    table_collision.primitives.push_back(primitive);
    table_collision.primitive_poses.push_back(box_pose);

    //2 leg front right
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.04;
    primitive.dimensions[2] = 0.69;
    //geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.95;
    box_pose.position.y =  -0.45;
    box_pose.position.z =  0.345;
    table_collision.primitives.push_back(primitive);
    table_collision.primitive_poses.push_back(box_pose);

    //3 leg back right
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.04;
    primitive.dimensions[2] = 0.69;
    //geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.45;
    box_pose.position.y =  -0.45;
    box_pose.position.z =  0.345;
    table_collision.primitives.push_back(primitive);
    table_collision.primitive_poses.push_back(box_pose);

    //4 leg back left
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.04;
    primitive.dimensions[2] = 0.69;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.45;
    box_pose.position.y =  0.45;
    box_pose.position.z =  0.345;
    table_collision.primitives.push_back(primitive);
    table_collision.primitive_poses.push_back(box_pose);

    table_collision.operation = table_collision.ADD;

    if (ActionAddObj("table", table_collision) < 0)
    {
        ROS_ERROR_STREAM("failed to add table collision!");
        return -1;
    }

    //-----------table 2--------------------
    table2_collision.header.frame_id = "world";
    table2_collision.id = "table2";
    
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.2;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 0.05;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.7;
    box_pose.position.y = -1.0;
    box_pose.position.z = 0.715;
    table2_collision.primitives.push_back(primitive);
    table2_collision.primitive_poses.push_back(box_pose);

    table2_collision.operation = table2_collision.ADD;

    if (ActionAddObj("table2", table2_collision) < 0)
    {
        ROS_ERROR_STREAM("failed to add table2 collision!");
        return -1;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::addPuckCollision(double radius)
{
    puck_collision.primitives.clear();
    puck_collision.primitive_poses.clear();

    puck_collision.header.frame_id = "world";
    puck_collision.id = "puck";

    Affine3d obj_affine;
    tf::poseMsgToEigen(object_in_base.pose, obj_affine);
    Vector3d T1 = obj_affine.translation();

    //puck object
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.14;
    primitive.dimensions[1] = radius;
    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;

    box_pose.position.x = T1(0);
    box_pose.position.y = T1(1);
    box_pose.position.z = T1(2) + 0.01 - 0.02;
    puck_collision.primitives.push_back(primitive);
    puck_collision.primitive_poses.push_back(box_pose);

    puck_collision.operation = puck_collision.ADD;

    if (ActionAddObj("puck", puck_collision) < 0)
    {
        ROS_ERROR_STREAM("failed to add puck collision!");
        return -1;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::addRackCollision()
{
    rack_collision.primitives.clear();
    rack_collision.primitive_poses.clear();

    rack_collision.header.frame_id = "world";
    rack_collision.id = "rack";

    Affine3d obj_affine;
    tf::poseMsgToEigen(rack_in_base.pose, obj_affine);
    Vector3d T1 = obj_affine.translation();
    Quaterniond Q1(obj_affine.linear());

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.55;
    primitive.dimensions[1] = 0.70;
    primitive.dimensions[2] = 0.10;
    
    geometry_msgs::Pose box_pose;
    box_pose.orientation.x = Q1.x();
    box_pose.orientation.y = Q1.y();
    box_pose.orientation.z = Q1.z();
    box_pose.orientation.w = Q1.w();

    box_pose.position.x = T1(0);
    box_pose.position.y = T1(1);
    box_pose.position.z = T1(2);
    rack_collision.primitives.push_back(primitive);
    rack_collision.primitive_poses.push_back(box_pose);

    rack_collision.operation = rack_collision.ADD;

    if (ActionAddObj("rack", rack_collision) < 0)
    {
        ROS_ERROR_STREAM("failed to add rack collision!");
        return -1;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::addObstacleCollision(double radius)
{
    obstacle_collision.primitives.clear();
    obstacle_collision.primitive_poses.clear();

    obstacle_collision.header.frame_id = "world";
    obstacle_collision.id = "obstacle";

    Affine3d obst_affine;
    tf::poseMsgToEigen(obstacle_in_base.pose, obst_affine);
    Vector3d T1 = obst_affine.translation();

    //puck object
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.14;
    primitive.dimensions[1] = radius;
    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;

    box_pose.position.x = T1(0);
    box_pose.position.y = T1(1);
    box_pose.position.z = T1(2) + 0.01 - 0.02;
    obstacle_collision.primitives.push_back(primitive);
    obstacle_collision.primitive_poses.push_back(box_pose);

    obstacle_collision.operation = obstacle_collision.ADD;

    // cout << "\nadd puck collision..." << endl;
    if (ActionAddObj("obstacle", obstacle_collision) < 0)
    {
        ROS_ERROR_STREAM("failed to add obstacle_collision!");
        return -1;
    }

    return 1;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int Manipulate::addToolCollision()
{
    tool_collision.meshes.clear();

    tool_collision.header.frame_id = "world";
    tool_collision.id = "tool";

    std::string object_mesh("file:///home/i2r/catkin_ws/tool_model/freecad_tool_45deg/tool_mesh_45deg.dae");
    //adding the mesh
    Vector3d scale(0.001, 0.001, 0.001);
    shapes::Mesh *m = shapes::createMeshFromResource(object_mesh, scale);

    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);

    shape_msgs::Mesh co_mesh;
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

    tool_collision.mesh_poses.resize(1);
    tool_collision.mesh_poses[0].position.x = tool_in_base.pose.position.x;
    tool_collision.mesh_poses[0].position.y = tool_in_base.pose.position.y;
    tool_collision.mesh_poses[0].position.z = tool_in_base.pose.position.z;

    tool_collision.mesh_poses[0].orientation.x = tool_in_base.pose.orientation.x;
    tool_collision.mesh_poses[0].orientation.y = tool_in_base.pose.orientation.y;
    tool_collision.mesh_poses[0].orientation.z = tool_in_base.pose.orientation.z;
    tool_collision.mesh_poses[0].orientation.w = tool_in_base.pose.orientation.w;

    tool_collision.meshes.push_back(co_mesh);

    tool_collision.operation = tool_collision.ADD;

    if (ActionAddObj("tool", tool_collision) < 0)
    {
        ROS_ERROR_STREAM("failed to add tool collision!");
        return -1;
    }

    return 1;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//generic add collision based on multiple mesh files
 int Manipulate::addCollision(vector< string > ids, vector< string > meshfiles, vector< geometry_msgs::Pose > poses)
{
    int size = ids.size();
    if(size < 1)
    {
        ROS_ERROR("no collision object id");
        return -1;
    }
    else if(meshfiles.size() < size)
    {
        ROS_ERROR("not enough meshfiles provided for each obj id");
        return -1;
    }
    else if (poses.size() < size )
    {
        ROS_ERROR("not enough geometry_pose provided for each obj id");
        return -1;
    }
    else
    {
        ROS_INFO_STREAM("size: " << size);
        if( meshfiles.size() != size || poses.size() != size )
            ROS_WARN_STREAM("meshsize/pose size doesnt match id size");    
    }
    
    vector< moveit_msgs::CollisionObject > add_list;
    for(int i = 0; i < size; i++)
    {
        int idx = -1;
        for(int j = 0; j < collision_list.size(); j++){
            if(collision_list.at(j).id == ids.at(i)){
                ROS_WARN_STREAM("collision_object [ " << ids.at(i) << " ] already exist -> overwrite");
                idx = j;
                break;
            }
        }

        moveit_msgs::CollisionObject colli;
        colli.header.frame_id = "world";
        colli.id = ids.at(i);

        //adding the mesh
        std::string object_mesh(meshfiles.at(i));
        Vector3d scale(0.001, 0.001, 0.001);
        shapes::Mesh *m = shapes::createMeshFromResource(object_mesh, scale);
        shape_msgs::Mesh co_mesh;
        shapes::ShapeMsg co_mesh_msg;
        shapes::constructMsgFromShape(m, co_mesh_msg);
        co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);

        colli.meshes.push_back(co_mesh);
        colli.mesh_poses.push_back(poses.at(i));
        colli.operation = colli.ADD;            

        add_list.push_back(colli);  //list for adding to scene

        if(idx < 0)
            collision_list.push_back(colli);    //add to record list
        else
            collision_list.at(idx) = colli;     //replace
    }

    for(int k = 0; k < size; k++)
    {    
        // cout << "\nadd tool collision..." << endl;
        if( ActionAddObj( add_list.at(k).id, add_list.at(k) ) < 0)
        {
            ROS_ERROR_STREAM("failed to add [ " << add_list.at(k).id << " ] collision!");
            return -1;
        }
    }

    return 1;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


int Manipulate::ActionSwitchTF(int num)
{
    monte_carlo::tf_switch srv;
    srv.request.value = num;
    if( client_tf_switch.call(srv) )
    {
        if(srv.response.okay)
            ROS_INFO_STREAM("tf send okay");
        else
            ROS_INFO_STREAM("tf send error");    
    }
    else
    {
        ROS_ERROR("Failed to call service tf_switch");
        return -1;
    }
    return 1;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

void Manipulate::AddMarker(Affine3d pose, string name) //rviz
{
  visualization_msgs::Marker mark_x, mark_y, mark_z;
  mark_x.type = visualization_msgs::Marker::ARROW;

  mark_x.header.frame_id = "/world";
  mark_x.header.stamp = ros::Time();
  marker_id ++;
  mark_x.id = marker_id;
  mark_x.action = visualization_msgs::Marker::ADD;

  mark_x.ns = name + "_x";
  mark_x.scale.x = 0.05;
  mark_x.scale.y = 0.002;
  mark_x.scale.z = 0.002;

  mark_x.color.g = mark_x.color.b = 0.0;
  mark_x.color.r = mark_x.color.a = 1.0;  //set color to red

  mark_y = mark_z = mark_x;

  mark_y.ns = name + "_y";
  marker_id++;
  mark_y.id = marker_id;

  mark_y.color.r = mark_y.color.b = 0.0;
  mark_y.color.g = mark_y.color.a = 1.0;  //set color to green

  mark_z.ns = name + "_z";
  marker_id++;
  mark_z.id = marker_id;

  mark_z.color.g = mark_z.color.r = 0.0;
  mark_z.color.b = mark_z.color.a = 1.0;  //set color to green

  geometry_msgs::Pose pose_tf, pose_tf_y, pose_tf_z;
  tf::poseEigenToMsg(pose, pose_tf);
  mark_x.pose = pose_tf;
  
  Affine3d pose_y, pose_z;
  pose_y =  pose * create_affine(0,0,M_PI/2.0,0,0,0);
  tf::poseEigenToMsg(pose_y, pose_tf_y);
  mark_y.pose = pose_tf_y;
  
  pose_z =  pose * create_affine(0,-M_PI/2.0,0,0,0,0);
  tf::poseEigenToMsg(pose_z, pose_tf_z);
  mark_z.pose = pose_tf_z;
  
  marker_List.markers.push_back(mark_x);
  marker_List.markers.push_back(mark_y);
  marker_List.markers.push_back(mark_z);
}

void Manipulate::publishMarker() //rviz
{
  ROS_WARN_STREAM(">> publishing rviz markers...");
  markers_pub.publish(marker_List);
  ros::spinOnce();
  sleep(sleep_time);
  ROS_INFO_STREAM(">> published rviz markers DONE...");
}

void Manipulate::clearMarker() //rviz
{
  marker_List.markers.clear();
    
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker_List.markers.push_back(mark);

  ROS_WARN_STREAM(">> clearing rviz markers...");
  markers_pub.publish(marker_List);
  ros::spinOnce();
  sleep(1);
  ROS_WARN_STREAM(">> spin done... ");
}