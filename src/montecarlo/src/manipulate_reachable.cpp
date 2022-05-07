#include <tool_expt/manipulate.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace Eigen;


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

bool Manipulate::EvReachable(bool PICK_FLAG) 
{

    ROS_ERROR_STREAM("DOING DIRECT PICK AND PLACE!");

    WAIST_INIT = -30.0;
    WAIST_TOOL = 60.0;

    OBST_RADIUS = 0.03;
    PUCK_RADIUS = 0.04;

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
        ActionSwitchTF(1);
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



    //clean up
	//remove puck collision
    cout << "\nremove puck collision..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    ActionRmObj("puck");
    ROS_INFO_STREAM("sleeping");
    sleep(3.0);



    //--- main here ---
    if( findObjectTargetLocation2_left() > 0 )
    {
        addTableCollision();
        cout << "Collsion added..." << endl << endl;

    	display_poses_matrix2();

    	//obtained angle and radian of obj2tar
        cout << "get TF for l_ee... DONT MOVE" << endl;
        ROS_WARN_STREAM("press enter to continue...");
        cin.ignore();

        Affine3d l_ee_affine;
        geometry_msgs::PoseStamped l_ee_pose;
        getCurrentTransform("l_ee", l_ee_pose);
        tf::poseMsgToEigen(l_ee_pose.pose, l_ee_affine);

        Vector3d ee_vec = l_ee_affine.translation();
        Vector3d obj_vec = obj_TF.translation();
		Vector3d tar_vec = tar_TF.translation();

	    //T_obj_task (direction)
	    Vector3d obj_task = obj_vec - ee_vec;   //task_dir
	    double angle_obj_task = atan2( obj_task(1), obj_task(0) ); //y/x
	    Affine3d T_obj_task = create_affine(0, 0, angle_obj_task,0,0,0);  
	    cout << "T_obj_task: \n" << T_obj_task.matrix() << endl;


	    //T_obj_task (direction)
	    Vector3d tar_task = tar_vec - ee_vec;   //task_dir
	    double angle_tar_task = atan2( tar_task(1), tar_task(0) ); //y/x
	    Affine3d T_tar_task = create_affine(0, 0, angle_tar_task,0,0,0);  
	    cout << "T_tar_task: \n" << T_tar_task.matrix() << endl;

		//functionality
		double hand_size = 0.5*0.08 + 0.025;
		double gap_size = 0.03;

		Vector3d objpos( PUCK_RADIUS + hand_size + gap_size, 0.01, -0.01);
	    double angle_func = atan2( 0, 1 );
	    Affine3d T_task_func = create_affine(0, 0, angle_func, -objpos(0), -objpos(1), objpos(2));
	    // cout << "T_task_func: \n" << T_task_func.matrix() << endl;
	    
	    Vector3d tarpos( PUCK_RADIUS + hand_size, 0.01, 0.01);
	    Affine3d T_task_func_2 = create_affine(0, 0, angle_func, -tarpos(0), -tarpos(1), tarpos(2));
	    
	    cout << "functionality done \n" << endl;

        Affine3d T_obj_func = T_obj_task * T_task_func;
        Affine3d T_world_obj = obj_TF;
        Affine3d T_world_func = T_world_obj * T_obj_func;
        
        Affine3d T_hf_x = create_affine( M_PI/2.0, 0, 0, 0, 0, 0);      //x by 90
        Affine3d T_hf_z = create_affine( 0, 0, M_PI/2.0, 0, 0, 0);      //z by 90                    
        Affine3d T_hand_func = T_hf_x * T_hf_z;

        Affine3d T_world_hand = T_world_func * T_hand_func.inverse();
    

        Affine3d T_tar_func = T_tar_task * T_task_func_2;
        Affine3d T_world_tar = tar_TF;
        Affine3d T_world_func_2 = T_world_tar * T_tar_func;
		Affine3d T_world_hand_2 = T_world_func_2 * T_hand_func.inverse();
    	Affine3d end_pose = T_world_hand_2;

		Affine3d return_pose = end_pose;
		Vector3d temp_watever = end_pose.translation();
    	temp_watever(2) += 0.15;  //set z to raise higher
		return_pose.translation() = temp_watever;

    	vector< Affine3d > goPoses;
    	goPoses.push_back( T_world_hand );
    	goPoses.push_back( end_pose );


        for(int g = 0; g < goPoses.size(); g++)
        {
            Affine3d this_loci_pose = goPoses.at(g); //world to tool * tool to grab * grab to hand orientation * hand orient to atk
            AddMarker( this_loci_pose, "goPoses_"+to_string(g)) ;
        }
        publishMarker();

	    bool failed = false;

        bool PLAN_ONLY_FLAG = true;
        cout << "check planning _ press enter" << endl; cin.ignore();
	    if( ActionGoToPos( goPoses, PLAN_ONLY_FLAG, "task_single_right", true, "l_ee") < 1 )
	    {
	    	sleep(sleep_time);
        	ROS_ERROR_STREAM("Failed");       
        	
	        failed = true;
	    }
	    sleep(sleep_time);


	    //----------------start experiment-------------------
	    if(!failed)
	    {
	    	cout << "PLANNING OKAY!" << endl;
        	addPuckCollision(PUCK_RADIUS);

		    //open fingers
	        cout << "open fingers left" << endl;
		    ROS_INFO_STREAM("Press anykey to continue...");
		    cin.ignore();
		    OpenFingers("left", 0);
		    sleep(sleep_time);


		    //go to grab position
	        cout << "go to grabbing _ " << endl; cin.ignore();
		    if( ActionGoToPos( goPoses.at(0), false, "task_single_right", true, "l_ee") < 1 )
		    {
		    	sleep(sleep_time);
	        	ROS_ERROR_STREAM("Failed");       
		        failed = true;
		    }
		    sleep(sleep_time);

		    if(!failed)
		    {
			    if(PICK_FLAG)
			    {
				    //attach puck
				    cout << "attach puck planner" << endl; 
				    ROS_INFO_STREAM("Press anykey to continue...");
				    cin.ignore();
				    ActionAttachObj("left", "puck", puck_collision);

				    //grab close fingers
			        cout << "close fingers left" << endl;
				    ROS_INFO_STREAM("Press anykey to continue...");
				    cin.ignore();
				    #ifdef USE_FINGER_ENC
			            powerFingers("left", 8.0, 1); //side, diameter, mode
				        sleep(sleep_time);
				    #else
				        MoveFingers("left", 0.60, 0); //side, %tage, mode
				        sleep(sleep_time);
				    #endif

				    //homing    
				    cout << "homing! press enter to continue:_" << endl; cin.ignore();
				    #ifdef USE_LOWER_TORSO
				        Home2(WAIST_INIT);
				    #else
				        Home(WAIST_INIT);
				    #endif
				    sleep(sleep_time);
				}
			

			    //go to target position
		        cout << "go to end pose _ " << endl; cin.ignore();
			    if( ActionGoToPos( goPoses.at(1), false, "cart_single_right", true, "l_ee") < 1 )
			    {
			    	sleep(sleep_time);
		        	ROS_ERROR_STREAM("Failed");       
			        failed = true;
			    }
			    sleep(sleep_time);
			}

		    if(!failed)
		    {
		    	if(PICK_FLAG)
		    	{
				    //open fingers
			        cout << "open fingers left" << endl;
				    ROS_INFO_STREAM("Press anykey to continue...");
				    cin.ignore();
				    OpenFingers("left", 0);
				    sleep(sleep_time);


				    //detach
			        cout << "detach puck planner" << endl; 
			        ROS_INFO_STREAM("Press anykey to continue...");
			        cin.ignore();
			        ActionDetachObj("left", "puck");
			    }

	            object_in_base.pose.position.x = tar_vec(0);
	            object_in_base.pose.position.y = tar_vec(1);
	            object_in_base.pose.position.z = tar_vec(2);
	            addPuckCollision(PUCK_RADIUS);
	            sleep(5.0);  

		        //return
		        cout << "go to return pose _ " << endl; cin.ignore();
			    if( ActionGoToPos( return_pose, false, "task_single_right", true, "l_ee") < 1 )
			    {
			    	sleep(sleep_time);
		        	ROS_ERROR_STREAM("Failed");       
			        failed = true;
			    }
			    sleep(sleep_time);
		    }

		    //homing    
		    cout << "homing! press enter to continue:_" << endl; cin.ignore();
		    #ifdef USE_LOWER_TORSO
		        Home2(WAIST_INIT);
		    #else
		        Home(WAIST_INIT);
		    #endif
		    sleep(sleep_time);

		    cout << "close fingers left" << endl;
		    ROS_INFO_STREAM("Press anykey to continue...");
		    cin.ignore();
		    CloseFingers("left", 0, 0);
		    sleep(sleep_time);

		    cout << "Finish expt~!" << endl;
		    cout << "-------------------------------------------------------" << endl;
    	}
    }

    return 1;
}




//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

bool Manipulate::EvReachable2()
{
    ROS_ERROR_STREAM("DOING PUSH!");

    WAIST_INIT = -30.0;
    WAIST_TOOL = 60.0;

    OBST_RADIUS = 0.03;
    PUCK_RADIUS = 0.04;

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
        ActionSwitchTF(1);
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


    //clean up
	//remove puck collision
    cout << "\nremove puck collision..." << endl;
    ROS_INFO_STREAM("Press anykey to continue...");
    cin.ignore();
    ActionRmObj("puck");
    ROS_INFO_STREAM("sleeping");
    sleep(3.0);


    //--- main here ---
    if( findObjectTargetLocation2_left() > 0 )
    {
        addTableCollision();
        cout << "Collsion added..." << endl << endl;
    	display_poses_matrix2();


        Vector3d obj_vec = obj_TF.translation();
		Vector3d tar_vec = tar_TF.translation();


	    //T_task (direction)
	    Vector3d task_vec = tar_vec - obj_vec;   //task_dir
	    double angle_task = atan2( task_vec(1), task_vec(0) ); //y/x
	    Affine3d T_task = create_affine(0, 0, angle_task,0,0,0);  
	    cout << "T_task: \n" << T_task.matrix() << endl;

		//functionality
		double hand_size = 0.5 * 0.10;	//horizontal
		double gap_size = 0.03;
		
		Vector3d objpos( PUCK_RADIUS + hand_size + gap_size, 0, -0.01);
	    double angle_func = atan2( 0, 1 );
	    Affine3d T_task_func = create_affine(0, 0, angle_func, -objpos(0), -objpos(1), objpos(2));
	    Vector3d tarpos( PUCK_RADIUS + hand_size, 0, 0.0);
	    Affine3d T_task_func_2 = create_affine(0, 0, angle_func, -tarpos(0), -tarpos(1), tarpos(2));
	    
	    cout << "functionality done \n" << endl;

        Affine3d T_obj_func = T_task * T_task_func;
        Affine3d T_world_obj = obj_TF;
        Affine3d T_world_func = T_world_obj * T_obj_func;
        
        //forehand
        Affine3d T_hf_x = create_affine( M_PI/2.0, 0, 0, 0, 0, 0);      //x by 90               
        Affine3d T_hand_func = T_hf_x;

        //backhand
		Affine3d T_hf_z = create_affine( 0, 0, M_PI, 0, 0, 0);      //z by 180               
		Affine3d T_hand_func_2 = T_hf_x * T_hf_z;



		//forehand
        Affine3d T_world_hand = T_world_func * T_hand_func.inverse();
        Affine3d T_tar_func = T_task * T_task_func_2;
        Affine3d T_world_tar = tar_TF;
        Affine3d T_world_func_2 = T_world_tar * T_tar_func;
		Affine3d T_world_hand_2 = T_world_func_2 * T_hand_func.inverse();
    	Affine3d end_pose = T_world_hand_2;

		Affine3d return_pose = end_pose;
		Vector3d temp_watever = end_pose.translation();
    	temp_watever(2) += 0.15;  //set z to raise higher
		return_pose.translation() = temp_watever;

    	vector< Affine3d > goPoses;
    	goPoses.push_back( T_world_hand );
    	goPoses.push_back( end_pose );

	    bool failed = false;

        bool PLAN_ONLY_FLAG = true;
        cout << "check planning _ press enter" << endl; cin.ignore();
	    if( ActionGoToPos( goPoses, PLAN_ONLY_FLAG, "task_single_right", true, "l_ee") < 1 )
	    {
	    	sleep(sleep_time);
        	ROS_ERROR_STREAM("Failed");       
        	
	        failed = true;
	    }
	    sleep(sleep_time);


	    if(failed)	//try back hand
	    {
	    	failed = false;
	    	goPoses.clear();

	    	//functionality
			hand_size += 0.025;	//horizontal
			
			objpos = Vector3d( PUCK_RADIUS + hand_size + gap_size, 0, -0.01);
		    T_task_func = create_affine(0, 0, angle_func, -objpos(0), -objpos(1), objpos(2));
		    tarpos = Vector3d( PUCK_RADIUS + hand_size, 0, 0.0);
		    T_task_func_2 = create_affine(0, 0, angle_func, -tarpos(0), -tarpos(1), tarpos(2));
		    

			//backhand
	         T_world_hand = T_world_func * T_hand_func_2.inverse();
	         T_tar_func = T_task * T_task_func_2;

	         T_world_tar = tar_TF;
	         T_world_func_2 = T_world_tar * T_tar_func;
			 T_world_hand_2 = T_world_func_2 * T_hand_func_2.inverse();
	    	 end_pose = T_world_hand_2;

			return_pose = end_pose;
			temp_watever = end_pose.translation();
	    	temp_watever(2) += 0.15;  //set z to raise higher
			return_pose.translation() = temp_watever;

	    	goPoses.push_back( T_world_hand );
	    	goPoses.push_back( end_pose );

	        cout << "check planning _ press enter" << endl; cin.ignore();
		    if( ActionGoToPos( goPoses, PLAN_ONLY_FLAG, "task_single_right", true, "l_ee") < 1 )
		    {
		    	sleep(sleep_time);
	        	ROS_ERROR_STREAM("Failed");       
	        	
		        failed = true;
		    }
		    sleep(sleep_time);
	    }

	    //----------------start experiment-------------------
	    if(!failed)
	    {
	    	cout << "PLANNING OKAY!" << endl;
	        for(int g = 0; g < goPoses.size(); g++)
	        {
	            Affine3d this_loci_pose = goPoses.at(g); //world to tool * tool to grab * grab to hand orientation * hand orient to atk
	            AddMarker( this_loci_pose, "goPoses_"+to_string(g)) ;
	        }
	        publishMarker();

			addPuckCollision(PUCK_RADIUS);

		    //go to grab position
	        cout << "go to grabbing _ " << endl; cin.ignore();
		    if( ActionGoToPos( goPoses.at(0), false, "task_single_right", true, "l_ee") < 1 )
		    {
		    	sleep(sleep_time);
	        	ROS_ERROR_STREAM("Failed");       
		        failed = true;
		    }
		    sleep(sleep_time);


		    if(!failed)
		    {
		    	//remove puck collision
			    cout << "\nremove puck collision..." << endl;
			    ROS_INFO_STREAM("Press anykey to continue...");
			    cin.ignore();
			    if (ActionRmObj("puck") < 0)
			    {
			        ROS_ERROR_STREAM("Cant remove puck collision -> quitting\n");
			        
			        cout << "homing" << endl;
			        ROS_INFO_STREAM("Press anykey to continue...");
			        cin.ignore();
			        
			        failed = true;
			    }
			    ROS_INFO_STREAM("sleeping");
			    sleep(3.0);
			}
			 
			if(!failed)
			{
			    //go to target position
		        cout << "go to end pose _ " << endl; cin.ignore();
			    if( ActionGoToPos( goPoses.at(1), false, "cart_single_right", true, "l_ee") < 1 )
			    {
			    	sleep(sleep_time);
		        	ROS_ERROR_STREAM("Failed");       
			        failed = true;
			    }
			    sleep(sleep_time);
	    	}

		    if(!failed)
		    {
	            object_in_base.pose.position.x = tar_vec(0);
	            object_in_base.pose.position.y = tar_vec(1);
	            object_in_base.pose.position.z = tar_vec(2);
	            addPuckCollision(PUCK_RADIUS);
	            sleep(5.0);    

		        //return
		        cout << "go to return pose _ " << endl; cin.ignore();
			    if( ActionGoToPos( return_pose, false, "task_single_right", true, "l_ee") < 1 )
			    {
			    	sleep(sleep_time);
		        	ROS_ERROR_STREAM("Failed");       
			        failed = true;
			    }
			    sleep(sleep_time);
		    }

		    //homing    
		    cout << "homing! press enter to continue:_" << endl; cin.ignore();
		    #ifdef USE_LOWER_TORSO
		        Home2(WAIST_INIT);
		    #else
		        Home(WAIST_INIT);
		    #endif
		    sleep(sleep_time);

		    cout << "Finish expt~!" << endl;
		    cout << "-------------------------------------------------------" << endl;
    	}
    }

    return 1;
}

