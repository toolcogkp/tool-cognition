#include <tool_expt/search.h>

using namespace std;
using namespace Eigen;


double cindex;

MCT_Search::MCT_Search()
{
    paramFlag = false;
    perceptFlag = false;

    m_create_tool = NULL;
    m_moveit_ik = NULL;
    m_kdl_ik = NULL;

    rec.setName("mcts");
}

void MCT_Search::init(Create_Tool *create_tool, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik)
{
    m_create_tool = create_tool;
    m_moveit_ik = moveit_ik;
    m_kdl_ik = kdl_ik;
}

void MCT_Search::init_params(int num_grasp, int num_atk_angle, int plays)
{
    N_grasp = num_grasp;
    N_atk_angle = num_atk_angle;
    if(N_atk_angle <= 0)
        N_atk_angle = 1;

    Nplays = plays;

    paramFlag = true;
    //display
	cout << "param initialised: " << endl;
    cout << "N_grasp=" << N_grasp 
         << ", N_atk_angle=" << N_atk_angle 
         << endl;
}

Affine3d MCT_Search::create_affine(double roll, double pitch, double yaw, double x, double y, double z) 
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

void MCT_Search::monte_carlo_tree()
{
    cout << "--------------------------------------------" << endl;
    cout << "creating tree..." << endl;
    S_tree.clear();
    P_tree.clear();
    A_tree.clear();
    G_tree.clear();

    TWT.clear();
    TWG.clear();


    #ifdef HARDCODE
        // cindex = 0;
        cindex = N_grasp - 1;
        N_grasp = 1;
    #endif

    //create P_tree
    for( int h = 0; h < N_grasp; h++)
    {
        vector< vector< vector< MCT_NODE >>> pi_vec;
        vector< vector< vector< vector< Affine3d >>>> TWG_i;
    
        for( int i = 0; i < N_atk_angle; i++)
        {
            vector< vector< MCT_NODE >> pj_vec;
            vector< vector< vector< Affine3d >>> TWG_j;
            
            //TODO CHANGE GRASP LOCI TO AFFINE
            Affine3d T_grasp, T_atk_angle, T_tool_grasp;
            
            #ifdef HARDCODE
                T_grasp = grasp_loci.at(cindex);
            #else
                T_grasp = grasp_loci.at(h);
            #endif

            Vector3d trans = T_grasp.translation();
            T_atk_angle = create_affine(0, 0, atk_angle(i), 0, 0, 0);
            T_tool_grasp = T_grasp * T_atk_angle; //rotate by atk_angle but dun change translation
            cout << "T_tool_grasp: \n" << T_tool_grasp.matrix() << endl;

            for( int j = 0; j < N_seg; j++)
            {
                vector< MCT_NODE > pk_vec;
                Vector3d nvec = nvec_seg.at(j);
                double angle_nvec = atan2( nvec(1), nvec(0) );  //angle of normal vector on this segment

                vector< vector< Affine3d >> TWG_k;
                vector< vector< Affine3d >> TWT_k;

                for( int k = 0; k < N_pts(j); k++)
                {
                    string pp = "p-" + to_string(h) + "-" + to_string(i) + "-" + to_string(j) + "-" + to_string(k);
                    MCT_NODE temp_p(pp, &rec, m_moveit_ik, m_kdl_ik);
                    cout << "P node: " << temp_p.label << endl;

                    vector< Affine3d > TWG_ipath;
                    vector< Affine3d > TWT_ipath;
                    Affine3d T_tool_func;

                    Affine3d TWT_initial;
                    vector< Affine3d > T_real_grasp;
                    vector< Affine3d > T_world_grasp_save;
                    vector< Affine3d > T_world_tool_save;

                    if(N_pts(j) == 1)   //if no pts between edges, take the middle
                    {
                        Vector3d trans2 = 0.5*( tool_seg.at(j).at(0) + tool_seg.at(j).at(1) );
                        T_tool_func = create_affine(0, 0, angle_nvec, trans2(0), trans2(1), trans2(2));
                    }
                    else    //excude extreme_pts/edges so +1
                    {
                        Vector3d trans2 = tool_seg.at(j).at(k+1);
                        T_tool_func = create_affine(0, 0, angle_nvec, trans2(0), trans2(1), trans2(2));    
                    }

                    bool collision_check = false;
                    for( int ipath = 0; ipath < N_path_pts; ipath++)
                    {
                        Affine3d T_world_tool;
                        T_world_tool = T_world_obj.at(ipath) * T_obj_func.at(ipath) * T_tool_func.inverse();
                        
                        Affine3d T_world_grasp = T_world_tool * T_tool_grasp;

                        Affine3d T_gh_z = create_affine( 0, 0, -M_PI/2.0, 0, 0, 0);      //z by -90
                        Affine3d T_gh_y = create_affine( 0, M_PI, 0, 0, 0, 0);           //y by 180
                        Affine3d T_grasp_hand = T_gh_z * T_gh_y;

                        Affine3d temp = T_world_grasp * T_grasp_hand /* T_atk_angle*/;

                        //raise hand higher by 10cm in z
                        Vector3d temp_tran = temp.translation();
                        temp_tran(2) += 0.10; //take away 4cm  
                        temp.translation() = temp_tran;
                        
                        T_real_grasp.push_back(temp); 
                        T_world_grasp_save.push_back(T_world_grasp);
                        T_world_tool_save.push_back(T_world_tool);
                        
                        cout << "  T_real_grasp[" << ipath << "]: \n" << temp.matrix() << endl;

                        TWT_ipath.push_back(T_world_tool);
                        TWG_ipath.push_back(temp);

                        Shape this_object = object;
                        Affine3d temp2 = T_world_obj.at(ipath);
                        Matrix3d R_temp2 = T_obj_task.rotation();
                        Vector3d temp2_trans = temp2.translation();
                        for(int m = 0; m < this_object.vertices.size(); m++)
                            this_object.vertices.at(m) = ( R_temp2 * object.vertices.at(m) ) + temp2_trans;

                        //rotate the tool to try other orientations
                        Affine3d temp3 = T_world_tool; //T_world_tool
                        Matrix3d R_temp = temp3.rotation();
                        Vector3d T_temp = temp3.translation();

                        bool pieceFlag = false;
                        if(!perceptFlag)
                        {
                            Shape this_subtool1 = subtool1;
                            Shape this_subtool2 = subtool2;
                            for(int n = 0; n < subtool1.vertices.size(); n++)
                                this_subtool1.vertices.at(n) = (R_temp*subtool1.vertices.at(n)) + T_temp;
                            for(int p = 0; p < subtool2.vertices.size(); p++)
                                this_subtool2.vertices.at(p) = (R_temp*subtool2.vertices.at(p)) + T_temp;
                            //ensure no collision at every path points
                            pieceFlag = m_gjk.collision_check(this_subtool1, this_object, max_iter) || m_gjk.collision_check(this_subtool2, this_object, max_iter);
                        }
                        else
                        {
                            vector< Shape > this_tool_pieces = tool_pieces;
                            //with lijun perception
                            for(int n = 0; n < this_tool_pieces.size(); n++)
                            {
                                for(int p =0; p < this_tool_pieces.at(n).vertices.size(); p++)
                                {
                                    this_tool_pieces.at(n).vertices.at(p) = (R_temp*tool_pieces.at(n).vertices.at(p) ) + T_temp;
                                }
                            }//rotate vertices



                            for(int q = 0; q < this_tool_pieces.size(); q++)
                            {   

                                if(  m_gjk.collision_check(this_tool_pieces.at(q), this_object, max_iter) )
                                {
                                    pieceFlag = true;
                                    break;
                                }
                            }//collision check
                        }

                        collision_check = collision_check | pieceFlag;

                        cout << "  p collide[" << ipath << "]: " << pieceFlag << endl;
                    }// end N_path_pts
                    cout << "----------------------------------------" << endl;
                
                    if( N_pts(j) <= SMALL_SEG_THRES)
                        temp_p.small_seg = true;

                    //collision flag for all path point
                    temp_p.collision_obj = collision_check;

                    //keep state of every path points
                    temp_p.state = T_real_grasp;
                    temp_p.state2 = T_world_grasp_save;
                    temp_p.state3 = T_world_tool_save;

                    temp_p.T_tf = T_tool_func;
                    temp_p.T_tg = T_tool_grasp;

                    TWG_k.push_back(TWG_ipath);
                    TWT_k.push_back(TWT_ipath);
                    
                    pk_vec.push_back(temp_p);
                }// end N_pts
                TWG_j.push_back(TWG_k);
                TWT.push_back(TWT_k);
                pj_vec.push_back(pk_vec);
            }// end N_segment
            TWG_i.push_back(TWG_j);
            pi_vec.push_back(pj_vec);
        }// end atk_angle
        TWG.push_back(TWG_i);
        P_tree.push_back(pi_vec);
    }// end grasp
    cout << ".. P_tree done \n";

    //create S tree
    for( int h = 0; h < N_grasp; h++)
    {
        vector< vector< MCT_NODE >> si_vec;
        for( int i = 0; i < N_atk_angle; i++)
        {
            vector< MCT_NODE > sj_vec;
            for( int j = 0; j < N_seg; j++)
            {
                string ss = "s-" + to_string(h) + "-" + to_string(i) + "-" + to_string(j);
                MCT_NODE temp_s(ss, &rec, m_moveit_ik, m_kdl_ik);
                temp_s.num_childs = N_pts(j);

                for(int k = 0; k < N_pts(j); k++)
                {
                    temp_s.childs.push_back( P_tree.at(h).at(i).at(j).at(k) );
                }// end N_pts
                sj_vec.push_back(temp_s);
                cout << "S";
            }// end N_segment
            si_vec.push_back(sj_vec);
        }// end atk_angle
        S_tree.push_back(si_vec);
    }// end grasp
    cout << ".. S_tree done\n";

    //create A_tree
    for( int h = 0; h < N_grasp; h++)
    {
        vector< MCT_NODE > ai_vec;
        for( int i = 0; i < N_atk_angle; i++)
        {
            string aa = "a-" + to_string(h) + "-" + to_string(i);
            MCT_NODE temp_a(aa, &rec, m_moveit_ik, m_kdl_ik);
            temp_a.num_childs = N_seg;

            for( int j = 0; j < N_seg; j++)
            {
                temp_a.childs.push_back( S_tree.at(h).at(i).at(j) );
            }// end N_segment
            ai_vec.push_back(temp_a);
            cout << "A";
        }// end atk_angle
        A_tree.push_back(ai_vec);
    }// end grasp
    cout << ".. A_tree done\n";

    //create G_tree
    for( int h = 0; h < N_grasp; h++)
    {
        string gg = "g-" + to_string(h);
        MCT_NODE temp_g(gg, &rec, m_moveit_ik, m_kdl_ik);
        temp_g.num_childs = N_atk_angle;

        for( int i = 0; i < N_atk_angle; i++)
        {
            temp_g.childs.push_back( A_tree.at(h).at(i) );
        }// end atk_angle
        G_tree.push_back(temp_g);
        cout << "G";
    }// end grasp
    cout << ".. G_tree done\n";

    //create root
    root = MCT_NODE("root", &rec, m_moveit_ik, m_kdl_ik);
    root.num_childs = N_grasp;
    for( int h = 0; h < N_grasp; h++)
    {
        root.childs.push_back( G_tree.at(h) );
    }// end grasp
    cout << "Root done\n";

}

void MCT_Search::monte_carlo_search()
{
    cout << "--------------------------------------------" << endl;
    cout << "performing tree search... Press enter to continue: \n_";
    cin.ignore(); 
    cout << "this will take awhile..." << endl;

    for(int i = 0; i < Nplays; i++)
    {
        if( (i % (int)floor(Nplays/10.0) )==0 )
        {
            cout << "============== play: " << to_string(i) << " ==============" << endl;
        }
        cout << "search no: " << i << endl;
        value = root.update();
        score = root.score;
        cout << "score: " << score << endl;
        cout << "++++++++++++++++++++++++++++" << endl;
    }

    ROS_WARN_STREAM("RESULTS");
    cout << "================ results ======================" << endl;
    cout << "finding node with best score... " << endl;
    
    best.clear();
    result_T_real_grasp.clear();
  
    MCT_NODE *ptr;
    ptr = &root;
    
    while( ptr->num_childs != 0 )
    {
        int best_idx = 0;
        double best_score = 0;
        int size = ptr->num_childs;
        cout << "label: " << ptr->label << endl;

        for(int n = 0; n < size; n++)
        {
            cout << "  child: " << ptr->childs.at(n).label << endl;
            double curr_score = ptr->childs.at(n).score;
            best_score = ptr->childs.at(best_idx).score;
            cout << "    curr score: " << curr_score << endl;
            // cout << "    best score: " << best_score << endl;

            if( curr_score > best_score )
                best_idx = n;
        }    
        best.push_back(best_idx);
        ptr = &(ptr->childs.at(best_idx));
    }cout << endl;
    

    max_score = score;
    result_T_real_grasp = ptr->state;              //return best path points to follow
    
    #ifdef HARDCODE
        result_grab_pos = grasp_loci.at( cindex ); //return best grabbing pos
    #else
        result_grab_pos = grasp_loci.at( best.at(0) ); //return best grabbing pos
    #endif

    result_atk_angle = atk_angle( best.at(1) );    //return best attack angle

    //for display only below:
    cout << "Best grasp position: \n";

    for(int i =0; i < N_path_pts; i++)
    {
        cout << "T_world_tool[" << i << "]: \n";
        cout << ptr->state3.at(i).matrix() << "\n" << endl;
    }cout << "=========================================" << endl;

    for(int i =0; i < N_path_pts; i++)
    {
        cout << "T_tool_func[" << i << "]: \n";
        cout << ptr->T_tf.matrix() << "\n" << endl;
    }cout << "=========================================" << endl;

    for(int i =0; i < N_path_pts; i++)
    {
        cout << "T_tool_grasp[" << i << "]: \n";
        cout << ptr->T_tg.matrix() << "\n" << endl;
    }cout << "=========================================" << endl;

    for(int i =0; i < N_path_pts; i++)
    {
        cout << "T_world_grasp[" << i << "]: \n";
        cout << ptr->state2.at(i).matrix() << "\n" << endl;
    }cout << "=========================================" << endl;

    for(int i =0; i < N_path_pts; i++)
    {
        cout << "T_real_grasp[" << i << "]: \n";
        cout << ptr->state.at(i).matrix() << "\n" << endl;
    }cout << "=========================================" << endl;

    cout << "best idx: [ ";
    for(int i =0; i < best.size(); i++)
        cout << best[i] << " ";
    cout << "]" << endl;

    cout << "best score: " << max_score << endl;
    cout << "=========================================" << endl;

    #ifdef RECORD_START
    cout << "saving to file" << endl;
    rec.save2File();
    rec.clearRecords();
    #endif

    return;
}


//main
void MCT_Search::search()
{
    if(m_create_tool == NULL || ( m_moveit_ik == NULL && m_kdl_ik == NULL ) || !paramFlag){
        cout << "run init and init_param first" << endl;
        return;
    }

    if(!perceptFlag)
    {
        //grasp_location
        grasp_loci.clear();
        for(int i=0; i < N_grasp; i++)
        {
            Affine3d temp_t(Translation3d(Vector3d( ((i+1)*0.08), 0, 0 )));
            Matrix4d temp_r;
            temp_r.setIdentity();

            Affine3d loci;
            loci = (temp_t * temp_r).matrix();
            grasp_loci.push_back(loci);
        }

        //attack angle PI
        atk_angle_step = 2.0*M_PI / N_atk_angle;
        atk_angle.resize(N_atk_angle);
        for(int i = 0; i < N_atk_angle; i++)
        {
            atk_angle(i) = (i)*atk_angle_step;
        }
    }
    else
    {
        grasp_loci = m_create_tool->grasp_loci;
        N_grasp = grasp_loci.size();
        
        #ifdef HARDCODE
            N_atk_angle = 1;
            atk_angle_step = 0;
            atk_angle.resize(N_atk_angle);
            atk_angle(0) = atk_angle_step;
        #else
            atk_angle_step = 2.0*M_PI / N_atk_angle;
            atk_angle.resize(N_atk_angle);
            for(int i = 0; i < N_atk_angle; i++)
            {
                atk_angle(i) = (i)*atk_angle_step;
            }
        #endif
        

        if(N_grasp <= 0)
        {
            ROS_ERROR_STREAM("ERROR: NO POSSIBLE GRASP LOCATION FOUND!");
            return;
        }
        else
        {
            ROS_WARN_STREAM("N_grasp = " << N_grasp);
        }
    }
    

    //N_seg & N_points from create_tool
    N_seg = m_create_tool->get_N_seg();         //int                       number of segments 
    N_pts = m_create_tool->get_n_pts();         //VectorXd                  number of points in each seg
    nvec_seg = m_create_tool->get_nvec_seg();   //vector<Vector3d>          normal vector for each seg
    tool_seg = m_create_tool->get_tool_seg();   //vector<vector<Vector3d>>  3d vector of each point in each seg
    angle_seg = m_create_tool->get_angle_seg();


    cout << "N_grasp: " << N_grasp << endl;

    cout << "N_atk_angle: " << N_atk_angle << endl;
    cout << "atk_angle: [" << atk_angle.transpose() << "]\n" <<endl;

    cout << "N_seg: " << N_seg << endl;
    cout << "N_pts: " << N_pts.transpose() << endl;

    ros::Time begin = ros::Time::now();
    monte_carlo_tree();
    ros::Duration elaspsed_time = ros::Time::now() - begin;
    ROS_WARN_STREAM("create MCTS Tree-> time elasped: " << elaspsed_time);

    sleep(0.5);

    ros::Time start = ros::Time::now();
    monte_carlo_search();
    ros::Duration diff = ros::Time::now() - start;
    ROS_WARN_STREAM("MCTS Tree Search-> time elasped: " << diff);

    sleep(0.5);

    return;
}