#include <tool_expt/search2.h>

using namespace std;
using namespace Eigen;


MCT_Search2::MCT_Search2()
{
    paramFlag = false;
    perceptFlag = false;

    m_create_tool = NULL;
    m_moveit_ik = NULL;
    m_kdl_ik = NULL;

    rec.setName("mcts2");
}

void MCT_Search2::init(Create_Tool *create_tool, MOVEIT_IK *moveit_ik, KDL_IK *kdl_ik, bool right)
{
    m_create_tool = create_tool;
    m_moveit_ik = moveit_ik;
    m_kdl_ik = kdl_ik;
    m_right = right;
}

void MCT_Search2::init_params(int num_grasp, int num_atk_angle, int plays)
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

Affine3d MCT_Search2::create_affine(double roll, double pitch, double yaw, double x, double y, double z) 
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

NODE_DATA MCT_Search2::set_ndata(int node_idx[7], bool cflag, bool sflag)
{
    NODE_DATA temp;

    for(int i=0; i<7; i++)
        temp.idx[i] = node_idx[i];
    
    temp.collision_obj = cflag;
    temp.small_seg = sflag;
    return temp;
}

void MCT_Search2::monte_carlo_tree()
{
    cout << "--------------------------------------------" << endl;
    cout << "creating tree..." << endl;
  
    int root_idx[] = {-1, -1, -1, -1, -1, -1, -1};
    NODE_DATA root_ndata = set_ndata(root_idx, false, false);   //node index, collision, small_seg

    root = MCT_NODE2("root", root_ndata, &rec, m_moveit_ik, m_kdl_ik);
    root.num_childs = N_via_candidates;

    MCT_NODE2 *ptr = &root;


    //via_candidates   lvl1
    for( int f = 0; f < N_via_candidates; f++)
    {
        string vv = "v-" + to_string(f);
        int vv_idx[] = {f, -1, -1, -1, -1, -1, -1};
        NODE_DATA vv_ndata = set_ndata(vv_idx, false, false);

        //already done in tool_expt
        vector< Affine3d > T_world_obj = all_T_world_obj.at(f);
	    vector< Affine3d > T_world_func = all_T_world_func.at(f);
	    
        vector< Affine3d > T_obj_func = all_T_obj_func.at(f);
        
        vector< Affine3d > T_world_obj_bb = all_T_world_obj_bb.at(f);
	    vector< Affine3d > T_world_obst_bb = all_T_world_obst_bb.at(f);	
	    
        vector< Vector3d > move_vec = all_move_vec.at(f); 

        //get bounding boxes for object and obstacles
        vector< Shape > all_obj_shape;  //at each path
        vector< Shape > all_obst_shape; //at each path
        for(int ipath = 0; ipath < N_path_pts; ipath++)
        {
            //object
            Shape this_object = object;
            Affine3d A_wo_bb = T_world_obj_bb.at(ipath);
            Matrix3d R_wo_bb = A_wo_bb.rotation();
            Vector3d T_wo_bb = A_wo_bb.translation();
            for(int x = 0; x < this_object.vertices.size(); x++)
                this_object.vertices.at(x) = ( R_wo_bb * this_object.vertices.at(x) ) + T_wo_bb;
            all_obj_shape.push_back(this_object);

            //obstacle
            Shape this_obstacle = obstacle;
            Affine3d A_wost_bb = T_world_obst_bb.at(ipath);
            Matrix3d R_wost_bb = A_wost_bb.rotation();
            Vector3d T_wost_bb = A_wost_bb.translation();
            for(int x = 0; x < this_obstacle.vertices.size(); x++)
                this_obstacle.vertices.at(x) = ( R_wost_bb * this_obstacle.vertices.at(x) ) + T_wost_bb;
            all_obst_shape.push_back(this_obstacle);        
        }//end ipath

        //create node
        ptr->create_child(vv, vv_ndata);
        ptr = ptr->childs.at(f);        //move down
        ptr->num_childs = N_grasp;
        cout << vv << endl;
	
        //grasp lvl2
        for( int h = 0; h < N_grasp; h++)
        {
            string gg = "g-" + to_string(f) + "-" + to_string(h);
            int gg_idx[] = {f, h, -1, -1, -1, -1, -1};
            NODE_DATA gg_ndata = set_ndata(gg_idx, false, false);   

            Affine3d T_grasp;
            T_grasp = grasp_loci.at(h);
            Vector3d trans = T_grasp.translation();    
            
            //create node
            ptr->create_child(gg, gg_ndata);
            ptr = ptr->childs.at(h);        //move down
            ptr->num_childs = N_atk_angle;
            cout << gg << endl;

            //atk_angles    lvl3
            for( int i = 0; i < N_atk_angle; i++)
            {
                string aa = "a-" + to_string(f) + "-" + to_string(h) + "-" + to_string(i);
                // vector<int> aa_idx = {f, h, i, -1, -1};
                int aa_idx[] = {f, h, i, -1, -1, -1, -1};
                NODE_DATA aa_ndata = set_ndata(aa_idx, false, false);
 
                Affine3d T_atk_angle, T_tool_grasp;    
                T_atk_angle = create_affine(0, 0, atk_angle(i), 0, 0, 0);
                T_tool_grasp = T_grasp * T_atk_angle; //rotate by atk_angle but dun change translation

                //create node
                ptr->create_child(aa, aa_ndata);
                ptr = ptr->childs.at(i);        //move down
                ptr->num_childs = N_seg;
                cout << aa << endl;

                //segments     lvl4
                for( int j = 0; j < N_seg; j++)
                {
                    string ss = "s-" + to_string(f) + "-" + to_string(h) + "-" + to_string(i) + "-" + to_string(j);
                    int ss_idx[] = {f, h, i, j, -1, -1, -1};

                    bool smallFlag = false;
                    if( N_pts(j) <= SMALL_SEG_THRES)
                        smallFlag = true;
                    Vector3d nvec = nvec_seg.at(j);
                    double angle_nvec = atan2( nvec(1), nvec(0) );  //angle of normal vector on this segment
                
                    //create node
                    NODE_DATA ss_ndata = set_ndata(ss_idx, false, smallFlag);
                    ptr->create_child(ss, ss_ndata);
                    ptr = ptr->childs.at(j);        //move down
                    ptr->num_childs = N_pts(j);
                    cout << ss << endl;

                    //points    lvl5
                    for( int k = 0; k < N_pts(j); k++)
                    {
                        //fixed variables
                        Affine3d T_gh_z, T_gh_y; 
                        if(m_right)
                        {
                            T_gh_z = create_affine( 0, 0, -M_PI/2.0, 0, 0, 0);      //z by -90
                            T_gh_y = create_affine( 0, M_PI, 0, 0, 0, 0);           //y by 180
                        }
                        else //left
                        {
                            T_gh_z = create_affine( 0, 0, M_PI/2.0, 0, 0, 0);      //z by 90
                            T_gh_y = create_affine( 0, M_PI, 0, 0, 0, 0);           //y by 180
                        }
                        Affine3d T_grasp_hand = T_gh_z * T_gh_y;
                        
                        // cout << "===========================================" << endl;
                        string pp = "p-" + to_string(f) + "-" + to_string(h) + "-" + to_string(i) + "-" + to_string(j) + "-" + to_string(k);
                        // vector<int> pp_idx = {f, h, i, j, k};
                        int pp_idx[] = {f, h, i, j, k, -1, -1};
                        cout << pp << endl;

                        Affine3d T_tool_func;
                        vector< Affine3d > T_real_grasp;
                        vector< Affine3d > T_world_grasp_save;
                        vector< Affine3d > T_world_tool_save;

                        //variables for all 
                        vector< Affine3d > T_world_tool_1st;
                        vector< Affine3d > T_world_grasp_1st;
                        vector< Affine3d > T_world_hand_1st; 

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

                        bool collision_Flag = false;    //overall collision check


                        int path_2_do = 2;
                        if(DOUBLE_SEARCH == 0)
                            path_2_do = N_path_pts;

                        //----------------------first leg only-----------------------
                        bool pieceFlag = false;
                        bool SV_collide_Flag = false;
                        bool hand_collide = false;

                        for( int ipath = 0; ipath < path_2_do; ipath++)
                        {
                            
                            //======================check possible grasp position=================================== 
                            Affine3d T_tool_obj = T_tool_func * T_obj_func.at(ipath).inverse();
                            Vector3d trans3 = T_tool_obj.translation();
                            Vector3d trans_grasp_func = trans - trans3;
                            if( trans_grasp_func.norm() < (GRAB_HAND + PUCK_RADIUS_DEFINE))
                                hand_collide = true;


                            //======================compute the grasp position=================================== 
                            Affine3d T_world_tool = T_world_func.at(ipath) * T_tool_func.inverse();
                            T_world_tool_1st.push_back(T_world_tool);
                            
                            Affine3d T_world_grasp = T_world_tool * T_tool_grasp;
                            T_world_grasp_1st.push_back(T_world_grasp);

                            Affine3d T_world_hand = T_world_grasp * T_grasp_hand /* T_atk_angle*/;
                            Vector3d temp_tran = T_world_hand.translation(); //raise hand higher by 10cm in z
                            temp_tran(2) += 0.06;
                            T_world_hand.translation() = temp_tran;
                            T_world_hand_1st.push_back(T_world_hand);
        

                            //======================compute the SV collision===================================
                            Affine3d A_w_t = T_world_tool; //T_world_tool
                            Matrix3d R_temp = A_w_t.rotation();
                            Vector3d T_temp = A_w_t.translation();

                            if( !(pieceFlag || SV_collide_Flag) )
                            {
                                if(!perceptFlag)    //without perception
                                {
                                    Shape this_subtool1 = subtool1;
                                    Shape this_subtool2 = subtool2;
                                    for(int n = 0; n < subtool1.vertices.size(); n++)   //vertices
                                        this_subtool1.vertices.at(n) = (R_temp*subtool1.vertices.at(n)) + T_temp;
                                    for(int p = 0; p < subtool2.vertices.size(); p++)   //vertices
                                        this_subtool2.vertices.at(p) = (R_temp*subtool2.vertices.at(p)) + T_temp;
                                    //ensure no collision at start
                                    pieceFlag = m_gjk.collision_check(this_subtool1, all_obj_shape.at(ipath), max_iter) || m_gjk.collision_check(this_subtool2, all_obj_shape.at(ipath), max_iter);
                                    // cout << "pieceFlag: =" << pieceFlag << endl;

                                    Shape SVsubtool1 = genSweptVolObj( this_subtool1, move_vec.at(ipath) );
                                    Shape SVsubtool2 = genSweptVolObj( this_subtool2, move_vec.at(ipath) );

                                    //ensure SV no collision with obstacle
                                    SV_collide_Flag = m_gjk.collision_check(SVsubtool1, all_obst_shape.at(ipath), max_iter) || m_gjk.collision_check(SVsubtool2, all_obst_shape.at(ipath), max_iter);
                                    // cout << "SV_flag: =" << SV_collide_Flag << endl;
                                }   
                                else                //with perception
                                {
                                    vector< Shape > this_tool_pieces = tool_pieces;
                                    for(int n = 0; n < this_tool_pieces.size(); n++)    //subtool
                                        for(int p =0; p < this_tool_pieces.at(n).vertices.size(); p++)  //vertices
                                            this_tool_pieces.at(n).vertices.at(p) = (R_temp*tool_pieces.at(n).vertices.at(p) ) + T_temp;
                                    
                                    for(int q = 0; q < this_tool_pieces.size(); q++)
                                    {   
                                        //check collision with object
                                        if(  m_gjk.collision_check(this_tool_pieces.at(q), all_obj_shape.at(ipath), max_iter) )
                                        {
                                            pieceFlag = true;
                                        }

                                        //check SV for collision with obstacle
                                        Shape SVpiece = genSweptVolObj( this_tool_pieces.at(q), move_vec.at(ipath) );
                                        if(  m_gjk.collision_check( SVpiece, all_obst_shape.at(ipath), max_iter) )
                                        {
                                            SV_collide_Flag = true;
                                        }

                                        if( pieceFlag && SV_collide_Flag )
                                            break;
                                    }//collision check
                                }//end if no perception
                            }//end if any flag true then skip check
                        }//end ipath
                        //--------------------------------------end first leg --------------------------------------------
                        collision_Flag = collision_Flag || pieceFlag || SV_collide_Flag || hand_collide;

                        //first leg done
                        NODE_DATA pp_ndata = set_ndata(pp_idx, collision_Flag, smallFlag);                        
                        //save to node (node data)
                        pp_ndata.state = T_real_grasp = T_world_hand_1st;
                        pp_ndata.state2 = T_world_grasp_save = T_world_grasp_1st;   //for checking only
                        pp_ndata.state3 = T_world_tool_save = T_world_tool_1st;    //for checking only
                        
                        pp_ndata.T_tf = T_tool_func;            //not rlly used
                        pp_ndata.T_tg = T_tool_grasp;           //not rlly used

                        ptr->create_child(pp, pp_ndata);
                        ptr = ptr->childs.at(k);                //move down!

                        //optional layers 6 and 7 (only if collision passed!)    
                        if(collision_Flag || N_path_pts <= 2 || DOUBLE_SEARCH == 0 )
                        {
                            //collision failed -> terminate
                            ptr->num_childs = 0;
                        }
                        else
                        {

                            //-----------------------------second leg--------------------------------
                        
                            int current_leg = 2;
                            // cout << "leg: 2" << endl;
                            //---------------------------------------------------------------//
                            //adjacent seg points   lvl6
                            vector< int > this_adj_indices = adj_indices.at(j);
                            ptr->num_childs = this_adj_indices.size();
                            
                            for(int l = 0; l < this_adj_indices.size(); l++)
                            {
                                string ll = "l-" + to_string(f) + "-" + to_string(h) + "-" + to_string(i) + "-" + to_string(j) + "-" + to_string(k) + "-" + to_string(l);
                                int ll_idx[] = {f, h, i, j, k, l, -1};
                                cout << ll << endl;
                                NODE_DATA ll_ndata = set_ndata(ll_idx, false, false);   //node index, collision, small_seg                            
                                
                                int adj_seg_idx = this_adj_indices.at(l);

                                //get number of points in adjacent indices
                                int N_pts2 = 0;
                                if(!obstruct_flag)
                                    N_pts2 = 1;
                                else
                                    N_pts2 = N_pts( adj_seg_idx );

                                bool smallFlag2 = false;
                                if( N_pts(adj_seg_idx) <= SMALL_SEG_THRES)
                                    smallFlag2 = true;
                                Vector3d nvec2 = nvec_seg.at(adj_seg_idx);
                                double angle_nvec2 = atan2( nvec2(1), nvec2(0) );  //angle of normal vector on this segment
                

                                ptr->create_child(ll, ll_ndata);
                                ptr = ptr->childs.at(l);        //move down
                                ptr->num_childs = N_pts2;

                                //points 2   lvl7
                                for(int m = 0; m < N_pts2; m++)
                                {
                                    // cout << "+++++++++++++++++++++++++++++++" << endl;
                                    string mm = "m-" + to_string(f) + "-" + to_string(h) + "-" + to_string(i) + "-" + to_string(j) + "-" + to_string(k) + "-" + to_string(l) + "-" + to_string(m);
                                    int mm_idx[] = {f, h, i, j, k, l, m};
                                    cout << mm << endl;

                                    Affine3d T_tool_func_2;
                                    //get tool 
                                    if(N_pts(adj_seg_idx) == 1)   //if no pts between edges, take the middle
                                    {
                                        Vector3d trans2 = 0.5*( tool_seg.at(adj_seg_idx).at(0) + tool_seg.at(adj_seg_idx).at(1) );
                                        T_tool_func_2 = create_affine(0, 0, angle_nvec2, trans2(0), trans2(1), trans2(2));
                                    }
                                    else    //excude extreme_pts/edges so +1
                                    {
                                        Vector3d trans2 = tool_seg.at(adj_seg_idx).at(m+1);
                                        T_tool_func_2 = create_affine(0, 0, angle_nvec2, trans2(0), trans2(1), trans2(2));    
                                    }
                                    // cout << "T_tool_func: \n" << T_tool_func.matrix() << endl;

                                    //reset collision check
                                    bool collision_Flag = false;
                                    bool pieceFlag = false;
                                    bool SV_collide_Flag = false;
                                    bool hand_collide = false;

                                    //variables for all 
                                    vector< Affine3d > T_world_tool_2nd;
                                    vector< Affine3d > T_world_grasp_2nd;
                                    vector< Affine3d > T_world_hand_2nd; 

                                    T_world_tool_2nd = T_world_tool_1st;
                                    T_world_grasp_2nd = T_world_grasp_1st;
                                    T_world_hand_2nd = T_world_hand_1st;

                                    for(int ipath = 0; ipath < 2; ipath++)
                                    {
                                        int current_path = current_leg + ipath;

                                        //======================check possible grasp position=================================== 
                                        Affine3d T_tool_obj = T_tool_func * T_obj_func.at(current_path).inverse();
                                        Vector3d trans3 = T_tool_obj.translation();
                                        Vector3d trans_grasp_func = trans - trans3;
                                        if( trans_grasp_func.norm() < (GRAB_HAND + PUCK_RADIUS_DEFINE))
                                            hand_collide = true;

                                        //======================compute the grasp position=================================== 
                                        Affine3d T_world_tool_2 = T_world_func.at(current_path) * T_tool_func_2.inverse();
                                        T_world_tool_2nd.push_back(T_world_tool_2);
                                        
                                        Affine3d T_world_grasp_2 = T_world_tool_2 * T_tool_grasp;
                                        T_world_grasp_2nd.push_back(T_world_grasp_2);

                                        Affine3d T_world_hand_2 = T_world_grasp_2 * T_grasp_hand /* T_atk_angle*/;
                                        Vector3d temp_tran = T_world_hand_2.translation(); //raise hand higher by 10cm in z
                                        temp_tran(2) += 0.06; 
                                        T_world_hand_2.translation() = temp_tran;
                                        T_world_hand_2nd.push_back(T_world_hand_2);
                    
                                        //======================compute the SV collision===================================
                                        // Affine3d A_w_t = T_world_tool_1st.at(0); //T_world_tool
                                        Affine3d A_w_t = T_world_tool_2; //T_world_tool
                                        Matrix3d R_temp = A_w_t.rotation();
                                        Vector3d T_temp = A_w_t.translation();

                                        if( !(pieceFlag || SV_collide_Flag) )
                                        {
                                            if(!perceptFlag)    //without perception
                                            {
                                                Shape this_subtool1 = subtool1;
                                                Shape this_subtool2 = subtool2;
                                                for(int n = 0; n < subtool1.vertices.size(); n++)   //vertices
                                                    this_subtool1.vertices.at(n) = (R_temp*subtool1.vertices.at(n)) + T_temp;
                                                for(int p = 0; p < subtool2.vertices.size(); p++)   //vertices
                                                    this_subtool2.vertices.at(p) = (R_temp*subtool2.vertices.at(p)) + T_temp;
                                                //ensure no collision at start
                                                pieceFlag = m_gjk.collision_check(this_subtool1, all_obj_shape.at(current_path), max_iter) || m_gjk.collision_check(this_subtool2, all_obj_shape.at(current_path), max_iter);
                                                // cout << "pieceFlag: =" << pieceFlag << endl;

                                                Shape SVsubtool1 = genSweptVolObj( this_subtool1, move_vec.at(current_path) );
                                                Shape SVsubtool2 = genSweptVolObj( this_subtool2, move_vec.at(current_path) );

                                                //ensure SV no collision with obstacle
                                                SV_collide_Flag = m_gjk.collision_check(SVsubtool1, all_obst_shape.at(current_path), max_iter) || m_gjk.collision_check(SVsubtool2, all_obst_shape.at(current_path), max_iter);
                                                // cout << "SV_flag: =" << SV_collide_Flag << endl;
                                            }   
                                            else                //with perception
                                            {
                                                vector< Shape > this_tool_pieces = tool_pieces;
                                                for(int n = 0; n < this_tool_pieces.size(); n++)    //subtool
                                                    for(int p =0; p < this_tool_pieces.at(n).vertices.size(); p++)  //vertices
                                                        this_tool_pieces.at(n).vertices.at(p) = (R_temp*tool_pieces.at(n).vertices.at(p) ) + T_temp;
                                                
                                                for(int q = 0; q < this_tool_pieces.size(); q++)
                                                {   
                                                    //check collision with object
                                                    if(  m_gjk.collision_check(this_tool_pieces.at(q), all_obj_shape.at(current_path), max_iter) )
                                                    {
                                                        pieceFlag = true;
                                                    }
                                                    
                                                    //check SV for collision with obstacle
                                                    Shape SVpiece = genSweptVolObj( this_tool_pieces.at(q), move_vec.at(current_path) );
                                                    if(  m_gjk.collision_check( SVpiece, all_obst_shape.at(current_path), max_iter) )
                                                    {
                                                        SV_collide_Flag = true;
                                                    }
                                                    
                                                    if( pieceFlag && SV_collide_Flag )
                                                        break;
                                                }//collision check
                                                // cout << "pieceFlag: =" << pieceFlag << endl;
                                                // cout << "SV_flag: =" << SV_collide_Flag << endl;    
                                            }//end if no perception
                                        }//end if any flag true then skip check                                    
                                    }//for each path
                                    //--------------------------------------end first leg --------------------------------------------
                                    collision_Flag = collision_Flag || pieceFlag || SV_collide_Flag || hand_collide;

                                    bool sf = smallFlag || smallFlag2;

                                    NODE_DATA mm_ndata = set_ndata(mm_idx, collision_Flag, sf);   //node index, collision, small_seg                            
                                    
                                        // cout << "T_world_tool size: " << T_world_tool_2nd.size() << endl;
                                        // cout << "T_world_grasp size: " << T_world_grasp_2nd.size() << endl;
                                        // cout << "T_world_hand size: " << T_world_hand_2nd.size() << endl;

                                    mm_ndata.state = T_real_grasp = T_world_hand_2nd;
                                    mm_ndata.state2 = T_world_grasp_save = T_world_grasp_2nd;   //for checking only
                                    mm_ndata.state3 = T_world_tool_save = T_world_tool_2nd;    //for checking only
                                    
                                    mm_ndata.T_tf = T_tool_func_2;            //not rlly used
                                    mm_ndata.T_tg = T_tool_grasp;           //not rlly used

                                    ptr->create_child(mm, mm_ndata);
                                    ptr = ptr->childs.at(m);                //move down!
                                    ptr->num_childs = 0;

                                    // cout << "++++++++++++++++++++++++++++++" << endl;

                                    ptr = ptr->parentNode;//move up
                                    // cout << ptr->label << endl;
                                    // cout << "return l" << endl;
                        
                                }//end lvl 7++ points 2

                                ptr = ptr->parentNode;//move up
                                // cout << ptr->label << endl;
                                // cout << "return k" << endl;
                            }//end lvl 6++ adj indicies
                                
                            //     current_leg = current_leg + 2;
                            // }//end while each leg

                        }//end pp collision check
                        // cout << "===========================================" << endl;
                        
                        ptr = ptr->parentNode;//move up
                        // cout << ptr->label << endl;
                        // cout << "return j" << endl;
                    }// end lvl 5 N_pts
                    ptr = ptr->parentNode;//move up
                    // cout << ptr->label << endl;
                    // cout << "return i" << endl;
                }// end lvl 4 N_segment
                ptr = ptr->parentNode;//move up
                // cout << ptr->label << endl;
                // cout << "return h" << endl;
            }// end lvl 3 atk_angle
            ptr = ptr->parentNode;//move up
            // cout << ptr->label << endl;
            // cout << "return f" << endl;
        }// end lvl 2 grasp
        ptr = ptr->parentNode;//move up
        // cout << ptr->label << endl;
        // cout << "return root" << endl;
    }// end lvl 1 via_pt

    cout << ".. mcts tree done \n";

}

void MCT_Search2::monte_carlo_search()
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
  
    MCT_NODE2 *ptr;
    ptr = &root;
    
    best = ptr->best_idx;
    max_score = ptr->best_score;
    result_T_real_grasp = ptr->best_state;

    //just to print score
    vector<int> best_old;
    while( ptr->num_childs != 0 )
    {
        int best_idx = 0;
        double best_score = 0;
        int size = ptr->num_childs;
        cout << "label: " << ptr->label << endl;

        for(int n = 0; n < size; n++)
        {
            cout << "  child: " << ptr->childs.at(n)->label << endl;
            double curr_score = ptr->childs.at(n)->score;
            best_score = ptr->childs.at(best_idx)->score;
            cout << "    curr score: " << curr_score << endl;
            // cout << "    best score: " << best_score << endl;

            if( curr_score > best_score )
                best_idx = n;
        }    
        best_old.push_back(best_idx);
        ptr = ptr->childs.at(best_idx);
    }cout << endl;
    cout << "old best idx: [ ";
    for(int i =0; i < best_old.size(); i++)
        cout << best_old[i] << " ";
    cout << "]" << endl;


    cout << "best idx: [ ";
    for(int i =0; i < best.size(); i++)
        cout << best[i] << " ";
    cout << "]" << endl;

    cout << "best score: " << max_score << endl;
    cout << "=========================================" << endl;
    
    
    if(max_score > 0 )
    {
        result_via_pt_index = best.at(0);
        #ifdef HARDCODE
            result_grab_pos = grasp_loci.at( cindex ); //return best grabbing pos
        #else
            result_grab_pos = grasp_loci.at( best.at(1) ); //return best grabbing pos
        #endif
        result_atk_angle = atk_angle( best.at(2) );    //return best attack angle
    }
    

    #ifdef RECORD_START
    cout << "saving to file" << endl;
    rec.save2File();
    rec.clearRecords();
    #endif

    return;
}

void MCT_Search2::computeAdjacentSeg()
{
    adj_indices.clear();
    for( int j = 0; j < N_seg; j++)
    {
        vector< int > indices;

        if(!obstruct_flag || SEARCH_ALL==1 )
        {
            for(int i = 0; i < N_seg; i++)
                indices.push_back(i);
        }
        else
        {
            for( int k = -N_ADJACENT; k < N_ADJACENT+1; k++)
            {
                int temp = j + k;
                if(temp < 0)
                    temp += N_seg;
                if(temp >= N_seg)
                    temp -= N_seg;
                indices.push_back(temp);
            }
        }
        adj_indices.push_back(indices);
    }

    cout << "adj_incdices: \n";
    for(int t = 0; t < adj_indices.size(); t++)
    {
        for( int u = 0; u < adj_indices.at(t).size(); u ++ )
        {
            cout << " " << adj_indices.at(t).at(u) << " ";
        }
        cout << "\n";
    }
}

//main
void MCT_Search2::search()
{
    if(m_create_tool == NULL || ( m_moveit_ik == NULL && m_kdl_ik == NULL ) || !paramFlag){
        cout << "run init and init_param first" << endl;
        return;
    }

    if(!perceptFlag)    //no perception
    {
        //hardcoded grasp location

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
    else    //using perception
    {
        //grasp_loci from create_tool
        grasp_loci = m_create_tool->grasp_loci;
        N_grasp = grasp_loci.size();

        //attack angle PI
        // N_atk_angle = 2;    //0 or 180deg
        atk_angle_step = 2.0*M_PI / N_atk_angle;
        atk_angle.resize(N_atk_angle);
        for(int i = 0; i < N_atk_angle; i++)
        {
            atk_angle(i) = (i)*atk_angle_step;
        }

        if(N_grasp == 0)
        {
            ROS_ERROR_STREAM("NO POSSIBLE GRABBING POSITION FOUND!");
            return;
        }
    }
    

    //N_seg & N_points from create_tool
    N_seg = m_create_tool->get_N_seg();         //int                       number of segments 
    N_pts = m_create_tool->get_n_pts();         //VectorXd                  number of points in each seg
    nvec_seg = m_create_tool->get_nvec_seg();   //vector<Vector3d>          normal vector for each seg
    tool_seg = m_create_tool->get_tool_seg();   //vector<vector<Vector3d>>  3d vector of each point in each seg
    angle_seg = m_create_tool->get_angle_seg();


    cout << "N_grasp: " << N_grasp << endl;

    cout << "grasp_loci: \n";
    for(int i=0; i < N_grasp; i++)
        cout << "[" << grasp_loci.at(i).matrix() << "]\n";
    cout << endl;

    cout << "press enter to continue...:_";
    cin.ignore();

    cout << "N_atk_angle: " << N_atk_angle << endl;
    cout << "atk_angle: [" << atk_angle.transpose() << "]\n" <<endl;

    cout << "N_seg: " << N_seg << endl;
    cout << "N_pts: " << N_pts.transpose() << endl;

    computeAdjacentSeg();

    ros::Time begin = ros::Time::now();
    monte_carlo_tree();
    ros::Duration elaspsed_time = ros::Time::now() - begin;
    ROS_WARN_STREAM("\ncreate MCTS Tree-> time elasped:\n" << elaspsed_time.toSec() << "secs");

    sleep(0.5);

    ros::Time start = ros::Time::now();
    monte_carlo_search();
    ros::Duration diff = ros::Time::now() - start;
    ROS_WARN_STREAM("\nMCTS Tree Search-> time elasped:\n" << diff.toSec() << "secs");

    sleep(0.5);

    ROS_ERROR_STREAM("\nTotal time elasped:\n" << elaspsed_time.toSec() << " + " <<  diff.toSec() << " = " <<  elaspsed_time.toSec()+diff.toSec());

    return;
}

