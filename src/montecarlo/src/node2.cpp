#include <tool_expt/node2.h>

using namespace std;
using namespace Eigen;

// #define USE_KDL

void MCT_NODE2::init()
{
    parentNode = nullptr;

    m_moveit_ik = NULL;
    m_kdl_ik = NULL;
    m_recorder = NULL;

    label = "";
    score = 0;
    num_visit = 0;

    node_value = 0;
    
    num_childs = 0;
    type = "";
    arm_len << 0.35, 0.34, 0.054;  
    
    childs.clear();

    m_ndata.collision_obj = false;
    m_ndata.small_seg = false;
    m_ndata.state.clear();
    m_ndata.state2.clear();
    m_ndata.state3.clear();
    
    for(int i = 0; i < 7; i++)
        m_ndata.idx[i] = 0;

    /* initialize random seed: */
    srand (time(NULL));
}

void MCT_NODE2::create_child( string str, NODE_DATA ndata)
{
    childs.push_back( new MCT_NODE2(this, str, ndata, m_recorder, m_moveit_ik, m_kdl_ik) );   
}

double MCT_NODE2::update()
{
    // cout << "node " << label << ": " << endl;

    if(num_childs > 0)
    {
        if( all_visited() )
        {
            //selection
            type = "selection";
            int best_idx = 0;
            double Cp = sqrt(2);

            int n_all = 0;
            for(int i = 0; i < num_childs; i++)
            {
                n_all += childs.at(i)->num_visit;
            }

            for(int i = 0; i < num_childs; i++)
            {
                int n = childs.at(i)->num_visit;
                int nbest = childs.at(best_idx)->num_visit;
                
                double ucb = childs.at(i)->score/n + Cp*sqrt( log(n_all)/n );
                double ucb_best = childs.at(best_idx)->score/nbest + Cp*sqrt( log(n_all)/nbest );

                if(ucb > ucb_best){
                    best_idx = i;
                }
            }
            node_value = childs.at(best_idx)->update();
        }
        else
        {
            //random rollout
            type = "rollout";
            int random_idx = rand() % num_childs;
            node_value = childs.at(random_idx)->update();
        }

        update_score(node_value);
    }
    else
    {
        // cout << "  terminal" << endl;
        type = "terminal";
     
        if(num_visit <= 0)
            node_value = calculate_value();

        update_score(node_value);
        //go back to root
        MCT_NODE2* p_pointer = parentNode;
        while( p_pointer->parentNode != NULL )
            p_pointer = p_pointer->parentNode;

        if(score > p_pointer->best_score)
        {
            p_pointer->best_score = score;
            p_pointer->best_idx.clear();
            for(int i = 0 ; i < 7; i++)
                p_pointer->best_idx.push_back(m_ndata.idx[i]);
            p_pointer->best_state = m_ndata.state;
        }
    }
    
    return node_value;
}
    
void MCT_NODE2::update_score(double val)
{
    score += val;
    num_visit += 1;

    #ifdef RECORD_START
    if(m_recorder == NULL)
        cout << "error, recorder NULL" << endl;
    else
    { 
        // cout << "m_recorder address: " << m_recorder << endl;
        write_dataset();
        m_recorder->saveData(data);
    }
    #endif

    return;
}

double MCT_NODE2::calculate_value()
{
    if(m_moveit_ik == NULL || m_kdl_ik == NULL)
    {
        ROS_ERROR_STREAM("error! forgot to set ik_solver!");
        return -1;
    }

    double coeff1 = 5.0;
    double coeff2 = -0.05;
    double coeff3 = -0.7;

    int N_path = m_ndata.state.size();
    double value = 0;
    double ik_value = 1;
    Vector3d f_arm_len = arm_len;

    // cout << "cal value@" << endl;
    cout << "  label: " << label << endl;
    // cout << "    N_path: " << N_path << endl;
    
    bool ikFlag = false;
    bool okFlag = true;

    #ifdef USE_KDL
        m_kdl_ik->reset();
    #else
        m_moveit_ik->reset();
    #endif

    if( m_ndata.collision_obj == true ){
        // cout << "collision failed" << endl;
        okFlag = false;
        value = 0;
    }

    int ipath = 0;
    while( ipath < N_path && okFlag )
    {
        ikFlag = false;
        Affine3d T_real_grasp = m_ndata.state.at(ipath);

        #ifdef USE_KDL
            ikFlag = m_kdl_ik->ik_solve(T_real_grasp);
        #else
            ikFlag = m_moveit_ik->ik_solve(T_real_grasp);
        #endif


        if( ikFlag == false ){
            okFlag = false;
            value = 0;
        }
        else
        {
            MatrixXd jacob, mat;
            double Manip;
            
            #ifdef USE_KDL
                jacob = m_kdl_ik->jacobian_solve();
            #else
                jacob = m_moveit_ik->jacobian_solve();
            #endif

            mat = jacob * jacob.transpose();
            Manip = sqrt(mat.determinant()) * coeff1;

            #ifdef USE_KDL
                KDL::JntArray desired_values;
                desired_values = m_kdl_ik->getDesired();
            #else
                vector< double > desired_values;
                desired_values = m_moveit_ik->desired_arm_values;
            #endif

            vector< double > maxl;
            maxl = m_moveit_ik->max_limit;
            vector< double > minl;
            minl = m_moveit_ik->min_limit;

            double limit_error =0;
            double sum_errors = 0;
            for(int i = 0; i < maxl.size(); i++)
            {   
                double mean = (maxl[i] + minl[i])/2.0;
                
                #ifdef USE_KDL
                    sum_errors += ((desired_values(i) - mean) * (desired_values(i) - mean));
                #else
                    sum_errors += ((desired_values[i] - mean) * (desired_values[i] - mean));
                #endif
            }
            limit_error = coeff2 * sqrt(sum_errors);    //coeff2 must be -ve to minus value off error
            
            double small_seg_error = 0;
            if(m_ndata.small_seg)
                small_seg_error = coeff3;

            double errors;
            errors = limit_error + small_seg_error; /* + Manip + exp(-coeff2*q3*q3)*/
            if(errors < -0.9)
                errors = -0.9;

            value = value + ik_value + errors;
        }
        ipath = ipath + 1;
        cout << "    --------------------" << endl;
    }// end while

    if(value != 0)
        value = value/N_path;

    return value;
}


bool MCT_NODE2::all_visited()
{
    bool flag = true;

    for(int i = 0; i < num_childs; i++)
    {
        if( childs.at(i)->num_visit < 1 )
        {
            flag = false;
        }
    }
    return flag;
}