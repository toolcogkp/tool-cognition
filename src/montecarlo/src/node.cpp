#include <tool_expt/node.h>

using namespace std;
using namespace Eigen;

void MCT_NODE::init()
{
    m_moveit_ik = NULL;
    m_kdl_ik = NULL;

    label = "";
    score = 0;
    num_visit = 0;
    small_seg = false;
    
    num_childs = 0;
    type = "";
    arm_len << 0.35, 0.34, 0.054;  
    
    collision_obj = false;
    childs.clear();
    state.clear();

    node_value = 0;
    small_seg = false;

    /* initialize random seed: */
    srand (time(NULL));

    idx_G = idx_A = idx_S = idx_P = -1;
}

double MCT_NODE::update()
{

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
                n_all += childs.at(i).num_visit;
            }

            for(int i = 0; i < num_childs; i++)
            {
                int n = childs.at(i).num_visit;
                int nbest = childs.at(best_idx).num_visit;
                
                double ucb = childs.at(i).score/n + Cp*sqrt( log(n_all)/n );
                double ucb_best = childs.at(best_idx).score/nbest + Cp*sqrt( log(n_all)/nbest );

                if(ucb > ucb_best){
                    best_idx = i;
                }
            }
            node_value = childs.at(best_idx).update();
        }
        else
        {
            //random rollout
            type = "rollout";
            int random_idx = rand() % num_childs;
            node_value = childs.at(random_idx).update();
        }
    }
    else
    {
        type = "terminal";

        if(num_visit <= 0)                  
            node_value = calculate_value();
    }

    update_score(node_value);
    return node_value;
}
    
void MCT_NODE::update_score(double val)
{
    score += val;
    num_visit += 1;

    write_dataset();
    m_recorder->saveData(data);
    return;
}

double MCT_NODE::calculate_value()
{
    if(m_moveit_ik == NULL && m_kdl_ik == NULL)
    {
        cout << "error! forgot to set ik_solver!" << endl;
        return -1;
    }

    double coeff1 = 5.0;
    double coeff2 = -0.05;
    double coeff3 = -0.7;

    int N_path = state.size();
    double value = 0;
    double ik_value = 1;
    Vector3d f_arm_len = arm_len;
    
    int ipath = 0;
    bool ikFlag = false;
    bool okFlag = true;

    m_moveit_ik->reset();

    while( ipath < N_path && okFlag)
    {
        ikFlag = false;
        Affine3d T_real_grasp = state.at(ipath);
        ikFlag = m_moveit_ik->ik_solve(T_real_grasp);
        if( (ikFlag == false) || (collision_obj == true) ){
            okFlag = false;
            value = 0;
        }
        else
        {
            MatrixXd jacob, mat;
            double Manip;
            
            jacob = m_moveit_ik->jacobian_solve();
            
            mat = jacob * jacob.transpose();
            Manip = sqrt(mat.determinant()) * coeff1;

            vector< double > desired_values;
            desired_values = m_moveit_ik->desired_joint_values;

            vector< double > maxl;
            maxl = m_moveit_ik->max_limit;
            vector< double > minl;
            minl = m_moveit_ik->min_limit;

            double limit_error =0;
            double sum_errors = 0;
            for(int i = 0; i < desired_values.size(); i++)
            {   
                double mean = (maxl[i] + minl[i])/2.0;
                sum_errors += ((desired_values[i] - mean) * (desired_values[i] - mean));
            }
            limit_error = coeff2 * sqrt(sum_errors);

            double small_seg_error = 0;
            if(small_seg)
                small_seg_error = coeff3;


            double errors;
            errors = limit_error + small_seg_error;
            if(errors < -0.9)
                errors = -0.9;
            value = value + ik_value + errors;
        }
        ipath = ipath + 1;
    }// end while

    if(value != 0)
        value = value/N_path;

    return value;
}



bool MCT_NODE::all_visited()
{
    bool flag = true;
    for(int i = 0; i < num_childs; i++)
    {
        if( childs.at(i).num_visit < 1 )
        {
            flag = false;
        }
    }
    return flag;
}
