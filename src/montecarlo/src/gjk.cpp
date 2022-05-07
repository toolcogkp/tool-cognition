#include <tool_expt/gjk.h>

using namespace std;
using namespace Eigen;

bool GJK::collision_check(Shape s1, Shape s2, int iterations)
{
    if( (s1.vertices.size() == 0) ) 
        cout << "gjk s1 vertices are empty!!!" << endl;
    if( (s2.vertices.size()==0) )
        cout << "gjk s2 vertices are empty!!!" << endl;
    
    //cout << "-----------------gjk start-------------------" << endl;
    bool flag = false;
    iteration = iterations;
    vec << 0.8, 0.5, 1.0;   
    Vector3d a, b, c, d;

    pickLine(s2, s1, vec, a, b);
    flag = pickTriangle(s2, s1, a, b, c);

    if(flag == true){
        flag = pickTetrahedron(s2, s1, a, b, c, d);
    }

    //cout << "------------------gjk end------------------" << endl;
    return flag;
}//end main collision check

void GJK::pickLine(Shape s1, Shape s2, Vector3d v, Vector3d &a, Vector3d &b)
{
    a = support(s2, s1,  v);
    b = support(s2, s1, -v);
}//end pick line

bool GJK::pickTriangle( Shape s1, Shape s2, Vector3d &a, Vector3d &b, Vector3d &c )
{   
    bool flag = false;

    Vector3d ab, ao, ac, abc, abp, acp, v;
    ab = b - a;
    ao = -a;
    v = (ab.cross(ao)).cross(ab);

    c = b;
    b = a;
    a = support(s2, s1, v);

    for(int i = 0; i < iteration; i++)
    {
        ab = b - a;
        ao = -a;
        ac = c - a;

        abc = ab.cross(ac);
        abp = ab.cross(abc);
        acp = abc.cross(ac);

        if( abp.dot(ao) > 0)
        {
            c = b;
            b = a;
            v = abp;
        }
        else if(acp.dot(ao) > 0)
        {
            b = a;
            v = acp;
        }
        else
        {
            flag = true;
            break;
        }
        a = support(s2, s1, v);
    }//end for

    return flag;
}//end pick triangle

bool GJK::pickTetrahedron( Shape s1, Shape s2, Vector3d &a, Vector3d &b, Vector3d &c, Vector3d &d )
{
    bool flag = false;
    
    Vector3d ab, ao, ac, ad, abc, acd, abp, adb, acp, v;
    ab = b - a;
    ac = c - a;

    abc = ab.cross(ac);
    ao = -a;

    if( abc.dot(ao) > 0)    //above
    {
        d = c;
        c = b;
        b = a;

        v = abc;
        a = support(s2, s1, v);
    }
    else                    //below
    {
        d = b;
        b = a;
        v = -abc;
        a = support(s2, s1, v);
    }//end ifelse
    
    for(int i = 0; i < iteration; i++)
    {
        ab = b - a;
        ao = -a;
        ac = c - a;
        ad = d - a;

        abc = ab.cross(ac);

        if( abc.dot(ao) > 0 )          //above ABC triangle
        {
            //no changes
        }
        else
        {
            acd = ac.cross(ad);

            if( acd.dot(ao) > 0 )       //above triangle ACD
            {
                b = c;
                c = d;
                ab = ac;
                ac = ad;
                abc = acd;
            }
            else if( acd.dot(ao) < 0 ) 
            {
                adb = ad.cross(ab);
                if( adb.dot(ao) > 0 )   //above triangle ADB
                {
                    c = b;
                    b = d;
                    ac = ab;
                    ab = ad;
                    abc = adb;
                }
                else
                {
                    flag = 1;
                    break;
                }//end ifelse .. adb
            }//end ifelse .. acd
        }//end ifelse .. acd
        
        //try again:
        if( abc.dot(ao) > 0 ) //above
        {
            d = c;
            c = b;
            b = a;
            v = abc;
            a = support(s2, s1, v);
        } 
        else    //below
        {
            d = b;
            b = a;
            v = -abc;
            a = support(s2, s1, v);
        }//end ifelse .. try again
    }//end for

    return flag;
}//end pickTetrahedron

Vector3d GJK::support(Shape s1, Shape s2, Vector3d v)
{
    Vector3d point1, point2;
    point1 = getFurthestlnDir(s1, v);
    point2 = getFurthestlnDir(s2,-v);
    return point1 - point2;
}//end support

Vector3d GJK::getFurthestlnDir(Shape s1, Vector3d v)
{
    int size = s1.vertices.size();

    VectorXd XData, YData, ZData, dot;
    XData.resize(size);
    YData.resize(size);
    ZData.resize(size);
    dot.resize(size);

    for(int i=0; i < size; i++)
    {
        Vector3d temp = s1.vertices.at(i);
        XData(i) = temp(0);
        YData(i) = temp(1);
        ZData(i) = temp(2);

        dot(i) = temp(0)*v(0) + temp(1)*v(1) + temp(2)*v(2);
    }// end for


    double max;
    int index;
    max = dot.maxCoeff(&index);

    Vector3d point;
    point << XData(index), YData(index), ZData(index);
    // cout << "point: " << point.transpose() << endl;
    // cout << "++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
    return point;
}//end getFurthestlnDir


//generate sweep volume
Shape genSweptVolObj( Shape tool, Vector3d move_vec) 
{
    // cout << "------------------genSV start------------------" << endl;
    // cout << "gen SV" << endl;
 
    if(tool.vertices.size() == 0)
        cout << "tool vertices are empty!!!" << endl;
 
    //generate tool
    Shape init_tool = tool;
    Shape final_tool = tool;
    for(int i = 0; i < final_tool.vertices.size(); i++)
        final_tool.vertices.at(i) = tool.vertices.at(i) + move_vec;

    //generate connecting indices
    vector< Vector2i > connect_v_indices; 
    for(int a = 0; a < tool.vertices.size(); a++)
    {   
        Vector2i indice;
        if(a == tool.vertices.size()-1)
            indice << a, 0;
        else
            indice << a, a+1;
        connect_v_indices.push_back(indice);
    }   

    //generate normals
    vector< Vector3d > edge_normal;
    int num_edges = connect_v_indices.size();

    Quaterniond temp_q;
	temp_q =  AngleAxisd(0, Vector3d::UnitX())					//roll, axis_x
			* AngleAxisd(0, Vector3d::UnitY())					//pitch, axis_y
			* AngleAxisd(M_PI/2.0, Vector3d::UnitZ());			//yaw, axis_z
    Matrix3d rot_z = temp_q.normalized().toRotationMatrix();

    //assume num_edges = num_vertices
    for( int b = 0; b < num_edges-1; b++)
    {
        Vector3d norm;
        norm = rot_z * ( tool.vertices.at(b+1)-tool.vertices.at(b) );
        edge_normal.push_back(norm);
    }
    edge_normal.push_back( rot_z * ( tool.vertices.at(0)-tool.vertices.at(num_edges-1) ) );
    

    //eliminating edges within SV
    vector< int > edges_init;
    vector< int > edges_final;
    double tol = 1e-12;
    for(int j = 0; j < num_edges; j++)
    {
        double val = edge_normal.at(j).transpose() * move_vec;
        if( val > tol)
            edges_final.push_back(j); //add
        else if( val < -tol)
            edges_init.push_back(j); //add
    }


    //get vertices for init and final
    vector< int > vertice_index_init;
    for(int k = 0; k < edges_init.size(); k++)  //usable edges
    {
        Vector2i indices = connect_v_indices.at( edges_init.at(k) );
        if( vertice_index_init.size() != 0)
        {   
            for(int l = 0; l < 2; l ++) //each vertices in indices
            {
                bool flag = false;
                for(int m = 0; m < vertice_index_init.size(); m++)  //check repeat
                {
                    if( vertice_index_init.at(m) == indices(l) )
                    {
                        flag = true;
                        break;
                    }            
                }

                if(!flag)
                    vertice_index_init.push_back( indices(l) );
            }
        }
        else
        {
            vertice_index_init.push_back(indices(0));
            vertice_index_init.push_back(indices(1));
        }
    }
    sort( vertice_index_init.begin(), vertice_index_init.end()); //sort


    vector< int > vertice_index_final;
    for(int k = 0; k < edges_final.size(); k++)  //usable edges
    {
        Vector2i indices = connect_v_indices.at( edges_final.at(k) );
        if( vertice_index_final.size() != 0)
        {   
            for(int l = 0; l < 2; l ++) //each vertices in indices
            {
                bool flag = false;
                for(int m = 0; m < vertice_index_final.size(); m++)  //check repeat
                {
                    if( vertice_index_final.at(m) == indices(l) )
                    {
                        flag = true;
                        break;
                    }            
                }

                if(!flag)
                    vertice_index_final.push_back( indices(l) );
            }
        }
        else
        {
            vertice_index_final.push_back(indices(0));
            vertice_index_final.push_back(indices(1));
        }
    }
    sort( vertice_index_final.begin(), vertice_index_final.end()); //sort

   
    Shape SV;
    for( int n = 0; n < vertice_index_init.size(); n++)
    {
        SV.vertices.push_back( init_tool.vertices.at( vertice_index_init.at(n) ) );
    }
    for( int p = 0; p < vertice_index_final.size(); p++)
    {
        SV.vertices.push_back( final_tool.vertices.at( vertice_index_final.at(p) ) );
    }

    // cout << "sv generated" << endl;
    // cout << "------------------genSV end------------------" << endl;
    
    return SV;
}
