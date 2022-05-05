#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <m3_moveit/MoveitSingleAction.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/Point.h>


double TABLE_HEIGHT, PUCK_HEIGHT, PUCK_RADIUS, TOOL_HANDLE_HEIGHT, HAND_THICKNESS, LIFT_TOOL_HEIGHT;

//geometry_msgs::Point pullbacktool_offset_right_hand, pullback_offset_right_hand, pullback_offset_left_hand, pushsideways_offset_right_hand, pushsideways_offset_left_hand, pushforward_offset_right_hand, pushforward_offset_left_hand;
geometry_msgs::Quaternion pullbacktool_orientation_right_hand, pullback_orientation_right_hand, pullback_orientation_left_hand, pushsideways_orientation_right_hand, pushsideways_orientation_left_hand, pushforward_orientation_right_hand, pushforward_orientation_left_hand;
//geometry_msgs::Point object_to_hand_position_offset;
geometry_msgs::Quaternion hand_orientation;
//geometry_msgs::Pose object_in_base;
geometry_msgs::Pose target;

//original
/*const double OFFSET_LONG = 0.12 + PUCK_RADIUS;
const double OFFSET_SHORT = 0.05;//0.1;
const double OFFSET_LAT = 0.067 + PUCK_RADIUS;
const double OFFSET_HT = 0.02;*/

//drawback
/*const double OFFSET_LONG = 0.12 + PUCK_RADIUS;
const double OFFSET_SHORT = 0.05+0.065;//0.1;
const double OFFSET_LAT = 0.067-0.05 + PUCK_RADIUS;
const double OFFSET_HT = 0.02;*/

//sideways
/*const double OFFSET_LONG = 0.12 + PUCK_RADIUS;
const double OFFSET_SHORT = 0.05+0.065;//0.1;
const double OFFSET_LAT = 0.067 + PUCK_RADIUS;
const double OFFSET_HT = 0.02;*/

//push
//const double OFFSET_LONG = 0.12 + PUCK_RADIUS;
//const double OFFSET_SHORT = 0.05+0.065;//0.1;
//const double OFFSET_LAT = 0.067 + PUCK_RADIUS;
//const double OFFSET_HT = 0.02;

//const double OFFSET_LONG = 0.17 + PUCK_RADIUS - 0.05-0.06004;//0.12 + PUCK_RADIUS
//const double OFFSET_SHORT = 0.05+0.065 - 0.05-0.06004;//0.05   //  0.1;
//const double OFFSET_LAT = 0.067-0.05 + PUCK_RADIUS; //0.067 + PUCK_RADIUS
//const double OFFSET_HT = 0.0;//0.02;

bool isReachable(geometry_msgs::Pose pose);



using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sim_reach_boundary");


//  TABLE_HEIGHT = 0.75;
//  PUCK_HEIGHT = 0.05;
//  PUCK_RADIUS = 0.04;
//  TOOL_HANDLE_HEIGHT = 0.12;
//  HAND_THICKNESS = 0.11;
//  LIFT_TOOL_HEIGHT = 0.02;
  TABLE_HEIGHT = 0.7;
  PUCK_HEIGHT = 0.09;
  PUCK_RADIUS = 0.04;
  TOOL_HANDLE_HEIGHT = 0.12;
  HAND_THICKNESS = 0.11;
  LIFT_TOOL_HEIGHT = 0.02;


  //offsets for hand
  //-----------------------------------
//  pullback_offset_right_hand.x = OFFSET_LAT;
//  pullback_offset_right_hand.y = 0.0;//-OFFSET_SHORT;
//  pullback_offset_right_hand.z = OFFSET_HT;
  pullback_orientation_right_hand.w = 0.0;
  pullback_orientation_right_hand.x = 0.0;
  pullback_orientation_right_hand.y = 0.70711;
  pullback_orientation_right_hand.z = 0.70711;

//  pushsideways_offset_right_hand.x = -OFFSET_SHORT;
//  pushsideways_offset_right_hand.y = -OFFSET_LAT;
//  pushsideways_offset_right_hand.z = OFFSET_HT;
  pushsideways_orientation_right_hand.w = 0.5;
  pushsideways_orientation_right_hand.x = 0.5;
  pushsideways_orientation_right_hand.y = 0.5;
  pushsideways_orientation_right_hand.z = 0.5;

//  pushforward_offset_right_hand.x = -OFFSET_LONG;
//  pushforward_offset_right_hand.y = 0.0;
//  pushforward_offset_right_hand.z = OFFSET_HT;
  pushforward_orientation_right_hand.w = 0.5;
  pushforward_orientation_right_hand.x = 0.5;
  pushforward_orientation_right_hand.y = 0.5;
  pushforward_orientation_right_hand.z = 0.5;

//  pullbacktool_offset_right_hand.x = -0.44+0.05;
//  pullbacktool_offset_right_hand.y = -0.065;
//  pullbacktool_offset_right_hand.z = OFFSET_HT;
  pullbacktool_orientation_right_hand.w = 0.5;
  pullbacktool_orientation_right_hand.x = 0.5;
  pullbacktool_orientation_right_hand.y = 0.5;
  pullbacktool_orientation_right_hand.z = 0.5;
  //-----------------------------------



  //Drawback
//  double x_lower = 0.35;
//  double x_upper = 0.65;
//  double y_lower = -0.45;
//  double y_upper = 0;
////  object_to_hand_position_offset = pullback_offset_right_hand;
//  hand_orientation = pullback_orientation_right_hand;

  //Sideways -- same as Push
//  double x_lower = 0.5;
//  double x_upper = 0.8;
//  double y_lower = -0.45;
//  double y_upper = 0;
////  object_to_hand_position_offset = pushsideways_offset_right_hand;
//  hand_orientation = pushsideways_orientation_right_hand;

  //Push
  double x_lower = 0.5;
  double x_upper = 0.8;
  double y_lower = -0.45;
  double y_upper = 0;
//  object_to_hand_position_offset = pushforward_offset_right_hand;
  hand_orientation =  pushforward_orientation_right_hand;

  //Drawback Tool
//  double x_lower = 1.0;
//  double x_upper = 1.15;
//  double y_lower = -0.45;
//  double y_upper = 0;
//  object_to_hand_position_offset = pullbacktool_offset_right_hand;
//  hand_orientation = pullbacktool_orientation_right_hand;

  int nx = 20;
  int ny = 20;
  double stepx =(x_upper-x_lower)/nx;
  double stepy =(y_upper-y_lower)/ny;
  cout << "stepx = " << stepx << endl;
  cout << "stepy = " << stepy << endl;

  //for resuming when moveit hangs halfway during run
//  stepx = 0.015;
//  nx = (x_upper-x_lower)/stepx;


  vector<vector<int> > reach_mat;
  reach_mat.resize(nx+1);
  for(int i=0; i<nx+1; i++)
      reach_mat.at(i).resize(ny+1);



  //Part 1: generate reachability matrix based on 2D grid on XY plane, and write to file
  //===================================================================================
//  ofstream myfile;
//  myfile.open ("reachability.txt", ios::app);
//  cout << "Writing reachability data to file...." << endl;

//  for(int i=0; i<nx+1; i++){
//      for(int j=0; j<ny+1; j++){
//          target.position.x = x_lower + (nx-i)*stepx;
//          target.position.y = y_lower + (ny-j)*stepy;
//          target.position.z = 0.75;
//          cout << target.position.x << "\t" << target.position.y <<"\t" << target.position.z << endl;
//          target.orientation = hand_orientation;
//          reach_mat.at(i).at(j) = isReachable(target);
//          myfile << reach_mat.at(i).at(j) << "\t" ;
//      }
//      myfile << endl;
//  }
//  cout << "Finished writing reachability data to file." << endl;
//  myfile.close();



  //Part 2: Load reachability data from file, compute boundary positions, and write to file
  //=======================================================================================
  ofstream myfile2;
  myfile2.open ("boundary.txt");
  ifstream myfile_read;
  myfile_read.open ("reachability.txt");

  reach_mat.clear();
  if (myfile_read.is_open() ){
      cout << "Reachability file is open. Reading file..." <<endl;
      string line;
      while (getline(myfile_read, line)) {
          stringstream ss(line);
          vector<int> tempvec;
          double dat;
          while(ss >> dat)
              tempvec.push_back(dat);
          reach_mat.push_back(tempvec);
      }
      myfile_read.close();
      cout << "File read and data stored in reach_mat." << endl;
      cout << "Size of reach_mat = " << reach_mat.size() << " X " << reach_mat.at(0).size() << endl;
  }
  else{
      cout << "Unable to open reachability file for reading!" <<endl;
      return false;
  }

  nx = reach_mat.size()-1;
  ny = reach_mat.at(0).size()-1;
  cout << "nx = " << nx << endl;
  cout << "ny = " << ny << endl;
  cout << "Writing boundary data to file...." << endl;
  vector<geometry_msgs::Point> boundary;
  geometry_msgs::Point boundary_pointX, boundary_pointY;
  double diffx, diffy;
  bool already_exist;
  double error;
  boundary_pointX.z = 0.75;
  boundary_pointY.z = 0.75;
  for(int i=0; i<nx+1; i++){
      for(int j=0; j<ny+1; j++){
          if(i<nx){
              diffx = reach_mat.at(i).at(j)-reach_mat.at(i+1).at(j);
              if(diffx!=0){
                  if(diffx<0)
                      boundary_pointX.x = x_lower + (nx-i-1)*stepx;
                  else if(diffx>0)
                      boundary_pointX.x = x_lower + (nx-i)*stepx;

                  boundary_pointX.y = y_lower + (ny-j)*stepy;
                  already_exist = false;

                  for(int k=0; k<boundary.size();k++){
                      error = fabs(boundary_pointX.x - boundary.at(k).x) + fabs(boundary_pointX.y - boundary.at(k).y) ;
                      if(error<1e-7){
                          already_exist = true;
                          cout << "Boundary point " << boundary_pointX << " already exists. Not adding this point." << endl;
                          continue;
                      }
                  }
                  if(!already_exist){
                      boundary.push_back(boundary_pointX);
                      myfile2 << boundary_pointX.x << "  " << boundary_pointX.y << "  " << boundary_pointX.z << "\n" ;
                  }
              }
          }
          if(j<ny){
              diffy = reach_mat.at(i).at(j)-reach_mat.at(i).at(j+1);
              if(diffy!=0){
                  if(diffy<0)
                      boundary_pointY.y = y_lower + (ny-j-1)*stepy;
                  else if(diffy>0)
                      boundary_pointY.y = y_lower + (ny-j)*stepy;
                  boundary_pointY.x = x_lower + (nx-i)*stepx;
                  cout << "=====================================" << endl;
                  cout << "i = " << i << endl;
                  cout << "j = " << j << endl;
                  cout << "boundary_pointY.x = " << boundary_pointY.x <<  endl;
                  cout << "boundary_pointY.y = " << boundary_pointY.y <<  endl;
                  already_exist = false;
                  for(int k=0; k<boundary.size();k++){
                      error = fabs(boundary_pointY.x - boundary.at(k).x) + fabs(boundary_pointY.y - boundary.at(k).y) ;
                      if(error<1e-7){
                          already_exist = true;
                          cout << "Boundary point\n" << boundary_pointY << " already exists. Not adding this point." << endl;
                          continue;
                      }
                  }
                  if(!already_exist){
                      boundary.push_back(boundary_pointY);
                      myfile2 << boundary_pointY.x << "    " << boundary_pointY.y << "    " << boundary_pointY.z << "\n" ;
                  }
              }
          }
      }
  }


  cout << "Finished writing boundary data to file." << endl;
  myfile2.close();

  return 0;
}


bool isReachable(geometry_msgs::Pose pose){

    actionlib::SimpleActionClient<m3_moveit::MoveitSingleAction> ac("cart_single", true);

    ac.waitForServer(ros::Duration(10.0));
    m3_moveit::MoveitSingleGoal target;
    m3_moveit::MoveitSingleResultConstPtr result;

    target.end_eff.poses.resize(1);
    target.link_name = "ee";
    target.end_eff.poses[0].position = pose.position;
    target.end_eff.poses[0].orientation = pose.orientation;
    target.plan_only = true;
    ac.sendGoal(target);
    ac.waitForResult(ros::Duration(10.0));
    result = ac.getResult();
    if(result->error_code == result->FAIL){
        //cout << "----------------------------" << endl;
        cout << "Not reachable by robot hand" << endl;
        //cout << pose << endl;
        //cout << "----------------------------" << endl;
        return false;
    }
    else{
        return true;
    }

}
