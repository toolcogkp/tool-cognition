/**
 * @file definitions.h
 * @author samuel_cheong@i2r.a-star.edu.sg
 * @brief 
 * Contains definition used and shared among all the classes and experiments
 * @version 0.1
 * @date 2019-03-01
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef DEFINITIONS_HPP_INCLUDED
#define DEFINITIONS_HPP_INCLUDED

/// puck radius -> object
#define PUCK_RADIUS_DEFINE 0.03

//--------------------------manipulate------------------------------------------
/// use finger encoder on olivia
#define USE_FINGER_ENC
/// use neck panning for object searching for olivia
#define USE_NECK_PAN

//-------------------------usign separate cameras---------------------------------
/// use overhead table camera 
#define USING_TABLE_CAM		
/// use tool rack camera
#define USING_RACK_CAM 		

//-------------------------create_tool--------------------------------------------
/// smallest length to divide tool for mcts search
#define LEN_SMALL		0.09		//smaller than this, divide by step size small
/// step size between point to segment tool
#define STEP_SIZE		0.03		//default
/// smallest step size 
#define STEP_SIZE_SMALL	0.01		//small seg
/// allowable grasping limit of tool -> limit by olivia hand size
#define GRAB_ALLOWED_LENGTH 0.16	
/// spacing between grasp location
#define SPACING 0.03 				


//---------------------------manipulate_mcts---------------------------------------
/// display on rviz
#define SHOW_ALL_GRASP_LOCI
/// number of additional viapoints
#define N_VIA_PTS       0
/// number of in between points
#define N_INTRA_PTS     0 
/// max iterations for GJK collision check 
#define MAX_GJK_ITER    10
/// number of possible grasping pose on each tool segment
#define N_GRASP         3   
/// number of attack angle for tool to object
#define N_ATK_ANGLE     2
/// number of mcts search
#define N_PLAYS         40000
/// number of via points if obstructed
#define N_VIA_CANDIDATES 2  
/// grabbing space allowed
#define GRAB_SPACE      0.025  
/// tool offset pose
#define OFFSET          0.00  

//------------------------search2---------------------------------------------------------
/// number of points in segment to consider as small
#define SMALL_SEG_THRES 2
/// perform double search
#define DOUBLE_SEARCH 1
/// search all layers
#define SEARCH_ALL 1
/// search up to 3 adjacent segments
#define N_ADJACENT 3
/// hand length for grabbing
#define GRAB_HAND 0.06	


//-------------------------tool_expt-----------------------------------------------------
/// scaling gain to generate via point 
#define VIA_PT_SCALING 2.8 


//--------------------------recorder-------------------------------------------------------
/// if tool experiment data will be recorded and exported to csv file when finished
#define RECORD_START

/// threshold between path
#define PATH_THRESHOLD 0.80

#endif //end DEFINITIONS_HPP_INCLUDED