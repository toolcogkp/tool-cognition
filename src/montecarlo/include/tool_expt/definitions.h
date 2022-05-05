#ifndef DEFINITIONS_HPP_INCLUDED
#define DEFINITIONS_HPP_INCLUDED


#define PUCK_RADIUS_DEFINE 0.03

//--------------------------manipulate------------------------------------------
#define USE_FINGER_ENC
#define USE_NECK_PAN

//-------------------------usign separate cameras---------------------------------
#define USING_TABLE_CAM		
#define USING_RACK_CAM 		

//-------------------------create_tool--------------------------------------------
#define LEN_SMALL		0.09		//smaller than this, divide by step size small
#define STEP_SIZE		0.03		//default
#define STEP_SIZE_SMALL	0.01		//small seg
#define GRAB_ALLOWED_LENGTH 0.16	
#define SPACING 0.03 				


//---------------------------manipulate_mcts---------------------------------------
#define SHOW_ALL_GRASP_LOCI
#define N_VIA_PTS       0
#define N_INTRA_PTS     0 
#define MAX_GJK_ITER    10
#define N_GRASP         3   
#define N_ATK_ANGLE     2
#define N_PLAYS         40000
#define N_VIA_CANDIDATES 2  
#define GRAB_SPACE      0.025  
#define OFFSET          0.00  

//------------------------search2---------------------------------------------------------
#define SMALL_SEG_THRES 2
#define DOUBLE_SEARCH 1
#define SEARCH_ALL 1
#define N_ADJACENT 3
#define GRAB_HAND 0.06	


//-------------------------tool_expt-----------------------------------------------------
#define VIA_PT_SCALING 2.8 


//--------------------------recorder-------------------------------------------------------
#define RECORD_START


#define PATH_THRESHOLD 0.80

#endif