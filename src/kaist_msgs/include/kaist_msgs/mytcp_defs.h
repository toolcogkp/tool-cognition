#ifndef MYTCP_DEFS_H
#define MYTCP_DEFS_H

// our custom port for communication
#define K_TCPSERVER_PORT_ALDAEMON      4000 // both directions
#define K_TCPSERVER_PORT_ALSTATUS      4500 // both directions
#define K_TCPSERVER_PORT_REMOTECONTROL 4501 // both directions
#define K_TCPSERVER_PORT_ROBOTSTATE    4502 // server to client only

// error codes
enum
{
    K_ERROR_ACCEPT        = -1,
    K_ERROR_BIND          = -2,
    K_ERROR_CREATE_SOCKET = -3,
    K_ERROR_LISTEN        = -4,
    K_ERROR_SELECT        = -5,
    K_ERROR_NONE          = 0,
};

// commands supported
enum
{
    // start at 10 for no reasons :)  just to ensure that we actually decode something correctly during debugging
    K_TCPSERVER_COMMAND_STATUS = 10,                    // for checking current status of the AL
    K_TCPSERVER_COMMAND_MOVE_PREDEFINED,                // for executing a predefined move
    K_TCPSERVER_COMMAND_MOVE_GESTURE,                   // for executing a gesture
    K_TCPSERVER_COMMAND_GET_CURRENT_ENCODER_VALUES,     // for returning the current encoder values
    K_TCPSERVER_COMMAND_GET_CURRENT_REFERENCE_VALUES,   // for returning the current reference values
    K_TCPSERVER_COMMAND_GET_CURRENT_TEMPERATURE_VALUES, // for returning the current velocity values
    K_TCPSERVER_COMMAND_GET_CURRENT_VELOCITY_VALUES,    // for returning the current velocity values
    K_TCPSERVER_COMMAND_MOVE_FINGERS,                   // for executing finger movements
    K_TCPSERVER_COMMAND_MOVE_IK,                        // for moving according to the IK parameters
    K_TCPSERVER_COMMAND_ROTATE_WRIST,                   // for rotating wrist
    K_TCPSERVER_COMMAND_ABORT_MOVE,                     // aborts the move in progress
    K_TCPSERVER_COMMAND_GET_PODOAL_STATUS,              // returns the status of the PODOALs
    K_TCPSERVER_COMMAND_MOVE_JOINT,                     // for moving a single joint
    K_TCPSERVER_COMMAND_REQUEST_CONTROL,                // OCU/Master Joypad request control for upper body
    K_TCPSERVER_COMMAND_RELEASE_CONTROL,                // OCU/Master Joypad release control for upper body
    K_TCPSERVER_COMMAND_TAKE_CONTROL,                   // command center requires OCU/Master Joypad to take control
    K_TCPSERVER_COMMAND_ADD_NOISE,                      // adds noise to current joints
    K_TCPSERVER_COMMAND_MOVE_PLANNED,                   // for executing a planned move
    K_TCPSERVER_COMMAND_UNKNOWN = 255,                  // unknown command
};

// response values
enum
{
    K_TCPSERVER_RESPONSE_CODE_OK = 0,
    K_TCPSERVER_RESPONSE_CODE_ERROR_MISSING_JOINTS,
    K_TCPSERVER_RESPONSE_CODE_ERROR_PREVIOUS_COMMAND_IN_PROGRESS,
    K_TCPSERVER_RESPONSE_CODE_ERROR_UNKNOWN_COMMAND,
    K_TCPSERVER_RESPONSE_CODE_ERROR_INVALID_ALNUMBER,
    K_TCPSERVER_RESPONSE_CODE_ERROR_SEND,
    K_TCPSERVER_RESPONSE_CODE_ERROR_IKMOVE_PATH_PLANNING,
};

// JSON keys
#define K_TCPSERVER_JSON_KEY_ACKREQUEST     "ackreq"     // if 1 AL tcpclient will reply with confirmation that command is received, else will be silent
#define K_TCPSERVER_JSON_KEY_ADDNOISE       "addnoise"   // add noise
#define K_TCPSERVER_JSON_KEY_ALSTATUS       "alstatus"   // current AL USER_COMMAND status
#define K_TCPSERVER_JSON_KEY_BASE_MODE      "basemode"   // either encoder or reference based values
#define K_TCPSERVER_JSON_KEY_COMMAND        "cmd"        // command to execute
#define K_TCPSERVER_JSON_KEY_ERRORCODE      "errorcode"  // error code
#define K_TCPSERVER_JSON_KEY_GESTURE_ID     "gestureid"  // gesture ID
#define K_TCPSERVER_JSON_KEY_PLANNED_ID     "plannedid"  // planned move ID
#define K_TCPSERVER_JSON_KEY_JOINT_ANGLE    "jointangle" // angle of single joint to move
#define K_TCPSERVER_JSON_KEY_JOINT_NAME     "jointname"  // name of single joint to move
#define K_TCPSERVER_JSON_KEY_MOVE_ID        "moveid"     // predefined move ID
#define K_TCPSERVER_JSON_KEY_PODOAL         "podoal"     // PODOAL
#define K_TCPSERVER_JSON_KEY_PODOAL_NUM     "alnum"      // PODOAL number
#define K_TCPSERVER_JSON_KEY_PODOAL_NAME    "alname"     // PODOAL name
#define K_TCPSERVER_JSON_KEY_PODOAL_STATUS  "alstatus"   // PODOAL status
#define K_TCPSERVER_JSON_KEY_REQUESTER_ID   "requesterid"// requester's ID
#define K_TCPSERVER_JSON_KEY_RESPONSE       "response"   // response values will be here
#define K_TCPSERVER_JSON_KEY_TIME_GESTURE   "gesturems"  // planned time in milliseconds
#define K_TCPSERVER_JSON_KEY_TIME_PLANNED   "plannedms"  // gesture time in milliseconds
#define K_TCPSERVER_JSON_KEY_TIME_IKMOVE    "ikmovems"   // IK move time in milliseconds
#define K_TCPSERVER_JSON_KEY_TIME_JOINTMOVE "jointmovems" // joint move time in milliseconds
#define K_TCPSERVER_JSON_KEY_TIME_READY     "readyms"    // ready time in milliseconds
#define K_TCPSERVER_JSON_KEY_TIME_WRIST     "wristms"    // wrist move time in milliseconds

enum
{
    K_TCPSERVER_BASEMODE_ENCODER,
    K_TCPSERVER_BASEMODE_REFERENCE,
};

// JSON keys to group different sets of values
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION "boardconnection"
#define K_TCPSERVER_JSON_KEY_CONTROL_STATUS   "controlstatus"
#define K_TCPSERVER_JSON_KEY_ENCODER          "encoder"
#define K_TCPSERVER_JSON_KEY_FT_SENSOR        "ftsensor"
#define K_TCPSERVER_JSON_KEY_IKDATA_ENCODER   "ikdataenc"
#define K_TCPSERVER_JSON_KEY_IKDATA_REFERENCE "ikdataref"
#define K_TCPSERVER_JSON_KEY_MCSTATUS         "mcstatus"
#define K_TCPSERVER_JSON_KEY_REFERENCE        "reference"
#define K_TCPSERVER_JSON_KEY_TEMPERATURE      "temperature"
#define K_TCPSERVER_JSON_KEY_VELOCITY         "velocity"

// JSON keys for the joint's board connection
// returns 0 if it's not turned on, 1 if turned on
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_LEB "bcleb" // left elbow
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_LSP "bclsp" // left shoulder pitch
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_LSR "bclsr" // left shoulder roll
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_LSY "bclsy" // left shoulder yaw
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_LWP "bclwp" // left wrist pitch
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_LWY "bclwy" // left wrist yaw
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_LW2 "bclw2" // left wrist rotation
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_NKP "bcnkp" // neck pitch
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_REB "bcreb" // right elbow
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_RSP "bcrsp" // right shoulder pitch
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_RSR "bcrsr" // right shoulder roll
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_RSY "bcrsy" // right shoulder yaw
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_RWP "bcrwp" // right wrist pitch
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_RWY "bcrwy" // right wrist yaw
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_RW2 "bcrw2" // right wrist rotation
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_TRP "bctrp" // torso pitch
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_WSP "bcwsp" // waist pitch
#define K_TCPSERVER_JSON_KEY_BOARD_CONNECTION_WSY "bcwsy" // waist yaw

// JSON Keys for FT Sensor
// float values of the sensors
#define K_TCPSERVER_JSON_KEY_LH_FTMX         "ftmxl" // left hand
#define K_TCPSERVER_JSON_KEY_LH_FTMY         "ftmyl"
#define K_TCPSERVER_JSON_KEY_LH_FTMZ         "ftmzl"
#define K_TCPSERVER_JSON_KEY_LH_FTFX         "ftfxl"
#define K_TCPSERVER_JSON_KEY_LH_FTFY         "ftfyl"
#define K_TCPSERVER_JSON_KEY_LH_FTFZ         "ftfzl"
#define K_TCPSERVER_JSON_KEY_LH_FTFTROLL     "ftftrolll"
#define K_TCPSERVER_JSON_KEY_LH_FTFTROLLVEL  "ftftrollvell"
#define K_TCPSERVER_JSON_KEY_LH_FTFTPITCH    "ftftpitchl"
#define K_TCPSERVER_JSON_KEY_LH_FTFTPITCHVEL "ftftpitchvell"
#define K_TCPSERVER_JSON_KEY_RH_FTMX         "ftmxr" // right hand
#define K_TCPSERVER_JSON_KEY_RH_FTMY         "ftmyr"
#define K_TCPSERVER_JSON_KEY_RH_FTMZ         "ftmzr"
#define K_TCPSERVER_JSON_KEY_RH_FTFX         "ftfxr"
#define K_TCPSERVER_JSON_KEY_RH_FTFY         "ftfyr"
#define K_TCPSERVER_JSON_KEY_RH_FTFZ         "ftfzr"
#define K_TCPSERVER_JSON_KEY_RH_FTFTROLL     "ftftrollr"
#define K_TCPSERVER_JSON_KEY_RH_FTFTROLLVEL  "ftftrollvelr"
#define K_TCPSERVER_JSON_KEY_RH_FTFTPITCH    "ftftpitchr"
#define K_TCPSERVER_JSON_KEY_RH_FTFTPITCHVEL "ftftpitchvelr"

// JSON keys for the IK data
// note that angles are in DEGREES
enum
{
    K_IKDATA_IKMODE_ENCODER = 0,
    K_IKDATA_IKMODE_REFERENCE,
};
enum
{
    K_IKDATA_PATH_PLANNER_NONE = 0,
    K_IKDATA_PATH_PLANNER_SIMPLE,
    K_IKDATA_PATH_PLANNER_RENJUN
};
#define K_TCPSERVER_JSON_KEY_IKDATA_GLOBAL_FRAME     "ikglobal" // global or hand frame/coordinates
#define K_TCPSERVER_JSON_KEY_IKDATA_IKDATA           "ikdata"   // array of data
#define K_TCPSERVER_JSON_KEY_IKDATA_IKMODE           "ikmode"   // encoder or reference mode
#define K_TCPSERVER_JSON_KEY_IKDATA_PATHPLANNER      "ikplanmethod" // method of path planner
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_POS_X         "ikposxl"  // left hand pos x
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_POS_Y         "ikposyl"  // left hand pos y
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_POS_Z         "ikposzl"  // left hand pos z
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_QUARTENION_W  "ikquarwl" // left hand quartenion w
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_QUARTENION_X  "ikquarxl" // left hand quartenion x
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_QUARTENION_Y  "ikquaryl" // left hand quartenion y
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_QUARTENION_Z  "ikquarzl" // left hand quartenion z
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_YAW           "ikyawl"   // left hand yaw
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_PITCH         "ikpitchl" // left hand pitch
#define K_TCPSERVER_JSON_KEY_IKDATA_LH_ROLL          "ikrolll"  // left hand roll
#define K_TCPSERVER_JSON_KEY_IKDATA_LEB_TORSO_ANGLE  "ikleb"    // left elbow torso angle
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_POS_X         "ikposxr"  // right hand pos x
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_POS_Y         "ikposyr"  // right hand pos y
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_POS_Z         "ikposzr"  // right hand pos z
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_QUARTENION_W  "ikquarwr" // right hand quartenion w
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_QUARTENION_X  "ikquarxr" // right hand quartenion x
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_QUARTENION_Y  "ikquaryr" // right hand quartenion y
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_QUARTENION_Z  "ikquarzr" // right hand quartenion z
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_YAW           "ikyawr"   // right hand yaw
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_PITCH         "ikpitchr" // right hand pitch
#define K_TCPSERVER_JSON_KEY_IKDATA_RH_ROLL          "ikrollr"  // right hand roll
#define K_TCPSERVER_JSON_KEY_IKDATA_REB_TORSO_ANGLE  "ikreb"    // right elbow torso angle
#define K_TCPSERVER_JSON_KEY_IKDATA_POS_CENTER_MASS1 "ikpcom1"  // position of center of mass 1
#define K_TCPSERVER_JSON_KEY_IKDATA_POS_CENTER_MASS2 "ikpcom2"  // position of center of mass 2
#define K_TCPSERVER_JSON_KEY_IKDATA_POS_CENTER_MASS3 "ikpcom3"  // position of center of mass 3

// JSON keys for the joints
// note that angles are in DEGREES
#define K_TCPSERVER_JSON_KEY_JOINT_LEB "leb" // left elbow
#define K_TCPSERVER_JSON_KEY_JOINT_LSP "lsp" // left shoulder pitch
#define K_TCPSERVER_JSON_KEY_JOINT_LSR "lsr" // left shoulder roll
#define K_TCPSERVER_JSON_KEY_JOINT_LSY "lsy" // left shoulder yaw
#define K_TCPSERVER_JSON_KEY_JOINT_LWP "lwp" // left wrist pitch
#define K_TCPSERVER_JSON_KEY_JOINT_LWY "lwy" // left wrist yaw
#define K_TCPSERVER_JSON_KEY_JOINT_LW2 "lw2" // left wrist rotation
#define K_TCPSERVER_JSON_KEY_JOINT_LF1 "lf1" // left finger gripper
#define K_TCPSERVER_JSON_KEY_JOINT_LF2 "lf2" // left finger trigger
#define K_TCPSERVER_JSON_KEY_JOINT_NKP "nkp" // neck pitch
#define K_TCPSERVER_JSON_KEY_JOINT_NKY "nky" // neck yaw
#define K_TCPSERVER_JSON_KEY_JOINT_REB "reb" // right elbow
#define K_TCPSERVER_JSON_KEY_JOINT_RSP "rsp" // right shoulder pitch
#define K_TCPSERVER_JSON_KEY_JOINT_RSR "rsr" // right shoulder roll
#define K_TCPSERVER_JSON_KEY_JOINT_RSY "rsy" // right shoulder yaw
#define K_TCPSERVER_JSON_KEY_JOINT_RWP "rwp" // right wrist pitch
#define K_TCPSERVER_JSON_KEY_JOINT_RWY "rwy" // right wrist yaw
#define K_TCPSERVER_JSON_KEY_JOINT_RW2 "rw2" // right wrist rotation
#define K_TCPSERVER_JSON_KEY_JOINT_RF1 "rf1" // right finger gripper
#define K_TCPSERVER_JSON_KEY_JOINT_RF2 "rf2" // right finger trigger
#define K_TCPSERVER_JSON_KEY_JOINT_TRP "trp" // torso pitch
#define K_TCPSERVER_JSON_KEY_JOINT_WSP "wsp" // waist pitch
#define K_TCPSERVER_JSON_KEY_JOINT_WSY "wsy" // waist yaw

// JSON keys for the joint's MCStatus
#define K_TCPSERVER_JSON_KEY_MCSTATUS_LEB "mcsleb" // left elbow
#define K_TCPSERVER_JSON_KEY_MCSTATUS_LSP "mcslsp" // left shoulder pitch
#define K_TCPSERVER_JSON_KEY_MCSTATUS_LSR "mcslsr" // left shoulder roll
#define K_TCPSERVER_JSON_KEY_MCSTATUS_LSY "mcslsy" // left shoulder yaw
#define K_TCPSERVER_JSON_KEY_MCSTATUS_LWP "mcslwp" // left wrist pitch
#define K_TCPSERVER_JSON_KEY_MCSTATUS_LWY "mcslwy" // left wrist yaw
#define K_TCPSERVER_JSON_KEY_MCSTATUS_LW2 "mcslw2" // left wrist rotation
#define K_TCPSERVER_JSON_KEY_MCSTATUS_NKP "mcsnkp" // neck pitch
#define K_TCPSERVER_JSON_KEY_MCSTATUS_REB "mcsreb" // right elbow
#define K_TCPSERVER_JSON_KEY_MCSTATUS_RSP "mcsrsp" // right shoulder pitch
#define K_TCPSERVER_JSON_KEY_MCSTATUS_RSR "mcsrsr" // right shoulder roll
#define K_TCPSERVER_JSON_KEY_MCSTATUS_RSY "mcsrsy" // right shoulder yaw
#define K_TCPSERVER_JSON_KEY_MCSTATUS_RWP "mcsrwp" // right wrist pitch
#define K_TCPSERVER_JSON_KEY_MCSTATUS_RWY "mcsrwy" // right wrist yaw
#define K_TCPSERVER_JSON_KEY_MCSTATUS_RW2 "mcsrw2" // right wrist rotation
#define K_TCPSERVER_JSON_KEY_MCSTATUS_TRP "mcstrp" // torso pitch
#define K_TCPSERVER_JSON_KEY_MCSTATUS_WSP "mcswsp" // waist pitch
#define K_TCPSERVER_JSON_KEY_MCSTATUS_WSY "mcswsy" // waist yaw

// JSON keys for temperature
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_LSP "blsp" // left shoulder pitch
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_LSR "blsr" // left shoulder roll
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_LSY "blsy" // left shoulder yaw
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_LWY "blwy" // left wrist yaw
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_LF  "blf"  // left finger
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_RSP "blsp" // right shoulder pitch
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_RSR "blsr" // right shoulder roll
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_RSY "blsy" // right shoulder yaw
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_RWY "blwy" // right wrist yaw
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_RF  "blf"  // right finger
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_WST "bwst" // waist
#define K_TCPSERVER_JSON_KEY_TEMPERATURE_TRK "btrk" // torso

// JSON keys for the velocity
#define K_TCPSERVER_JSON_KEY_VELOCITY_LEB "vleb" // left elbow
#define K_TCPSERVER_JSON_KEY_VELOCITY_LSP "vlsp" // left shoulder pitch
#define K_TCPSERVER_JSON_KEY_VELOCITY_LSR "vlsr" // left shoulder roll
#define K_TCPSERVER_JSON_KEY_VELOCITY_LSY "vlsy" // left shoulder yaw
#define K_TCPSERVER_JSON_KEY_VELOCITY_LWP "vlwp" // left wrist pitch
#define K_TCPSERVER_JSON_KEY_VELOCITY_LWY "vlwy" // left wrist yaw
#define K_TCPSERVER_JSON_KEY_VELOCITY_LW2 "vlw2" // left wrist rotation
#define K_TCPSERVER_JSON_KEY_VELOCITY_NKP "vnkp" // neck pitch
#define K_TCPSERVER_JSON_KEY_VELOCITY_REB "vreb" // right elbow
#define K_TCPSERVER_JSON_KEY_VELOCITY_RSP "vrsp" // right shoulder pitch
#define K_TCPSERVER_JSON_KEY_VELOCITY_RSR "vrsr" // right shoulder roll
#define K_TCPSERVER_JSON_KEY_VELOCITY_RSY "vrsy" // right shoulder yaw
#define K_TCPSERVER_JSON_KEY_VELOCITY_RWP "vrwp" // right wrist pitch
#define K_TCPSERVER_JSON_KEY_VELOCITY_RWY "vrwy" // right wrist yaw
#define K_TCPSERVER_JSON_KEY_VELOCITY_RW2 "vrw2" // right wrist rotation
#define K_TCPSERVER_JSON_KEY_VELOCITY_TRP "vtrp" // torso pitch
#define K_TCPSERVER_JSON_KEY_VELOCITY_WSP "vwsp" // waist pitch
#define K_TCPSERVER_JSON_KEY_VELOCITY_WSY "vwsy" // waist yaw

#endif // MYTCP_DEFS_H

