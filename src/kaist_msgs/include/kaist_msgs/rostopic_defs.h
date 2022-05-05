#ifndef ROSTOPIC_DEFS_H
#define ROSTOPIC_DEFS_H

// robot control request topics
#define K_ROSTOPIC_CMDCENTREREQUEST       "CommandCenterControlRequest"
#define K_ROSTOPIC_ESTOPREQUEST           "EStopRequest"
#define K_ROSTOPIC_FINGERMOVEREQUEST      "FingerMoveRequst"
#define K_ROSTOPIC_FTSENSORCONTROLREQUEST "FTSensorControlRequest"
#define K_ROSTOPIC_GESTUREMOVEREQUEST     "GestureMoveRequest"
#define K_ROSTOPIC_PLANNEDMOVEREQUEST     "PlannedMoveRequest"
#define K_ROSTOPIC_IKDATAMOVEREQUEST      "IKDataMoveRequest"
#define K_ROSTOPIC_MOVEJOINTREQUEST       "MoveJointRequest"
#define K_ROSTOPIC_PREDEFINEDMOVEREQUEST  "PredefinedMoveRequest"
#define K_ROSTOPIC_WRISTMOVEREQUEST       "WristMoveRequest"

// robot control response topics
#define K_ROSTOPIC_CMDCENTRERESPONSE       "CommandCenterControlResponse"
#define K_ROSTOPIC_ESTOPRESPONSE           "EStopResponse"
#define K_ROSTOPIC_FINGERMOVERESPONSE      "FingerMoveResponse"
#define K_ROSTOPIC_FTSENSORCONTROLRESPONSE "FTSensorControlResponse"
#define K_ROSTOPIC_GESTUREMOVERESPONSE     "GestureMoveResponse"
#define K_ROSTOPIC_PLANNEDMOVERESPONSE     "PlannedMoveResponse"
#define K_ROSTOPIC_IKDATAMOVERESPONSE      "IKDataMoveResponse"
#define K_ROSTOPIC_MOVEJOINTRESPONSE       "MoveJointResponse"
#define K_ROSTOPIC_PREDEFINEDMOVERESPONSE  "PredefinedMoveResponse"
#define K_ROSTOPIC_WRISTMOVERESPONSE       "WristMoveResponse"

// robot state topics
#define K_ROSTOPIC_BOARD_CONNECTION       "BoardConnection"
#define K_ROSTOPIC_FTSENSOR               "FTSensor"
#define K_ROSTOPIC_IKDATA_JOINT_ENCODER   "IKDataJointEncoder"
#define K_ROSTOPIC_IKDATA_JOINT_REFERENCE "IKDataJointReference"
#define K_ROSTOPIC_JOINT_ENCODER          "JointEncoder"
#define K_ROSTOPIC_JOINT_REFERENCE        "JointReference"
#define K_ROSTOPIC_MCSTATUS               "MCStatus"
#define K_ROSTOPIC_TEMPERATURE            "Temperature"

// AL control request topics
#define K_ROSTOPIC_INITFINDHOMEREQUEST    "InitFindHomeRequest"
#define K_ROSTOPIC_INITLOADPARAMREQUEST   "InitLoadParamRequest"
#define K_ROSTOPIC_STARTALREQUEST         "StartALRequest"
#define K_ROSTOPIC_STOPALREQUEST          "StopALRequest"

// AL control response topics
#define K_ROSTOPIC_INITFINDHOMERESPONSE   "InitFindHomeResponse"
#define K_ROSTOPIC_INITLOADPARAMRESPONSE  "InitLoadParamResponse"
#define K_ROSTOPIC_STARTALRESPONSE        "StartALResponse"
#define K_ROSTOPIC_STOPALRESPONSE         "StopALResponse"

// other topics
#define K_ROSTOPIC_DATA_ERRORS            "DataErrors"
#define K_ROSTOPIC_ADD_NOISE_REQUEST      "AddNoiseRequest"
#define K_ROSTOPIC_ADD_NOISE_RESPONSE     "AddNoiseResponse"
#define K_ROSTOPIC_NECK_PAN_REQUEST       "NeckPanRequest"
#define K_ROSTOPIC_PODOAL_STATUS_REQUEST  "PODOALStatusRequest"
#define K_ROSTOPIC_PODOAL_STATUS_RESPONSE "PODOALStatusResponse"

// Yuan Wei listens to this instead of K_ROSTOPIC_JOINT_REFERENCE
#define K_ROSTOPIC_JOINT_STATES           "drchubo_joint_states"

#endif // ROSTOPIC_DEFS_H
