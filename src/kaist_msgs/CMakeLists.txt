cmake_minimum_required(VERSION 2.8.3)
project(kaist_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs
  message_generation
)

add_message_files(
    FILES
    AddNoiseRequest.msg
    AddNoiseResponse.msg
    BoardConnectionStatus.msg
    CommandCenterControlRequest.msg
    CommandCenterControlResponse.msg
    DataErrorsStatus.msg
    EStopRequest.msg
    EStopResponse.msg
    FingerMoveRequest.msg
    FingerMoveResponse.msg
    FTSensorControlRequest.msg
    FTSensorControlResponse.msg
    FTSensorStatus.msg
    GestureMoveRequest.msg
    GestureMoveResponse.msg
    GestureNeckPan.msg
    IKData.msg
    IKDataJointEncoderStatus.msg
    IKDataJointReferenceStatus.msg
    IKDataMoveRequest.msg
    IKDataMoveResponse.msg
    InitFindHomeRequest.msg
    InitFindHomeResponse.msg
    InitLoadParamRequest.msg
    InitLoadParamResponse.msg
    JointEncoderStatus.msg
    JointReferenceStatus.msg
    MCStatus.msg
    MoveJointRequest.msg
    MoveJointResponse.msg
    PODOALStatusRequest.msg
    PODOALStatusResponse.msg
	PlannedMoveRequest.msg
	PlannedMoveResponse.msg
    PredefinedMoveRequest.msg
    PredefinedMoveResponse.msg
    StartALRequest.msg
    StartALResponse.msg
    StopALRequest.msg
    StopALResponse.msg
    TemperatureStatus.msg
    WristMoveRequest.msg
    WristMoveResponse.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
)


catkin_package(
    INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

