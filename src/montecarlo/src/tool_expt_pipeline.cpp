#include <tool_expt/machine.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "tool_expt_pipeline");

  Machine drc_hubo;
  drc_hubo.initiate();
  drc_hubo.process_event( EvStart() );

  return 0;
}

