#include "px4_offboard/include.h"
#include "CtrlPx4.cpp"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"px4_controller");
  CtrlPx4 controller;
  ros::Rate loop_rate(200);

  while(ros::ok())
  {
    controller.commandUpdate();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
