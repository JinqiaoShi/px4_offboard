#include "px4_offboard/include.h"
#include "CtrlPx4.cpp"

int main(int argv, char **argc) {
  ros::init(argv, argc, "px4_controller");
  CtrlPx4 controller;
  ros::Rate loop_rate(100);

  while (ros::ok()) {
    controller.commandUpdate();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
