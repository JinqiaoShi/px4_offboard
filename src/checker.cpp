#include <ros/ros.h>
// #include <posedetection_msgs/ObjectDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include "px4_offboard/JoyCommand.h"

px4_offboard::JoyCommand g_command;
bool is_update = 0;

void checkerCallback(const geometry_msgs::PoseStamped);

int main(int argc, char **argv) {
  ros::init(argc, argv, "checker");
  ros::NodeHandle nh;
  // publish joy command
  ros::Publisher state_pub =
      nh.advertise<px4_offboard::JoyCommand>("/checker/cmd_mav", 100);
  // listen to checkerboard
  ros::Subscriber checker_sub = nh.subscribe(
      "/checkerdetector/objectdetection_pose", 100, &checkerCallback);
  // checkerboard always armed
  g_command.arm = 1;
  g_command.offboard = 1;

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (is_update) {
      is_update = 0;
      state_pub.publish(g_command);
      ROS_INFO("Publishing setpoint: x %f  y %f yaw %f", g_command.position.x,
               g_command.position.y, g_command.yaw);
      // clear buffer
      g_command.position.x = 0;
      g_command.position.y = 0;
      g_command.yaw = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void checkerCallback(const geometry_msgs::PoseStamped checker_pose) {
  double linear = 0.5;
  double angular = 0.1;
  is_update = 1;
  if (checker_pose.pose.orientation.x > 0.25)
    g_command.position.x = -linear;
  else if (checker_pose.pose.orientation.x < -0.25)
    g_command.position.x = linear;

  if (checker_pose.pose.orientation.y > 0.25)
    g_command.position.y = -linear;
  else if (checker_pose.pose.orientation.y < -0.25)
    g_command.position.y = linear;

  if (checker_pose.pose.orientation.z > 0.25)
    g_command.yaw = -angular;
  else if (checker_pose.pose.orientation.z < -0.25)
    g_command.yaw = angular;
}
