#ifndef CtrlPx4_H_
#define CtrlPx4_H_

#define POSITION 1 // PreCompiling Flags
#define VELOCITY 0
#define SITL 1
#define PX4 0

class CtrlPx4 {
public:
  CtrlPx4();

  bool commandUpdate();
  bool off_sw_;
  my_state state_set_{0, 0, 0, 0}, state_read_{0, 0, 0, 0};

private:
  // subscriber callbacks from MAV
  void stateCallback(const mavros_msgs::State);
  void radioCallback(const mavros_msgs::RCIn);
  void poseCallback(const geometry_msgs::PoseStamped);
  void velCallback(const geometry_msgs::TwistStamped);
  // subscriber callback from joystick
  void joyCallback(const px4_offboard::JoyCommand);

  // flight controller
  bool takeoff(double altitude, double velcity);
  bool land(double velocity);
  void hover();

  bool stateCmp();
  bool commandUpdate(my_control);      // TODO
  bool commandUpdate(int, my_control); // TODO

  // node initialization
  ros::NodeHandle nh_;
  // Publisher
  ros::Publisher mavros_vel_pub_;
  ros::Publisher mavros_pos_pub_;
  ros::Publisher mavros_acc_pub_;

  // mode change service
  ros::ServiceClient mavros_set_mode_client_;
  ros::ServiceClient mavros_armed_client_;

  // subscriber
  ros::Subscriber local_pos_sub_;
  ros::Subscriber local_vel_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber radio_sub_;
  ros::Subscriber joy_sub_;

  // publishing msgs
  mavros_msgs::SetMode set_mode_;
  mavros_msgs::CommandBool set_armed_;
  geometry_msgs::TwistStamped fcu_vel_setpoint_;
  geometry_msgs::PoseStamped fcu_pos_setpoint_;
};

#endif
