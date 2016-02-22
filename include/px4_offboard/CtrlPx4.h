// Nothing happens here yet
#ifndef CtrlPx4_H_
#define CtrlPx4_H_

#define VELOCITY

class CtrlPx4
{
public:
  CtrlPx4();
  bool stateCmp();
  bool commandUpdate();
  // flight state
  myState state_set{0,0,VELOCITY}, state_read{0,0,VELOCITY};


private:
  // subscriber callbacks
  void stateCallback(const mavros_msgs::State);
  void radioCallback(const mavros_msgs::RCIn);
  void joyVelCallback(const geometry_msgs::Twist);
  void joyStateCallback(const std_msgs::Byte);


  bool commandUpdate(myControl); // TODO
  bool commandUpdate(int, myControl);


  bool OffSw;
  // node initialization
  ros::NodeHandle nh;
  ros::Publisher     mavros_vel_pub;
  ros::Publisher     mavros_pos_pub;
  ros::Publisher     mavros_acc_pub;

  ros::ServiceClient mavros_set_mode_client;
  ros::ServiceClient mavros_armed_client;
  ros::Subscriber state_sub;
  ros::Subscriber radio_vel_sub;
  ros::Subscriber joy_vel_sub;
  ros::Subscriber joy_state_sub;


  //publishing msgs
  mavros_msgs::SetMode set_mode;
  mavros_msgs::CommandBool set_armed;

  #ifdef ACCELERATION
  geometry_msgs:: Vector3Stamped fmu_controller_setpoint;
  #endif

  #ifdef VELOCITY
  geometry_msgs::TwistStamped fmu_controller_setpoint;
  #endif

  #ifdef POSE
  geometry_msgs::PoseStamped  fmu_controller_setpoint;
  #endif
};

#endif
