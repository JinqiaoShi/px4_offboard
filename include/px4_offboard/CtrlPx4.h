// Nothing happens here yet
#ifndef CtrlPx4_H_
#define CtrlPx4_H_

#define VELOCITY // POSE

#define SITL //PX4


class CtrlPx4
{
public:
  CtrlPx4();

  bool commandUpdate();
  // flight state
  myState state_set{0,0,0,0}, state_read{0,0,0,0};


private:
  // subscriber callbacks
  void stateCallback(const mavros_msgs::State);
  void radioCallback(const mavros_msgs::RCIn);
  void joyVelCallback(const geometry_msgs::Twist);
  void joyStateCallback(const std_msgs::ByteMultiArray);
  void poseCallback(const geometry_msgs::PoseStamped);
  void velCallback(const geometry_msgs::TwistStamped);

  bool takeoff(double altitude, double velcity);
  bool land   (double velocity);
  void hover   ();


  bool stateCmp();
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

  ros::Subscriber local_pos_sub;
  ros::Subscriber local_vel_sub;
  ros::Subscriber state_sub;
  ros::Subscriber radio_mode_sub;
  ros::Subscriber joy_vel_sub;
  ros::Subscriber joy_state_sub;


  //publishing msgs
  mavros_msgs::SetMode set_mode;
  mavros_msgs::CommandBool set_armed;

  geometry_msgs::TwistStamped    fcu_vel_setpoint;
  geometry_msgs::PoseStamped     fcu_pos_setpoint;

};

#endif
