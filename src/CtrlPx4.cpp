#include "px4_offboard/CtrlPx4.h"
#include "px4_offboard/include.h"

CtrlPx4::CtrlPx4() {

#if SITL
  off_sw_ = 1;
#endif

  mavros_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 100);
  mavros_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 100);

  mavros_set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_armed_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  local_vel_sub_ = nh_.subscribe("/mavros/local_position/velocity", 100,
                                 &CtrlPx4::velCallback, this);
  local_pos_sub_ = nh_.subscribe("/mavros/local_position/pose", 100,
                                 &CtrlPx4::poseCallback, this);
  state_sub_ =
      nh_.subscribe("/mavros/state", 100, &CtrlPx4::stateCallback, this);
  radio_sub_ =
      nh_.subscribe("/mavros/rc/in", 100, &CtrlPx4::radioCallback, this);
  joy_sub_ = nh_.subscribe("/joy/cmd_mav", 100, &CtrlPx4::joyCallback, this);
  // joy_state_sub_ =
  //     nh_.subscribe("/joy/state", 100, &CtrlPx4::joyStateCallback, this);
};

bool CtrlPx4::commandUpdate() {
  if (off_sw_) {
    if (!stateCmp()) {
      // arming
      // set_armed.request.value = state_set_.armed;
      // mavros_armed_client_.call(set_armed);
      // TODO local takeoff and land
      // if (state_set_.armed)
      // {
      //   set_mode_.request.custom_mode = "OFFBOARD";
      //   mavros_set_mode_client_.call(set_mode_);
      //   takeoff(2,1);
      // }
      // else
      //   land(0.3);

      if (state_set_.offboard) {
        set_mode_.request.custom_mode = "OFFBOARD";
        mavros_set_mode_client_.call(set_mode_);
      }
    }

    if (state_set_.armed) {
#if VELOCITY
      mavros_vel_pub__.publish(fcu_vel_setpoint_);
#endif
#if POSITION
      mavros_pos_pub_.publish(fcu_pos_setpoint_);
#endif
    }

    ros::spinOnce();
  }

  else {
    ROS_INFO("In Manual Mode");
  }
}

bool CtrlPx4::stateCmp() {
  my_state *read = &state_read_;
  my_state *set = &state_set_;

  bool arm = (read->armed == set->armed);
  bool offboard = (read->offboard == set->offboard);
  bool in_the_sky = (arm && set->armed == 1); //&& read->takeoff;
  bool on_the_ground =
      (arm && set->armed == 0); //&& read->land; TODO local takeoff/land

  return offboard;
}

bool CtrlPx4::takeoff(double altitude, double velocity) {
  double current_height = state_read_.pos.pz;
  ROS_INFO("Current Height: %f", current_height);
  if (current_height < altitude) {
    fcu_vel_setpoint_.twist.linear.z = velocity;
    mavros_vel_pub_.publish(fcu_vel_setpoint_);
  } else {
    ROS_INFO("Finished Takeoff");
    hover();
  }
  state_read_.takeoff = current_height > altitude;

  return current_height > altitude;
}

bool CtrlPx4::land(double velocity) {}

void CtrlPx4::hover() {
#if VELOCITY
  fcu_vel_setpoint_.twist.linear.x = 0;
  fcu_vel_setpoint_.twist.linear.y = 0;
  fcu_vel_setpoint_.twist.linear.z = 0;
  mavros_vel_pub_.publish(fcu_vel_setpoint_);
#endif

#if POSITION
// Does Not Apply
#endif
};

void CtrlPx4::joyCallback(const px4_offboard::JoyCommand joy) {
#if VELOCITY
  fcu_vel_setpoint_.twist.linear.x = joy.position.x;
  fcu_vel_setpoint_.twist.linear.y = joy.position.y;
  fcu_vel_setpoint_.twist.linear.z = joy.position.z;
  fcu_vel_setpoint_.twist.angular.z = joy.quarternion.z;
  ROS_INFO("Velocity Command: [x: %f y:%f z: %f, yaw:%f]",
           fcu_vel_setpoint_.twist.linear.x, fcu_vel_setpoint_.twist.linear.y,
           fcu_vel_setpoint_.twist.linear.z, fcu_vel_setpoint_.twist.angular.z);
#endif

#if POSITION // only add on current position reading

  // decompose local setpoint to navigation setpoint;
  Vector3f pos_body, pos_nav;
  Quaternion<float> q_n2b(state_read_.pos.q);
  pos_body << joy.position.x, joy.position.y, joy.position.z;
  pos_nav = q_n2b._transformVector(pos_body);

  // Addition to yaw angle
  float w_temp = cos(0.5 * (state_read_.pos.yaw + joy.yaw));
  float z_temp = sin(0.5 * (state_read_.pos.yaw + joy.yaw));

  ROS_INFO("Pos_nav is %f", pos_nav(0));
  // note this is now the NED Frame!! duhh.. that's kinda dumb...
  // We want ENU frame setpoint
  fcu_pos_setpoint_.pose.position.x = pos_nav(0) + state_read_.pos.px;
  fcu_pos_setpoint_.pose.position.y = pos_nav(1) + state_read_.pos.py;
  fcu_pos_setpoint_.pose.position.z = -pos_nav(2) + state_read_.pos.pz; // Down
  fcu_pos_setpoint_.pose.orientation.w = w_temp; // TODO add local
  fcu_pos_setpoint_.pose.orientation.z = z_temp;

  ROS_INFO("Pos_nav is: [x: %f y:%f z: %f, yaw:%f]", pos_nav(0), pos_nav(1),
           pos_nav(2), joy.yaw);

#endif
  state_set_.offboard = joy.offboard;
  state_set_.armed = joy.arm;
}

void CtrlPx4::stateCallback(const mavros_msgs::State vehicle_state) {
  state_read_.armed = (vehicle_state.armed == 128);
  state_read_.offboard = (strcmp(vehicle_state.mode.c_str(), "OFFBOARD") == 0);
}

void CtrlPx4::radioCallback(const mavros_msgs::RCIn rc_in) {
  off_sw_ = (rc_in.channels[6] > 1000);
}

void CtrlPx4::poseCallback(const geometry_msgs::PoseStamped pos_read) {
  state_read_.pos.px = pos_read.pose.position.x;
  state_read_.pos.py = pos_read.pose.position.y;
  state_read_.pos.pz = pos_read.pose.position.z;

  state_read_.pos.q(0) = pos_read.pose.orientation.w; // qw
  state_read_.pos.q(1) = pos_read.pose.orientation.x; // qx
  state_read_.pos.q(2) = pos_read.pose.orientation.y; // qy
  state_read_.pos.q(3) = pos_read.pose.orientation.z; // qz

  state_read_.pos.yaw = acos(pos_read.pose.orientation.w) * 2;
};
void CtrlPx4::velCallback(const geometry_msgs::TwistStamped vel_read) {
  state_read_.vel.vx = vel_read.twist.linear.x;
  state_read_.vel.vy = vel_read.twist.linear.y;
  state_read_.vel.vz = vel_read.twist.linear.z;
};
