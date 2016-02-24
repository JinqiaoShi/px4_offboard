#include "px4_offboard/include.h"
#include "px4_offboard/CtrlPx4.h"

CtrlPx4::CtrlPx4()
{
  mavros_acc_pub         =   nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/setpoint_accel/accel",50);
  mavros_pos_pub         =   nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",50);
  mavros_vel_pub         =   nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",50);

  mavros_set_mode_client =   nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_armed_client    =   nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  local_vel_sub          =   nh.subscribe("/mavros/local_position/velocity",100, &CtrlPx4::velCallback,this);
  local_pos_sub          =   nh.subscribe("/mavros/local_position/pose",100, &CtrlPx4::poseCallback,this);
  state_sub              =   nh.subscribe("/mavros/state", 100, &CtrlPx4::stateCallback,this);
  radio_mode_sub         =   nh.subscribe("/mavros/rc/in", 100,&CtrlPx4::radioCallback,this);
  joy_vel_sub            =   nh.subscribe("/joy/cmd_vel",  100,&CtrlPx4::joyVelCallback,this);
  joy_state_sub          =   nh.subscribe("/joy/state",    100,&CtrlPx4::joyStateCallback,this);
};

bool CtrlPx4::commandUpdate(){
//  if (OffSw) {
    if (!stateCmp()){
        // arming
        set_armed.request.value = state_set.armed;
        mavros_armed_client.call(set_armed);

        // switch to offboard
        if (state_set.offboard)
          set_mode.request.custom_mode = "OFFBOARD";

        ros::spinOnce();
        mavros_set_mode_client.call(set_mode);
    }

    // if state is armed publish fcu commnad
    if (state_set.armed){
      if (state_set.takeoff)
          takeoff(2,0.5); // takeoff to 2m with velocity 0.5m/s
      else if (state_set.land)
          land(0.3);
      else{
      mavros_vel_pub.publish(fcu_vel_setpoint);
      ROS_INFO("Velocity Command: [x: %f y:%f z: %f, yaw:%f]", fcu_vel_setpoint.twist.linear.x,fcu_vel_setpoint.twist.linear.y,fcu_vel_setpoint.twist.linear.z,fcu_vel_setpoint.twist.angular.z);
      }
  }
  //}
  //else
  //{
  //  ROS_INFO("In Manual Mode");
  //}


}

bool CtrlPx4::stateCmp()
{
  myState *read = &state_read;
  myState *set  = &state_set;
  return ((read->armed == set->armed)&&(read->offboard == set->offboard)&&(read->control == set->control));
}


void CtrlPx4::takeoff()
{

}

void CtrlPx4::land()
{

}

void CtrlPx4::joyVelCallback(const geometry_msgs::Twist twist)
{
    #ifdef VELOCITY
    fcu_vel_setpoint.twist.linear.x = twist.linear.x;
    fcu_vel_setpoint.twist.linear.y = twist.linear.y;
    fcu_vel_setpoint.twist.linear.z = twist.linear.z;
    fcu_vel_setpoint.twist.angular.x = twist.angular.x;
    fcu_vel_setpoint.twist.angular.y = twist.angular.y;
    fcu_vel_setpoint.twist.angular.z = twist.angular.z;
    #endif


    #ifdef POSE
    fcu_pos_setpoint.pose.position.x = twist.linear.x;
    fcu_pos_setpoint.pose.position.y = twist.linear.y;
    fcu_pos_setpoint.pose.position.z = twist.linear.z;
    #endif


}

void CtrlPx4::stateCallback(const mavros_msgs::State state)
{
	 state_read.armed = (state.armed==128);
	 state_read.offboard = (strcmp(state.mode.c_str(),"OFFBOARD")==0);
}

void CtrlPx4::radioCallback(const mavros_msgs::RCIn rc_in)
{
    OffSw = (rc_in.channels[5]>1700);
}

void CtrlPx4::joyStateCallback(const std_msgs::ByteMultiArray state)
{
    switch (state.data.at(0)) // arming status
    {
      //DISARM
      case 0:
        ROS_INFO("Standby");
        state_set.armed    =   0;
        state_set.offboard =   0;
        break;
      //ARM
      case 1:
        ROS_INFO("Armed");
        state_set.armed    =   1;
        state_set.offboard =   1;
        break;

     //ARMED BUT NO OFFBOARD COMMAND (DANGEROUS)
      case 2:
        state_set.armed    =   1;
        state_set.offboard =   0;
        break;
    }

    if (state.data.at(1)>100) // takeoff detected
        state_set.takeoff = 1;
    else
        state_set.takeoff = 0;

    if (state.data.at(1)<-100)
        state_set.land = 1;
    else
        state_set.land = 0;


}

void CtrlPx4::poseCallback(const geometry_msgs::PoseStamped)
{

};
void CtrlPx4::velCallback(const geometry_msgs::TwistStamped)
{

};
