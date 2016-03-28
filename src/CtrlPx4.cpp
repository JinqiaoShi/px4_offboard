#include "px4_offboard/include.h"
#include "px4_offboard/CtrlPx4.h"


CtrlPx4::CtrlPx4()
{
  mavros_pos_pub         =   nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
  mavros_vel_pub         =   nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",100);

  mavros_set_mode_client =   nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_armed_client    =   nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

  local_vel_sub          =   nh.subscribe("/mavros/local_position/velocity",100, &CtrlPx4::velCallback,this);
  local_pos_sub          =   nh.subscribe("/mavros/local_position/pose",100, &CtrlPx4::poseCallback,this);

  state_sub              =   nh.subscribe("/mavros/state", 100, &CtrlPx4::stateCallback,this);
  radio_mode_sub         =   nh.subscribe("/mavros/rc/in", 100,&CtrlPx4::radioCallback,this);
  joy_vel_sub            =   nh.subscribe("/joy/cmd_vel",  100,&CtrlPx4::joyVelCallback,this);
  joy_state_sub          =   nh.subscribe("/joy/state",    100,&CtrlPx4::joyStateCallback,this);


  #ifdef SITL
  OffSw = 1;
  #endif

};

bool CtrlPx4::commandUpdate(){

 if (OffSw) {
    if (!stateCmp()){

        // arming
        set_armed.request.value = state_set.armed;
        mavros_armed_client.call(set_armed);
        // TODO local takeoff and land; arm = takeoff, disarm = land
        // if (state_set.armed)
        // {
        //   set_mode.request.custom_mode = "OFFBOARD";
        //   mavros_set_mode_client.call(set_mode);
        //   takeoff(2,1);
        // }
        // else
        //   land(0.3);
        if (state_set.offboard)
          set_mode.request.custom_mode = "OFFBOARD";
        else
          set_mode.request.custom_mode = "MANUAL";

        ros::spinOnce();
        mavros_set_mode_client.call(set_mode);
    }

    if (state_set.armed){
      #ifdef VELOCITY
      mavros_vel_pub.publish(fcu_vel_setpoint);
      ROS_INFO("Velocity Command: [x: %f y:%f z: %f, yaw:%f]", fcu_vel_setpoint.twist.linear.x,fcu_vel_setpoint.twist.linear.y,fcu_vel_setpoint.twist.linear.z,fcu_vel_setpoint.twist.angular.z);
      #endif

      #ifdef POSITION
      mavros_pos_pub.publish(fcu_pos_setpoint);
      ROS_INFO("Position Command: [x: %f y:%f z: %f, yaw:%f]", fcu_pos_setpoint.pose.position.x,fcu_pos_setpoint.pose.position.y,fcu_pos_setpoint.pose.position.z,fcu_pos_setpoint.pose.orientation.w);
      #endif

      }
    ros::spinOnce();
  }
  
  else
  {
   ROS_INFO("In Manual Mode");
  }


}

bool CtrlPx4::stateCmp()
{
  myState *read = &state_read;
  myState *set  = &state_set;

  bool in_the_sky    =  (read->armed == set->armed && set->armed == 1);//&& read->takeoff;
  bool on_the_ground =  (read->armed == set->armed && set->armed == 0);//&& read->land; TODO local takeoff/land

  return (in_the_sky||on_the_ground)&&(set->offboard == read->offboard);
}


bool CtrlPx4::takeoff(double altitude, double velocity)
{
  double current_height = state_read.pos.pz;
  ROS_INFO("Current Height: %f", current_height);
  if (current_height < altitude)
  {
    fcu_vel_setpoint.twist.linear.z = velocity;
    mavros_vel_pub.publish(fcu_vel_setpoint);
  }
  else
  {
    ROS_INFO("Finished Takeoff");
    hover();
  }
  state_read.takeoff =  current_height>altitude;

  return current_height>altitude ;
}

bool CtrlPx4::land(double velocity)
{

}

void CtrlPx4::hover()
{
  #ifdef VELOCITY
  fcu_vel_setpoint.twist.linear.x = 0;
  fcu_vel_setpoint.twist.linear.y = 0;
  fcu_vel_setpoint.twist.linear.z = 0;
  mavros_vel_pub.publish(fcu_vel_setpoint);
  #endif

  #ifdef POSITION

  #endif


};




void CtrlPx4::joyVelCallback(const geometry_msgs::Twist twist)
{
    #ifdef VELOCITY
    fcu_vel_setpoint.twist.linear.x = twist.linear.x;
    fcu_vel_setpoint.twist.linear.y = twist.linear.y;
    fcu_vel_setpoint.twist.linear.z = twist.linear.z;
    fcu_vel_setpoint.twist.angular.z = twist.angular.z;
    #endif


    #ifdef POSITION　//only add on current position reading

      fcu_pos_setpoint.pose.position.x    = twist.linear.x + state_read.pos.px;
      fcu_pos_setpoint.pose.position.y    = twist.linear.y + state_read.pos.py;
      fcu_pos_setpoint.pose.position.z    = twist.linear.z + state_read.pos.pz;
      fcu_pos_setpoint.pose.orientation.w = twist.angular.z+ state_read.pos.yaw;
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

    }

};



void CtrlPx4::poseCallback(const geometry_msgs::PoseStamped posRead)
{

  state_read.pos.px  = posRead.pose.position.x;
  state_read.pos.py  = posRead.pose.position.y;
  state_read.pos.pz  = posRead.pose.position.z;
  state_read.pos.yaw = posRead.pose.orientation.w;
};
void CtrlPx4::velCallback(const geometry_msgs::TwistStamped velRead)
{
  state_read.vel.vx = velRead.twist.linear.x;
  state_read.vel.vy = velRead.twist.linear.y;
  state_read.vel.vz = velRead.twist.linear.z;
};
