#include "px4_offboard/include.h"
#include "px4_offboard/CtrlPx4.h"       //TODO


void stateCallback(const mavros_msgs::State);
void joyVelCallback(const geometry_msgs::Twist);
void joyStateCallback(const std_msgs::Byte);


//desired state and current state
myState state_set{0,0,VELOCITY}, state_read{0,0,VELOCITY};
bool stateCmp(myState *, myState *);


//edit msgs globally, wrap in class later
geometry_msgs::TwistStamped         fmu_controller_setvel;
// geometry_msgs::Vector3Stamped       fmu_controller_setacel;
geometry_msgs::PoseStamped          fmu_controller_setpoint;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic");
  ros::NodeHandle nh;
  // ros::Publisher mavros_control_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1000); // here the number is the buffer
  ros::Publisher mavros_control_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1000); // here the number is the buffer
  // ros::Publisher mavros_control_pub = nh.advertise<mavros_msgs::ManualControl>("/mavros/manual_control/control",1000);
  // ros::Publisher mavros_control_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/setpoint_accel/accel",1000); // here the number is the buffer

  //setup offboard service call
  ros::ServiceClient mavros_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_msgs::SetMode set_mode;

  //setup arm service call
  ros::ServiceClient mavros_armed_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool armed;

  ros::Subscriber state_sub = nh.subscribe("/mavros/state", 100, stateCallback);
  ros::Subscriber joy_vel   = nh.subscribe("/joy/cmd_vel",100,joyVelCallback);
  ros::Subscriber joy_state = nh.subscribe("/joy/state",100,joyStateCallback);


  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    // make service call for state
    if (!stateCmp(&state_set,&state_read))
    {
        if (state_set.offboard)
          set_mode.request.custom_mode = "OFFBOARD";
        else
          set_mode.request.custom_mode = "MANUAL";
        armed.request.value = state_set.armed;

      	ros::spinOnce();
      	mavros_armed_client.call(armed);
        mavros_set_mode_client.call(set_mode);
    }

    // write stream of velocity command
    if (state_read.armed)
 		   	   mavros_control_pub.publish(fmu_controller_setvel);

		ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void joyVelCallback(const geometry_msgs::Twist twist)
{
    ROS_INFO("keyboard received");
    fmu_controller_setvel.twist.linear.x = twist.linear.x;
    fmu_controller_setvel.twist.linear.y = twist.linear.y;
    fmu_controller_setvel.twist.linear.z = twist.linear.z;
    fmu_controller_setvel.twist.angular.x = twist.angular.x;
    fmu_controller_setvel.twist.angular.y = twist.angular.y;
    fmu_controller_setvel.twist.angular.z = twist.angular.z;

    fmu_controller_setacel.vector.x = twist.linear.x;
    fmu_controller_setacel.vector.y = twist.linear.y;
    fmu_controller_setacel.vector.z = twist.linear.z;

    fmu_controller_setpoint.pose.position.x = twist.linear.x;
    fmu_controller_setpoint.pose.position.y = twist.linear.y;
    fmu_controller_setpoint.pose.position.z = twist.linear.z;

    // fmu_controller_manual.x = twist.linear.x;
    // fmu_controller_manual.y = twist.linear.y;
    // fmu_controller_manual.z = twist.linear.z;

}

void stateCallback(const mavros_msgs::State state)
{
	 state_read.armed = (state.armed==128);
	 state_read.offboard = (strcmp(state.mode.c_str(),"OFFBOARD")==0);
   ROS_INFO("ENTER STATE CALLBACK");
}

void joyStateCallback(const std_msgs::Byte state)
{
    switch (state.data)
    {
      //DISARM
      case 0:
        state_set.armed    =   0;
        state_set.offboard =   0;
        break;
      //ARM
      case 1:
        state_set.armed    =   1;
        state_set.offboard =   1;
        break;
     //ARMED BUT NO OFFBOARD COMMAND (DANGEROUS)
      case 2:
        state_set.armed    =   1;
        state_set.offboard =   0;
        break;
    }
}

bool stateCmp(myState *read, myState *set)
{
  return ((read->armed == set->armed)&&(read->offboard == set->offboard)&& (read->control == set->control));
}
