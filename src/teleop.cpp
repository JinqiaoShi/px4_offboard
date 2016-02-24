#include "px4_offboard/include.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <time.h>

#define SITL


#ifdef SITL
// Control 1 : Arrow Keys
#define KEYCODE_RIGHT 0x43          //  right
#define KEYCODE_LEFT  0x44          //  left
#define KEYCODE_UP    0x41          // forward
#define KEYCODE_DOWN  0x42          // backward

#endif

#ifdef PX4
#define KEYCODE_UP 0x43          //  right
#define KEYCODE_DOWN  0x44          //  left
#define KEYCODE_RIGHT  0x41          // forward
#define KEYCODE_LEFT  0x42          // backward
#endif

//Control 2 : WASD
#define KEYCODE_W   0x77            // throttle up
#define KEYCODE_S   0x73            // throttle down
#define KEYCODE_D   0x64            // yaw right
#define KEYCODE_A   0x61            // yaw left

#define KEYCODE_SPACE 0x20          // hover space

// Buttons TODO switch to offboard mode after taken off

#define KEYCODE_Q   0x71    //q   //ARM and offboard
#define KEYCODE_E   0x65      //e   //DISARM
#define KEYCODE_C   0x63    //c   //ARM but no offboard

void copyTwist(geometry_msgs::Twist*, geometry_msgs::Twist);

class TeleopPx4
{
public:
  TeleopPx4();
  void keyLoop();

private:
  ros:: NodeHandle nh_;
  double linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher state_pub_;
};

TeleopPx4::TeleopPx4():
linear_(0.3),
angular_(0.5),
l_scale_(0.5),
a_scale_(0.5)
{
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/joy/cmd_vel", 100);
    state_pub_ = nh_.advertise<std_msgs::ByteMultiArray>("/joy/state", 100);
}

int kfd = 0;
struct termios cooked, raw;
void quit (int sig)
{
    tcsetattr(kfd,TCSANOW,&cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"px4_teleop");
  TeleopPx4 teleop_px4;
  signal(SIGINT,quit);
  teleop_px4.keyLoop();
  return (0);
}

void TeleopPx4::keyLoop()
{
  char c, c_last;
  bool dirty = false;
  // last velocity setpoint (use for relative control)
  geometry_msgs::Twist lastTwist;
  geometry_msgs::Twist twist;

  tcgetattr(kfd,&cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the drone.");

  std_msgs::ByteMultiArray state;
  bool keyheld;
  state.data.clear();
  state.data.resize(10);

  ros::Rate loop_rate(50);

  for(;;)
  {

    clock_t  now = clock();
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

    geometry_msgs::Twist hover;
    switch(c)
    {
      case KEYCODE_E:
        ROS_INFO("DISARM");
        state.data.at(0) = 0; //Disarm
        dirty = true;
        break;

      case KEYCODE_Q:
        ROS_INFO("ARMING");
        state.data.at(0) = 1; //arm
        dirty = true;
        break;

      case KEYCODE_C:
        ROS_INFO("ARMED BUT NO OFFBOARD COMMAND");
        state.data.at(0) = 2;
        dirty = true;
        break;

      case KEYCODE_SPACE:
        ROS_INFO("HOVER");
        copyTwist(&twist, hover);
        dirty = true;
        break;

      case KEYCODE_LEFT:
        ROS_INFO("LEFT");

        twist.linear.y = lastTwist.linear.y+linear_;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_INFO("RIGHT");
        twist.linear.y = lastTwist.linear.y-linear_;
        dirty = true;
        break;

      case KEYCODE_UP:
        ROS_INFO("Forward");
        twist.linear.x = lastTwist.linear.x+linear_;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_INFO("Backward");
        twist.linear.x = lastTwist.linear.x-linear_;
        dirty = true;
        break;

      case KEYCODE_W:
        ROS_INFO("UP");
        state.data.at(1) += (state.data.at(1)<127) && 1 ;
        twist.linear.z = lastTwist.linear.z+linear_;
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_INFO("DOWN");
        state.data.at(1) -= (state.data.at(1)>-127) && 1;
        twist.linear.z = lastTwist.linear.z-linear_;
        dirty = true;
        break;
    }


    keyheld = ((clock()-now)>400||state.data.at(1)==127||state.data.at(1)==-127);

    ROS_INFO("Keyheld time is %d", keyheld);
    if(dirty && keyheld) //detect if land or takeoff command also issued
    {
      if (state.data.at(1)>100)
      {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
      }

      vel_pub_.publish(twist);
      state_pub_.publish(state);
      dirty=false;
      state.data.at(1)=0;
    }

    copyTwist(&lastTwist,twist);
    loop_rate.sleep();
  }


  return;
}

void copyTwist(geometry_msgs::Twist* twistTemp, geometry_msgs::Twist twist)
{
  (*twistTemp).linear.x = twist.linear.x;
  (*twistTemp).linear.y = twist.linear.y;
  (*twistTemp).linear.z = twist.linear.z;
  (*twistTemp).angular.x = twist.angular.x;
  (*twistTemp).angular.y = twist.angular.y;
  (*twistTemp).angular.z = twist.angular.z;

}
