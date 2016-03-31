#include "px4_offboard/include.h"
#include "px4_offboard/JoyCommand.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <time.h>

#define SITL

#ifdef SITL
// Control 1 : Arrow Keys
#define KEYCODE_RIGHT 0x43 //  right
#define KEYCODE_LEFT 0x44  //  left
#define KEYCODE_UP 0x41    // forward
#define KEYCODE_DOWN 0x42  // backward

#endif

#ifdef PX4
#define KEYCODE_UP 0x43    //  right
#define KEYCODE_DOWN 0x44  //  left
#define KEYCODE_RIGHT 0x41 // forward
#define KEYCODE_LEFT 0x42  // backward
#endif

// Control 2 : WASD
#define KEYCODE_W 0x77 // throttle up
#define KEYCODE_S 0x73 // throttle down
#define KEYCODE_D 0x64 // yaw right
#define KEYCODE_A 0x61 // yaw left

#define KEYCODE_SPACE 0x20 // hover space

// Buttons TODO switch to offboard mode after taken off

#define KEYCODE_Q 0x71 // q   //ARM and offboard
#define KEYCODE_E 0x65 // e   //DISARM
#define KEYCODE_C 0x63 // c   //ARM but no offboard

void copyTwist(px4_offboard::JoyCommand *, px4_offboard::JoyCommand);

class TeleopPx4 {
public:
  TeleopPx4();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  double linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher state_pub_;
};

TeleopPx4::TeleopPx4()
    : linear_(0.5), angular_(0), l_scale_(0.5), a_scale_(0.5) {
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  vel_pub_ = nh_.advertise<px4_offboard::JoyCommand>("/joy/cmd_mav", 100);
}

int kfd = 0;
struct termios cooked, raw;
void quit(int sig) {
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "px4_teleop");
  TeleopPx4 teleop_px4;
  signal(SIGINT, quit);
  teleop_px4.keyLoop();
  return (0);
}

void TeleopPx4::keyLoop() {
  char c, c_last;
  bool dirty = false;
  // last velocity setpoint (use for relative control)
  px4_offboard::JoyCommand last_command;
  px4_offboard::JoyCommand command;

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the drone.");

  std_msgs::Byte state;
  ros::Rate loop_rate(50);

  for (;;) {

    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

    px4_offboard::JoyCommand hover;
    hover.arm = true;
    hover.offboard = true;

    switch (c) {
    case KEYCODE_E:
      ROS_INFO("DISARM");
      command.arm = false; // Disarm
      command.offboard = false;
      dirty = true;
      break;

    case KEYCODE_Q:
      ROS_INFO("ARMING");
      command.arm = true; // arm
      command.offboard = true;
      dirty = true;
      break;

    case KEYCODE_SPACE:
      ROS_INFO("HOVER");
      copyTwist(&command, hover);
      dirty = true;
      break;

    case KEYCODE_LEFT:
      ROS_INFO("LEFT");
      command.position.y = last_command.position.y + linear_;
      dirty = true;
      break;
    case KEYCODE_RIGHT:
      ROS_INFO("RIGHT");
      command.position.y = last_command.position.y - linear_;
      dirty = true;
      break;

    case KEYCODE_UP:
      ROS_INFO("Forward");
      command.position.x = last_command.position.x + linear_;
      dirty = true;
      break;
    case KEYCODE_DOWN:
      ROS_INFO("Backward");
      command.position.x = last_command.position.x - linear_;
      dirty = true;
      break;

    case KEYCODE_W:
      ROS_INFO("UP");
      command.position.z = last_command.position.z + linear_;
      dirty = true;
      break;

    case KEYCODE_S:
      ROS_INFO("DOWN");
      command.position.z = last_command.position.z - linear_;
      dirty = true;
      break;

    case KEYCODE_A:
      ROS_INFO("Yaw Left");
      angular_ += 0.5;                               // yaw angle decrease
      command.orientation.z = sin(0.5 * (angular_)); // convert to q
      command.orientation.w = cos(0.5 * (angular_));
      dirty = true;
      break;

    case KEYCODE_D:
      ROS_INFO("Yaw Right");
      angular_ -= 0.5;                               // yaw angle increase
      command.orientation.z = sin(0.5 * (angular_)); // convert to q
      command.orientation.w = cos(0.5 * (angular_));
      dirty = true;
      break;
    }

    if (dirty) // detect if land or takeoff command also issued
    {
      vel_pub_.publish(command);
      dirty = false;
    }

    copyTwist(&last_command, command);
    loop_rate.sleep();
  }

  return;
}

void copyTwist(px4_offboard::JoyCommand *command_temp,
               px4_offboard::JoyCommand command) {
  (*command_temp).position.x = command.position.x;
  (*command_temp).position.y = command.position.y;
  (*command_temp).position.z = command.position.z;
  (*command_temp).orientation.w = command.orientation.w;
  (*command_temp).orientation.z = command.orientation.z;
}
