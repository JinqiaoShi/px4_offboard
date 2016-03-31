#ifndef INCLUDE_H_
#define INCLUDE_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <Eigen/Dense>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include "std_msgs/String.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Byte.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "px4_offboard/JoyCommand.h"
#include <sstream>

using namespace Eigen;

typedef enum control_t { VELOCITY, POSITION, RAW } my_control;

typedef struct vel_s {
  double vx;
  double vy;
  double vz;
} my_vel;

typedef struct pos_s {
  double px; // current POSITION + yaw
  double py;
  double pz;
  double yaw;
  Vector4f q; // current rotation
} my_pos;

typedef struct state_s {
  bool dirty;
  bool armed;
  bool offboard;
  bool takeoff;
  bool land;

  my_control control;
  my_vel vel;
  my_pos pos;
} my_state;

#endif
