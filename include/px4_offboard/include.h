#ifndef INCLUDE_H_
#define INCLUDE_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
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
#include <sstream>




typedef enum control_t{
  VELOCITY,POSITION,RAW
}myControl;

typedef struct vel_s{
  double vx;
  double vy;
  double vz;
}myVel;

typedef struct pos_s{
  double px;
  double py;
  double pz;
  double yaw;
}myPos;

typedef struct state_s{
  bool armed;
  bool offboard;
  bool takeoff;
  bool land;

  myControl control;
  myVel vel;
  myPos pos;
}myState;


#endif
