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
#include "std_msgs/Byte.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

typedef enum control_t{
  VELOCITY,POSITION,RAW
}myControl;

typedef struct state_t{
  bool armed;
  bool offboard;
  myControl control;
}myState;


#endif
