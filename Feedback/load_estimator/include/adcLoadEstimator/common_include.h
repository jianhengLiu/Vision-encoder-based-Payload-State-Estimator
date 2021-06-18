//
// Created by root on 2020/5/29.
//

#ifndef USB2XXX_COMMON_INCLUDE_H
#define USB2XXX_COMMON_INCLUDE_H
#define OS_UNIX
// System
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <ctime>

// USB2XXX
#include "USB2XXX/usb_device.h"
#include "USB2XXX/usb2adc.h"

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

// boost
#include <boost/bind.hpp>



#endif //USB2XXX_COMMON_INCLUDE_H
