/**
 *@author Ruben Acevedo
 *@file motionModule.cpp
 *@brief This is the ".cpp" file for the motionModule Class 
 *@copyright [2017] Ruben Acevedo
 *
 * This file implements the methods and attributes of the
 * motionModule Class.
 */
/**
 * MIT License
 *
 * Copyright 2017 Ruben Acevedo
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software. THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE. © 2017 GitHub, Inc.
 */

#include <nav_msgs/Odometry.h>
#include <math.h>
#include <ros/ros.h>
#include "position.hpp"
#include "motionModule.hpp"


//! Class Constructor
/**
 * @brief This code constructs the class.
 * @param nothing
 * @return nothing
 */
motionModule::motionModule() {
}

//! checks to see if the robot is in a region
/**
 * @brief This function checks if the robot is in a region
 * The robot should be with in 0.125 m of the pos for it to
 * return that it is within the region
 * @param a position reference repersenting the target region
 * @return bool repesenting whether the robot is in the region or not
 */
bool motionModule::inRegion(position pos) {
  auto p = pos.getPos();
  auto cl = currentLoc.getPos();
  auto d = pow((pow((p[0] - cl[0]), 2) + pow((p[1] - cl[1]), 2)), 0.5);
  if (d <= 0.125) {
    return true;
  }
  return false;
}

//! gets the robots current location
/**
 * @brief This function returns the robots currentLoc position
 * @param nothing
 * @return a position repesenting the currentLoc value
 */
position motionModule::getCurrentLoc() {
  return currentLoc;
}

//! sets the current location of the robot
/**
 * @brief This function sets the current location of the robot.
 * it takes the nav_msgs::Odometry message and converts it to a position.
 * It sets that postion to be currentLoc
 * @param a const nav_msgs::Odometry message reference from the odometry sensor
 * @return nothing
 */
void motionModule::setCurrentLocationCallBack
  (const nav_msgs::Odometry& odo_msg) {
  auto x = odo_msg.pose.pose.position.x;
  auto y = odo_msg.pose.pose.position.y;
  auto theta = odo_msg.pose.pose.orientation.z;
  currentLoc.setPosition(x, y, theta);
  ROS_INFO("Received Odometry Message");
  ROS_INFO("xPos: %f", x);
  ROS_INFO("yPos: %f", x);
  ROS_INFO("theta: %f", x);
}
