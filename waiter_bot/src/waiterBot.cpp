/**
 *@author Ruben Acevedo
 *@file waiterBot.cpp
 *@brief This is the ".cpp" file for the waiterBot Class 
 *@copyright [2017] Ruben Acevedo
 *
 * This file implements the methods and attributes of the
 * waiterBot Class.
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

#include "waiterBot.hpp"
#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>
#include "distSensor.hpp"
#include "motionModule.hpp"
#include "forceSensor.hpp"
#include "position.hpp"


//! Class Constructor
/**
 * @brief This code constructs the class.
 * It sets the target locations to be 
 * (0,0) (2,0) (2,2) (2,0)
 * It sets the status to "in target location 1"
 * Sets the stopB to false;
 * Sets the right_direction to false;
 * @param nothing
 * @return nothing
 */
waiterBot::waiterBot(): status("in target location 1"),
  stopB(false), right_direction(false) {
    position p;
    p.setPosition(0, 0, 0);
    targetLocs.push_back(p);
    p.setPosition(2, 0, 0);
    targetLocs.push_back(p);
    p.setPosition(2, 2, 0);
    targetLocs.push_back(p);
    p.setPosition(0, 2, 0);
    targetLocs.push_back(p);
}

//! return the robot's status function
/**
 * @brief This function returns the robot's status function
 * @param nothing
 * @return a string representing the robot's status
 */
std::string waiterBot::getStatus() {
  return status;
}

//! gets the robot's target locations
/**
 * @brief This function returns the robots target locations
 * @param nothing
 * @return a position vector list of all the target locations
 */
std::vector<position> waiterBot::getTargetLocs() {
  return targetLocs;
}

//! sees if the robot stopped function
/**
 * @brief This function to see whether the robot stopped or not
 * @param nothing
 * @return a bool repersenting whether or not the robot stopped
 */
bool waiterBot::didStop() {
  return stopB;
}

//! get the angle difference function
/**
 * @brief This function gets the angle differnce between heading and direction.
 * It find the angle heading that the robot should be in to reach its location
 * It then calulates and returns the difference between what the robots heading
 * is and what it should be
 * @param a position repersenting a target location
 * @return a float repersenting the angle difference in radians
 */
float waiterBot::getAngleDiff(position tl) {
  auto cpv = mm.getCurrentLoc().getPos();
  auto tlv = tl.getPos();
  auto x = tlv[0]-cpv[0];
  auto y = tlv[1]-cpv[1];
  float heading;
  /**
   * Find what the heading should be
   */
  if (x == 0 && y == 0) {
    return 0;
  } else if (x > 0) {
      if (y == 0) {
        heading = 0;  // check point (1,0)
      } else if (y > 0) {
          heading = atan(y/x);  // check points in top right quad
        } else {
            heading = 3*3.14/2-atan(y/x);  // check points in bottom right quad
          }
    } else if (x < 0) {
        if (y == 0) {
          heading = 3.14;  // check point (-1,0)
        } else if (y > 0) {
            heading = 3.14 - atan(y/x);  // check points in top left quad
          } else {
              heading = 3.14 + atan(y/x);  // check points in bottom left quad
            }
      } else {
          if (y > 0) {
            heading = 3.14/2;  // check point (0,1)
          } else {
              heading = 3*3.14/2;  // check point (0,-1)
            }
        }
  /**
   * find the angle difference
   */
  return heading - cpv[2];
}

//! sees if the robot stopped function
/**
 * @brief This function moves the robot
 * It checks its current location
 * It checks/sets where it needs to go
 * It checks if it has food
 * It checks it there is an obstical
 * It checks to see if their is an obstical in the way
 * It sets the velocity command
 * @param nothing
 * @return a geometry_msgs::Twist message repersenting velocity commands
 */
geometry_msgs::Twist waiterBot::move() {
  geometry_msgs::Twist vel;
  /**
   * Check to see if the robot is in collision
   */  
  if (ds.inCollision()) {
    vel.linear.x = 0;
    vel.angular.z = 0;
    stopB = true;
    return vel;  // robot stops
  } else {
      stopB = false;
    }

  /**
   * Check to see if the robot has food
   */
  ROS_DEBUG("getWeight: %f", fs.getWeight());
  if (fs.getWeight() == 0) {
    status = "heading to target location 1";
    ROS_INFO("GO pick up food!!!!!");
  }

  ros::Duration delay(10);

  /**
   * Check to see if the robot is in location 1
   */
  if (mm.inRegion(targetLocs[0])) {
    if (status != "in target location 1"
      && status != "heading to target location 2") {
        status = "in target location 1";
        ROS_INFO("in target location 1");
        delay.sleep();
        right_direction = false;
    } else {
        status = "heading to target location 2";
      }
  }

  /**
   * Check to see if the robot is in location 2
   */
  if (mm.inRegion(targetLocs[1])
    && status != "heading to target location 1") {
      if (status != "in target location 2"
        && status != "heading to target location 3") {
          status = "in target location 2";
          ROS_INFO("in target location 2");
          delay.sleep();
          right_direction = false;
        } else {
            status = "heading to target location 3";
          }
  }

  /**
   * Check to see if the robot is in location 3
   */  
  if (mm.inRegion(targetLocs[2])
    && status != "heading to target location 1") {
      if (status != "in target location 3"
        && status != "heading to target location 4") {
          status = "in target location 3";
          ROS_INFO("in target location 3");
          delay.sleep();
          right_direction = false;
        } else {
            status = "heading to target location 4";
          }
  }

  /**
   * Check to see if the robot is in location 4
   */
  if (mm.inRegion(targetLocs[3])
    && status != "heading to target location 1") {
      if (status != "in target location 4"
        && status != "heading to target location 2") {
          status = "in target location 4";
          ROS_INFO("in target location 4");
          delay.sleep();
          right_direction = false;
        } else {
            status = "heading to target location 2";
          }
  }

  /**
   * Checks status for "heading to target location 1
   */
  if (status == "heading to target location 1") {
    if (getAngleDiff(targetLocs[0]) > -0.025 &&
      getAngleDiff(targetLocs[0]) < 0.025 ) {
        right_direction = true;
        ros::Duration(3).sleep();
      }
      if (right_direction) {
        vel.linear.x = 0.2;
        vel.angular.z = 0;
        return vel;
      } else {
          vel.linear.x = 0;
          vel.angular.z = 0.05;
          return vel;
        }
  }

  /**
   * Checks status for "heading to target location 2
   */
  if (status == "heading to target location 2") {
    if (getAngleDiff(targetLocs[1]) > -0.025 &&
      getAngleDiff(targetLocs[1]) < 0.025 ) {
        right_direction = true;
        ros::Duration(3).sleep();
      }
      if (right_direction) {
        vel.linear.x = 0.2;
        vel.angular.z = 0;
        return vel;
      } else {
          vel.linear.x = 0;
          vel.angular.z = 0.05;
          return vel;
        }
  }

  /**
   * Checks status for "heading to target location 3
   */
  if (status == "heading to target location 3") {
    if (getAngleDiff(targetLocs[2]) > -0.025 &&
      getAngleDiff(targetLocs[2]) < 0.025 ) {
      right_direction = true;
        ros::Duration(3).sleep();
      }
      if (right_direction) {
        vel.linear.x = 0.2;
        vel.angular.z = 0;
        return vel;
      } else {
          vel.linear.x = 0;
          vel.angular.z = 0.05;
          return vel;
        }
  }

  /**
   * Checks status for "heading to target location 4
   */
  if (status == "heading to target location 4") {
    if (getAngleDiff(targetLocs[3]) > -0.025 &&
      getAngleDiff(targetLocs[3]) < 0.025 ) {
        right_direction = true;
        ros::Duration(3).sleep();
      }
      if (right_direction) {
        vel.linear.x = 0.2;
        vel.angular.z = 0;
        return vel;
      } else {
          vel.linear.x = 0;
          vel.angular.z = 0.05;
          return vel;
        }
  }

  /**
   * if status is incorrect or in a region robot does nothing
   */
  vel.linear.x = 0;
  vel.angular.z = 0;
  return vel;
}
