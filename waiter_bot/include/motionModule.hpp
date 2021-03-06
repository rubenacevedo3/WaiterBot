/**
 *@author Ruben Acevedo
 *@file motionModule.hpp
 *@brief This is the ".hpp" file for the motionModule Class
 *@copyright [2017] Ruben Acevedo
 *
 * This file will define the methods and attributes of the
 * motionModule Class
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
#ifndef CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_MOTIONMODULE_HPP_
#define CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_MOTIONMODULE_HPP_

#include <nav_msgs/Odometry.h>
#include "position.hpp"

//! A motion module Class
/**
 * @brief This class keeps track of the robot's motion in the map
 */
class motionModule {
  //! Public Methods
 public:
  //! Class Constructor
  /**
   * @brief This code constructs the class.
   * @param nothing
   * @return nothing
   */
  motionModule();

  //! checks to see if the robot is in a region
  /**
   * @brief This function checks if the robot is in a region
   * The robot should be with in 0.2 m of the pos for it to
   * return that it is within the region
   * @param a position reference repersenting the target region
   * @return bool repesenting whether the robot is in the region or not
   */
  bool inRegion(position pos);

  //! gets the robots current location
  /**
   * @brief This function returns the robots currentLoc position
   * @param nothing
   * @return a position repesenting the currentLoc value
   */
  position getCurrentLoc();

  //! sets the current location of the robot
  /**
   * @brief This function sets the current location of the robot.
   * it takes the nav_msgs::Odometry message and converts it to a position.
   * It sets that postion to be currentLoc
   * @param a const nav_msgs::Odometry message reference from the odometry sensor
   * @return nothing
   */
  void setCurrentLocationCallBack(const nav_msgs::Odometry& odo_msg);

  //! sets the current location's theta of the robot
  /**
   * @brief This function sets the current location theta of the robot.
   * this function is purely to make testing easier
   * @param a const float repesenting the theta
   * @return nothing
   */
  void setThetaValueForTests(const float& t);

  //! Private Attributes
 private:
  //! current location
  /**
   * @brief this position repersents the robots current location 
   */
  position currentLoc;
};

#endif  // CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_MOTIONMODULE_HPP_
