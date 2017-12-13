/**
 *@author Ruben Acevedo
 *@file waiterBot.hpp
 *@brief This is the ".hpp" file for the waiterBot Class
 *@copyright [2017] Ruben Acevedo
 *
 * This file will define the methods and attributes of the
 * waiterBot Class
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
#ifndef CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_WAITERBOT_HPP_
#define CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_WAITERBOT_HPP_

#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>
#include "distSensor.hpp"
#include "motionModule.hpp"
#include "forceSensor.hpp"
#include "position.hpp"


//! A waiterBot Class
/**
 * @brief This class is the complete robot
 */
class waiterBot {
  //! Public Methods and Attributes
 public:
  //! Class Constructor
  /**
   * @brief This code constructs the class.
   * It sets the target locations to be 
   * (0,0) (10,0) (10,10) (10,0)
   * It sets the status to "in target location 1"
   * Sets the stopB to false;
   * @param nothing
   * @return nothing
   */
  waiterBot();

  //! return the robot's status function
  /**
   * @brief This function returns the robot's status function
   * @param nothing
   * @return a string representing the robot's status
   */
  std::string getStatus();

  //! gets the robot's target locations
  /**
   * @brief This function returns the robots target locations
   * @param nothing
   * @return a position vector list of all the target locations
   */
  std::vector<position> getTargetLocs();

  //! sees if the robot stopped function
  /**
   * @brief This function to see whether the robot stopped or not
   * @param nothing
   * @return a bool repersenting whether or not the robot stopped
   */
  bool didStop();

  //! get the angle difference function
  /**
   * @brief This function gets the angle differnce between heading and direction.
   * It find the angle heading that the robot should be in to reach its location
   * It then calulates and returns the difference between what the robots heading
   * is and what it should be
   * @param a position repersenting a target location
   * @return a float repersenting the angle difference in radians
   */
  float getAngleDiff(position tl);

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
  geometry_msgs::Twist move();

  //! the waiterBot's distance Sensor
  /**
   * @brief this distSensor repersents the robot's distance sensor 
   */
  distSensor ds;

  //! the waiterBot's motion module
  /**
   * @brief this motionModule repersents the robot's motion module
   */
  motionModule mm;

  //! the waiterBot's force sensor
  /**
   * @brief this forceSensor repersents the robot's force sensor
   */
  forceSensor fs;

  //! Private Attributes
 private:
  //! the waiterBot's target locations
  /**
   * @brief this position vector is a list of all the target locations
   * the first element is location (0,0)
   * the second element is location (0,10)
   * the third element is location (10,10)
   * the forth element is location (10,0)
   */
  std::vector<position> targetLocs;

  //! the waiterBot's status
  /**
   * @brief this string states the status of the robot
   * The potential statuses are:
   * "in target location 1"
   * "in target location 2"
   * "in target location 3"
   * "in target location 4"
   * "heading to target location 1"
   * "heading to target location 2"
   * "heading to target location 3"
   * "heading to target location 4"
   * "stopped"
   */
  std::string status;

  //! Stop boolean
  /**
   * @brief this boolean states whether or not the robot is suppose to stop 
   */
  bool stopB;
};

#endif  // CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_WAITERBOT_HPP_
