/**
 *@author Ruben Acevedo
 *@file distSensor.hpp
 *@brief This is the ".hpp" file for the distSensor Class
 *@copyright [2017] Ruben Acevedo
 *
 * This file will define the methods and attributes of the
 * distSensor Class
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
#ifndef CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_DISTSENSOR_HPP_
#define CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_DISTSENSOR_HPP_

#include <sensor_msgs/LaserScan.h>

//! A distance sensor Class
/**
 * @brief This class stores data from the distance sensor
 */
class distSensor {
  //! Public Methods
 public:
  //! Class Constructor
  /**
   * @brief This code constructs the class.
   * It initializes the distReading to be 1000000
   * @param nothing
   * @return nothing
   */
  distSensor();

  //! checks for collision function
  /**
   * @brief This function checks for collision
   * If the distReading is under 0.75 meters then it will mark
   * that there is something in front of it
   * @param nothing
   * @return bool repesenting whether it senses a collision or not
   */
  bool inCollision();

  //! gets the distReading function
  /**
   * @brief This function returns the distReading value
   * @param nothing
   * @return float repesenting distReading value
   */
  float getDistReading();

  //! set the distance reading function
  /**
   * @brief This function sets the distReading value.
   * This function finds the smallest value of the LaserScan message and 
   * sets it to be the distReading value. If the value is infitiy it set
   * the reading distance to be 100
   * @param a const sensor_msgs::LaserScan::ConstPtr reference message type
   * repersenting the values that the distance sensor reads. 
   * @return nothing
   */
  void setDistReadingCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  //! Private Attributes
 private:
  //! minimum distance reading
  /**
   * @brief this float repersents the minimum distance that the distance 
   * sensor senses
   */
  float distReading;
};

#endif  // CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_DISTSENSOR_HPP_
