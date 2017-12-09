/**
 *@author Ruben Acevedo
 *@file forceSensor.hpp
 *@brief This is the ".hpp" file for the force sensor Class
 *@copyright [2017] Ruben Acevedo
 *
 * This file will define the methods and attributes of the
 * force sensor Class
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
#ifndef CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_FORCESENSOR_HPP_
#define CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_FORCESENSOR_HPP_

#include <std_msgs/Float32.h>

//! A force sensor Class
/**
 * @brief This class stores data from the force sensor
 */
class forceSensor {
  //! Public Methods
 public:
  //! Class Constructor
  /**
   * @brief This code constructs the class.
   * It initializes the weight to be -1
   * @param nothing
   * @return nothing
   */
  forceSensor();

  //! get the weight function
  /**
   * @brief This function gets the weight value
   * @param nothing
   * @return float repesenting the weight value
   */
  float getWeight();

  //! set the weight function
  /**
   * @brief This function sets the weight value
   * @param a const std_msgs::Float32 reference message type
   * repersenting the values that the force sensor reads
   * @return nothing
   */
  void setWeightCallBack(const std_msgs::Float32& force_msg);

  //! Private Attributes
 private:
  //! weight
  /**
   * @brief this float repersents the weight the force sensor senses
   * on the robots tray
   */
  float weight;
};

#endif  // CATKIN_WS_SRC_WAITERBOT_WAITER_BOT_INCLUDE_FORCESENSOR_HPP_
