/**
 *@author Ruben Acevedo
 *@file distSensor.cpp
 *@brief This is the ".cpp" file for the distSensor Class 
 *@copyright [2017] Ruben Acevedo
 *
 * This file implements the methods and attributes of the
 * distSensor Class.
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

#include <sensor_msgs/LaserScan.h>
#include "distSensor.hpp"

//! Class Constructor
/**
 * @brief This code constructs the class.
 * It initializes the distReading to be 1000000
 * @param nothing
 * @return nothing
 */
distSensor::distSensor(): distReading(1000000) {
}

//! checks for collision function
/**
 * @brief This function checks for collision
 * If the distReading is under 0.1 meters then it will mark
 * that there is something in front of it
 * @param nothing
 * @return bool repesenting whether it senses a collision or not
 */
bool distSensor::inCollision() {
  if (distReading < 0.1) {
    return true;
  }
  return false;
}

//! gets the distReading function
/**
 * @brief This function returns the distReading value
 * @param nothing
 * @return float repesenting distReading value
 */
  float distSensor::getDistReading() {
    return distReading;
  }

//! set the distance reading function
/**
 * @brief This function sets the distReading value.
 * This function finds the average value of the LaserScan message and 
 * sets it to be the distReading value
 * @param a const sensor_msgs::LaserScan::ConstPtr reference message type
 * repersenting the values that the distance sensor reads. 
 * @return nothing
 */
void distSensor::setDistReadingCallBack
  (const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
  float sum = 0;
  for (auto d : scan_msg->ranges) {
    sum = sum + d;
  }
  distReading = sum/scan_msg->ranges.size();
}
