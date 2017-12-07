/**
 *@author Ruben Acevedo
 *@file forceSensor.cpp
 *@brief This is the ".cpp" file for the force sensor Class 
 *@copyright [2017] Ruben Acevedo
 *
 * This file implements the methods and attributes of the
 * force sensor Class.
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

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "forceSensor.hpp"

//! Global Variable
float w= -1;

//! weight call back function 
/**
 * @brief This function sets the global variable to be the force_msg
 * @param nothing
 * @return nothing
 */
void getWeightCallBack(const std_msgs::Float32::ConstPtr& force_msg) {
      w = force_msg->data;
} 

//! Class Constructor
/**
 * @brief This code constructs the class.
 * It initializes the weight to be 0
 * @param nothing
 * @return nothing
 */
forceSensor::forceSensor(): weight(0) {
}

//! set the weight function 
/**
 * @brief This function sets the weight to be the subscribed message value
 * @param nothing
 * @return nothing
 */
void forceSensor::setWeight() {
  weight = w;
}

//! get the weight function 
/**
 * @brief This function gets the weight value
 * @param nothing
 * @return bool
 */
float forceSensor::getWeight() {
  return weight;
}

//! Checks to see if it has food funciton
/**
 * @brief This code checks to see if the weight is 0
 * @param nothing
 * @return bool
 */
bool forceSensor::isEmpty() {
  /**
   * reset the global variable
   */
  w = -1; 

  /**
   * Create the force topic subscriber 
   */
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("force", 1000, getWeightCallBack);
  setWeight();
  ros::spin();

 /**
  * Checks to see if the subscribere is reading a zero value
  */ 
 if (weight == 0) {
    return true;
  } else {
      return false;
    }
}

