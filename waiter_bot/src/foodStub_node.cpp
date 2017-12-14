/**
 *@author Ruben Acevedo
 *@file foodStub_node.cpp
 *@brief This is the ".cpp" file for the foodStub node
 *@copyright [2017] Ruben Acevedo
 *
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

#include <std_msgs/Float32.h>
#include <ros/ros.h>
#include "distSensor.hpp"
#include "motionModule.hpp"
#include "foodStub.hpp"


/**
* @brief This node runs the food stub
* When the robot is in target location 1 it publishes 25N 
* of food. Everytime it reaches a target location it
* reduces the food by 1/5 the original value. Everytime the 
* robot is stopped it also reduces the food by 1/5 the original value.
* This node publishes std_msgs::Float32 messages to the force topic.
* This node subscribes to the /odom topic
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "foodStub_node");
  foodStub f;
  ros::NodeHandle n;

  /**
   * subscribe the motionModule to the /odom topic
   */
  ros::Subscriber motionModuleSub = n.subscribe("/odom", 10,
    &motionModule::setCurrentLocationCallBack, &f.r.mm);

  /**
   * subscribe the distSensor to the /scan topic
   */ 
  ros::Subscriber distSensorSub = n.subscribe("/scan", 100,
    &distSensor::setDistReadingCallBack, &f.r.ds);

  /**
   * publish the the food weight to the force topic
   * the weights are gstd_msgs::Float32 message type
   */
  ros::Publisher foodPub = n.advertise<std_msgs::Float32>
    ("force", 100);

  ros::Rate rate(1);

  while (ros::ok()) {
    auto food_msg = f.pubFood();
    auto m = f.r.move();
    foodPub.publish(food_msg);
    ROS_DEBUG("Published Food Weight");
    ROS_INFO("Food Weight: %f", food_msg.data);
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
