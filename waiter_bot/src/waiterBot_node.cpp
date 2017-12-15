/**
 *@author Ruben Acevedo
 *@file waiterBot_node.cpp
 *@brief This is the ".cpp" file for the waiterBot node
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sstream>
#include "waiterBot.hpp"
#include "distSensor.hpp"
#include "motionModule.hpp"
#include "forceSensor.hpp"


/**
* @brief This node run the waiterBot Demo
* The robot moves to its target location, stoping every
* time there is an obstical, and it retruns to the food/drink
* pick up location only once it has no more food
* It publishes geometry_msgs::Twist messages from the 
* "/mobile_base/commands/velocity" topic
* It subscribes to the the "/scan", "force", "/odom" topic 
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "waiterBot_node");
  waiterBot r;
  ros::NodeHandle n;

  /**
   * subscribe the distSensor to the /scan topic
   */ 
  ros::Subscriber distSensorSub = n.subscribe("/scan", 100,
    &distSensor::setDistReadingCallBack, &r.ds);

  /**
   * subscribe the forceSensor to the force topic
   */
  ros::Subscriber forceSensorSub = n.subscribe("force", 100,
    &forceSensor::setWeightCallBack, &r.fs);

  /**
   * subscribe the motionModule to the /odom topic
   */
  ros::Subscriber motionModuleSub = n.subscribe("/odom", 100,
    &motionModule::setCurrentLocationCallBack, &r.mm);

  /**
   * publish the velocity commands to the /mobile_base/commands/velocity topic
   * the velocity comands are geometry_msgs::Twist message type
   */
  ros::Publisher velCommandPub = n.advertise<geometry_msgs::Twist>
    ("/mobile_base/commands/velocity", 100);

  ros::Rate rate(5);

  while (ros::ok()) {
    auto vel_msg = r.move();
    velCommandPub.publish(vel_msg);
    std_msgs::String msg;
    std::stringstream ss;
    ss << r.getStatus();
    msg.data = ss.str();
    ROS_INFO("Published Velocity Command");
    ROS_DEBUG("robot status: %s", msg.data.c_str());
    ROS_DEBUG("linear.x: %f", vel_msg.linear.x);
    ROS_DEBUG("angular.z: %f", vel_msg.angular.z);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
