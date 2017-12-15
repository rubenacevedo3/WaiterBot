/**
 *@author Ruben Acevedo
 *@file test_node.cpp
 *@brief This is the ".cpp" file for the test node
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

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "distSensor.hpp"

/**
* @brief This node run is used for testing
* The node publishes sensor_msgs::LaserScan data
* This node also publishes nav_msgs::Odometry data
* 
*/
int main(int argc, char **argv) {
  /**
   * This is creating a sensor_msgs::LaserScan publisher
   * to test the distSensor class and waiterBot_node
   */
  ros::init(argc, argv, "test_node");
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);
  unsigned int num_readings = 100;
  float v = 0.05;  // This should indicate that the robot is in collision

  /**
   * This is creating a nav_msgs::Odometry publisher
   * to test the waiterBot_node
   */
  ros::Publisher odo_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  /**
   * This sets the robot location to target location 2;
   */
  float x = 2;
  float y = 0;


  ros::Rate r(1.0);
  int t = 0;
  while (t < 5) {
    // generate some fake data for our laser scan
    sensor_msgs::LaserScan scan;
    scan.ranges.resize(num_readings);
    int i = 0;
    while (i < num_readings) {
      scan.ranges[i] = v;
      ROS_DEBUG("%f", scan.ranges[i]);
      i++;
    }
    scan_pub.publish(scan);
    ROS_INFO("Publishing Scan Data");

    nav_msgs::Odometry odo_msg;
    odo_msg.pose.pose.position.x = x;
    odo_msg.pose.pose.position.y = y;

    odo_pub.publish(odo_msg);
    ROS_INFO("Publishing Odom Data");

    r.sleep();
    t++;
  }
  return 0;
}
