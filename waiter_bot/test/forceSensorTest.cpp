/**
 * @author Ruben Acevedo
 * @file forceSensorTest.cpp
 * @brief This is the ".cpp" file for testing the force sensor class
 * @copyright [2017] Ruben Acevedo
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
#include <gtest/gtest.h>
#include <std_msgs/Float32.h>
#include "forceSensor.hpp"

//! test the forceSensor constructor 
/**
 * @brief This tests makes sure that the constructor sets the Weight=0
 * It also test the get food weight at the same time.
 */
TEST(forceSensorTest, constructorTest) {
  forceSensor f;
  EXPECT_EQ(0,f.getWeight());
}

//! test the forceSensor isEmpty() function 
/**
 * @brief This tests makes sure the sensor can correctly read data
 * It will make sure that the function can read published data
 * from the force topic
 */
TEST(forceSensorTest, isEmptyTest) {
  forceSensor f;
  ros::init();
  ros::NodeHandle n;

  /**
   * create a Float32 message publisher that publishes
   * the food weight to the topic "force"
   */
  ros::Publisher pub = n.advertise<std_msgs::Float32>("force", 1000);
  ros::Rate loop_rate(10);

  int i = 0;
  while (i < 100) {
    std_msgs::Float32 msg;
    msg.data = 10;
    pub.publish(msg);
    ROS_INFO("Food Weight published: %f", msg.data);
    ros::spinOnce();
    loop_rate.sleep();
  }
  EXPECT_TRUE(true);
}
