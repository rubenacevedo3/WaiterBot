/**
 * @author Ruben Acevedo
 * @file nodeInteractionsTest.cpp
 * @brief This is the ".cpp" file for testing the node interations
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

#include <gtest/gtest.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include "ros/ros.h"

float x;
float t;
bool velB;
float w;
bool forceB;

void velCallBack(const geometry_msgs::Twist& vel) {
  x = vel.linear.x;
  t = vel.angular.z;
  velB = true;
}

void forceCallBack(const std_msgs::Float32& f_msg) {
  w = f_msg.data;
  forceB = true;
}

//! test the waiterBot_node
/**
 * @brief This tests the waiterBot_node
 * Given that the distSensor should be recieving
 * a collision statement. The waiterBot should be
 * publishing linear.x = angular.z = 0
 */
TEST(nodeInteractionsTest, waiterBot_nodeTest) {
  ros::NodeHandle n;
  ros::Subscriber vel_sub = n.subscribe("/mobile_base/commands/velocity",
    50, velCallBack);
  ros::Duration(5).sleep();
  ros::spinOnce();
  EXPECT_FLOAT_EQ(0, x);
  EXPECT_FLOAT_EQ(0, t);
  EXPECT_TRUE(velB);
}

//! test the foodStub_node
/**
 * @brief This tests the foodStub_node
 * Given that the robot should be at location (0,2) the
 * food stub should be publishing data = 6.033
 */
TEST(nodeInteractionsTest, foodStub_nodeTest) {
  ros::NodeHandle n;
  ros::Subscriber force_sub = n.subscribe("force", 50, forceCallBack);
  ros::Duration(5).sleep();
  ros::spinOnce();
  EXPECT_FLOAT_EQ(6.0029998, w);
  EXPECT_TRUE(forceB);
}
