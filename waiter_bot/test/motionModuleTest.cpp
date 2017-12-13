/**
 * @author Ruben Acevedo
 * @file motionModuleTest.cpp
 * @brief This is the ".cpp" file for testing the motionModule class
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
#include <nav_msgs/Odometry.h>
#include "motionModule.hpp"
#include "position.hpp"

//! test the getCurrentLoc function
/**
 * @brief This tests makes sure that it returns the current location
 */
TEST(motionModuleTest, getCurrentLocTest) {
  motionModule m;
  auto p = m.getCurrentLoc();
  auto v = p.getPos();
  EXPECT_EQ(0, v[0]);
  EXPECT_EQ(0, v[1]);
  EXPECT_EQ(0, v[2]);
}

//! test the inRegion function
/**
 * @brief This tests makes test the inRegion function
 * it should output true if current location is
 * with in a 0.125 m distance from the input position
 */
TEST(motionModuleTest, inRegionTest) {
  motionModule m;

  position p1;
  p1.setPosition(0.1, 0, 0.5);
  EXPECT_TRUE(m.inRegion(p1));

  p1.setPosition(0, 0.125, 1);
  EXPECT_TRUE(m.inRegion(p1));

  p1.setPosition(10, 10, 0.23);
  EXPECT_FALSE(m.inRegion(p1));
}

//! test the set current location function
/**
 * @brief This tests the setCurrentLocationCallBack
 * it should set currentLoc to what ever the 
 * nav_msgs::Odometry message is
 * with in a 0.125 m distance from the input position
 */
TEST(motionModuleTest, setCurrentLocationCallBackTest) {
  motionModule m;
  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = 6;
  msg.pose.pose.position.y = 2;
  msg.pose.pose.orientation.z = 1;
  m.setCurrentLocationCallBack(msg);
  auto p = m.getCurrentLoc();
  auto v = p.getPos();
  EXPECT_EQ(6, v[0]);
  EXPECT_EQ(2, v[1]);
  EXPECT_EQ(1, v[2]);
}
