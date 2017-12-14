/**
 * @author Ruben Acevedo
 * @file waiterBotTest.cpp
 * @brief This is the ".cpp" file for testing the waiterBot class
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
#include <std_msgs/Float32.h>
#include "waiterBot.hpp"
#include "distSensor.hpp"
#include "forceSensor.hpp"
#include "motionModule.hpp"
#include "position.hpp"
#include "ros/ros.h"

//! test the waiterBot constructor
/**
 * @brief This tests makes sure that the constructor works
 * It should set the target locations to be 
 * (0,0) (10,0) (10,10) (10,0)
 * It should set the status to "in target location 1"
 * It should set the stopB to false;
 * It also test the getTargetLocs() and getStatus() didStop()
 * at the same time
 */
TEST(waiterBotTest, constructorTest) {
  waiterBot r;
  auto v = r.getTargetLocs();

  auto pv = v[0].getPos();
  EXPECT_EQ(pv[0], 0);
  EXPECT_EQ(pv[1], 0);
  EXPECT_EQ(pv[2], 0);

  pv = v[1].getPos();
  EXPECT_EQ(pv[0], 2);
  EXPECT_EQ(pv[1], 0);
  EXPECT_EQ(pv[2], 0);

  pv = v[2].getPos();
  EXPECT_EQ(pv[0], 2);
  EXPECT_EQ(pv[1], 2);
  EXPECT_EQ(pv[2], 0);

  pv = v[3].getPos();
  EXPECT_EQ(pv[0], 0);
  EXPECT_EQ(pv[1], 2);
  EXPECT_EQ(pv[2], 0);

  EXPECT_EQ("in target location 1", r.getStatus());
  EXPECT_FALSE(r.didStop());
}

//! test the getAngleDiff funtion
/**
 * @brief This tests makes sure its returns the angle difference correctly
 * Given a location it should say how much you need to turn in radians to head
 * straight for that location. Tests 9 differnt locations in all 4 quadrants
 */
TEST(waiterBotTest, getAngleDiffTest) {
  waiterBot r;
  position p;

  p.setPosition(0, 0, 0.1);
  EXPECT_EQ(0, r.getAngleDiff(p));

  p.setPosition(0, 1, 2);
  EXPECT_FLOAT_EQ(3.14/2, r.getAngleDiff(p));

  p.setPosition(1, 1, 3);
  EXPECT_FLOAT_EQ(0.78539819, r.getAngleDiff(p));

  p.setPosition(1, 0, 4);
  EXPECT_FLOAT_EQ(0, r.getAngleDiff(p));

  p.setPosition(1, -1, -5);
  EXPECT_FLOAT_EQ(3*3.14/2 + 0.78539819, r.getAngleDiff(p));

  p.setPosition(0, -1, 6);
  EXPECT_FLOAT_EQ(3*3.14/2, r.getAngleDiff(p));

  p.setPosition(-1, -1, 7);
  EXPECT_FLOAT_EQ(3.14+0.78539819, r.getAngleDiff(p));

  p.setPosition(-1, 0, 8);
  EXPECT_FLOAT_EQ(3.14, r.getAngleDiff(p));

  p.setPosition(-1, 1, 9);
  // EXPECT_FLOAT_EQ(2.3553982, r.getAngleDiff(p));
}

//! test the stop section of the move funtion
/**
 * @brief This tests the stop sections of code in the move function
 * If the distSensor says it is in collision the robot should stop
 */
TEST(waiterBotTest, moveStopTest) {
  waiterBot r;
  EXPECT_FALSE(r.ds.inCollision());
  EXPECT_FALSE(r.didStop());

  /**
   * Get collision data
   */ 
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 50,
    &distSensor::setDistReadingCallBack, &r.ds);
  ros::Duration(1).sleep();
  ros::spinOnce();

  // ros::Duration(5).sleep();
  auto m = r.move();
  // EXPECT_EQ(0, m.linear.x);
  EXPECT_EQ(0, m.linear.y);
  EXPECT_EQ(0, m.linear.z);
  EXPECT_EQ(0, m.angular.x);
  EXPECT_EQ(0, m.angular.y);
  EXPECT_EQ(0, m.angular.z);
  // EXPECT_TRUE(r.didStop());
}

//! test the no food section of the move funtion
/**
 * @brief This tests the no food sections of code in the move function
 * If the forceSensor states that there is no food
 * then the robot should change its status to heading to location 1
 */
TEST(waiterBotTest, moveNoFoodTest) {
  waiterBot r;

  /**
   * setting force message to be 0
   */
  std_msgs::Float32 msg;
  msg.data = 0;
  r.fs.setWeightCallBack(msg);

  /**
   * place robot not in location 1
   * since here it can never not have food
   */
  nav_msgs::Odometry omsg;
  omsg.pose.pose.position.x = 6;
  omsg.pose.pose.position.y = 2;
  r.mm.setCurrentLocationCallBack(omsg);

  auto m = r.move();
  EXPECT_EQ("heading to target location 1", r.getStatus());
}

//! test the in target location section of the move funtion
/**
 * @brief This tests the in target location sections of code in the move function
 * If the robot gets to a target location it changes its status to
 * "in target location #" If the robot has already been marked as being in
 * that specific target location it sets its status to move to the next location
 */
TEST(waiterBotTest, moveInTargetLocationTest) {
  waiterBot r;

  /**
   * place robot in location 2
   */
  nav_msgs::Odometry omsg;
  omsg.pose.pose.position.x = 2;
  omsg.pose.pose.position.y = 0;
  r.mm.setCurrentLocationCallBack(omsg);
  auto m = r.move();
  EXPECT_EQ("in target location 2", r.getStatus());
  auto m11 = r.move();
  EXPECT_EQ("heading to target location 3", r.getStatus());

  /**
   * place robot in location 3
   */
  omsg.pose.pose.position.x = 2;
  omsg.pose.pose.position.y = 2;
  r.mm.setCurrentLocationCallBack(omsg);
  auto m2 = r.move();;
  EXPECT_EQ("in target location 3", r.getStatus());
  auto m3 = r.move();
  EXPECT_EQ("heading to target location 4", r.getStatus());

  /**
   * place robot in location 4
   */
  omsg.pose.pose.position.x = 0;
  omsg.pose.pose.position.y = 2;
  r.mm.setCurrentLocationCallBack(omsg);
  auto m4 = r.move();
  EXPECT_EQ("in target location 4", r.getStatus());
  auto m5 = r.move();
  EXPECT_EQ("heading to target location 2", r.getStatus());

  /**
   * place robot in location 1
   */
  omsg.pose.pose.position.x = 0;
  omsg.pose.pose.position.y = 0;
  r.mm.setCurrentLocationCallBack(omsg);
  auto m6 = r.move();
  EXPECT_EQ("heading to target location 2", r.getStatus());
}
