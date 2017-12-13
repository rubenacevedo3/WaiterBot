/**
 * @author Ruben Acevedo
 * @file distSensorTest.cpp
 * @brief This is the ".cpp" file for testing the distSensor class
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
#include <sensor_msgs/LaserScan.h>
#include "distSensor.hpp"
#include "ros/ros.h"

//! test the distSensor constructor
/**
 * @brief This tests makes sure that the constructor works
 * It should set the distReading to be 1000000
 * It also test the getDistReading() at the same time
 */
TEST(distSensorTest, constructorTest) {
  distSensor ds;
  EXPECT_EQ(1000000, ds.getDistReading());
}

//! test the inCollison function
/**
 * @brief This tests the inCollision function
 * to see if it can state whether or not the distReading
 * idicates that something is in front of it 
 */
TEST(distSensorTest, inCollisionTest) {
  distSensor ds;
  EXPECT_FALSE(ds.inCollision());
}

//! test the setDistReadingCallBack function
/**
 * @brief This tests the setDistReadingCallBack function
 * to see if can get the sensor data correctly 
 * It also retests the inCollision() function to make
 * sure it marks that it is in collision
 */
TEST(distSensorTest, setDistReadingCallBackTest) {
  distSensor ds;
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 50,
    &distSensor::setDistReadingCallBack, &ds);
  ros::Duration(1).sleep();
  ros::spinOnce();
  EXPECT_FLOAT_EQ(0.05, ds.getDistReading());
  EXPECT_TRUE(ds.inCollision());
}

