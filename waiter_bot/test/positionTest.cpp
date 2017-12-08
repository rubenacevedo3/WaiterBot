/**
 * @author Ruben Acevedo
 * @file positionTest.cpp
 * @brief This is the ".cpp" file for testing the position class
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
#include <vector>
#include "position.hpp"

//! test the position constructor
/**
 * @brief This tests makes sure that the constructor sets everything to 0
 * It also test the getPos() at the same time.
 */
TEST(positionTest, constructorTest) {
  position pos;
  auto v = pos.getPos();
  EXPECT_EQ(0, v[0]);
  EXPECT_EQ(0, v[1]);
  EXPECT_EQ(0, v[2]);
}

//! test the setPosition function
/**
 * @brief This tests that you can set the position
 * It also test the getPos() at the same time.
 */
TEST(positionTest, setPositionTest) {
  position pos;
  pos.setPosition(6, 2, 0.5);
  auto v = pos.getPos();
  EXPECT_EQ(6, v[0]);
  EXPECT_EQ(2, v[1]);
  EXPECT_EQ(0.5, v[2]);
}
