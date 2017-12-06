/**
 * @author Ruben Acevedo
 * @file foodTest.cpp
 * @brief This is the ".cpp" file for testing the food class
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
#include "food.hpp"

//! test the food constructor 
/**
 * @brief This tests makes sure that the constructor sets the foodWeight=0
 * It also test the get food weight at the same time.
 *
 */
TEST(foodTest1, constructorTest) {
  food f;
  EXPECT_EQ(0,f.getFoodWeight());
}

//! test the addFood function 
/**
 * @brief This tests makes sure when the addFood function is called it sets it to 
 * 25 Newtons
 */
TEST(foodTest, addFoodTest) {
  food f;
  f.addFood();
  EXPECT_EQ(25,f.getFoodWeight());
  f.addFood();
  EXPECT_EQ(25,f.getFoodWeight());
}

//! test the removeFood function 
/**
 * @brief This tests makes sure that when removeFood is called the weight reduces. 
 * It also checks that the foodweight can never be negative
 */
TEST(foodTest, removeFoodTest) {
  food f;
  f.removeFood();
  EXPECT_EQ(0,f.getFoodWeight());
  f.addFood();
  f.removeFood();
  EXPECT_EQ(20,f.getFoodWeight());
  f.removeFood();
  EXPECT_EQ(15,f.getFoodWeight());
  f.removeFood();
  EXPECT_EQ(10,f.getFoodWeight());
  f.removeFood();
  EXPECT_EQ(5,f.getFoodWeight());
  f.removeFood();
  EXPECT_EQ(0,f.getFoodWeight());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

