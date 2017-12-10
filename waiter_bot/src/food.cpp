/**
 *@author Ruben Acevedo
 *@file food.cpp
 *@brief This is the ".cpp" file for the food Class
 *@copyright [2017] Ruben Acevedo
 *
 * This file implements the methods and attributes of the
 * food Class.
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
#include <std_msgs/Float32.h>
#include "food.hpp"


//! Global Variable
/**
 * This sets the max food weight
 * to be 25 Newtons
 */
float maxFoodWeight = 25;

//! Class Constructor
/**
 * @brief This code constructs the class.
 * It initializes the food weight to be 0
 * @param nothing
 * @return nothing
 */
food::food(): foodWeight(0) {
}

//! get food weight function
/**
 * @brief This code returns the food weight
 * @param nothing
 * @return float
 */
float food::getFoodWeight() {
  return foodWeight;
}

//! increase food weight function
/**
 * @brief This function increased the food weight to 25 N
 * @param nothing
 * @return nothing
 */
void food::addFood() {
  foodWeight = maxFoodWeight;
}

//! decrease food weight function
/**
 * @brief This function decreases the food weight by 5N
 * @param nothing
 * @return nothing
 */
void food::removeFood() {
  if (foodWeight - 0.2*maxFoodWeight < 0) {
    foodWeight = 0;
    return;
  }
  foodWeight = foodWeight - 0.2*maxFoodWeight;
}


