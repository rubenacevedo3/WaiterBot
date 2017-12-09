/**
 *@author Ruben Acevedo
 *@file position.cpp
 *@brief This is the ".cpp" file for the position Class 
 *@copyright [2017] Ruben Acevedo
 *
 * This file implements the methods and attributes of the
 * position Class.
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

#include <vector>
#include "position.hpp"

//! Class Constructor
/**
 * @brief This code constructs the class.
 * It initializes the xpos=ypos=theta=0
 * @param nothing
 * @return nothing
 */
position::position(): xpos(0), ypos(0), theta(0) {
}

//! Set the Position Function
/**
 * @brief This sets the postion
 * @param a const float reference for the x coordinate
 * @param a const float reference for the y coordinate
 * @param a const float reference for the theta coordinate
 * @return nothing
 */
void position::setPosition(const float& x, const float& y, const float& t) {
  xpos = x;
  ypos = y;
  theta = t;
}

//! get the position function
/**
 * @brief This function returns the stored position
 * in the form of a vector where
 * the first element is xpos, the second is
 * y pos, and the third is theta
 * @param nothing
 * @return a float vector repesenting the position
 */
std::vector<float> position::getPos() {
  std::vector<float> v;
  v.push_back(xpos);
  v.push_back(ypos);
  v.push_back(theta);
  return v;
}
