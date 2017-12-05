# WaiterBot
[![Build Status](https://travis-ci.org/rubenacevedo3/WaiterBot.svg?branch=master)](https://travis-ci.org/rubenacevedo3/WaiterBot)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Product Description

Acme WaiterBot is a mobile robotic platform that delivers appetizers and drink trays at wedding receptions, networking events, and fundraisers during the cocktail hour. The motivation for this product is that human workers can focus their attention on making the food and drinks without having to spend the time walking around with trays and waiting for the guest to remove the food off of them. The fact that the waiters are robots adds another level of excitement to the event that they are being implemented in making the whole experience more enjoyable for the guest thus adding value to the people hosting the event. In addition, the WaiterBot would in time save the caterer business money since it is just an initial one time front cost to buy the whole robot but then you do not have to pay for its services by the hour like how you would for a human waiter. 

The way the robot works is that it will carry a tray of food or drinks from the food/drink pick up location to specific locations on the reception floor that will be hubs for a lot of people to gather around with the food/drink pickup location being the origin of the map. The robot will use a PID controller to control the linear and angular velocities that the robot should move at in order to reach the target locations while checking its current location using its odometry sensor. In addition, there will be sensor on the robot that can tell if the robot is about to run into a person. In this scenario the robot will stop and wait for the person to move before continuing on its path towards the target location. The robot will also have a force senor to measure the weight of the food on its tray. Whenever the weight measurement drops to zero indicates that the tray has no more food, so the robot will have to return to the food/pick up location to get more. This could happen at any time regardless if the WaiterBot reached its previous targeted destination or not since people would be free to approach the robot at any location to get food from the tray. It is important to note the robot will never return to the food/drink pick up location unless it has no more food on its tray. It will keep looping to the other locations until the food has been removed.

The main assumption for this platform is that all the obstacles are dynamic. The target locations on the map will be set so that the only potential obstacles will be humans moving around in the reception floor. For this reason it is feasible to assume that people will be able move out of the robots path eventually. Other assumptions are that the chefs will place the food on the robot platform and that the guest will remove it. This is simulated by a food class that is a force sensor publisher. 

The whole robot system is simulated for the demo using Gazebo with the TurtleBot acting as the base platform. The PID controller that is used is an open source software and that can be found using this link: http://wiki.ros.org/pid. The demo shows the TurtleBot with a force senor on its head starting from the food/drink pick up location (0,0) moving to 3 target locations (0,10), (10,10), and (10,0). In the simulation there are moving blocks representing people. The robot will stop every time a block is in its way or if it reached its target location. The assumption for the demo is that every time the robot stops it is because people are removing food from its tray. For that reason the food class will publish (current weight -1/5 of the total weight) whenever it stops unless it is at the food/pick up location where the food class will publish the total food weight of 25 N. In the demo the TurtleBot will loop around all the target locations, stopping every time a block is in its way and it will wait 30 seconds at each target location. The TurtleBot returns back to the food/drink station only once its sense that all the food has been removed from its tray. The whole demo will continue to repeat until the program is terminated.


## SIP Code Development Process

Below is a link to the Product Backlog, Iteration Backlog, and Work Log for the development of this ROS Package

https://docs.google.com/spreadsheets/d/1DUyqNCOFJtVztRsu9-k8D0shkn-WgMdPOilfRRZVXlw/edit?usp=sharing

Below is a link to the SIP Notes Document

https://docs.google.com/document/d/1JN21H65LgtKyDHanWxHyH0hLVRiJsucrz1gQGTqj9Qk/edit?usp=sharing

## License

MIT License

Copyright 2017 Ruben Acevedo 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
© 2017 GitHub, Inc.

## Dependencies

* Ubuntu 
* ROS Kinetic
* c++11/14
* Catkin
* Gazebo
* Turtlebot Simulation stack
* roscpp package
* std_msgs package
* message_generation package
* sensor_msgs package

## Known Issues/Bugs

* bla
* bla
* bla

## Demo Default Values

* Travel velocity = 1 m/s
* Food Weight =  25 N
* Loaction 1 (Food/Drink Pick-Up Location) = (0,0) meters
* Location 2 = (0,10) meters
* Location 3 = (10,10) meters
* Location 4 = (10,0) meters
* Collision Distance = 0.1 meters

## ROS Messages, Services, and Actions

Messages:
* sensor_msgs::LaserScan subscribing to the topic "/scan"
* std_msgs::float32 subscribing and publishing to the topic "force"
* geometry_msgs::Twist publishing to the topic "/mobile_base/commands/velocity"

Services:
(none)

Actions:
* moveToPos.action 

## Steps to Build Demo

First, create and build a catkin workspace if you do not have one already

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

To add your catkin workspace into the ROS enviroment you need to source the generated setup file.

```
$ . ~/catkin_ws/devel/setup.bash
```

To download this repository to your catkin workpace do the following steps:

```
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/rubenacevedo3/WaiterBot.git
```

To build any catkin projects found in the src folder use: 
```
# In a catkin workspace
$ catkin_make
```

To build any test found in the test folder use: 
```
# In a catkin workspace
$ catkin_make test
```

## Steps to Run Demo

To install Turtlebot simulation stack type:
```
$ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```

To run the launch file:

(In terminal 1)
```
$ roscore
```

(In terminal 2)
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch WaiterBot waiterBotDemo.launch
# press ctrl+C to stop
```

## Steps to Run Tests
(In terminal 2)
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rostest beginner_tutorials talkerTest.launch
```

## Steps to Run cpplint 

Use cpplint to identify potential source code issues that are in conflict with the Google C++ style guide. 

To install and run from terminal:

```
$ sudo apt-get install python-pip
$ sudo pip install cpplint
$ cd ~/catkin_ws/src/RoombaTurtleBot/roomba_turtle_bot
$ cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )
```

## Steps to Run cppcheck 

Use cppcheck for static code analysis.

To run from terminal:

```
$ cd ~/catkin_ws/src/RoombaTurtleBot/roomba_turtle_bot
$ cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

## About Me

I am a 2nd year PhD student in the University of Maryland, College Park (UMD) Mechanical Engineering Department focusing on soft robotics. My PI is Dr. Hugh Bruck in the Advance Manufacturing Laboratory and I also collaborate with Dr. Ryan Sochol in the Bioinspired Advance Manufacturing Laboratory and Dr. S.K Gupta at the University of Southern California (USC). 

I am from California and Mexico and I got my bachelor’s degree in mechanical engineering from the University of California Los Angeles (UCLA). My interests are in bioinspired robotics, mobile robotics, soft robotics, and medical robotics. I am very new to coding but I have loved this learning experience to develop my ROS programing skills. 
