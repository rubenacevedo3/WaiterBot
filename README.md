# WaiterBot
[![Build Status](https://travis-ci.org/rubenacevedo3/WaiterBot.svg?branch=master)](https://travis-ci.org/rubenacevedo3/WaiterBot)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


## Product Description

Acme WaiterBot is a mobile robotic platform that delivers appetizers and drink trays at wedding receptions, networking events, and fundraisers during the cocktail hour. The motivation for this product is that human workers can focus their attention on making the food and drinks without having to spend the time walking around with trays and waiting for the guest to remove the food off of them. The fact that the waiters are robots adds another level of excitement to the event that they are being implemented in making the whole experience more enjoyable for the guest thus adding value to the people hosting the event. In addition, the WaiterBot would in time save the caterer business money since it is just an initial one time front cost to buy the whole robot but then you do not have to pay for its services by the hour like how you would for a human waiter. 

The way the robot works is that it carries a tray of food or drinks from the food/drink pick up location to specific target locations on the reception floor that will be hubs for a lot of people to gather around with the food/drink pickup location being the origin of the map. The robot first turns in place until its facing the target location. Then the robot moves forward until it reaches the target location. It is important to note that the robot checks its current location using its odometer sensor. In addition, there is a sensor on the robot that can tell if the robot is about to run into a person. In this scenario the robot will stop and wait for the person to move before continuing on its path towards the target location. The robot will also have a force senor to measure the weight of the food on its tray. Whenever the weight measurement drops to zero indicates that the tray has no more food, so the robot returns to the food/pick up location to get more. This can happen at any time regardless if the WaiterBot reached its previous targeted destination or not since people are free to approach the robot at any location to get food from the tray. It is important to note the robot will never return to the food/drink pick up location unless it has no more food on its tray. It will keep looping to the other locations until the food has been removed.

The main assumption for this platform is that all the obstacles are dynamic. The target locations on the map will be set so that the only potential obstacles will be humans moving around in the reception floor. For this reason it is feasible to assume that people will be able move out of the robots path eventually. Other assumptions are that the chefs will place the food on the robot platform and that the guest will remove it. This is simulated by a foodStub class that is a force sensor publisher. Whenever the robot is in the food/drink pick up location the foodStub publishes 9N. Every time the robot reaches a target location or is stopped due to an obstacle the foodStub publishes 1/3 of the initial food weight.  

The whole robot system is simulated for the demo using Gazebo with the TurtleBot acting as the base platform. The demo shows the TurtleBot starting from the food/drink pick up location (0,0) moving to 3 target locations (0,2), (2,2), and (2,0) meters. In the simulation there is one block representing a person. The robot will stop every time the block is in its way or if it reached its target location. It is improtant to note that in the demo, the block representing a person does not move on its own. However, it can be moved in real time using the Gazebo interface to fully show the waiterBot’s functionality. 

In the demo the TurtleBot will loop around all the target locations, stopping every time a block is in its way and it will wait 10 seconds at each target location. The TurtleBot returns back to the food/drink station only once its sense that all the food has been removed from its tray. The whole demo will continue to repeat until the program is terminated.


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
* tf package


## Known Issues/Bugs

The reason for why the target locations are about 2 meters apart is due to the fact that the robot is not calibrated enough to  reach further locations. The turning section of the code does not orient the robot perfectly to the target location.  The robot has to travel small distances in order for the errors in the turning section to be negligible for the robot’s overall performance success. With that being said, after multiple loops, the errors add up eventually and the robot will never reach the target location.

In the future this code should implement a PID controller in order to insure that the robot reach its target locations every time and at any distance away.  This was the original plan but due to time constraints it had to be pushed back to future works. 

Another issue is that the block representing a person does not load up in the Gazebo Simulator. When the world is open locally the block is there but when the launch file opens the same world file through the launch file the block is missing. This could be due to the fact that the Gazebo on my Virtual Machine is slow to upload worlds. In the future this issue would be thoroughly investigated. 
   

## Demo Default Values

* Travel velocity = 0.2 m/s
* Turning velocity = 0.05 rad/s
* Initial Food Weight = 9 N
* Location 1 (Food/Drink Pick-Up Location) = (0,0) meters
* Location 2 = (0,2) m
* Location 3 = (2,2) m
* Location 4 = (2,0) m
* Minimum Collision Distance = 0.75 m
* Wait time in target location = 10s
* Target Location Radius = 0.2 m
* Record Rosbag Default Setting "doRosbag = false"


## ROS Messages, Services, and Actions

Messages:
* sensor_msgs::LaserScan subscribing to the topic "/scan"
* std_msgs::float32 subscribing and publishing to the topic "force"
* geometry_msgs::Twist publishing to the topic "/mobile_base/commands/velocity"
* nav_msgs::Odometry subscribing to the topic "/odom"

Services: (none)

Actions: (none)


## ROS Nodes

waiterBot_node: 
* Runs the waiterBot's logic
* Subscribes to sensor_msgs::LaserScan messages from the "/scan" topic
* Subscribes to std_msgs::float32 messages from the "force" topic
* Subscribes to nav_msgs::Odometry messages from the "/odom" topic
* Publishes geometry_msgs::Twist messages to the "/mobile_base/commands/velocity" topic

foodStub_node:
* Simulates food being placed and removed off of the robot's tray
* Subscribes to sensor_msgs::LaserScan messages from the "/scan" topic
* Subscribes to nav_msgs::Odometry messages from the "/odom" topic
* Publishes std_msgs::float32 messages to the "force" topic

test_node:
* Simulates test scenarios in order to test node interactions 
* Publishes sensor_msgs::LaserScan messages to the "/scan" topic indicating that the robot is in collision 
* Publishes nav_msgs::Odometry messages to the "/odom" topic indicating that the robot is in location (2,0)


## Future Works

* Impliment a PID controller or use a Navigation Stack
* Create more tests to get 100% code coverage
* Create moving obsticals in Gazebo


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
$ roslaunch waiter_bot waiter_bot_demo.launch
# press ctrl+C to stop
```


## Steps for recording bag files with the launch file

(In terminal 2)
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ roslaunch waiter_bot waiter_bot_demo.launch  doRosbag:=true
# press ctrl+C to stop recording 
```


## Steps for inspecting the bag file

(In terminal 2)
```
$ cd ~/.ros
$ rosbag info waiterBotBagFile.bag
```


## Steps for playing back the bag file

(In terminal 2)
```
$ cd ~/.ros
$ rosbag play waiterBotBagFile.bag
```


## Steps to Run Tests

(In terminal 2)
```
$ cd ~/catkin_ws
$ source ./devel/setup.bash
$ rostest waiter_bot mainTest.launch
# it may take a minute to complete
```


## Step to See Code Coverage 

(In terminal 2)
```
$ sudo apt-get install lcov
$ cd ~/catkin_ws/build
$ lcov --directory . --capture --output-file coverage.info
$ lcov --list coverage.info
```


## Steps to Generate Doxygen Documentation

First install doxygen.

```
sudo apt install doxygen
```

To generate Doxygen Documentation in HTML and LaTEX, follow the next steps:

```
cd <path to repository>
mkdir <folder name>
cd <folder name>
doxygen -g <doxygen file name>
```
Inside the doxygen file change the following:

```
PROJECT_NAME = 'your project name'
INPUT = ../app ../include ../test
```
Run and generate the documents by running the next command:

```
doxygen <doxygen file name>
```

To view the documents go to the directory that you ran doxygen and open the directory called html. Then click on the file index.html to open it using you favorite browser.


## Steps to Run cpplint 

Use cpplint to identify potential source code issues that are in conflict with the Google C++ style guide. 

To install and run from terminal:

```
$ sudo apt-get install python-pip
$ sudo pip install cpplint
$ cd ~/catkin_ws/src/src/WaiterBot/waiter_bot
$ cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )
```


## Steps to Run cppcheck 

Use cppcheck for static code analysis.

To run from terminal:

```
$ cd ~/catkin_ws/src/WaiterBot/waiter_bot
$ cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```

 ## Five-minute Technical Presentation Video
 
 YouTube link:
 


## About Me

I am a 2nd year PhD student in the University of Maryland, College Park (UMD) Mechanical Engineering Department focusing on soft robotics. My PI is Dr. Hugh Bruck in the Advance Manufacturing Laboratory and I also collaborate with Dr. Ryan Sochol in the Bioinspired Advance Manufacturing Laboratory and Dr. S.K Gupta at the University of Southern California (USC). 

I am from California and Mexico and I got my bachelor’s degree in mechanical engineering from the University of California Los Angeles (UCLA). My interests are in bioinspired robotics, mobile robotics, soft robotics, and medical robotics. I am very new to coding but I have loved this learning experience in developing my ROS programing skills. 
