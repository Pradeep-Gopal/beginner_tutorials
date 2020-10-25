# beginner_tutorials
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Author

Pradeep Gopal

## Overview

This short tutorial on implementation of basic ROS C++ Publisher and Subscriber.
ROS publisher sends the messages and ROS subscriber receives the messages.

Talker (src/talker.cpp): Publisher
Listener (src/listener.cpp): Subscriber

## Dependencies

ROS Melodic should be installed on your computer (preferably Ubuntu 18.04).
Catkin workspace must be set up.

## Set up and build

- Suppose your catkin workspace is 'catkin_ws' which has build, src and devel folders.
- Open new terminal and run the command 
```
git clone --recursive https://github.com/Pradeep-Gopal/beginner_tutorials

```
- Move the cloned folder to catkin_ws/src.
- Open terminal and run the following commands:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  catkin_make

```
## Steps to run the publisher and subscriber using rosrun

- Open catkin_ws in a terminal and source your workspace's setup.sh file by running the following commands:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  
```
- Run the following command to start the master
```
  roscore
  
```
- Open new terminal and run the following commands to run the publisher:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun beginner_tutorials talker

```
- Open new terminal and run the following commands to run the subscriber:
```
  cd ~/catkin_ws
  source ./devel/setup.bash
  rosrun beginner_tutorials listener

```

## To terminate

press Ctrl-C to terminate both the listener and the talker

## Running cpp-check

```
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./src/")

```

## Running cpp-lint

```
cpplint $( find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./build/" -e "^./vendor/" -e "^./docs/" -e "^./results" )

```

