# Project_Finder : Finding Individuals for Disaster and Emergency Response 
A search and rescure bot which will detect humans in hazardous location and navigate them to nearest exit

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://app.travis-ci.com/naitri/Project_Finder.svg?branch=main)]( https://app.travis-ci.com/github/naitri/Project_Finder)
[![Coverage Status](https://coveralls.io/repos/github/naitri/Project_Finder/badge.svg?branch=main)](https://coveralls.io/github/naitri/Project_Finder?branch=main)

## Project contibutors

1) [Naitri Rajyaguru](https://github.com/naitri), Robotics Graduate Student at University of Maryland, College Park. 
2) [Mayank Joshi](https://github.com/mjoshi07), Robotics Graduate Student at University of Maryland, College Park. 

## Overview
Firefighters  all  over  the  world,  put  the  safety  of others  before  their  own.  But  still  they  are  not  equipped  with the latest tools and technology to combat the disastrous environment. By providing a better idea and map of the unknown environment, the firefighters can minimize the search time and can rescue or take necessary actions as quickly as possible to save human lives.Search and rescue  robots are  an upcoming type  of robots that can  work  along  with  the  firefighters  and  behave  as  the  first responder  to  any  situation.  They  would  search for  the  desired target,  rescue  if  possible  or  at  least  notify  the  location  of  the target to the team. We  propose  a  search  and  rescue  robot  which  could  work  in any indoor scenario. We  would also like to incorporate Acme Robotics state-of-the-art human detection and tracking module which  was  developed  by  us  last  year.  The  human  detection module  would  enhance  the  bot’s  capability  to  search  for humans in the environment because they are the desired target in  most  of  the  rescue  operations.  We  plan  to use  move_base for navigation, onboard LiDAR for dynamic obstacle avoidance and the RGB camera for human detection. Together  they  would  work  as  a  search  and  rescue  system. Testing  and  Simulation  would  be  done  on  ROS and  Gazebo with a single robot, which could be scaled to ‘N’ number of robots depending upon the environment size and other factors.

## Deliverables
* Project: Project Finder is search and rescue robot for hazardous environment
* Overview of prosposed work, including risks, mitigation, timeline
* UML and activity diagrams
* Travis code coverage setup with Coveralls
* Developer-level documentation

## Dependencies with licenses
* OpenCV 4.5.0 (covered under the open-source Apache 2 License)
* ROS Noetic 
* Move_base 
* Gazebo 
* GTest

## Development Process
Following the Agile Iterative Process for Development, we switch roles of driver and navigator. Product backlog, iteration backlog and worklog can be found [here](https://docs.google.com/spreadsheets/d/1rBFfK4g2CC1IsPqGAZWokGkGCv19n7TvAoxRm0TySUQ/edit?usp=sharing) and Sprint planning with review notes can be found [here](https://docs.google.com/document/d/1XgyItVMLgMgZYTeKPQwQybjKTmsq5tjbA4Ds0ETw-6w/edit?usp=sharing)

## World
* We created a custom gazebo world which could simulate the requirements for our project
![image](https://github.com/mjoshi07/project_finder/blob/dev_phase2/data/gazebo_world.png)
* For the custom world, we also generated a binary occupancy grid, loaded in rviz
![image](https://github.com/mjoshi07/project_finder/blob/dev_phase2/data/rviz_map.png)
* Generated map can be found [here](https://github.com/mjoshi07/project_finder/blob/dev_phase2/maps/)

## Static Code analysis
* Cppcheck results can be found [here](https://github.com/mjoshi07/project_finder/blob/dev_phase2/results/cppcheck.txt)
* Cpplint results can be found [here](https://github.com/mjoshi07/project_finder/blob/dev_phase2/results/cpplint.txt)

## Installation

# Install TurtleBot3
```
cd ~/finderbot_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b noetic-devel
cd ~/finderbot_ws
catkin_make
source devel/setup.bash
```
# Clone git repository
```
cd finderbot_ws/src
git clone --recursive https://github.com/naitri/project_finder
```

# Execution of code
```
cd ~/finderbot_ws/src
catkin_make
source devel/setup.bash
```
In terminal 1
```
roslaunch project_finder main.launch
```
In terminal 2
```
rosrun project_finder finder
```




