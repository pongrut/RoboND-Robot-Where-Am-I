# RoboND-Jetbot-Where-Am-I
Project 2 of Udacity Robotics Software Engineer Nanodegree Program
[![Demo_Video](/videos/RoboND-Robot-Where-Am-I.gif)](https://youtu.be/imqXVSOmmfs)
![Jetbot_Model2](images/jetbot_model_2.png)  
## Overview  
This project contains two ROS packages inside `catkin_ws/src`: the `my_robot` and the `ball_chaser`. The Jetbot will be programmed to chase the white-colored ball inside the custom house. 
In this project you'll utilize ROS AMCL package to accurately localize a mobile robot inside a map in the Gazebo simulation environments.

## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  

sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl

## Run the project  
* Clone this repository
* Open the repository and make  
```
cd /home/catkin_ws/
catkin_make
```
* Launch my_robot in Gazebo to load both the world and plugins  
```
cd /home/workspace/catkin_ws/
source devel/setup.bash
roslaunch my_robot world.launch
```  
* Launch ball_chaser and process_image nodes  
```
cd /home/workspace/catkin_ws/
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```  
* Visualize  
```
cd /home/workspace/catkin_ws/
source devel/setup.bash
rosrun rqt_image_view rqt_image_view  
```  
