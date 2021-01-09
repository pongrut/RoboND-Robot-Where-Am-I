# Jetbot-ROS-Go-Chase-It
Project 2 of Udacity Robotics Software Engineer Nanodegree Program
[![Demo_Video](/videos/RoboND-Robot-Where-Am-I.gif)](https://youtu.be/imqXVSOmmfs)
![Jetbot_Model2](images/jetbot_model_2.png)  
## Overview  
This project contains two ROS packages inside `catkin_ws/src`: the `my_robot` and the `ball_chaser`. The Jetbot will be programmed to chase the white-colored ball inside the custom house. 

## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  

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
