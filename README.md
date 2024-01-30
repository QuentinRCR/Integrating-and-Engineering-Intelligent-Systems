# Integrating and Engineering Intelligent Systems

This repository contains all the code that has been used for do this project.
Each script/code-snippet has its own README file that explains the usage.
The `home` directory mimics the file structure on the ROS client.

The code can be used on all the available devices.

## ROS master

roscore-one: `172.16.1.108`  
roscore-two: `172.16.1.109`  
roscore-three: `172.16.1.110`  
roscore-four: `172.16.1.111`

## ROS clients (username: 'ros' | password: 'ros')

turtlebot-one: `172.16.1.105`  
turtlebot-two: `172.16.1.106`  
turtlebot-three: `172.16.1.107`  
turtlebot-four: `172.16.1.112`

## ROS robots (username: 'ubuntu' | password: 'turtlebot')

tb3-doc: `172.16.1.101`  
tb3-sunshine: `172.16.1.102`  
tb3-peanut: `172.16.1.103`  
tb3-happy: `172.16.1.104`

## Launching commands

We use the screen command to run multiple processes without blocking the shell and manage each of the more easily.  
The keyboard shortcuts in the following command succession are used to detach a screen.  
You can see all running screens with `screen -ls` and attach with `screen -r screen_name`

```
# roscore
ssh ros@adress_roscore # password: 'ros'
roscore -p 11311

# turtlebot
ssh ubuntu@adress_turtlebot # password: 'turtlebot'
screen -S turtlebot3_robot roslaunch turtlebot3_bringup turtlebot3_robot.launch  # launch turtlebot
ctrl + A ; ctrl + D
screen -S turtlebot3_rpicamera roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch  # launch the camera module
ctrl + A ; ctrl + D
cd //home/ubuntu/catkin_ws/src/microrecognition/scripts
./sprecognition.py

# ros client
ssh ros@adress_ros_client # password: 'ros'
screen -S rosbridge_websocket roslaunch rosbridge_server rosbridge_websocket.launch # launch rosbridge
ctrl + A ; ctrl + D
screen -S obstacle_detection 
cd //home/ros/catkin_ws/src/project/scripts/
./obstacle_detection.py # launch obstacle detection
ctrl + A ; ctrl + D
screen -S action_server
cd //home/ros/catkin_ws/src/turtlebot_move_action/scripts
./action_server.py # launch the action server
ctrl + A ; ctrl + D
screen -S map roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/ros/catkin_ws/src/project/map/own_map.yaml # launch the slam action server

# Computer
Run python scripts:
- Image recognition
- Language processing
- Mic

Go in Agent_dictator and run `gradle`
```