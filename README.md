# Integrating and Engineering Intelligent Systems

This repository contains the different codes used to do this project. They all contain their own readme. The `home` directory corresponds to the structure of the files on the ros client.

This code could be used on several devices with the following addresses:
## Roscore
ssh: `ros@172.16.1.108` (roscore-one)  
ssh: `ros@172.16.1.109` (roscore-two)  
ssh: `ros@172.16.1.110` (roscore-three)  
ssh: `ros@172.16.1.111` (roscore-four)  

## ROS clients
ssh: `ros@172.16.1.105` (turtlebot-one)  
ssh: `ros@172.16.1.106` (turtlebot-two)  
ssh: `ros@172.16.1.107` (turtlebot-three)  
ssh: `ros@172.16.1.112` (turtlebot-four)  

## Turtlebot
ssh: `ubuntu@172.16.1.101` (tb3-doc)  
ssh: `ubuntu@172.16.1.102` (tb3-sunshine)  
ssh: `ubuntu@172.16.1.103` (tb3-peanut)  
ssh: `ubuntu@172.16.1.104` (tb3-happy)  

## To launch everything, the following command should be performed:
```
# roscore
ssh ros@adress_roscore
ros # password
roscore -p 11311

# turtlebot
ssh ubuntu@adress_turtlebot
turtlebot # password
screen -S turtlebot3_robot roslaunch turtlebot3_bringup turtlebot3_robot.launch  #bot in general
ctrl + A ; ctrl + D
screen -S turtlebot3_rpicamera roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch  #camera module
ctrl + A ; ctrl + D
cd //home/ubuntu/catkin_ws/src/microrecognition/scripts
./sprecognition.py

# ros client
ssh ros@adress_ros_client
ros # password
screen -S rosbridge_websocket roslaunch rosbridge_server rosbridge_websocket.launch # ros bridge
ctrl + A ; ctrl + D
screen -S obstacle_detection #screen -r obstacle_detection
cd //home/ros/catkin_ws/src/project/scripts/
./obstacle_detection.py
ctrl + A ; ctrl + D
screen -S action_server
cd //home/ros/catkin_ws/src/turtlebot_move_action/scripts
./action_server.py
ctrl + A ; ctrl + D
screen -S map roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/ros/catkin_ws/src/project/map/own_map.yaml # launch slam action server

# Computer
Run python scripts:
- Image recognition
- Language processing
- Mic

Go in Agent_dictator and run `gradle`
```