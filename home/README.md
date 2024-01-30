# ROS client

This repository contains all the files on the ROS client. The action server can be found in `home\ros\catkin_ws\src\turtlebot_move_action\scripts\action_server.py`and the obstacle detection can be found in `home\ros\catkin_ws\src\project\scripts\obstacle_detection.py`.

## Obstacle detection algorithm

### Purpose of this program
It detects near obstacles in front, in the back, right and left of the robot. Once one an obstacle in one direction is detected, a message containing the direction is published on `/obstacle`.

### How to use the program
1. Place the program on the script folder of a node
4. Run it from the command line

### Principle of the detection
To detect obstacles, the program perform the following tasks:
1. Get the most recent lidar value
2. Convert the received message format to get the distance to the nearest obstacle as a list of list for the 360 positions
3. Get the values of the 6 lidar rays closer to the direction watched
4. Compare the distances with the distance detecting an obstacle and get the average of the binarised distances
5. If enough of the 6 rays detected an obstacle then a message is published in `/obstacle`.

### Note
- The binarization prevents very small or big distances from having a great impact on the average. It creates a protection against possible wrong values sent by the lidar
- The program uses 6 rays to increase accuracy. Indeed, sometimes the lidar will send a wrong value for a ray, which can create false alarms. The average reduce the number of false alarms

### Possible improvements:
- It would be possible to convert more orientation to bring the detecting even closer to the physical dimensions of the robot (ex: the diagonals are longer so the sides need to detect object from further away to take them into account)

## Action server
This program allows the robot to move using manual commands. It takes as input the direction of the movement (forward, backward, left and right) and the distance (in meters or degrees). It then calculated how long the robot needs to move at a defined speed to fulfill the command. Once the action is realized, it returns the info of its success to the client.  
If a command is sent while another action is being realized, the current action is interrupted and the new one is executed. A failure is then returned to the client as a result of the first command being interrupted.