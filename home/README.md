# ROS Client

This repository contains all the files on the ROS client.
The action server script can be found in `home\ros\catkin_ws\src\turtlebot_move_action\scripts\action_server.py` and the obstacle detection script can be found in `home\ros\catkin_ws\src\project\scripts\obstacle_detection.py`.

## Obstacle Detection Algorithm

### Purpose of this program

This program detects nearby obstacles in the front, the back, right and left of the robot. Once an obstacle in one direction is detected, a message containing the direction is published on the `/obstacle` topic.

### Quick Start
1. Place the program on the script folder of a node
2. Run it from the command line

### Detection Method
To detect obstacles, the program performs the following tasks:
1. Obtain the most recent lidar value
2. Convert the received message format to get the distance to the nearest obstacle as a list corresponding to all 360 directions
3. Get the values of the 6 closest lidar rays to the desired direction
4. Compare the distances with the detection threshold and get compute the average of binarized distances
5. If enough of the 6 rays detected an obstacle, then a message is published on `/obstacle`.

### Notes
- Binarization prevents very small or big distances from having a great impact on the average. It creates a protection against potentially wrong values sent by the lidar
- The program uses 6 rays to increase accuracy, as it may produce some wrong values. The average reduces the impact of the error

### Improvements to Consider
- Cover more directions to match the shape of the robot more precisely to avoid having accidents in blind spots

## Action Server
This program allows the robot to move using manual commands. It takes as input the direction of the movement (forward, backward, left and right) and a distance value (in meters or degrees). Then it calculates for how long the robot should move at a fixed speed to achieve the given distance. Once the action is realized, it declares its success to the client via a topic.

If a new command is sent while another action is being realized, the current action is interrupted and the new one is executed. A failure message is returned to the client as a result of the first command being interrupted.