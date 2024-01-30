# Integrating and Engineering Intelligent Systems

## I - Introduction

This project aims to develop a self-sufficient robot that can perform certain tasks (e.g. transport items, craft items) while being aware of its surrounding environment.
Our design allows the robot to identify locations of workstations and item storages using signs and handle a crafting request that would use the registered locations.
The robot is designed to respond to changes in its environment, such as reacting to traffic lights or navigating around new obstacles.
Additionally, it is equipped with a microphone that makes it controllable with vocal commands.
The project uses ROS to control the robot and connect all its components.

## II - Organization
![Organization Diagram](ros-organization.jpg)

### 1. Functionnalities

#### Microphones
The system is composed of 2 microphones, one on the robot, and one on a computer. Both microphone inputs are transcribed into text using the Google speech-to-text API. The resulting string becomes available on the `/sprecognition` topic which the `Command Interpreter` subscribes to. This node will then publish formatted commands to the `/audio_command` topic. It also generates a string that describes the command onto the `/speech` topic which will be converted to audio and fed to the speaker in order to provide immediate feedback to the operators.

#### Speaker
The `Audio Synthesizer` subscribes to the `/speech` topic which holds the text to read out, and it publishes the generated audio to the speaker. This feature was not implemented on the robot because it wasn't equipped with one, although a demo using a computer speaker could have been considered.

#### Lidar
Data from the lidar on the `/scan` topic is handled by the `Obstacle Detection` node. When it detects an obstacle, it publishes its relative position (front, right, left, back) to `/obstacles`.

#### Camera
Data from the camera is handled by the `Image Processing` node on a laptop to offload the computationally heavy image processing task to a more powerful device. This node identifies signs in the image and determines their relative position to the robot (distance and angle). The identified information gets published to the `/sign` topic.

#### Mapper
The `Mapper` node is used at the very beginning to generate a full map of the environment that will be exploited by the SLAM process.

#### Action Dictator
The `Action Dictator` is implemented using JaCaMo and rosbridge. It serves to coordinate the multiple missions/goals that the robot would have to achieve. When it receives a command from the command interpreter, movement commands will be forwarded to the action server to manage all movement of the robot in a unified way. If it is a mission, it uses its internal reasoning as well as the position of the signs to determine the required action sequence. It then uses the `SLAM Action Server` to move the robot to a precise location. If an obstacle is detected, the node tries to plan the following actions to avoid it, and realizes it by using the custom action server. The speaker could be used by the node to express its current goal as well as occurring events.


### 2. Topic message formats
### /speech
A string containing the text to read out: `std_msgs/String`.

#### /audio
An audio format understood by the speaker.

#### /cmd_arm
In theory, this would control precisely the movement of the robot arm. In our project, this will simply be a string containing the different possible actions, for instance, `pick` or `drop`. Therefore, `std_msgs/String`.

#### /sign
Contains the type and relative position (distance and angle) to the detected signs in the following custom format (`project/sign_info`):

```json
{
    "data":
    {
        "sign":"oneStrip",
        "distance": 0.2, // in meters,
        "angle": 20.4 // in degrees
    }
}
```

#### /sprecognition
Contains the text version of the understood speech (`std_msgs/String`), which corresponds to the string returned by the Google API.

#### /obstacles
A string containing the direction of the detected obstacle (`std_msgs/String`). It can be `left`,`right`,`front` or `back`

#### /audio_command (/command is a reserved name)
Contains the commands extracted from the audio. The format is the following (`project/command`):

```json
{
    "data":{
        "command": "move",
        "args": {
            "type": "left", // "left", "right", "front", "back"
            "value": 10.36 // in degrees for turns and in meters for straight lines
        }
    }
}
```

or

```json
{
    "data":{
        "command": "action",
        "args": {
            "type": "make",
            "value": 1 // object id
        }
    }
}
```

or

```json
{
    "data":{
        "command": "toggle_sign",
        "args": {
            "type": "off"
        }
    }
}
```

## III- ROS Nodes
### 1. Obstacle Detection
To detect obstacles, the program performs the following tasks:
1. Obtain the most recent lidar value
2. Convert the received message format to get the distance to the nearest obstacle as a list corresponding to all 360 directions
3. Get the values of the 6 closest lidar rays to the desired direction
4. Compare the distances with the detection threshold and get compute the average of binarized distances
5. If enough of the 6 rays detected an obstacle, then a message is published on `/obstacle`.

### 2. Command Interpreter
To be able to extract commands from a string published on `/sprecognition`, the following actions are performed:
1. Add a list of synonyms to the basic commands to be able to detect a wider variety of phrasing.
2. Transform the text into tokens for analysis.
3. Transform the words to their radical so that a wider number of words can be analyzed (fait => faire).
4. Get the intersection for all the detectable commands and the radicals to detect a command
5. Transform any synonym commands to the basic one
6. Process specific commands:
    - The detected command is `faire ... object`, the command is analyzed to detect numbers and their position. The number before the word `object` is the amount of item to craf and the number after is the ID. The correct message is then published on `/audio_command`.
    - The detected command is a movement command (`droite`, `gauche`, `avancer`, `reculer`, `stopper`). The program extracts the value of the movement. By default, this value is in meters for forward and backward, and in degrees for turns. If the command contains the word `centimètre` or `radian`, the value is converted. The correct message is then publish on `/audio_command`.
    - The detected command is for sign processing activation. Depending on the detected command (`activer` or `désactiver`), the correct message is publish on `/audio_command`.
    - For other commands that do not yet have an implementation, the command is simply printed.

### 3. Image Processing
To detect signs, this program performs the following tasks:
1. Perform a vertical Sobel on the image to get the horizontal lines
2. Transform the sobel image in a binary image according to a fixed threshold
3. For each column of the sobel image:
    - Calculate the distance between horizontal lines
    - Using the ratio of distances, determine which sign is most likely to fit this pattern
4. If the same sign was detected on enough adjacent columns, a sign is successfully found
5. Get the coordinates of the top left and bottom right corners of the sign
6. Identify the type of sign using colors (basic signs could be red, green or white)
7. Determine the distance and the angle of the sign relative to the turtlebot based on its position on the image
8. Publish all the information on the corresping topic

### 4. Action Dictator
This application serves as the central intelligence of the system, interpreting messages from topics and responding accordingly:
- Sign Detection:
    1. If a red sign is detected, it restricts any movement outside the SLAM action server until a green light is seen.
    2. If a green sign is detected, it allows movements to resume.
    3. If a sign represents the position of a workstation and the application can update sign positions, it replaces the sign on the map if it significantly differs from the previous location.
- Obstacle Detection:
    - If the SLAM action server is not activated:
        - Obstacle on the right: The robot turns left.
        - Obstacle on the left: The robot turns right.
        - Obstacle in front of the robot: It moves backward.
        - Obstacle at the back of the robot: It moves forward.
    - If the SLAM action server is activated, the application issues a warning, indicating it cannot react to avoid interference with the SLAM action server.
- Robot Position Received:
    - The position in the belief base is updated.
- Command Received:
    - If it is a movement command, the action server is invoked to execute the command.
    - If it is a command to activate/deactivate sign position updates, it permits or disallows the update.
    - If it is a command to create an object, it retrieves the object's recipe, gathers all the items to the workbench, crafts the object, and transports it to storage.

### 5. Action Server
This program allows the robot to move using manual commands. It takes as input the direction of the movement (forward, backward, left and right) and a distance value (in meters or degrees). Then it calculates for how long the robot should move at a fixed speed to achieve the given distance. Once the action is realized, it declares its success to the client via a topic.

If a new command is sent while another action is being realized, the current action is interrupted and the new one is executed. A failure message is returned to the client as a result of the first command being interrupted.

## IV - Considered Improvements
### 1. Obstacle Detection
- Extend coverage in more directions to precisely match the robot's shape and avoid accidents in blind spots.

### 2. Command Interpreter
- Expand the program to handle commands such as `transporter`, `prendre`, and `déposer`.

### 3. Image Processing
- Address the 10-15 seconds delay issue between perception and reality by considering the following:
    - Compress the image before sending it to prevent saturating rosbridge.
    - Avoid outsourcing image processing to a computer, as the delay is caused by image transfer speed.
    - Investigate other parameters influencing this delay.
- Verify strip colors when testing signs with 2 and 3 strips to prevent misidentifications, even though it might hinder the detection of certain signs.
- Optimize sign detection by running it continuously only when necessary, saving computational resources.

### 4. Action Dictator
- Implement advanced obstacle avoidance for the robot to navigate around obstacles instead of just moving away.
- Allow the creation of multiple objects instead of a single object, based on user commands.
- Enhance red light handling to enable movement further away from the red light.
- Address lidar obstacle detection issues when crafting objects by adjusting the position of workstations on the map.
- Improve the stop command in the robot program to prevent long-term motor damage.
- Introduce the capability to move an object from one point to another, adding value to the simulation.
- Consider transforming the single-agent architecture into a multi-agent one for better management of robot features, enabling finer control and reducing unexpected conflicts and bugs.
To achieve this, it would be possible to create an agent for each monitored topic (using `watch` in hypermedea) and assign it a corresponding plan. For instance, when the agent managing the SLAM is performing an action, it would transmit a belief to the obstacle detection agent, notifying it of the ongoing action. This belief would effectively prevent any action from the obstacle detection agent. Once the agent managing the SLAM completes its action, it would retract the belief, and the obstacle detection agent could resume normal operation. Similarly, it would be feasible to deactivate any image processing during SLAM.
To facilitate the easy management of agents' activation and deactivation in specific situations, one could register any affected agent in the group "Interrupted_Slam" at the beginning. Subsequently, informing any agent within this group would efficiently communicate the occurrence of an interruption to the relevant agents. For enhanced interoperability, these groups could be assigned within the RDF, fostering better coordination among multiple turtlebots. Overall, this improvement would enable finer control of performed actions, resulting in fewer unexpected conflicts and reduced bugs.

### 5. Microphones
- Investigate and decrease the delay in the robot's microphone functionality.
- Reduce the delay in audio-to-text conversion by exploring local speech-to-text models.
- Implement a push-to-talk module to address issues where the microphone fails to detect the end of a command in noisy environments.

### 6. General
- Enhance realism in the simulation by adding a speaker and an arm to the robot.
- Utilize computer speakers to complete the simulation, similar to the approach taken with the microphone.

## V - Notes
### 1. Obstacle detection
- Binarization prevents very small or large distances from having a significant impact on the average. It safeguards against potentially incorrect values sent by the lidar.
- The program uses 6 rays to enhance accuracy, mitigating the impact of potential errors. The average calculation further reduces error influence.
- Before employing JaCaMo to handle obstacle avoidance, an initial program was implemented in Python without using the action server. It directly published on `/cmd_vel`.
- An issue arose when a lidar data processing task took more time than the frequency of new data arrivals, resulting in a gap between the actual environment and the robot's perception. This caused the robot to execute somewhat random actions, making it challenging to pinpoint the issue's location.

### 2. Image Processing
- To prevent false detections on small details, a minimum sign height is imposed.
- To improve computational time, the image quality is reduced by a factor of 2.
- Tolerances on the previously stated rules were added and fine-tuned to achieve the best possible results.
- To publish information about sign detection, a new custom message was created in ROS.
- This part of the project was the most challenging. We explored various methods:
    - Initially, we attempted edge detection by identifying rectangles in the image, but this technique yielded poor results due to false detections and high computational intensity.
    - Next, we tried multi-scale pattern matching by finding a distorted image of a known sign in a larger image. While this solution worked better, it was still computationally intensive.
    - We considered a machine learning algorithm, but without a labeled database for training, it was deemed impossible due to time constraints.
- Identifying the delay source was challenging. We tested the camera directly on the robot, showing low latency. The program execution time was also not the issue, as it only took a couple of tenths of a second. The noticeable difference between the quick live camera and the slow program pointed to rosbridge as the problem. To alleviate stress on rosbridge, we attempted to reduce the number of images per second published on the topic to 2, but it didn't resolve the issue.
- We made various small changes to the program's tolerances (e.g., how many pixels can be skipped on a line to still be considered a line) to ensure it detects all signs in as many different situations as possible.
- To detect colors even in poor lighting conditions, we converted them to the HSV format and ignored the luminosity component.
- To improve image processing time, we reduced the image size by two, slightly impacting detection performance.
- For debugging purposes, we added information to the image, ensuring two separate images to avoid contaminating sign detection with debugging info.

### 3. Action Dictator
- To easily swap robots and topic names, generic representations as RDF tuples in a turtle file are converted into usable beliefs when the program starts. This enhances the system's interoperability and integration capabilities within a larger project.
- The addition of the sign update toggling addresses the delay between sign perception and information publication on the topic. If the robot is moving, this minimizes inaccuracies caused by latency differences between the robot's position and sign information.


### 4. Slam Action Server
- The SLAM action server is by default configured for open spaces, resulting in a large obstacle detection distance. To adapt to smaller spaces, we decreased the detection distance and robot speed, facilitating navigation in the factory model.
- To avoid obstacle problems, a workstation is considered reached if the robot is near enough.

### 5. Microphones
- To enhance interaction with the robot, we opted not to use a trigger word. This decision avoids slowing down interaction for minimal benefits since language processing extracts useful information from provided text, minimizing the risk of accidentally triggering a command. On the privacy side, using or not using a trigger word in this context does not change anything.

### 6. General
- For a better understanding of the different programs, take a look at the codes which are commented