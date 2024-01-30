# Integrating and Engineering Intelligent Systems

## I - Introduction
The goal of this project was to develop a self-sufficient robot capable of analyzing its surroundings to identify the location of materials. Once identified, the robot is programmed to construct a predefined object using these materials. The robot is designed to respond to changes in its environment, such as reacting to traffic lights or navigating around new obstacles. Additionally, it is equipped to respond to verbal commands.
The project uses ROS to achieve this automatisation.

## II - Organization of the project
![Diagram of the organization](ros-organization.jpg)

Our organization of the different components of the ros infrastructure is represented by the above image. Relations to topics are represented by arrows with a P for publishing and S for subscribing. Interactions with action servers are client-server interaction represented by C/S arrows.

### 1. Different functions

#### Microphones
The system is composed of 2 microphones, one on the robot, and one on a computer. Both microphone inputs are transcribed into text using the Google speech-to-text API. The resulting string becomes available on the `/sprecognition` topic which the `Command Interpreter` subscribes to. This node will then publish formatted commands to the `/command` topic. It also generates a string that describes the command onto the `/speech` topic which will be converted to audio and fed to the speaker in order to provide immediate feedback to the operators.

#### Speaker
The `Audio Synthesizer` subscribes to the `/speech` topic which holds the text to read out, and it publishes the generated audio to the speaker. This feature was not implemented on the robot because of a lack of speakers. It could have been implemented on a computer.

#### Lidar
Data from the lidar on the `/scan` topic is handled by the `Obstacle Detection` node. When it detects an obstacle, it publishes its relative position (front, right, left, back) to `/obstacles`.

#### Camera
Data from the camera is handled by the `Image Processing` node on a laptop to offload the computationally heavy image processing task to a more powerful device. This node identifies signs in the image and determines their relative position to the robot (distance and angle). The identified information gets published to the `/signs` topic.

#### Mapper
The `Mapper` node is used at the very beginning to generate a full map of the environment that will be exploited by the SLAM process.

#### Action dictator
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

## III- How does the different node works
### 1. Obstacle detection
To detect obstacles, the program perform the following tasks:
1. Get the most recent lidar value
2. Convert the received message format to get the distance to the nearest obstacle as a list of list for the 360 positions
3. Get the values of the 6 lidar rays closer to the observed direction
4. Compare the distances with the distance detecting an obstacle and get the average of the binarised distances
5. If enough of the 6 rays are detecting an obstacle then a message is published in `/obstacle`.

### 2. Command interpreter
To be able to extract the correct actions from a text published on `/sprecognition`, the following actions are performed:
1. Add a list of synonyms to the basic commands to be able to detect a wider variety of phrasing (ex: avancer <=> avant, devant...).  
1. Transform the text into tokens so that it can be analyzed.
2. Transform the tokens to their radicals so that a wider number of words can be analyzed (fait => faire).
3. Get the intersection for all the detectable commands and the radicals to detect a command
4. Transform any detected synonym command to the initial word
5. Process specific commands:
    - The detected command is `faire ... object`, the command is analyzed to detect numbers and their position. The number before the word `object` is the quantity of object to be crafted and the number after is the id. The correct message is then published on `/audio_command`.
    - The detected command is a movement command (droite,gauche,avancer,reculer,stopper). The program extracts the value of the movement. By default, this value is in meters for forward and backward, and in degrees for turns. If the command contains the word centimeter or radian, the value is converted. The correct message is then published on `/audio_command`.
    - The detected command is a sign processing activation. Depending on the detected command (activer or désactiver), the correct message is published on `/audio_command`.
    - The detected command is different from any of the previous ones. These commands are not yet implemented and the detected command is only printed.

### 3. Image Processing
To detect signs, this program perform the following tasks:
1. Perform a vertical sobel on the image to get the horizontal lines
2. Transform the sobel image in a binary image, based on a threshold
3. For each column of the sobel image:
    - Calculate the distance between the horizontal lines
    - By looking at the ratio of distances, determine which sign is most likely to be in this column
4. Go through each column. If the same sign was detected on enough adjacent columns, then a sign is indeed present
5. Get the coordinates of the top left and bottom right corner of the sign
6. Identify which type of sign it is by looking at the color (for basic signs that are white, red and green)
7. Determine the distance and the angle of the sign compared to the turtlebot based on its position on the image
8. Publish all the info on the sign topic

### 4. Action dictator
This application is the brain of the system. it reads messages from topics and reacts accordingly:
- A sign is detected:
    1. The sign is red: it prevents any movement outside of the slam action server as long as it does not see a green light
    2. The sign is green: it allows movements again
    3. The sign is the position of a workstation: If the application can update the position of the signs and the sign is at a very different place than previously thought, the sign is replace on the map
- An obstacle is detected:
    - The slam action server is not activated:
        - Obstacle on the right: the robot turns left
        - Obstacle on the left: the robot turns right
        - Obstacle on front of the robot: it moves backward
        - Obstacle on the back of the robot: it moves forward
    - The slam action server is activated:
        - The application warn that it can't react to avoid interfering with the SLAM action server
- A position of the robot is received:
    - The position in the believe base is updated
- A command is received:
    - It is a movement command: The action server is called to perform the command
    - It is a command to activate/deactivate the sign position update: It allows the update
    - It is a command to create an object: It gets the recipe of the object, brings all the items to the workbench, crafts it and brings it to the storage.

### 5. Action server
This node allows the robot to move using manual commands. It takes as input the direction of the movement (forward, backward, left and right) and the distance (in meters or degrees). It then calculated how long the robot needs to move at a defined speed to fulfill the command. Once the action is realized, it returns the info of its success to the client.  
If a command is sent while another action is being realized, the current action is interrupted and the new one is executed. A failure is then returned to the client as a result of the first command being interrupted.

## IV - Possible improvements
### 1. Obstacle detection
- It would be possible to cover more orientation (ex: 8 orientations) to bring the obstacle detection even closer to the physical dimensions of the robot (ex: the diagonals are longer so the distance to detect on obstacle need to be greater on the sides to take them into account)

### 2. Command interpreter
- This program could be completed to handle the commands `transporter`, `prendre` and `déposer`

### 3. Image Processing
- This program suffers from a 5 second delay between what's perceived and the reality. To fix this it would be possible to:
    - Compress the image before sending it to prevent saturating the network
    - Avoid outsourcing the processing to a computer since this delay appears to only appear on external devices
    - Investigate more the source of the problem to find improvements
- Testing the color of the strips while testing signs with 2 and 3 strips would prevent some misidentifications. However, it might prevent the detection of said sign all together
- Currently, the detection of signs is running continuously, which requires significant computational resources. However, to update the location of the workstations, a feature needs to be toggled. An improvement would be to activate the sign processing only when this feature is activated or when the robot is near a traffic light. This would save a lot of ressources

### 4. Action dictator
- It would be possible to implement a more advanced obstacle avoidance so that the robot goes around the obstacle instead of just going away from the obstacle
- Add the possibility to create several objects (currently it will always create a single object, even if the command asked for 2)
- It would be possible to create a smarter red light handling, for instance allowing a movement to go further away from the red light
- If we leave the signs indicating the workstations while trying to craft an object, the robot doesn't want to reach the position because the lidar is detecting an obstacle. A solution would be to stick the signs on the walls and shift the position of the workstation ever so slightly inside the map. This would allow for a true live workstation positioning.
- Currently, to stop the robot, an instruction to go one millimeter forward is given. This is not ideal, however it simplifies the program since it eliminates the need to publish on the cancel topic of the action server. Adding a real stop would improve the system, since sending a small distance forward might damage the motor on the long run.
- Adding the possibility to move an object from one point to another would add value to the simulation
- Transforming the single agent architecture into a multi-agent one could help to better manage the different features of the robot.  
To do so, it would be possible to simply create an agent for every watched topic and give it its corresponding plan. Then, for instance when the agent managing the slam would be performing an action, it would send a belief to the obstacle detection agent informing it that it is performing an action. This belief would effectively prevent any action from this agent. When the agent managing the slam would have finish its action, it would untell the believe and the obstacle detection agent would be able to work normally. In a similar manner, it would be possible to deactivate any image processing during slam.  
To manage the agents’ activation and deactivation in certain situations easily, it would be possible to register any impacted agent in the group "Interupted_Slam" at the beginning. It would then be very easy to inform the correct agents when an interruption occurs by informing any agent of the group. For even more interoperability, it would be possible to assign these groups inside the RDF. This would allow for a better coordination between several turtlebots.  
Overall, this improvement would allow for a finer control of the performed actions which would lead to less unexpected conflicts and less bugs.  

### 5. Microphones
- The microphone on the robot is working but has a delay of several minutes. The reason for this delay has not been investigated. A possible improvement could be to understand and decrease this delay. The microphone of the computer was used to get around this problem.
- The delay to convert the audio to the text is a bit long. Investigating with local speech-to-text models could help reducing this delay
- Sometimes, the microphone doesn't detect the end a the command. It thus doesn't send the audio to the google API and the command is delayed. To solve this, it would be possible to implement a push to talk module that starts recording when a key is pressed, and stop when it's releases. This feature would be very handy in noisy environment when the end of a command is hard to detect.   

### 6. General
- Adding a speaker and an arm to the robot could help to make this simulation more realistic.
- In a similar manner that what was done with the microphone, it would be possible to use the speakers of a computer to complete the simulation.

## V - Notes

### 1. Obstacle detection
- The binarization of the distance to the obstacle prevents very small or big distances from having a great impact on the average. It creates a protection against possible wrong values sent by the lidar
- The program uses 6 rays to increase accuracy. Indeed, sometimes the lidar will send a wrong value for a ray, which can create false alarms or miss an obstacle. The average reduces the number of false alarms.
- Before using Jacamo to handle the obstacle avoidance, a first program was realized in python without using the actions server. It was directly publishing on `/cmd_vel`
- We encountered an issue when a process for handling lidar data started taking more time than the frequency of new data arrivals. This resulted in a gap between the actual environment and what the robot was perceiving. Consequently, the robot started carrying out somewhat random actions, making it challenging to pinpoint the issue's location.  

### 2. Image Processing
- To be able to publish the information of the detection of a sign, a new custom message was created in ROS
- This part of the project was the most complicated to realize. Indeed, we tried different methods before:
    - We first tried to detect the edges of the signs by detecting rectangles in the image. However, this technique gave very bad results because it creates a lot of false detections, was computationally intensive and most sign were not even detected because they don't appear as perfect rectangles on camera.      
    - We then tried to do a multi-scale pattern matching by trying to find a distorted image of a known sign in a bigger image. This solution worked better but was also very companionably intensive
    - We thought about doing a machine learning algorithm, however without any labeled database to do the training, it was impossible. The labeling of each sign would have taken too much time
- We had a hard time figuring out where the delay was coming from. We checked by testing the camera directly on the robot, and it showed low latency. We also looked into the program execution time, but that's not the issue because it only takes a couple of tenths of a second. The noticeable difference between the quick live camera and the slow program points to rosbridge being the problem.  
To diminish the stress on rosbridge, we tried to diminish the number of images per second published on the topic to 2 but it didn't solve the issue.
- We did a lot of little changes on the tolerances of the program (ex: how many pixel can be skipped of a line to still be considered a line) so that it detects all signs in as many different situation as possible
- To be able to detect colors even with bad lighting conditions, we converted them to the HSV format and ignored the luminosity component
- To improve the processing time of an image, we diminished its size by two. It decreased slightly the performance of the detection
- For debug purposes, we were adding info on the image. However, we made sure to have 2 separate images in order to avoid polluting the sign detection because of the debugging info

### 3. Action dictator
- To be able to easily swap robots as well as the name of the topic, anything that is generic is represented as rdf tuples in a turtle file. When the program starts, it converts these tuples into usable beliefs. This improves the interoperability of the system as well as its capacity to be integrated inside a bigger project.
- The addition of the toggling of the sign update was added because of the delay between the sign perception and the publication of the info on the topic. Indeed, if the robot is moving, it will place the sign at the wrong place because of the difference of latency between the robot position and the information of a sign.


### 4. Slam action server
- The slam action server is by default configured to be used in open spaces. It means that its obstacle detection distance is very big by default. To be able to make it work in a smaller space, we decreased the detection distance as well as the speed of the robot. This allowed the robot to navigate the factory model much more easily.
- To avoid obstacle problems, a workstation is considered reached if the robot is near enough

### 5. Microphones
- To improve the quality of the interaction with the robot, we choose to not use a trigger word. Indeed, it would have made the interaction much slower for little benefits. Indeed, the language processing only extracts useful information from the provided text, so the risk of giving a command accidentally is low. On the privacy side, we were forced to send any audio processed to Google to extract the trigger word, so the use or not of a trigger word on this subject doesn't change anything.

### 6. General
- For a better understanding of the different programs, every code is commented

## VI - Repartition of the work
The time required is mentioned between brackets

**Arcady:**
- Setup (LOW)
- Action server (MEDIUM)
- Action dictator (HIGH)
- RDF (MEDIUM)

**Quentin:**
- Audio processing (MEDIUM)
- Image processing (HIGH)
- Microphones (MEDIUM)
- Custom messages (LOW)
- Obstacle detection (MEDIUM)