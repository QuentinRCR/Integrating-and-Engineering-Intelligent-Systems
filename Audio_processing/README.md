# Voice processing algorism

This repository is composed of 2 programs:
- A microphone program that listen and publishes the text to the corresponding topic
- A language processing program that takes a text from a topic as input and extract the different commands 

## How to use both program
1. Make sure to be connected to the same network than the turtlebot
2. Make sure rosbridge is launched
3. Specify the address of the raspberry with the screen in the function `connect_to_ros_bridge`
4. Run the program

## The microphone program
To be able to transform a vocal command to a text, the microphone program do the following actions:
1. Calibrate the threshold that determines when the user finished talking (to adapt to ambient noise) 
2. In a loop as long as rosbridge is connected:
    - Listen for a command
    - Send this sound to the google API
    - Get the text back
    - publish the understood text on `/sprecognition`

## The language processing program
To be able to extract the correct actions from a text published on `/sprecognition`, the following actions are performed:
1. Add a list of synonyms to the basic commands to be able to detect a wider variety of phrasing.  
1. Transform the text into tokens so that it can be analyzed.
2. Transform the words to their radical so that a wider number of words can be analyzed (fait => faire).
3. Get the intersection for all the detectable commands and the radicals to detect a command
4. Transform any synonym command to the basic one
5. Process specific commands:
    - The detected command is `faire ... object`, the command is analyzed to detect numbers and their position. The number before the word `object` is the quantity of object to be crafted and the number after is the id. The correct message is then publish on `/audio_command`.
    - The detected command is a movement command (droite,gauche,avancer,reculer,stopper). The program extracts the value of the movement. By default, this value is in meters for forward and backward, and in degrees for turns. If the command contains the word centimeter or radian, the value is converted. The correct message is then publish on `/audio_command`.
    - The detected command is a sign processing activation. Depending on the detected command (activer or désactiver), the correct message is publish on `/audio_command`.
    - The detected command is different than any of the previous one. These command are not implemented and the detection is only printed.

## Note
- For more details on the program, please refer to the commented code

## Possible improvements:
- This program could be completed to handle the commands `transporter`, `prendre` and `déposer`