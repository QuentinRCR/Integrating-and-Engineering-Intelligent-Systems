# Voice Processing Algorithm

This repository contains 2 programs:
- A microphone program that captures audio, extracts speech from it, and publishes the resulting string to a topic
- A language processing program that takes text from a topic and tries to extract commands from it, to then publish to another topic

## Quick Start
1. Make sure to be connected to the same network as the turtlebot
2. Make sure rosbridge is launched
3. Specify the address of the raspberry with the screen in the function `connect_to_ros_bridge`
4. Run the program

## Microphone Program
To be able to transform a vocal command to text, the microphone program does the following:
1. Calibrate the threshold that determines when the user finished talking (to adapt to ambient noise) 
2. In a loop (as long as rosbridge is connected):
    - Listen for audio
    - Send the audio to the Google API
    - Receive the resulting text
    - Publish the string on `/sprecognition`

## Language Processing Program
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

## Notes
- For more details on the program, please refer to the commented code

## Improvements to Consider:
- This program could be completed to handle the commands `transporter`, `prendre` and `déposer` since the Action Dictator is able to handle them