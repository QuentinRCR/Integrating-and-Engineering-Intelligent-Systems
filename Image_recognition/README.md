# Sign detection algorism

## How to use the program
1. Make sure to be connected to the same network than the turtlebot
2. Make sure rosbridge is launched
3. Specify the address of the raspberry with the screen in the function `connect_to_ros_bridge`
4. Run the program

## Principle of the detection
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

## Note
- To prevent false detection on small details, a minimum sign hight is imposed
- To improve the computational time, the quality of the image is reduced by a factor of 2
- Some tolerances on the previously stated rules were added and tuned to get the best possible results
- For more details on the program, please refer to the commented code

## Possible improvements:
- This program suffers from a 5 second delay between what's perceived and the reality. To fix this it would be possible to:
    - Compress the image before sending it to prevent saturating rosbridge
    - Avoid outsourcing the processing to a computer since this delay appears to only appear on external devices
    - Investigate more the source of the problem to find improvements
- Testing the color of the strips while testing signs with 2 and 3 strips would prevent some misidentifications. However, it might prevent the detection of said sign all together
