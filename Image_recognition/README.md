# Sign Detection Algorithm

## Quick Start
1. Make sure to be connected to the same network as the turtlebot
2. Make sure rosbridge is launched
3. Specify the address of the raspberry with the screen in the function `connect_to_ros_bridge`
4. Run the program

## Detection Method
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

## Notes
- To prevent false detection on small details, a minimum sign height is imposed
- To improve the computational time, the quality of the image is reduced by a factor of 2
- Some tolerances on the previously stated rules were added and tuned to get the best possible results
- For more details on the program, please refer to the commented code

## Improvements to Consider
- This program suffers from a 10-15 seconds delay between what is perceived and the reality. To fix this it would be possible to:
    - Compress the image before sending it to prevent saturating rosbridge
    - Avoid outsourcing the image processing to a computer since this delay is caused by the image transfer speed
    - Investigate further on other paramters influencing this delay
- Verifying the color of the strips while testing signs with 2 and 3 strips would prevent some misidentifications. However, it might prevent the detection of said sign all together
