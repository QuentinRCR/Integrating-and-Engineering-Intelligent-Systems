import roslibpy
import matplotlib.pyplot as plt
import cv2
import base64
import numpy as np
from datetime import datetime
import colorsys
import time

# ============================= FUNCTIONS ===================================================

def connect_to_ros_bridge():
    client = roslibpy.Ros(host='172.16.1.105', port=9090)
    client.run()
    print('Is ROS connected?', client.is_connected)
    return client

# Get any image coming from rosbridge and update the value of the most recent image  
def image_callback(msg):
    global most_recent_image

    # Decode the base64-encoded string
    decoded_bytes = base64.b64decode(bytes(msg['data'], 'utf-8'))
    array_from_base64 = np.frombuffer(decoded_bytes, dtype=np.uint8)

    # reshape the image and flip it because the camera is upside down
    image = np.flipud(array_from_base64.reshape((msg['height'],msg['width'],3)))
    
    # Convert this image in RGB to 
    most_recent_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)/255

# Function to input an image from the file folder to make tests
def image_callback_bis():
    global most_recent_image

    image = cv2.imread("images/15_01_2024_19_56_23-brut.png")

    most_recent_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)/255


def percentage_distances(dist1,dist2):
    return np.abs((dist1-dist2)/dist2)

# Return the color inputted as a string
def get_color_name(color):
    H,S,V = (colorsys.rgb_to_hsv(*color)) # transform to HSV to get read of the luminosity
    H*=360

    red = color[0]
    green = color[1]
    blue = color[2]
    identified_color = "undetermined"

    if(V < 0.15):
        identified_color = "black"
    elif((abs(green - red) + abs(green - blue) < 0.15) and  V > 0.4): # empirical white test to see what works best
        identified_color = "white"
    elif( H <35 or H > 330): # very wide color test because we only are interested in 2 colors
        identified_color = "red"
    elif( 60 < H < 240):
        identified_color = "green"

    return identified_color

# Return the distance between ones, for a line composed of zeros and ones
def measure_distances_line(line): 
    index = 0
    distances = []
    line_length = len(line)
    while(index < line_length):
        distance = 0
        while(index < line_length and line[index] == 0):
            index += 1
            distance += 1
        if(distance > vertical_line_allowed_gap): # if the gap is not big enough, it ignores it. 
            distances.append({"distance": distance, "endValue": index})
        else:
            index += 1
    return distances

# Return the sum of the vertical sobel on the red and green image to have better details
def generate_sobel(image):
    red = image[:,:,0] # get the red canal
    green = image[:,:,1] # get the green canal
    sobel_red = cv2.Sobel(red, cv2.CV_64F, 0, 1, ksize=3)
    sobel_green = cv2.Sobel(green, cv2.CV_64F, 0, 1, ksize=3)
    
    sobel = ((np.sqrt(sobel_red**2) + np.sqrt(sobel_green**2) / 2)).astype(float)
    return sobel

# For a column of an image, recognize the sign corresponding to the spacing and the color
def get_ratio_from_distances(distances,image_column):

    # try to detect 3 strips
    index = 0
    len_distance = len(distances)
    while index < (len_distance-8): # if there is not enough detected vertical lines, it means is can't be this sign 
        index += 1

        distance_1 = distances[index]["distance"]
        distance_2 = distances[index+1]["distance"]
        distance_3 = distances[index+2]["distance"]
        distance_4 = distances[index+3]["distance"]
        distance_5 = distances[index+4]["distance"]
        distance_3_strips = distance_1+distance_2+distance_3+distance_4+distance_5

        # test if 3 strips are identical, separated by 2 identical lines 
        if((percentage_distances(distance_1,distance_3)< 0.2) and #black
            (percentage_distances(distance_3,distance_5)< 0.2) and #black
            (percentage_distances(distance_2,distance_4)< 0.2) and #white
            # making sur the sign is big enough (to avoid false detections)
            (distance_3_strips*2.51 > min_sign_hight)): # the 2.51 correspond to ratio between the detected section and the total sign height
            start = distances[index-1]["endValue"] - distance_1
            end = distances[index+7]["endValue"]
            return [start,end,"three_strips"] #return coordinate of the start and the end of the value, as well as the type
            
    # try to detect 2 strips
    index = 0
    while index < (len_distance-6):
        index += 1

        distance_1 = distances[index]["distance"]
        distance_2 = distances[index+1]["distance"]
        distance_3 = distances[index+2]["distance"]
        distance_4 = distances[index+3]["distance"]
        distance_5 = distances[index+4]["distance"]
        distance_white = distance_1+distance_2+distance_3+distance_4+distance_5

        # detect the three white band and the 2 black one
        if((percentage_distances(distance_1,distance_3)< 0.2) and # white
            (percentage_distances(distance_3,distance_5)< 0.2) and # white
            (percentage_distances(distance_2,distance_4)< 0.2) and # black
             # making sur the sign is big enough (to avoid false detections)
            (distance_white*1.58>min_sign_hight)): # the 1.58 correspond to ratio between the detected section and the total sign height
            start = distances[index-1]["endValue"] - distance_1
            end = distances[index+5]["endValue"]
            return [start,end,"two_strips"] #return coordinate of the start and the end of the value
            
    # try to detect 1 strips
    index = 0
    while index < (len_distance-5):
        index += 1

        distance_1 = distances[index]["distance"]
        distance_2 = distances[index+1]["distance"]
        distance_3 = distances[index+2]["distance"]
        distance_4 = distances[index+3]["distance"]
        distance_5 = distances[index+4]["distance"]
        distance_total = distance_1+distance_2+distance_3+distance_4+distance_5

        # detect the black,white,band,white,black
        if((percentage_distances(distance_2,distance_4)< 0.2) and # white ratio
            (percentage_distances(distance_1,distance_5)< 0.3) and #black ratio
            (get_color_name(image_column[distances[index+1]["endValue"]-distance_2//2])=='white') and #white band 1 is white
            (get_color_name(image_column[distances[index+2]["endValue"]-distance_3//2])=='black') and #black band is black
            (get_color_name(image_column[distances[index+3]["endValue"]-distance_4//2])=='white') and #white band 2 is white
            (distance_total)>min_sign_hight):
            start = distances[index]["endValue"] - distance_1
            end = distances[index+4]["endValue"]
            return [start,end,"one_strip"] #return coordinate of the start and the end of the value


    # try to detect red-green and green-red signs
    index = 0
    while index < (len_distance-4):
        index += 1

        distance_1 = distances[index]["distance"]
        distance_2 = distances[index+1]["distance"]
        distance_3 = distances[index+2]["distance"]
        distance_4 = distances[index+3]["distance"]
        distance_total = distance_1+distance_2+distance_3+distance_4

        # detect black, color, color, black
        if((percentage_distances(distance_2,distance_3)< 0.2) and # red and green same size
            (percentage_distances(distance_1,distance_4)< 0.2) and # black same size
            (distance_total>min_sign_hight)):

            if ((get_color_name(image_column[distances[index+1]["endValue"]-distance_2//2])=='red') and # red part is red
                (get_color_name(image_column[distances[index+2]["endValue"]-distance_3//2])=='green')): # green part is green
                start = distances[index]["endValue"] - distance_1
                end = distances[index+3]["endValue"]
                return [start,end,"red_green"] #return coordinate of the start and the end of the value
            elif((get_color_name(image_column[distances[index+1]["endValue"]-distance_2//2])=='green') and
                (get_color_name(image_column[distances[index+2]["endValue"]-distance_3//2])=='red')):
                start = distances[index]["endValue"] - distance_1
                end = distances[index+3]["endValue"]
                return [start,end,"green_red"] #return coordinate of the start and the end of the value

    # try to detect basic
    a, b, c, type, tolerance = [72,233,72,"basic",0.2]
    index = 0
    while (index < (len_distance-3)):
        index += 1

        distance_1 = distances[index]["distance"]
        distance_2 = distances[index+1]["distance"]
        distance_3 = distances[index+2]["distance"]
        distance_total = distance_1+distance_2+distance_3
        
        # if the ratio is correct on the 3 first distances
        # and the sign is not too far away
        # return the top and the bottom of the sign
        if((np.abs(a/c - distance_1/distance_3) < tolerance*distance_total/(2*min_sign_hight)) and # ratio 1 good
           (np.abs(a/b - distance_1/distance_2) < tolerance*distance_total/(2*min_sign_hight)) and # ratio 2 good
           (get_color_name(image_column[distances[index]["endValue"]-distance_1//2])=='black') and # black top is black
           (get_color_name(image_column[distances[index+2]["endValue"]-distance_3//2])=='black') and #black bottom is black
           (distance_total > min_sign_hight)):
            start = distances[index]["endValue"] - distance_1
            end = distances[index+2]["endValue"]
            return [start,end,type] #return coordinate of the start and the end of the value

    # if no pattern was found, return -1 and -1
    return [-1,-1,"None"]


# take an image and return the sign detected for every column of the image
def extract_matching_vertical_coords(sobel,image,display_image):
    # extract matching vertical coords 
    vertical_cords = []
    for line_position in range(np.shape(sobel)[1]): #for every column 
        sobel_column = sobel[:,line_position]
        image_column = image[:,line_position]

        #get the lines that are marked enough
        sobel_column = sobel_column>sobel_binarization_threshold

        # get the distance between each ones on the line
        distances = measure_distances_line(sobel_column)

        # identify the matching ratios
        vertical_cord = get_ratio_from_distances(distances,image_column)

        if display_mode: # draw the matching ratio
            display_vertical_ratios(vertical_cord,line_position,display_image)

        # add to the list of already matching coords
        vertical_cords.append(vertical_cord)
    return vertical_cords

# Display the found matching ratio with different colors depending on the type (for debug purposes)
def display_vertical_ratios(vertical_cord,line_position,display_image):
    if vertical_cord is not None:
        if(vertical_cord[2]=='basic'):
            color = np.array([119, 3, 252])/255 #purple
        elif(vertical_cord[2]=='one_strip'):
            color = np.array([252, 128, 3])/255 #orange
        elif(vertical_cord[2]=='two_strips'):
            color = np.array([2, 250, 205])/255 #cyan
        elif(vertical_cord[2]=='three_strips'):
            color = np.array([250, 2, 238])/255 #pink
        else:
            color = [0.0,1.0,0.0]
        display_image[vertical_cord[0]-2:vertical_cord[0]+2,line_position] = color
        display_image[vertical_cord[1]-2:vertical_cord[1]+2,line_position] = color

# take a list of detected vertical coords and, this enough of the same time are next to one another, it's a sign 
def extract_rectangle_from_vertical_coords(vertical_cords):
    top_list, bottom_list,varieties = np.transpose(vertical_cords)

    #force to int again after the transpose turned everything to str 
    top_list = top_list.astype(int) 
    bottom_list = bottom_list.astype(int)

    rectangle_coords = []
    width = 0
    number_missed = 0 #number of values that do no align
    top_value_serie = [] # list of all the top values in a serie of aligned coords
    bottom_value_serie = [] # list of all the top values in a serie of aligned coords

    # if enough top vertical coords of the same type are next to each other, it is a sign
    for i in range(1,len(top_list)):
        top = top_list[i]

        # we allow some discontinuity as long as its not to much
        # if it is not the first pixel and different sign is detected, then it's a miss
        if(top==-1 or (width>0 and varieties[i]!=varieties[i-1-number_missed])):
            number_missed +=1
        else:
            number_missed = 0 #if it's a hit, reset the miss counter

        #if the line is somewhat horizontal, increase the width
        if(top_list[i-1-number_missed]!=-1 and (abs(top-top_list[i-1-number_missed]) < allowed_horizontal_shift)): 
            width +=1
            top_value_serie.append(top_list[i-1-number_missed])
            bottom_value_serie.append(bottom_list[i-1-number_missed])


        # if the line doesn't continue anymore
        if(number_missed>horizontal_line_allowed_gap):

            # if the width is more that 70% of what it should be, then we have a sign
            if((width !=0) and (width > 0.7 * vertical_horizontal_sign_ratio * (bottom_list[i-4]-top_list[i-4]))):
                #get coords of the top left corner - take the median so that if the first if the wrong type, it still gives a good position
                left_top = [int(np.median(top_value_serie)),i-4-width]
                right_bottom = [int(np.median(bottom_value_serie)),i-4] #get coords of the bottom right corner
                rectangle_coords.append((left_top,right_bottom,varieties[i-4]))

            width = 0
            top_value_serie = []
            bottom_value_serie = []

    return rectangle_coords

# Add the rectangle on the image and write its type
def display_rectangles(rectangle_coords, display_image):
    global most_recent_image
    rectangleWeight = 2

    for top_left, bottom_right, variety in rectangle_coords:
        # Print the rectangle
        y_top = top_left[0]
        x_left = top_left[1]
        y_bottom = bottom_right[0]
        x_right = bottom_right[1]

        display_image[y_top:y_bottom,x_left-rectangleWeight:x_left+rectangleWeight]=[1,0.0,0.0] #left
        display_image[y_top:y_bottom,x_right-rectangleWeight:x_right+rectangleWeight]=[1,0.0,0.0] #right
        display_image[y_top-rectangleWeight:y_top+rectangleWeight,x_left:x_right]=[1,0.0,0.0] #top
        display_image[y_bottom-rectangleWeight:y_bottom+rectangleWeight,x_left:x_right]=[1,0.0,0.0] #bottom

        # Print the text
        middle_coordinates = [(y_top+y_bottom)//2,(x_right+x_left)//2]

        # if its the basic type, need to specify the color
        if(variety=='basic'):
            color_middle = most_recent_image[middle_coordinates[0],middle_coordinates[1]]
            color_name = get_color_name(color_middle)
            variety += "_"+color_name

        cv2.putText(display_image,variety,(middle_coordinates[1]-40,middle_coordinates[0]),cv2.FONT_HERSHEY_SIMPLEX ,1,(0.0,1.0,0.0),2)

    return display_image

# Combine all the functions and determine the distance of the sign
def identify_sign(image):
    focal = 3.04 # mm
    sensor_height = 2.76
    pixel_size = 1.12e-3
    real_height = 201 # 210 - 2 * 4.5 mm
    image_height, image_width, _ = np.shape(image)
    sobel = generate_sobel(image)
    display_image = most_recent_image.copy()
    vertical_coordinates = extract_matching_vertical_coords(sobel, image,display_image)
    rectangle_coordinates = extract_rectangle_from_vertical_coords(vertical_coordinates)
    if display_mode:
        img = display_rectangles(rectangle_coordinates,display_image)
        h.set_data(img) #dynamical update the value of the image 
        plt.draw(),plt.pause(1) #the plt.pause matters


    data = []
    for tl, br, type in rectangle_coordinates:
        y_mid = (tl[0] + br[0]) // 2 # get the center of the sign
        x_mid = (tl[1] + br[1]) // 2
        height = abs(tl[0] - br[0])
        horizontal_distance = x_mid - image_width//2
        distance = (real_height * focal * image_height) / (height * sensor_height)
        angle = np.arctan(horizontal_distance * pixel_size / focal)
        if(type == 'basic'):
            type += f'_{get_color_name(image[y_mid, x_mid])}'
        data.append({"distance": distance/1000, "angle": np.degrees(angle), "sign": type})
    
    return data
    
    

# ===================================================================================================================================================
#                                   BEGIN OF THE PROGRAM

# global parameters
vertical_line_allowed_gap = 4
horizontal_line_allowed_gap = 3
min_sign_hight = 50
sobel_binarization_threshold = 0.30
vertical_horizontal_sign_ratio = 100/450
allowed_horizontal_shift = 3
display_mode = True # display the computed image or note

client = connect_to_ros_bridge()

# connect to the topics
image_listener = roslibpy.Topic(client, '/camera/image', 'sensor_msgs/Image', queue_size=1)
image_listener.subscribe(image_callback)
sign_talker = roslibpy.Topic(client, '/sign', 'project/sign_info')

global most_recent_image
most_recent_image = None

# image_callback_bis() #used to manually input an image 

if display_mode: #prepare dynamic plot
    fg= plt.figure()
    ax = fg.gca()
    h = ax.imshow(np.zeros((480, 640, 3)))  # set initial display dimensions

try:
    while client.is_connected:
        if most_recent_image is not None:

            start_time = time.process_time()
        
            extracted_data = identify_sign(most_recent_image)

            image_name = f'images/{datetime.now().strftime("%d_%m_%Y_%H_%M_%S")}-brut.png' 
            cv2.imwrite(image_name, most_recent_image*255)

            for extracted in extracted_data: # publish each sign independently
                sign_talker.publish(extracted)

            print("Elapsed time during the whole program in seconds: ", time.process_time() - start_time)
except KeyboardInterrupt:
    client.terminate()