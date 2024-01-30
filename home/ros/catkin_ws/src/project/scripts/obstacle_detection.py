#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np

def lidar_callback(msg):
    global len_lidar_msg # to allow one single initialization and improve performances
    if(len_lidar_msg < 0): # set the len of the lidar data with the first value
        len_lidar_msg = len(msg.ranges)

    # get the distances out of the lidar data
    distances = np.array(msg.ranges)

    # use a slight angle and take the mean in case one lidar ray fails
    range_width = 6
    detection_distance_front = 0.25
    detection_distance_back = 0.15
    detection_distance_sides = 0.15
    distance_front =  (np.concatenate((distances[np.arange(range_width//2)],distances[-np.arange(1,range_width//2+1)])))
    distance_left = (distances[range(len_lidar_msg//4-range_width//2,len_lidar_msg//4+range_width//2)])
    distance_back = (distances[range(len_lidar_msg//2-range_width//2,len_lidar_msg//2+range_width//2)])
    distance_right = (distances[range(3*len_lidar_msg//4-range_width//2,3*len_lidar_msg//4+range_width//2)])

    distance_front[distance_front==0.0]= 10.0 # if infinite is detected, set it to 10 meters.
    distance_left[distance_left==0.0]= 10.0 # if infinite is detected, set it to 10 meters.
    distance_back[distance_back==0.0]= 10.0 # if infinite is detected, set it to 10 meters.
    distance_right[distance_right==0.0]= 10.0 # if infinite is detected, set it to 10 meters.

    # binaries the distance to avoid having one value dragging the mean up 
    binary_distance_front = np.average((distance_front < detection_distance_front).astype(int))
    binary_distance_left = np.average((distance_left < detection_distance_sides).astype(int))
    binary_distance_back = np.average((distance_back < detection_distance_back).astype(int))
    binary_distance_right = np.average((distance_right < detection_distance_sides).astype(int))

    # Check if there is an obstacle
    if (binary_distance_front > 0.8):
        obstacles_pub.publish("front")
    if (binary_distance_left > 0.8):
        obstacles_pub.publish("left")
    if (binary_distance_back > 0.8):
        obstacles_pub.publish("back")
    if (binary_distance_right > 0.8):
        obstacles_pub.publish("right")

# initialization
rospy.init_node('obstacle_detection', anonymous=True)

len_lidar_msg = -1

lidar_sub = rospy.Subscriber('/scan', LaserScan, lidar_callback)
obstacles_pub = rospy.Publisher('/obstacles', String, queue_size=1)

rate = rospy.Rate(10) # perform the action x time per second

print("Obstacle detection successfully started")

while not rospy.is_shutdown():
    # Sleep to control the rate
    rate.sleep()
