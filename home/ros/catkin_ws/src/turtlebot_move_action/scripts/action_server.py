#!/usr/bin/env python3
# Authors: Boissier, Badeig 

import rospy
import actionlib  #Action
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import turtlebot_move_action.msg
import numpy as np

# Turtlebot3 variable
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

BURGER_WIDTH = 0.16
BURGER_WHEEL = 0.066

DEFAULT_PRECISION = 0.001
DEFAULT_LINEAR_SPEED = 0.06
DEFAULT_ANGULAR_SPEED = 0.4

class MoveAction():

    # function to wait for turtlebot3 initialization (i.e. connection on topic cmd_vel > 0) 
    def waitRobotConnection(self):
        while self.pub.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscriber to connect")
            rosrate.sleep()
    
    # initialization du node 
    # * message for action server
    # * local variable success, currente distance, node name, ...
    # * action server with class SimpleActionServer
    # * topic to control turtlebot3 velocity
    def __init__(self, name):
        print("My name is:")
        print(name)
        self._action_name = name
        self._feedback = turtlebot_move_action.msg.Turtlebot_moveFeedback()
        self._feedback.step_distance = []
        self._result = turtlebot_move_action.msg.Turtlebot_moveResult()
        self._success = True
        self._final_distance = 0.0
        self._as = actionlib.SimpleActionServer(self._action_name, turtlebot_move_action.msg.Turtlebot_moveAction, execute_cb = self.goal_cb, auto_start = False)
        self._as.register_preempt_callback(self.preempt_cb)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)      
        self.waitRobotConnection() 
        self._as.start()

    # preempt callback with result message : failure and the distance already covered
    def preempt_cb(self):
        self._success = False
        self._result.status = "failure"
        self._result.final_distance = self._final_distance
        rospy.loginfo('%s: preempted' %(self._action_name))
        
    # goal callback (default thread for simpleactionserver)
    # * decrypt the goal instruction
    # * publish turtlebot3 velocity
    # * handle feedback message
    # * pay attention to new goal message or cancel message
    def goal_cb(self,goal):
        self._success = True
        self._final_distance = 0.0
        self._feedback.step_distance = []
        self.ccontrol = goal.direction
        self.cdistance = goal.distance
        self.interpret_goal()
        rospy.loginfo('%s: executing, moving %s %s' % (self._action_name, self.ccontrol,self.cdistance))
        self.move()
        if self._success:
            self._result.status = "success"
            self._result.final_distance = self._final_distance
            rospy.loginfo('%s: succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    # function to compute the duration of the action
    def interpret_goal(self):
        if (self.ccontrol=='left') or (self.ccontrol=='right'):
            self.total_time = np.radians(self.cdistance) / DEFAULT_ANGULAR_SPEED
        elif (self.ccontrol=='forward') or (self.ccontrol=='backward'):
            self.total_time = self.cdistance / DEFAULT_LINEAR_SPEED

    # function to control the turtlebot3
    def move(self):        
        twist = Twist()
        if(self.ccontrol=='left'):
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = DEFAULT_ANGULAR_SPEED
        elif (self.ccontrol=='right'):
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -DEFAULT_ANGULAR_SPEED
        elif (self.ccontrol=='forward'):
            twist.linear.x = DEFAULT_LINEAR_SPEED; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        elif (self.ccontrol=='backward'):
            twist.linear.x = -DEFAULT_LINEAR_SPEED; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        else:
            rospy.loginfo('bad command %s',self.ccontrol)
            self._as.set_preempted(self._result)
            self._success = False
            return
        self.pub.publish(twist)
        start_time = rospy.Time.now().to_sec()
        while (not rospy.is_shutdown()) & (rospy.Time.now().to_sec() - start_time <= self.total_time) & (not self._as.is_preempt_requested()) :
            mytime = rospy.Time.now().to_sec() - start_time
            if (self.ccontrol=='left') or (self.ccontrol=='right'):
                self._final_distance = np.degrees(mytime *  DEFAULT_ANGULAR_SPEED)
                self._feedback.step_distance.append(self._final_distance) 
            elif (self.ccontrol=='forward') or (self.ccontrol=='backward'):
                self._final_distance = mytime * DEFAULT_LINEAR_SPEED
                self._feedback.step_distance.append(self._final_distance)  
            self._as.publish_feedback(self._feedback)
            self.pub.publish(twist)
            rosrate.sleep()
        if self._as.is_preempt_requested():
            self._as.set_preempted(self._result)
            self._success = False
            return
        rospy.loginfo('time %f', rospy.Time.now().to_sec()-start_time)
        rospy.loginfo('time %f', self.total_time)
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        self.pub.publish(twist)
 
# main method to create node et initialize MoveACtion object       
def main():    
    try:
        rospy.init_node('move_action')
        global rosrate
        rosrate = rospy.Rate(10) #10 - 30         
        Robotc = MoveAction(rospy.get_name())
    except rospy.ROSInterruptException:
        rospy.loginfo('ROS exception')
        pass

if __name__ == '__main__':
    main()
    rospy.spin()
