#!/usr/bin/python
# -*- coding: utf-8 -*-

# importing the packages and modules for turtlesim simulation
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from pkg_ros_basics.msg import command



# defining subscriber callback function

def command_callback(msg):
    global COMMAND
    COMMAND = msg.command

# function for the turtle simulation

def move_turtle():
    global COMMAND
    

    # initializing node
    rospy.init_node('move', anonymous=True)

    # initailizing publisher....
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('my_command', command, command_callback)

    rate = rospy.Rate(10)  # Rate = 10Hz
    
    vel = Twist()  # Assigning publisher
    
    while not rospy.is_shutdown():

        vel.linear.x=3.5
        vel.linear.y=0
        vel.linear.z=0

        vel.angular.x=0
        vel.angular.y=0
        vel.angular.z=1

        rate.sleep()
        
        # calling publisher........
        pub.publish(vel)

        rospy.loginfo('hello')

       

        

        

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass