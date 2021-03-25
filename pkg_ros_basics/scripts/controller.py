#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import matplotlib.pyplot as plt
import cv2


global State_wall
State_wall=0
int_error = []
x_pos = []
y_pos = []

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    x_pos.append(data.pose.pose.position.x)
    y_pos.append(data.pose.pose.position.y)

def laser_callback(msg):
    global regions
    regions = {
        'bright',
        'fright', 
        'front', 
        'fleft',  
        'bleft',  
    wall_state_check()
    }

   



def Waypoints(t):
    # This determines the y positon of the bot for a given x on the curve
    x  = t
    y  = sin(x*2)*sin(x/2)*e**(0.01)
    return [x,y]


def path_tracing():
    #This funciton is used to compute the next point where it needs to move and assigns the velocities
    data = Twist()
    pub.publish(data)
        
def control_loop():
    #This is the main loop which starts and runs path tracking first and then obstacle avoidance
    rospy.init_node('turtle_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    while True:
        try:
            pose
            regions
            break
        except:
            continue
    rate = rospy.Rate(10)
    path_tracing()


if __name__ == "__main__":
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
