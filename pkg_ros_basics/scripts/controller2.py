#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


global pose, regions



def get_dist(x1,y1,x2,y2):
    return(math.sqrt((x1-x2) * 2 + (y1-y2) * 2))


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]


def Waypoints(x_present):
    x  =  x_present+.3
    y  =  2*math.sin(x_1)*math.sin(x_1/2)*e**(0.01)
    return [x,y]


def laser_callback(data):
    global regions
    regions = {
        'bright': min(min(data.ranges[0:180]),30),
        'fright': min(min(data.ranges[181:300]),30),
        'front': min(min(data.ranges[301:420]),30),
        'fleft': min(min(data.ranges[421:540]),30),
        'bleft': min(min(data.ranges[541:720]),30),
    }




def loop():
    global pose, regions

    
    pose = [0,0,0]
    regions = {
        'bright': 30,
        'fright': 30,
        'front' : 30,
        'fleft' : 30,
        'bleft' : 30,
    }

   
    rospy.init_node('ebot_controller')
    
   
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    
    rate = rospy.Rate(10) 

    
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0

    # Publishing initial velocity
    pub.publish(velocity_msg)

    while not rospy.is_shutdown(): 

        # Tracing the given path
        #if pose[0] <= math.pi*2:
        if False:
            
            
            waypoint = Waypoints(pose[0]) 
            
            
            dist = get_dist(pose[0],pose[1],waypoint[0],waypoint[1]) 

            
            ang_diff = (math.atan2(waypoint[1]-pose[1], waypoint[0]-pose[0]) - pose[2])

            
            angular_velocity=4*ang_diff
            
            
            if abs(ang_diff) <.1:
                linear_velocity = 1
             
            elif abs(ang_diff) > .3:
                linear_velocity = 0
            
            else:
                linear_velocity = .5/(1+2*abs(ang_diff))
        
        
        else:

            
            goal_x=-25
            goal_y=0

            
            dist = get_dist(pose[0],pose[1],goal_x,goal_y)   

            
            ang_diff = (math.atan2(goal_y-pose[1], goal_x-pose[0]) - pose[2])

            
            if ang_diff > math.pi:
                ang_diff = -(2*math.pi - ang_diff)
            
            elif ang_diff < -math.pi :
                ang_diff = 2*math.pi + ang_diff

            
            linear_velocity = 0
            angular_velocity = 0

            
            if dist < 0.2:
                linear_velocity=0
                angular_velocity=0
                rospy.loginfo('Reached goal!!')
            
           
            else:

                
                 if abs(ang_diff) >= math.pi:
                     # Rotate the bot in a loop
                     while abs(ang_diff) > 3 and not rospy.is_shutdown():
                         linear_velocity = 0
                         angular_velocity = 1
                         
                         velocity_msg.linear.x = linear_velocity
                         velocity_msg.angular.z = angular_velocity
                         ang_diff = (math.atan2(goal_y-pose[1], goal_x-pose[0]) - pose[2])

                         
                         pub.publish(velocity_msg)


                
                 if abs(ang_diff) >.02:
                    
                    
                    if regions['front']>=1 and regions['fleft'] >=1 and regions['fright'] >=1:
                        
                        angular_velocity = 3 * ang_diff
                        linear_velocity = 1
                    
                    
                    elif regions['front']<1 and regions['fleft'] >=1 and regions['fright'] >=1:
                        
                        angular_velocity = 3 * ang_diff
                        linear_velocity = 0

                      
                    elif regions['front'] >=1 and regions['fright'] <1 and regions['fleft'] >=1:
                        
                        angular_velocity = 0
                        linear_velocity = 1

                   
                    elif regions['front'] >=1 and regions['fright'] >=1 and regions['fleft'] <1:
                        
                        angular_velocity = 0
                        linear_velocity = 1

                    
                    elif regions['front'] <1 and regions['fright'] <1 and regions['fleft'] >=1:
                        
                        angular_velocity = 1
                        linear_velocity = 0

                    
                    elif regions['front'] <1 and regions['fright'] >=1 and regions['fleft'] <1:
                        
                        angular_velocity = -1
                        linear_velocity = 0

                    
                    elif regions['front'] <1 and regions['fright'] <1 and regions['fleft'] <1:
                        
                        angular_velocity = 1
                        linear_velocity = 0

                    
                    elif regions['front'] >=1 and regions['fright'] <1 and regions['fleft'] <1:
                        
                        angular_velocity = 0
                        linear_velocity = 1

                    
                    else:
                        print(regions['front'])
                        print(regions['fright'])
                        print(regions['fleft'])
                        print('unknown')                        

               
                 else:

                    
                    if regions['front'] >= 1:
                       
                        linear_velocity = 1
                        angular_velocity = 0

                    
                    else:
                        linear_velocity = 0
                        angular_velocity = 1
        
       
    	velocity_msg.linear.x = linear_velocity
        velocity_msg.angular.z = angular_velocity

        
    	pub.publish(velocity_msg)
    	
       
    	rate.sleep()


if __name__ == '__main__':
    try:
        loop()
    except rospy.ROSInterruptException:
        pass