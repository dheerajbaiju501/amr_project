#! /usr/bin/env python3
import rospy
from demo import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import atan2
from math import atan
 
#start is x:0, y:0
x1 = 8.0
y1 = 8.0
theta1 = atan2(y1, x1)    #current angle of robot
 
#import ipdb; ipdb.set_trace()
 
def callback1(msg):
    global x1
    global y1
    global theta1
 
    x1 = msg.pose[1].position.x
    y1 = msg.pose[1].position.y
    rot_q1 = msg.pose[1].orientation
    #q=Quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)
    (roll1, pitch1, theta1) = euler_from_quaternion([rot_q1.x, rot_q1.y, rot_q1.z, rot_q1.w])
    #print(roll,pitch,theta)
 
 
rospy.init_node ('subscriber')
sub1 = rospy.Subscriber('/gazebo/model_states', ModelStates, callback1) #use odom topic
pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
 
speed1 = Twist()
 
r = rospy.Rate(4)
 
goal = Point()
goal.x = x+7
goal.y = y+5
 
while not rospy.is_shutdown():
    inc_x = goal.x - x1                      #distance robot to goal in x
    inc_y = goal.y - y1                      #distance robot to goal in y
    angle_to_goal = atan2 (inc_y, inc_x)    #calculate angle through distance from robot to 
    print(str(angle_to_goal) + " " +str(theta1))

    if abs(angle_to_goal - theta1) > 0.1:
    	speed.linear.x = 0.0
    	if angle_to_goal > theta1:
    	 speed.angular.z = 0.5
    	else:
    	 speed.angular.z = -0.5    	
    else:
        speed.linear.x = 0.3                #drive towards goal
        speed.angular.z = 0.0
        
        if (inc_x==0):
        	speed.linear.x = 0.0
        	speed.angular.z = 0.0
        	break
        	
 
    pub1.publish(speed)
    
speed=Twist()
pub1.publish(speed)
r.sleep()
