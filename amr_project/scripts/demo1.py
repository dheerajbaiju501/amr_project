#! /usr/bin/env python3
import rospy

from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from math import atan2
from math import atan
 
#start is x:0, y:0
x = 8.0
y = 8.0
theta = atan2(y, x)    #current angle of robot
 
#import ipdb; ipdb.set_trace()
 
def callback(msg):
    global x
    global y
    global theta
 
    x = msg.pose[1].position.x
    y = msg.pose[1].position.y
    rot_q = msg.pose[1].orientation
    #q=Quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #print(roll,pitch,theta)
 
 
rospy.init_node ('subscriber')
sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback) #use odom topic
pub = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
 
speed = Twist()
 
r = rospy.Rate(4)
 
goal = Point()
goal.x = 7.0
goal.y = 5.1
 
while not rospy.is_shutdown():
    inc_x = goal.x - x                      #distance robot to goal in x
    inc_y = goal.y - y                      #distance robot to goal in y
    angle_to_goal = atan2 (inc_y, inc_x)    #calculate angle through distance from robot to 
    print(str(angle_to_goal) + " " +str(theta))

    if abs(angle_to_goal - theta) > 0.1:
    	speed.linear.x = 0.0
    	if angle_to_goal > theta:
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
        	
 
    pub.publish(speed)
    
speed=Twist()
pub.publish(speed)
r.sleep()
