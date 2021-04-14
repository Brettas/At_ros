#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from time import sleep

def listener_callback(data):
    global pos_x
    pos_x = data.pose.pose.position.x
            
def talker_main(x, y):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd = Twist()
    cmd.linear.x = x
    cmd.linear.y = y
    rospy.loginfo(cmd)
    pub.publish(cmd)

def controller(sp_x):
    
    max_lin = 0.70  # m/s, maximum linear speed

    if pos_x > sp_x:
        talker_main(-max_lin, -0.5)
    elif pos_x < sp_x:
        talker_main(max_lin, 0.5)
    else:
        talker_main(0.0 , 0.0)

if __name__ == '__main__':
    
    pos_x = 0.0
    goal_x = 0.0
    
    rospy.init_node('pose_control')

    while not rospy.is_shutdown():
        rospy.Subscriber('/odom', Odometry, listener_callback)
        controller(goal_x)
        sleep(1)
