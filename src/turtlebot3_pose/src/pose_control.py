#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from time import sleep

def listener_callback(data):
    global pos_x
    pos_x = data.pose.pose.position.x

def talker_main(x):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    cmd = Twist()
    cmd.linear.x = x
    rospy.loginfo(cmd)
    pub.publish(cmd)

def controller(sp_x):

    kp = 1          # proportional gain
    vel_x = 0.0     # m/s, current linear speed
    max_lin = 0.26  # m/s, maximum linear speed

    vel_x = kp*(sp_x - pos_x)
    if vel_x > max_lin:
        talker_main(max_lin)
    elif vel_x < -max_lin:
        talker_main(-max_lin)
    else:
        talker_main(vel_x)    
    
            
if __name__ == '__main__':
        
    pos_x = 0.0

    rospy.init_node('pose_control')
    
    goal_x = eval(input("Digite a posição do eixo X com formato 0.0: "))

    while not rospy.is_shutdown():
            rospy.Subscriber('/odom', Odometry, listener_callback)    
            controller(goal_x)
            sleep(1)            
    

        
        
        
    
