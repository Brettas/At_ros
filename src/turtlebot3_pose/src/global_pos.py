#!/usr/bin/env python3

import rospy 
import math
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

roll = pitch = yaw = 0.0
odom_x = 0

def dados(data):
    global odom_x
    odom_x = data.pose.pose.position.x

def callback(data):
    #Adicionado para fazer a Questão 2
    rospy.loginfo(" X posicao = %s m", '{:.2f}'.format(data.pose.pose.position.x))
    rospy.loginfo(" Y posicao = %s m", '{:.2f}'.format(data.pose.pose.position.y))
    #Adicionado para fazer a Questão 3
    global roll, pitch, yaw
    quat = data.pose.pose.orientation
    orientation_list = [quat.x, quat.y, quat.z, quat.w]
    (roll, pitch ,yaw) = euler_from_quaternion (orientation_list)
    quatyaw = math.degrees(yaw)
    rospy.loginfo(" Yaw  = %s graus", '{:.4f}'.format(quatyaw))
    
    
def Questão6(data):
    pose = data.position[0]
    brac_pos = odom_x - 0.14 + (0.25*math.sin(pose))
    rospy.loginfo(" Posição = %s m", '{:.2f}'.format(brac_pos))
    

    
def global_pose():
    rospy.init_node('global_pose', anonymous=True)
    rospy.Subscriber("rrbot/joint_states", JointState, Questão6)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    global_pose() 

