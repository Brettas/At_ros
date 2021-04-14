#!/usr/bin/env python3

import rospy
import cv2 as cv 
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class Follower:

	def __init__(self):
		self.bridge = CvBridge()
		cv.namedWindow("Imagem", 1)
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.twist = Twist()

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		
		#convert BGR to HSV
		hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
		
		# dinine range of yellow color HSV
		lower_yellow = numpy.array([ 10, 10, 10])
		upper_yellow = numpy.array([255, 255, 250])
		
		#Threshold the HSV image to get only yellow colors 
		mask = cv.inRange(hsv, lower_yellow, upper_yellow)
		
		
		h, w, d = image.shape
		search_top = 3*h/4
		search_bot = 3*h/4 + 10
		
		
        
		M = cv.moments(mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv.circle(image, (cx, cy), 20, (0,0,255), -1)
#The proportional controller is implemented in the following four lines	which
#is reposible of linear scaling of an error to drive the control output.	
			err = cx - w/2
			self.twist.linear.x = 0.26
			self.twist.angular.z = -float(err) / 100
			self.cmd_vel_pub.publish(self.twist)
		cv.imshow("Imagem", image)
		cv.waitKey(3)

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
