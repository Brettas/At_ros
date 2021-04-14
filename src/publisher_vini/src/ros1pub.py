#!/usr/bin/env python3

import rospy
import random

from std_msgs.msg import Float64

def talker():
	pub = rospy.Publisher('vinicius', Float64, queue_size=0)
	rospy.init_node('publisher_vini')
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		data = random.uniform(1, 100)
		rospy.loginfo(data)
		pub.publish(data)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
