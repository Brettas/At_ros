#!/usr/bin/env python3
import rospy 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')       
    edges = cv2.Canny(cv_image, 50, 200)
    cv2.imshow('image', edges)
    cv2.waitKey(0)    
    
def get_image():
    rospy.init_node('Detector', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    get_image()
  
    
