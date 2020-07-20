#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
import pytesseract
from std_msgs.msg import String

class Img_to_str:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', 
										  Image, self.image_callback)
		self.str_pub = rospy.Publisher('img_to_str', String, queue_size=1)
		self.rate = rospy.Rate(10)

	def image_callback(self, msg):
		
		################################################
		# Yellow Mask
		
		image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_black = numpy.array([ 0, 0, 0])
		upper_black = numpy.array([ 0, 0, 0])
		mask = cv2.inRange(hsv, lower_black, upper_black)
		
		###################################################
		# OCR-- Pass masked image to 'pytesseract.image_to_string' to
		# extract text from the image in the English language. 
		
		text = pytesseract.image_to_string(mask, lang = "eng")
		print(text)
		
		self.str_pub.publish(text)
		
		self.rate.sleep()
				
		cv2.imshow("window_image", image) # raw image
		cv2.imshow("window_mask", mask) # only black color
		cv2.waitKey(1)

rospy.init_node('img_to_str')
img_to_str = Img_to_str()
rospy.spin()
