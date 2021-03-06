#!/usr/bin/env python3 
# Python libs
import sys, time
from pyzbar.pyzbar import decode

# numpy and scipy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import rospy

# Ros Messages
from sensor_msgs.msg import Image
from std_msgs.msg import String
from object_msgs.msg import Objects
from stand_pose.msg import Stand


VERBOSE=True
SHOW_VIDEO = False

class stand_identifier:

	def __init__(self):
		self.subscriber = rospy.Subscriber("/stand_projector/objects", Objects, self.callback, queue_size = 1)
		self.publisher = rospy.Publisher("/stand_detector/Stand", Stand, queue_size=10)
		if VERBOSE :
			print("subscribed to /stand_projector/objects")


	def callback(self, data):
		#Callback function of subscribed topic. 
		#Here images from stand detection algorithm are decoded using pyzbar. The decoded stand is posted to a topic.

		for det in data.objects :
			im = np.frombuffer(det.image.data, dtype=np.uint8).reshape(det.image.height, det.image.width, -1)	# Reading image from message
			# Image is cropped for performance and perspective reasons.
			y,x,c = im.shape
			startx = x//2 
			crop = im[:, startx-5:startx+5, :]
			res = decode(crop) # Decodes the barcode in image using pyzbar.

			if res != []:
				# Forwarding found stand _including ID_ and excluding image to other recipients
				msg = Stand()
				msg.header.stamp = det.header.stamp
				if VERBOSE : 
					print("Detected stand:", res[0][0].decode("utf-8") )
				msg.id = res[0][0].decode("utf-8")
				msg.pose = det.poses[0].pose
				msg.known = True
				self.publisher.publish(msg)
		
			if VERBOSE and SHOW_VIDEO:
				cv2.imshow('image', crop)
				k = cv2.waitKey(50)
				if k == 13 or k == 27:
					break

def main(args):
	'''Initializes and cleanup ros node'''
	rospy.init_node('stand_identifier', anonymous=False)
	print ("Node online")
	try:
		standidentifier = stand_identifier()
		rospy.spin()
	except KeyboardInterrupt:
		print( "Shutting down stand identifier module")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)


'''
rosrestart
roslaunch handirob_bringup handirob_sensors.launch camera:=true (ctrl+r sensors)
roslaunch handirob_navigation handirob_navigation.launch  (ctrl+r navi)
roslaunch object_launch stand_tracker.launch
'''
