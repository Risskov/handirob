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


VERBOSE=True

class stand_identifier:

	def __init__(self):
		self.subscriber = rospy.Subscriber("/stand_projector/objects", Objects, self.callback, queue_size = 1)
		if VERBOSE :
			print("subscribed to /stand_projector/objects")


	def callback(self, data):
		#Callback function of subscribed topic. 
		#Here images get converted

		for det in data.objects :
			im = np.frombuffer(det.image.data, dtype=np.uint8).reshape(det.image.height, det.image.width, -1)	# Reading image from message
			#crop = im[im.width]
			y,x,c = im.shape
			startx = x//2
			starty = y//2   
			crop = im[:, startx-5:startx+5, :]
			res = decode(crop) # Decodes the barcode in image using pyzbar.

			if res != []:
				# Stand database service call goes here
				print(res)	# Prints decoded data
		
			if VERBOSE:
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
