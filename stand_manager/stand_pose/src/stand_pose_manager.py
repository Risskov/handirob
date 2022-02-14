#!/usr/bin/env python3
from os import ST_APPEND
from sre_parse import Verbose
import rospy

from geometry_msgs.msg import Pose
from stand_pose.msg import Stand

import json
import rospkg
import numpy as np

####### Note: 31/01/2022
# 	using dict to keep position and such in memory..

#	If all stand data is kept in memory, one can schedule a non-volatile writedown every x seconds to avoid further overhead, albeit only when dict has changed.
#	A dict should be convertable to a file relatively easily. With minimal googling.


VERBOSE = False
class StandManager():

	def __init__(self, _dir):
		self.dir = _dir
		self.stand_dict = {}
		self.load_all_stand_data()
		self.subscriber = rospy.Subscriber("/stand_detector/Stand", Stand, self.update_stand_callback, queue_size = 1)
		rospy.loginfo("running stand_pose_manager")
		

	def update_stand_callback(self, data):
		# Should subscribe to topic published by stand identification algorithm.
		# This mentioned topic can then also be subscribed to by the 'recently detected' node in the behavior tree!
		# Future: Maybe compare distance to see if change is big enough to save to non-volatile
		self.stand_dict[data.id] = data
		self.save_all_stand_data()
		if VERBOSE:
			print(self.stand_dict.keys())
			print("recently updated:" , data.id)
	
	def save_all_stand_data(self):
		# Save data to non-volatile file
		# This could be run every x seconds
		if len(self.stand_dict) > 0:
			np.save(self.dir + "/stand_data.npy", self.stand_dict)
		if VERBOSE:
			print("Memory saved to " + self.dir + "/stand_data.npy")

	def load_all_stand_data(self):
		# Load data from non-volatile file. This will run only on initialization
		try:
			self.stand_dict = np.load(self.dir + "/stand_data.npy", allow_pickle='TRUE').item()
		except:
			print("file", self.dir + "/stand_data.npy", " not found")
		else:
			print(self.stand_dict)
			print("Loaded ", str(len(self.stand_dict)) , " entries from ", self.dir + "/stand_data.npy")

if __name__ == '__main__':
	rospy.init_node('stand_manager', anonymous=False)
	dir = rospy.get_param('~data_directory')
	StandManager(dir)
	rospy.spin()