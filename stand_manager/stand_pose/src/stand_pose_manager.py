#!/usr/bin/env python3
from os import ST_APPEND
import rospy

from geometry_msgs.msg import Pose
from stand_pose.msg import Stand

import json
import rospkg

####### Note: 31/01/2022
# 	using dict to keep position and such in memory..

#	If all stand data is kept in memory, one can schedule a non-volatile writedown every x seconds to avoid further overhead, albeit only when dict has changed.
#	A dict should be convertable to a file relatively easily. With minimal googling.



class StandManager():

	def __init__(self, _dir):
		self.stand_dict = {}
		self.load_all_stand_data()
		self.subscriber = rospy.Subscriber("/stand_detector/Stand", Stand, self.callback, queue_size = 1)
		

	def update_stand_callback(self, data):
		# Should subscribe to topic published by stand identification algorithm.
		# This mentioned topic can then also be subscribed to by the 'recently detected' node in the behavior tree!
		self.stand_dict[data.id] = data
	
	def save_all_stand_data(self, standID):
		# Save data to non-volatile file
		# This should be run every x seconds
		if len(self.stand_dict) > 0:
			file = open(self.dir + "/stand_data.json", "w")
			print(self.dir)
			file.write(json.dumps(self.stand_dict))

	def load_all_stand_data(self, standID):
		# Load data from non-volatile file. This will run only on initialization
		file = open(self.dir + "/stand_data.json", "r")
		self.stand_dict = json.load(file)
		file.close()

if __name__ == '__main__':
	rospy.init_node('stand_manager', anonymous=False)
	StandManager()
	rospy.spin()