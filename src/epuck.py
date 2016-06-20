#!/usr/bin/env python

import rospy
from epuck.ePuck import ePuck

class EPuckDriver():

	def __init__(self, ttydev="/dev/ttyO0", epuck_name):
		self._driver = ePuck(ttydev, False)
		self._name = epuck_name


	def start(self):
		# Connect to the ePuck
		rospy.loginfo("Connecting to ePuck 1106")
		self._driver.connect()

		# Setup the necessary sensors.
		self.setup_sensors()

		# Disconnect when rospy is going to down
		rospy.on_shutdown(self.disconnect)

		self.greeting()

		self._driver,step()


	def greeting():
		rospy.loginfo("Greetings from ePuck 1106")


	def disconnect(self):
		"""
		Close bluetooth connection
		"""
		self._driver.close()


def run():
	rospy.init_node("epuck_drive", anonymous=True)
	rospy.loginfo("Starting ePuck 1106")
	epuck_name = rospy.get_param("~epuck_name", "epuck")
	EPuckDriver(epuck_name=epuck_name).start()


if __name__ == "__main__":
	run()
