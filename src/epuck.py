#!/usr/bin/env python

import rospy
import time
from ePuck import ePuck

class EPuckDriver():

	def __init__(self, ttydev="/dev/ttyO0", epuck_name="epuck"):
		self._driver = ePuck(ttydev, False)
		self._name = epuck_name


	def start(self):
		# Connect to the ePuck
		rospy.loginfo("Connecting to ePuck 1106")
		self._driver.connect()

		# Setup the necessary sensors.
		#self.setup_sensors()

		# Disconnect when rospy is going to down
		rospy.on_shutdown(self.disconnect)

		self.greeting()

		self._driver.step()

                # Spin almost forever
                #rate = rospy.Rate(7)   # 7 Hz. If you experience "timeout" problems with multiple robots try to reduce this value.
                self.startTime = time.time()
                while not rospy.is_shutdown():
                    self._driver.step()


	def greeting(self):
		rospy.loginfo("Greetings from ePuck 1106")
                self._driver.set_led(1, 1)


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
