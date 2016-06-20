#!/usr/bin/env python

import rospy
import time
import subprocess
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

		# setup publisher
		self.accel_publisher = rospy.Publisher('accel', Imu)    # Only "linear_acceleration" vector filled.
		self.selector_publisher = rospy.Publisher('selector', Marker)
		self.light_publisher = rospy.Publisher('light', Marker)
		self.motor_speed_publisher = rospy.Publisher('motor_speed', Marker)
		self.microphone_publisher = rospy.Publisher('microphone', Marker)
		self.floor_publisher = rospy.Publisher('floor', Marker)

		# Spin almost forever
		#rate = rospy.Rate(7)   # 7 Hz. If you experience "timeout" problems with multiple robots try to reduce this value.
		self.startTime = time.time()
		while not rospy.is_shutdown():
			self._driver.step()
			self.update_sensors()


	def update_sensors(self):
		# Accelerometer
		accel = self._driver.get_accelerometer()
        # accel_msg = Imu()
        # accel_msg.header.stamp = rospy.Time.now()
        # accel_msg.header.frame_id = self._name+"/base_link"
        # accel_msg.linear_acceleration.x = (accel[1]-2048.0)/800.0*9.81 # 1 g = about 800, then transforms in m/s^2.
        # accel_msg.linear_acceleration.y = (accel[0]-2048.0)/800.0*9.81
        # accel_msg.linear_acceleration.z = (accel[2]-2048.0)/800.0*9.81
        # accel_msg.linear_acceleration_covariance = [0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01]
        # #print "accel raw: " + str(accel[0]) + ", " + str(accel[1]) + ", " + str(accel[2])
        # #print "accel (m/s2): " + str((accel[0]-2048.0)/800.0*9.81) + ", " + str((accel[1]-2048.0)/800.0*9.81) + ", " + str((accel[2]-2048.0)/800.0*9.81)
        # accel_msg.angular_velocity.x = 0
        # accel_msg.angular_velocity.y = 0
        # accel_msg.angular_velocity.z = 0
        # accel_msg.angular_velocity_covariance = [0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01]
        # q = tf.transformations.quaternion_from_euler(0, 0, 0)
        # accel_msg.orientation = Quaternion(*q)
        # accel_msg.orientation_covariance = [0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01]
        # self.accel_publisher.publish(accel_msg)

	def greeting(self):
		"""
		Hello by robot.
		"""
		rospy.loginfo("Greetings from ePuck 1106")
		player = subprocess.Popen(["mplayer", "r2d2.mp3", "-ss", "30"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

		# Set initial velocity to zero
		self._driver.set_motors_speed(0, 0)

		for i in range(10):
			self._driver.set_body_led(1)
			self._driver.set_front_led(0)
			rospy.sleep(0.2)
			self._driver.set_body_led(0)
			self._driver.set_front_led(1)
			rospy.sleep(0.2)
		self._driver.set_body_led(0)
		self._driver.set_front_led(0)


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
