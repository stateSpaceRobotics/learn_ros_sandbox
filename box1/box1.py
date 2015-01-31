#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

'''
This sandbox will cause the simulated robot to drive in a large circle.
'''

class sandbox_1(object):

	def __init__(self):
		'''
		Constructor for sandbox_1 class.
		'''
		rospy.init_node("sandbox_1")						# initialize this as a ROS node named 'sandbox_1'
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)		# create publisher that can be used to publish twist messages over /cmd_vel topic

		
	def run(self):
		'''
		When this function is called, commands will continuously be sent to the
		 simulated robot that will make the robot drive in a circle.
		'''
		while not rospy.is_shutdown():
			twist = Twist()  # create an empty twist message

			# Populate the twist message with values (these should be self-explanatory)!
			twist.angular.x = 0.0	# Roll
			twist.angular.y = 0.0	# Pitch
			twist.angular.z = -0.15 # Yaw

			twist.linear.x = 0.75
			twist.linear.y = 0.0
			twist.linear.z = 0.0

			self.cmd_vel_pub.publish(twist)		# publish our twist message to the cmd_vel topic
			rospy.sleep(0.1)					# sleep briefly so ROS doesn't die


if __name__ == "__main__":
	sandbox = sandbox_1()
	sandbox.run()
