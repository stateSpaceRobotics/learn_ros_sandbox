#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

'''
This sandbox provides an example of how to get odometry from the simulated robot.
It will also show you how to use the ROS parameter server.
'''

class sandbox_3(object):

	def __init__(self):
		'''
		Constructor for sandbox_3 node.
		'''
		rospy.init_node("sandbox_3")									# initialize this as a ROS node named 'sandbox_3'
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)			# create publisher that can be used to publish twist messages over /cmd_vel topic

		
	def run(self):
		'''
		When this function is called, commands will continuously be sent to the
		 simulated robot that will make the robot drive in a circle.
		'''
		while not rospy.is_shutdown():
			twist = Twist()		# create an empty twist message

			

			self.cmd_vel_pub.publish(twist)		# publish our twist message to cmd_vel topic

			rospy.sleep(0.1)					# sleep briefly so ROS doesn't die


if __name__ == "__main__":
	sandbox = sandbox_3()
	sandbox.run()
