#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

'''
This robot drives forward and uses its laser range finder to avoid obstacles.
'''

class sandbox_2(object):

	def __init__(self):
		'''
		Constructor for sandbox_2 class.
		'''
		rospy.init_node("sandbox_2")									# initialize this as a ROS node named 'sandbox_1'
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)			# create publisher that can be used to publish twist messages over /cmd_vel topic
		rospy.Subscriber("base_scan", LaserScan, self.laser_callback) 	# subscribe to /base_scan topic using self.laser_callback as the callback function

		self.obstacle_detected = False 	# this variable will keep track of whether or not the robot currently sees an obstacle

	def laser_callback(self, data):
		'''
		This function gets called everytime a message is published over the /base_scan topic.
		'''
		
		comfort_threshold = 1.0 # how close to an obstacle we are comfortable getting

		flag = False
		# for each laser value in data.ranges
		for i in xrange(0, len(data.ranges)):
			# if the range value i is less than comfort_threshold, there is an obstacle in front of robot
			if data.ranges[i] < comfort_threshold:
				flag = True
		
		self.obstacle_detected = flag		
		

		
		
	def run(self):
		'''
		When this function is called, commands will continuously be sent to the
		 simulated robot that will make the robot drive in a circle.
		'''
		while not rospy.is_shutdown():
			twist = Twist()		# create an empty twist message

			if self.obstacle_detected:
				# if an obstacle is detected, just turn
				twist.angular.z = 0.25
				self.cmd_vel_pub.publish(twist)
			else:
				# otherwise, go forward
				twist.linear.x = 0.75   # forward
				twist.angular.z = 0.05  # make it turn a little bit

			self.cmd_vel_pub.publish(twist)		# publish our twist message to cmd_vel topic

			rospy.sleep(0.1)					# sleep briefly so ROS doesn't die


if __name__ == "__main__":
	sandbox = sandbox_2()
	sandbox.run()
