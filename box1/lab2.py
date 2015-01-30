#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math
from math import *

obstacle_force = [0,0]
robot = [0,0,0]

def add_forces(a, b):
    #This function adds to force vectors together and returns the result
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c

def wrap_angle(angle):
    #This function will take any angle and wrap it into the range [-pi, pi]
    while angle >= math.pi:
        angle = angle - 2*math.pi
        
    while angle <= -math.pi:
        angle = angle + 2*math.pi
    return angle

def goal_force(angle_to_goal):
    #This function determines and returns the attractive force [s_x,s_y] to the goal, given the angle to the goal

    #Parameter : MODIFY
    strength = 0.75 #This is the magnitude of the attractive goal force
    #End of Parameters

    s_x = strength * cos(angle_to_goal)
    s_y = strength * sin(angle_to_goal)

    return [s_x,s_y]

def drive_from_force(force,twist):
    #This function takes in a force [x,y] and determines the drive command (Twist) that should be sent to the robot motors
    global robot

    #Parameters : MODIFY
    turn_multiplier = 0.1 #This is multiplied by the angle difference to get the turn command
    spin_threshold = math.pi #If the difference between the robot's yaw and the force direction is greater than this, we only spin
    drive_multiplier = .15 #This is multiplied by the magnitude of the force vector to get the drive forward commnd
    #End of Parameters

    #Determine the angle of the force
    force_angle = atan2(force[1], force[0])
    force_mag = hypot(force[0], force[1])
    
    # print("Force angle: " + str(degrees(force_angle)))
    # print("Force mag: " + str(force_mag))
    # print("Robot angle: " + str(degrees(robot[2])))
    #Get difference to robot's current yaw
    a_diff = wrap_angle(force_angle - robot[2])

    #Get turn speed
    twist.angular.z = turn_multiplier * a_diff

    #Do we just spin
    if abs(a_diff) < spin_threshold:
        twist.linear.x = drive_multiplier * force_mag

    return twist

def laserCallback(data):
    #This function looks at the current laser reading and computes the obstacle avoidance force vector
    #which is set in the global variable 'obstacle force'

    global obstacle_force  #format = [x,y]
    global robot  #format = [x_position, y_position, yaw]
    
    #Make sure this is zeroed out before beginning
    obstacle_force = [0,0]

    #Parameters: MODIFY
    strength = -1 #This is the magnitude of the repulsive force from each individual laser scan data point
    obstacle_distance_threshold = 1.5 #How close to the obstacle do we have to be to feel repulsed
    #End of Parameters

    print 'Got new laser scan at ', rospy.Time.now()

    cur_angle = data.angle_min #cur_angle will always have the relative angle between the robot's yaw and the current laser reading

    for i in range(len(data.ranges)):
        if data.ranges[i] < obstacle_distance_threshold:
            strength_x = 0.0
            strength_y = 0.0
            #TASK 2: BEGIN CODE: (Compute obstacle avoidance force from this laser reading and save as [strength_x, strength_y)
            if (degrees(cur_angle) < 75 and degrees(cur_angle) > -75): 
                yaw = robot[2]
                strength_x = strength * cos(yaw + cur_angle)
                strength_y = strength * sin(yaw + cur_angle)

            #TASK 2: END CODE
            obstacle_force = add_forces(obstacle_force, [strength_x, strength_y]) 
            
        
        cur_angle = cur_angle + data.angle_increment
    ob_len = hypot(obstacle_force[0], obstacle_force[1])
    if (ob_len != 0):
        obstacle_force = [obstacle_force[0]/ob_len, obstacle_force[1]/ob_len]
    
def robotCallback(data):
    #This function updates the robots position and yaw, based on the ground truth (we don't have localization yet)
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]


if __name__ == '__main__':
    rospy.init_node('lab2', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan", LaserScan, laserCallback) #Subscribe to the laser scan topic
    rospy.Subscriber("base_pose_ground_truth", Odometry, robotCallback) #Subscribe to the robot pose topic

    global obstacle_force #format [x,y]
    global robot #format [x_position, y_position, yaw]

    #Hard coded goal location for this lab
    goal_x = -6
    goal_y = 10
    
    while not rospy.is_shutdown():
        twist = Twist()
        
        #Get vector from robot to goal
        angle_to_goal = 0.0
        #TASK 1: BEGIN CODE (Compute the angle from robot to the goal, save in 'angle_to_goal' variable)
        
        robot_x = robot[0]
        robot_y = robot[1]
        angle_to_goal = atan2((goal_y - robot_y), (goal_x - robot_x))
        
        #TASK 1: END CODE

        #Compute attractive force to goal
        g_force = goal_force(angle_to_goal)
        
        #Add to obstacle avoidance forces
        # print("=====================")
        # print("Obstacle angle: " + str(degrees(atan2(obstacle_force[1], obstacle_force[0]))))
        total_force = add_forces(g_force, obstacle_force)
        #Get final drive command from total force
        twist = drive_from_force(total_force, twist) 

        #Publish drive command, then sleep
        pub.publish(twist)
        rospy.sleep(0.1)
        

    print 'Done'

    twist = Twist()
    pub.publish(twist)

