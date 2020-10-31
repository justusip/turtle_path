#!/usr/bin/env python3
import math
import rospy
from turtleservice.srv import *
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def getPose():
	return rospy.wait_for_message("turtle1/pose", Pose)

def onSetOrientation(args):
	while True:
		pose = getPose()
		diff = math.fmod(args.orientation - pose.theta + math.pi + 2 * math.pi, 2 * math.pi) - math.pi
		if abs(diff) < 0.05:
			break

		t = Twist()
		t.angular.z = diff
		p.publish(t)
	return True

def onWalkDistance(args):
	dist = args.distance
	if dist < 0:
		return False

	pose = getPose()
	destX = pose.x + math.cos(pose.theta) * dist
	destY = pose.y + math.sin(pose.theta) * dist
	if destX < 0 or destX > 11 or destY < 0 or destY > 11:
		return False

	while True:
		pose = getPose()
		if not math.sqrt((pose.x - destX) ** 2 + (pose.y - destY) ** 2) >= 0.05:
			break

		t = Twist()
		t.linear.x = 1
		p.publish(t)
	return True

rospy.init_node("path_manager")
p = rospy.Publisher(
		"turtle1/cmd_vel", 
		Twist, 
		queue_size = 1
	)
rospy.Service("set_orientation", SetOrientation, onSetOrientation)
rospy.Service("walk_distance", WalkDistance, onWalkDistance)
rospy.spin()
