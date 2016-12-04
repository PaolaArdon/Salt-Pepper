#!/usr/bin/env python
import math
import numpy as np
import argparse
import time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from naoqi import ALProxy

def callback(data):
	motionProxy  = ALProxy("ALMotion", "10.42.0.76", 9559)

	if data.buttons[7] == 1:
		rospy.loginfo("Going to sleep.")
		motionProxy.rest()
	if data.buttons[5] == 1:
		rospy.loginfo("Waking up.")
		motionProxy.wakeUp()

	headMove = np.sum(data.buttons[0:4])
	rospy.loginfo(headMove)
	if headMove > 0:
		radian = 0.0174533
		names  = ["HeadYaw","HeadPitch"]
		useSensor = True
		fractionMaxSpeed  = 0.5

		angleCurrent = motionProxy.getAngles(names, useSensor)
		newYaw = angleCurrent[0]
		newPitch = angleCurrent[1]
		# left:
		if data.buttons[0] == 1:
			newYaw = angleCurrent[0]+data.buttons[0]*radian*2.0
		# right: Working
		if data.buttons[2] == 1:
			newYaw = angleCurrent[0]-data.buttons[2]*radian*2.0
		# top: Working
		if data.buttons[3] == 1:
			newPitch = angleCurrent[1]-data.buttons[3]*radian*2.0
		# down
		if data.buttons[1] == 1:
			newPitch = angleCurrent[1]+data.buttons[1]*radian*2.0

		angles = [newYaw, newPitch]
		rospy.loginfo(angles)
		motionProxy.setAngles(names, angles, fractionMaxSpeed)
		

	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(100) # 100hz
	vel = Twist()
	vel.linear.x = data.axes[5]/5.0
	vel.angular.z = data.axes[4]/5.0
	rospy.loginfo(vel)
	pub.publish(vel)
	rate.sleep()
def fromJoy():
	print "Initilizing node"
	rospy.init_node('joy_to_pepper', anonymous=True)
	rospy.Subscriber("/joy", Joy, callback)
	rospy.spin()

if __name__ == '__main__':
	fromJoy()
