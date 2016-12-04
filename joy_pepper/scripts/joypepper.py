#!/usr/bin/env python
import math
import numpy as np

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from naoqi import ALProxy

class JoyPepper:
	def __init__(self):
		self.ROBOT_IP		= "192.168.150.7"
		self.ROBOT_PORT		= 9559
		self.motionProxy	= ALProxy("ALMotion", self.ROBOT_IP, self.ROBOT_PORT)
		self.sub 			= rospy.Subscriber("/joy", Joy, self.joyCallback)
		self.pub 			= rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	def joyCallback(self, data):

		if data.buttons[7] == 1:
			rospy.loginfo("Going to sleep.")
			self.motionProxy.rest()
		if data.buttons[5] == 1:
			rospy.loginfo("Waking up.")
			self.motionProxy.wakeUp()

		headMove = np.sum(data.buttons[0:4])
		rospy.loginfo(headMove)
		if headMove > 0:
			radian = 0.0174533
			names  = ["HeadYaw","HeadPitch"]
			useSensor = True
			fractionMaxSpeed  = 0.5

			angleCurrent = self.motionProxy.getAngles(names, useSensor)
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
			self.motionProxy.setAngles(names, angles, fractionMaxSpeed)
		rate = rospy.Rate(100) # 100hz
		vel = Twist()
		if math.fabs(data.axes[4]) > 0:
			vel.linear.x = data.axes[5]/16.0
			vel.angular.z = data.axes[4]/16.0
		else:
			vel.linear.x = data.axes[5]/8.0
		rospy.loginfo(vel)
		self.pub.publish(vel)
		rate.sleep()

def main():
	jp = JoyPepper()
	rospy.init_node('joy_to_pepper', anonymous=True)
	ROBOT_IP     = "192.168.150.7"
  	ROBOT_PORT   = 9559
  	proxy  = ALProxy("ALTextToSpeech", ROBOT_IP, ROBOT_PORT)
  	# proxy.say("Localization mode is activated!");
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")

if __name__ == '__main__':
	main()
