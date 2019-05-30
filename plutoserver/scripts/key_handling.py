#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int16
import rospy

class send_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_server')
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		self.key_value =0
		self.cmd = PlutoMsg()
		self.cmd.rcRoll =1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX1 =1500
		self.cmd.rcAUX2 =1500
		self.cmd.rcAUX3 =1500
		self.cmd.rcAUX4 =1000
		self.cmd.commandType = 0
		self.cmd.trim_roll = 0
		self.cmd.trim_pitch = 0

		rospy.Subscriber('/input_key', Int16, self.indentify_key )

	def arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1000
		self.cmd.rcAUX4 =1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def box_arm(self):
		self.cmd.rcRoll=1500
		self.cmd.rcYaw=1500
		self.cmd.rcPitch =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcAUX4 =1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.5)

	def disarm(self):
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.5)

	def indentify_key(self, msg):
		self.key_value = msg.data

		if self.key_value == 70:
			if(self.cmd.rcAUX4 == 1500):
				self.disarm()
			else:
				self.arm()
		elif self.key_value == 10:
			self.forward()
		elif self.key_value == 30:
			self.left()
		elif self.key_value == 40:
			self.right()
		elif self.key_value == 80:
			self.reset()
		elif self.key_value == 50:
			self.increase_height()
		elif self.key_value == 60:
			self.decrease_height()
		elif self.key_value == 110:
			self.backward()
		elif self.key_value == 130:
			self.take_off()
		elif self.key_value == 140:
			self.land()
		elif self.key_value == 150:
			self.left_yaw()
		elif self.key_value == 160:
			self.right_yaw()
		self.command_pub.publish(self.cmd)

	def forward(self):
		self.cmd.rcPitch =1600
		self.command_pub.publish(self.cmd)
	def backward(self):
		self.cmd.rcPitch =1400
		self.command_pub.publish(self.cmd)
	def left(self):
		self.cmd.rcRoll =1400
		self.command_pub.publish(self.cmd)
	def right(self):
		self.cmd.rcRoll =1600
		self.command_pub.publish(self.cmd)
	def left_yaw(self):
		self.cmd.rcYaw = 1200
		self.command_pub.publish(self.cmd)
	def right_yaw(self):
		self.cmd.rcYaw = 1800
		self.command_pub.publish(self.cmd)
	def reset(self):
		self.cmd.rcRoll =1500
		self.cmd.rcThrottle =1500
		self.cmd.rcPitch =1500
		self.cmd.rcYaw = 1500
		self.cmd.commandType = 0
		self.command_pub.publish(self.cmd)
	def increase_height(self):
		self.cmd.rcThrottle = 1800
		self.command_pub.publish(self.cmd)
	def decrease_height(self):
		self.cmd.rcThrottle =1300
		self.command_pub.publish(self.cmd)
	def take_off(self):
		self.disarm()
		self.box_arm()
		self.cmd.commandType = 1
		self.command_pub.publish(self.cmd)
	def land(self):
		self.cmd.commandType = 2
		self.command_pub.publish(self.cmd)


if __name__ == '__main__':
	test = send_data()
	while not rospy.is_shutdown():
		rospy.spin()
		sys.exit(1)
