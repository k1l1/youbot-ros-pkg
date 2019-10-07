#!/usr/bin/env python
import rospy
import tf
import numpy as np
import math
import sys, termios, tty, os, time



from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from brics_actuator.msg import JointValue
from brics_actuator.msg import JointPositions

msg = Pose()

msg.position.x = 0.55
msg.position.y = 0.0
msg.position.z = 0.0

msg.orientation.x = 0.0
msg.orientation.y = 0.0
msg.orientation.z = 0.0
msg.orientation.w = 1.0

increment = 0.01 # 1cm

def getch():
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	return ch
	

def kinematics_node():
	global msg

	update_rate = 10 #10Hz
	rospy.init_node('youbot_manual_cart_control',anonymous=False)
	
	pub_pose = rospy.Publisher('endeffector_pose_command', Pose , queue_size=1)
	
	rate = rospy.Rate(update_rate)
	
	fd = sys.stdin.fileno()
	fl = fcntl.fcntl(fd, fcntl.F_GETFL)
	fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
	
	while not rospy.is_shutdown():
	
		pub_pose.publish(pos)

		print("pos: " + str(xyz) + " rpy: " + str(rpy) + " qtn: " + str(qtn))  
		rate.sleep()
		
		keystroke = getch()
		
		if keystroke == "a":
			msg.position.x = msg.position.x + increment
		if keystroke == "y":
			msg.position.x = msg.position.x - increment
		if keystroke == "s":
			msg.position.y = msg.position.y + increment
		if keystroke == "x":
			msg.position.y = msg.position.y - increment
		if keystorke == "d":
			msg.position.z = msg.position.z + increment
		if keystroke == "c":
			msg.position.z = msg.position.z - increment
	
def stop():
	rospy.loginfo("STOP")
	
if __name__ == '__main__':
	try:
		kinematics_node()

	except KeyboardInterrupt:
		stop()
	except rospy.ROSInterruptException:
		stop()
		pass



