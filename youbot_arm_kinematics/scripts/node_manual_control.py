#!/usr/bin/env python
import rospy
import tf
import numpy as np
import math
import curses



from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from brics_actuator.msg import JointValue
from brics_actuator.msg import JointPositions

msg = Pose()

msg.position.x = 0.0
msg.position.y = 0.0
msg.position.z = 0.55

msg.orientation.x = 0.0
msg.orientation.y = 0.0
msg.orientation.z = 1.0
msg.orientation.w = 0.0
increment = 0.01 # 1cm

increment2 = 0.1

euler = tf.transformations.euler_from_quaternion((0.0,0.0,1.0,0.0))
rpy = np.array([euler[0],euler[1],euler[2]])
def manual_control_node(stdscr):
	global msg, increment, increment2

	update_rate = 10 #10Hz
	rospy.init_node('youbot_manual_cart_control',anonymous=False)
	
	pub_pose = rospy.Publisher('endeffector_pose_commands', Pose , queue_size=1)
	
	rate = rospy.Rate(update_rate)
	
	stdscr.nodelay(1)
	
	while not rospy.is_shutdown():
	
		rate.sleep()
		
		keystroke = stdscr.getch()
		print(keystroke)
		
		if keystroke == 97:
			msg.position.x = msg.position.x + increment
		if keystroke == 121:
			msg.position.x = msg.position.x - increment
		if keystroke == 115:
			msg.position.y = msg.position.y + increment
		if keystroke == 120:
			msg.position.y = msg.position.y - increment
		if keystroke == 100:
			msg.position.z = msg.position.z + increment
		if keystroke == 99:
			msg.position.z = msg.position.z - increment

		if keystroke == 102:
			rpy[0] = rpy[0] + increment2
		if keystroke == 118:
			rpy[0] = rpy[0] - increment2

		if keystroke == 102:
			rpy[1] = rpy[1] + increment2
		if keystroke == 118:
			rpy[1] = rpy[1] - increment2

		if keystroke == 102:
			rpy[2] = rpy[2] + increment2
		if keystroke == 118:
			rpy[2] = rpy[2] - increment2


		q = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
		
		msg.orientation.x = q[0]
		msg.orientation.y = q[1]
		msg.orientation.z = q[2]
		msg.orientation.w = q[3]

		pub_pose.publish(msg)

	
def stop():
	rospy.loginfo("STOP")
	
if __name__ == '__main__':
	try:
		curses.wrapper(manual_control_node)

	except KeyboardInterrupt:
		stop()
	except rospy.ROSInterruptException:
		stop()
		pass



