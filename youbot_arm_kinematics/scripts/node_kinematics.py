#!/usr/bin/env python
import rospy
import tf
import numpy as np
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from brics_actuator.msg import JointValue
from brics_actuator.msg import JointPosition

from youbot_arm_kinematics.kinematics import Kinematics
from youbot_arm_kinematics.dh_parameters import YouBotDHParameters


def callback_incomming_pose()

def kinematics_node():
	
	update_rate = 10 #10Hz
	rospy.init_node('youbot_kinematics_node',anonymous=False)
	
	#pub_pose = rospy.Publisher('arm_1/endeffector_pose', Pose)
	pub_joint = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions)
	
	#rospy.Subscriber('endeffector_pose_commands', Pose, callback_incomming_pose)
	
	rate = rospy.Rate(update_rate)
	joint_msg = JointPositions()
	joint_msg.positions = [bricks_actuator.msg.JointValue()]
	
	joint_msg.positions[0].joint_uri = "arm_joint_1"
	joint_msg.positions[1].joint_uri = "arm_joint_2"
	joint_msg.positions[2].joint_uri = "arm_joint_3"
	joint_msg.positions[3].joint_uri = "arm_joint_4"
	joint_msg.positions[4].joint_uri = "arm_joint_5"
	
	for i in range(0,4):
		joint_msg.position[i].unit = "rad"
	
	
	while not rospy.is_shutdown():
	
		#do something
		joint_msg.positions[0].value  = 2.10046
		joint_msg.positions[1].value  = 2.36209
		joint_msg.positions[2].value  = -3.07113
		joint_msg.positions[3].value  = 1.08429
		joint_msg.positions[4].value  = 3.77257
		
		rate.sleep()
		
def stop():
	rospy.loginfo("STOP")
	
if __name__ == '__main__':
	try:
	kinematics_node()
	except KeyboardInterrupt:
		stop()
	execpt rospy.ROSInterruptException:
		stop()
		pass



