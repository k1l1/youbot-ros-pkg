#!/usr/bin/env python
import rospy
import tf
import numpy as np
import math

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from brics_actuator.msg import JointValue
from brics_actuator.msg import JointPositions

from youbot_arm_kinematics.kinematics import Kinematics
from youbot_arm_kinematics.dh_parameters import YouBotDHParameters

ks = Kinematics(YouBotDHParameters.DH_A, YouBotDHParameters.DH_ALPHA,
YouBotDHParameters.DH_D, YouBotDHParameters.DH_THETA)

pos = Pose()


def callback(msg):
	
	q = np.array([msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4]])

	xyz, qtn, rpy, h = ks.forward(q)
	pos.position.x = xyz[0]
	pos.position.y = xyz[1]
	pos.position.z = xyz[2]

	pos.orientation.x = qtn[0]
	pos.orientation.y = qtn[1]
	pos.orientation.z = qtn[2]
	pos.orientation.w = qtn[3]
	

def kinematics_node():
	update_rate = 10 #10Hz
	rospy.init_node('youbot_kinematics_info_node',anonymous=False)
	
	pub_pose = rospy.Publisher('endeffector_cart_state', Pose , queue_size=10)
	
	rospy.Subscriber('/joint_states', JointState, callback)
	
	rate = rospy.Rate(update_rate)
	
	while not rospy.is_shutdown():
	
		pub_pose.publish(pos)
		rate.sleep()
		
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



