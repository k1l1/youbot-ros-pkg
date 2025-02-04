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

joint_msg = JointPositions()

for i in range(0, 5):
	joint_msg.positions.append(JointValue())
	joint_msg.positions[i].unit = "rad"
	
joint_msg.positions[0].joint_uri = "arm_joint_1"
joint_msg.positions[1].joint_uri = "arm_joint_2"
joint_msg.positions[2].joint_uri = "arm_joint_3"
joint_msg.positions[3].joint_uri = "arm_joint_4"
joint_msg.positions[4].joint_uri = "arm_joint_5"
	
joint_msg.positions[0].value  = 2.10046
joint_msg.positions[1].value  = 2.36209
joint_msg.positions[2].value  = -3.07113
joint_msg.positions[3].value  = 1.08429
joint_msg.positions[4].value  = 3.77257

def check_ranges(q,i):

	if not 0.0100692 < q[i,0] < 5.84014:
		return False
	if not  0.0100692 < q[i,1] < 2.61799:
		return False
	if not  -5.02655 < q[i,2] < -0.015708:
		return False
	if not  -0.0221239 < q[i,3] < 3.4292:
		return False
	if not  0.110619 < q[i,4] < 5.64159:
		return False
	
	return True	

def callback_incomming_pose(msg):
	#incomming Pose commands
	q = (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
	
	euler = tf.transformations.euler_from_quaternion(q)
	rpy = np.array([euler[0],euler[1],euler[2]])
	xyz = np.array([msg.position.x,msg.position.y,msg.position.z])
	
	qs, solves = ks.inverse(xyz,rpy)
	#if solves != [False,False]:
	#
	#	return
	#	print("No solution found")
	solution_in_range = []
	
	for i in range(qs.shape[0]):
		solutionValid = check_ranges(qs,i)
		solution_in_range.append(solutionValid)

		if solutionValid is True:
			joint_msg.positions[0].value = qs[i,0]
			joint_msg.positions[1].value = qs[i,1]
			joint_msg.positions[2].value = qs[i,2]
			joint_msg.positions[3].value = qs[i,3]
			joint_msg.positions[4].value = qs[i,4]
			
	print(solution_in_range)
	print(qs)
	print(solves)

def kinematics_node():
	update_rate = 10 #10Hz
	rospy.init_node('youbot_kinematics_control_node',anonymous=False)
	
	#pub_pose = rospy.Publisher('arm_1/endeffector_pose', Pose)
	pub_joint = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions, queue_size=1)
	
	rospy.Subscriber('endeffector_pose_commands', Pose, callback_incomming_pose)
	
	rate = rospy.Rate(update_rate)
	
	while not rospy.is_shutdown():
	
		
		pub_joint.publish(joint_msg)
		
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


