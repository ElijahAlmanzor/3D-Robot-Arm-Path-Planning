#!/usr/bin/env python

import rospy
import intera_interface
import ik_service_client
import numpy as np
import math
from gazebo_msgs.msg import ModelStates 
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler, euler_from_quaternion
reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)
test = rospy.Subscriber("/gazebo/model_states", ModelStates, callback) 
from geometry_msgs.msg import(
	Point,
	Quaternion,
	Pose,
	PoseStamped
)


rospy.init_node("test")
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper()
pose = Pose()
fin_pos = [0.8996526516230774, 0.110300850025158, 0.5800905399641927]
fin_ori_q = [0.45397200528693116, 0.541973281397203, 0.5421077589188734, 0.4541955067889456]
final_pose = Pose()
final_pose.position.x = fin_pos[0]
final_pose.position.y = fin_pos[1]
final_pose.position.z = fin_pos[2]
final_pose.orientation.x = fin_ori_q[0]
final_pose.orientation.y = fin_ori_q[1]
final_pose.orientation.z = fin_ori_q[2]
final_pose.orientation.w = fin_ori_q[3]  


def initialise_pose_orientation():
	current_pose = limb.endpoint_pose()
	pose.position.x = current_pose['position'].x
	pose.position.y = current_pose['position'].y
	pose.position.z = current_pose['position'].z
	

def initialise_pose_position():
	current_pose = limb.endpoint_pose()
	pose.orientation.x = current_pose['orientation'].x
	pose.orientation.y = current_pose['orientation'].y
	pose.orientation.z = current_pose['orientation'].z
	pose.orientation.w = current_pose['orientation'].w
	#0.13754 takes into account the robot tool tip length

	
def moveTo(next_pose):
	limb_joints = limb.ik_request(next_pose, "right_gripper_tip")
	#send the cartesian and quaternion coordinates to ik_service_client
	if limb_joints != False:
		limb.move_to_joint_positions(limb_joints)	
		return True
	else:
		return False



def moveSawyer(index):
	if index == 0:
		pose.position.x += 0.05
	elif index == 1:
		pose.position.x -= 0.05
	elif index == 2:
		pose.position.y += 0.05
	elif index == 3:
		pose.position.y -= 0.05
	elif index == 4:
		pose.position.z += 0.05
	elif index == 5:
		pose.position.z -= 0.05
	#can't seem to make it move properly.
	elif index == 6:
	#atm it euler rotates at 10 degrees
		temp_euler = list(euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))
		temp_euler[0] += 0.2618 #5 degrees
		temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
		pose.orientation.x = temp_ori[0]
		pose.orientation.y = temp_ori[1]
		pose.orientation.z = temp_ori[2]
		pose.orientation.w = temp_ori[3]
	elif index == 7:
		temp_euler = list(euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))
		temp_euler[0] -= 0.2618 
		temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
		pose.orientation.x = temp_ori[0]
		pose.orientation.y = temp_ori[1]
		pose.orientation.z = temp_ori[2]
		pose.orientation.w = temp_ori[3]
	elif index == 8:
		temp_euler = list(euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))
		temp_euler[1] += 0.2618 #0.0873 5 degrees
		temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
		pose.orientation.x = temp_ori[0]
		pose.orientation.y = temp_ori[1]
		pose.orientation.z = temp_ori[2]
		pose.orientation.w = temp_ori[3]
	elif index == 9:
		temp_euler = list(euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))
		temp_euler[1] -= 0.2618 #0.0873
		temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
		pose.orientation.x = temp_ori[0]
		pose.orientation.y = temp_ori[1]
		pose.orientation.z = temp_ori[2]
		pose.orientation.w = temp_ori[3]
	elif index == 10:
		temp_euler = list(euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))
		temp_euler[2] += 0.2618 
		temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
		pose.orientation.x = temp_ori[0]
		pose.orientation.y = temp_ori[1]
		pose.orientation.z = temp_ori[2]
		pose.orientation.w = temp_ori[3]
	elif index == 11:
		temp_euler = list(euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))
		temp_euler[2] -= 0.2618 
		temp_ori = quaternion_from_euler(temp_euler[0], temp_euler[1], temp_euler[2])
		pose.orientation.x = temp_ori[0]
		pose.orientation.y = temp_ori[1]
		pose.orientation.z = temp_ori[2]
		pose.orientation.w = temp_ori[3]
     
	if moveTo(pose):
		return True
	else: 
		return False
	
limb.move_to_neutral()	
reset()	
limb.move_to_neutral()
initialise_pose_position() #should only be used to initialised
initialise_pose_orientation() #should only be used to initialised

	
#obs_move = [4,4,4,3,3,3,0,0,0,0,0,0,0,7,2,2,7,4,7,4,7,4,7,4,0,0,7,4,7,4,7,4,7]
obs_move = [3, 7, 2, 10, 11, 3, 0, 5, 2, 7, 7, 9, 3, 9, 7, 8, 10, 6, 8, 9, 1, 4, 2, 0, 11, 2, 10, 7, 2, 5]
for i, index in enumerate(obs_move):
	print(index)
	moveSawyer(index)
moveTo(final_pose)
test.unregister()



'''
if not moveSawyer(index):
		print("Invalid Path")
		break
		'''
print("Endpoint cartesian")
cart = [pose.position.x,pose.position.y,pose.position.z]
print(cart)
print("Endpoint orientation")
orient = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
print(orient)
print("Endpoint orientation in Euler")
euler = list(euler_from_quaternion(orient))
print(euler)

