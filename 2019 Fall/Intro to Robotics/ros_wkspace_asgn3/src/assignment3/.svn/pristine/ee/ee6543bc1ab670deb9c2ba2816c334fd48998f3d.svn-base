#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock
import time

'''This is a class which will perform both cartesian control and inverse
kinematics'''
class CCIK(object):
	def __init__(self):
	#Load robot from parameter server
		self.robot = URDF.from_parameter_server()

	#Subscribe to current joint state of the robot
		rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

	#This will load information about the joints of the robot
		self.num_joints = 0
		self.joint_names = []
		self.q_current = []
		self.joint_axes = []
		self.get_joint_info()

	#This is a mutex
		self.mutex = Lock()

	#Subscribers and publishers for for cartesian control
		rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
		self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
		self.joint_velocity_msg = JointState()

		#Subscribers and publishers for numerical IK
		rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
		self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
		self.joint_command_msg = JointState()

	'''This is a function which will collect information about the robot which
	   has been loaded from the parameter server. It will populate the variables
	   self.num_joints (the number of joints), self.joint_names and
	   self.joint_axes (the axes around which the joints rotate)'''
	def get_joint_info(self):
		link = self.robot.get_root()
		while True:
			if link not in self.robot.child_map: break
			(joint_name, next_link) = self.robot.child_map[link][0]
			current_joint = self.robot.joint_map[joint_name]
			if current_joint.type != 'fixed':
			    self.num_joints = self.num_joints + 1
			    self.joint_names.append(current_joint.name)
			    self.joint_axes.append(current_joint.axis)
			link = next_link

	'''This is the callback which will be executed when the cartesian control
	   recieves a new command. The command will contain information about the
	   secondary objective and the target q0. At the end of this callback, 
	   you should publish to the /joint_velocities topic.'''
	def get_cartesian_command(self, command):
		self.mutex.acquire()
		#--------------------------------------------------------------------------
		#FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE

		joint_transforms, b_T_ee_curr = self.forward_kinematics(self.q_current)
		b_T_ee_des_T = tf.transformations.translation_matrix((command.x_target.translation.x,command.x_target.translation.y,command.x_target.translation.z))
		# print("desired T in base coordinate",b_T_ee_des_T)
		b_T_ee_des_R = tf.transformations.quaternion_matrix([command.x_target.rotation.x,command.x_target.rotation.y,command.x_target.rotation.z,command.x_target.rotation.w])
		# print("desired R in base coordinate ",b_T_ee_des_R)
		b_T_ee_des = numpy.dot(b_T_ee_des_T,b_T_ee_des_R)
		# print("desired T&R in base coordinate ",b_T_ee_des)
		ee_curr_T_b = numpy.linalg.inv(b_T_ee_curr)
		curr_T_des = ee_curr_T_b.dot(b_T_ee_des)
		# print("desired T in ee coordinate ",curr_T_des)
		translation_ee = tf.transformations.translation_from_matrix(curr_T_des)
		rotation_ee = self.rotation_from_matrix(curr_T_des)
		rotation_ee = rotation_ee[0]*rotation_ee[1]
		# print(translation_ee,"translation in ee")
		# print("start here",rotation_ee,"rotation in ee")
		# print("Vee",curr_T_des)
		t_max = numpy.abs(translation_ee).max(0)
		r_max = numpy.abs(rotation_ee).max(0)
		if t_max > 0.1:
			translation_ee = (0.1/t_max) * translation_ee
		if r_max > 2:
			rotation_ee = (1/r_max) * rotation_ee
		Vee = numpy.array([translation_ee, rotation_ee]).reshape(6)
		
		J = self.get_jacobian(b_T_ee_curr, joint_transforms)
		J_plus = numpy.linalg.pinv(J)
		q_desired = J_plus.dot(Vee)
		if command.secondary_objective:
			ps = 5
			q_sec = numpy.zeros(self.num_joints)
			q_sec[0] = ps*(command.q0_target - self.q_current[0])
			J_plus_dot_J = J_plus.dot(J)
			q_null = (numpy.identity(J_plus_dot_J.shape[1])-J_plus_dot_J)
			q_null = q_null.dot(q_sec)
			q_desired += q_null
		#q_desired = q_desired / (numpy.linalg.norm(q_desired))
		q_max = numpy.abs(q_desired).max(0)
		gain = 1
		if q_max > gain:
			q_desired = (gain / q_max ) * q_desired
		velocity = JointState()
		velocity.name = self.joint_names
		velocity.velocity = q_desired
		
		# print(q_desired)
		self.velocity_pub.publish(velocity)
		#--------------------------------------------------------------------------
		self.mutex.release()



	def skew(self, x):
    		return numpy.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

	def get_jacobian(self, b_T_ee_curr, joint_transforms):
		J = numpy.zeros((6,self.num_joints))
		#--------------------------------------------------------------------------
		#FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
		#for each joint
		i = 0
		for each in joint_transforms:
			# print(self.joint_names[i])
			b_T_J = each
			J_T_ee = numpy.linalg.inv(each).dot(b_T_ee_curr) 
			ee_T_J = numpy.linalg.inv(J_T_ee)
			B_R_A = ee_T_J[0:3,0:3]
			A_t_B = J_T_ee[0:3,3:4]
			axis = numpy.array([0.0,0.0,0.0])			
			axis = numpy.append(axis,self.joint_axes[i])
			axis = numpy.expand_dims(axis.reshape(6),1)
			V_J = numpy.zeros((6,6))
			V_J[0:3,0:3],V_J[3:6,3:6] = B_R_A,B_R_A 
			V_J[0:3,3:6] = -1*B_R_A.dot(self.skew(A_t_B))
			J[:,i] = V_J.dot(axis).flatten()
			
			i+=1
		#--------------------------------------------------------------------------
		return J;

	'''This is the callback which will be executed when the inverse kinematics
	   recieve a new command. The command will contain information about desired
	   end effector pose relative to the root of your robot. At the end of this
	   callback, you should publish to the /joint_command topic. This should not
	   search for a solution indefinitely - there should be a time limit. When
	   searching for two matrices which are the same, we expect numerical
	   precision of 10e-3.
	'''
	def get_ik_command(self, command):
		self.mutex.acquire()
		#start timer
		# print("start")
		command_cc = CartesianCommand()
		command_cc.x_target.translation = command.translation
		command_cc.x_target.rotation = command.rotation
		b_T_ee_des_T = tf.transformations.translation_matrix((command.translation.x,command.translation.y,command.translation.z))
		# print("desired T in base coordinate",b_T_ee_des_T)
		b_T_ee_des_R = tf.transformations.quaternion_matrix([command.rotation.x,command.rotation.y,command.rotation.z,command.rotation.w])
		# print("desired R in base coordinate ",b_T_ee_des_R)
		b_T_ee_des = numpy.dot(b_T_ee_des_T,b_T_ee_des_R)
		p = 1
		for i in range(3):
			#initial random q
			time_out = 10 #10 sec
			time_start = time.time()
			self.q_current = random.sample(numpy.arange(0.0,2*3.14,0.1),self.num_joints)
			joint_transforms, b_T_ee_curr = self.forward_kinematics(self.q_current)
			error =  1
			#(time.time() - time_start < time_out) and 
			while (time.time() - time_start < time_out) and (error > 0.01):
				# print("while loop start")
				joint_transforms, b_T_ee_curr = self.forward_kinematics(self.q_current)
				# print("start cartisain command")
				ee_curr_T_b = numpy.linalg.inv(b_T_ee_curr)
				curr_T_des = ee_curr_T_b.dot(b_T_ee_des)
				# print("desired T in ee coordinate ",curr_T_des)
				translation_ee = tf.transformations.translation_from_matrix(curr_T_des)
				rotation_ee = self.rotation_from_matrix(curr_T_des)
				rotation_ee = rotation_ee[0]*rotation_ee[1]
				Vee = numpy.array([translation_ee, rotation_ee]).reshape(6)
				J = self.get_jacobian(b_T_ee_curr, joint_transforms)
				J_plus = numpy.linalg.pinv(J)
				q_desired = J_plus.dot(Vee)
				velocity = JointState()
				velocity.name = self.joint_names
				velocity.velocity = q_desired
				# print("caculate q_curent intergration")
				self.q_current += p*q_desired
				error = numpy.max(numpy.abs(b_T_ee_curr - b_T_ee_des))
				print(i,error,(time.time() - time_start))
			if error < 0.01:
				q = JointState()
				q.name = self.joint_names
				q.position = self.q_current
				self.joint_command_pub.publish(q)
				# print("published")
				break
		# print("quit")
		pass
		self.mutex.release()

	'''This function will return the angle-axis representation of the rotation
	   contained in the input matrix. Use like this: 
	   angle, axis = rotation_from_matrix(R)'''
	def rotation_from_matrix(self, matrix):
		# print(self.joint_names)
		R = numpy.array(matrix, dtype=numpy.float64, copy=False)
		R33 = R[:3, :3]
		# axis: unit eigenvector of R33 corresponding to eigenvalue of 1
		l, W = numpy.linalg.eig(R33.T)
		i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
		if not len(i):
			raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
		axis = numpy.real(W[:, i[-1]]).squeeze()
		# point: unit eigenvector of R33 corresponding to eigenvalue of 1
		l, Q = numpy.linalg.eig(R)
		i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
		if not len(i):
			raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
		# rotation angle depending on axis
		cosa = (numpy.trace(R33) - 1.0) / 2.0
		if abs(axis[2]) > 1e-8:
			sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
		elif abs(axis[1]) > 1e-8:
			sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
		else:
			sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
		angle = math.atan2(sina, cosa)
		return angle, axis

	'''This is the function which will perform forward kinematics for your 
	   cartesian control and inverse kinematics functions. It takes as input
	   joint values for the robot and will return an array of 4x4 transforms
	   from the base to each joint of the robot, as well as the transform from
	   the base to the end effector.
	   Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
	#return the skew matrix 3X3 only

	def forward_kinematics(self, joint_values):
		joint_transforms = []

		link = self.robot.get_root()
		T = tf.transformations.identity_matrix()

		while True:
			if link not in self.robot.child_map:
			    break

			(joint_name, next_link) = self.robot.child_map[link][0]
			joint = self.robot.joint_map[joint_name]

			T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
			T = numpy.dot(T, T_l)

			if joint.type != "fixed":
			    joint_transforms.append(T)
			    q_index = self.joint_names.index(joint_name)
			    T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
			    T = numpy.dot(T, T_j)

			link = next_link
		return joint_transforms, T #where T = b_T_ee

	'''This is the callback which will recieve and store the current robot
	   joint states.'''
	def get_joint_state(self, msg):
		self.mutex.acquire()
		self.q_current = []
		for name in self.joint_names:
			self.q_current.append(msg.position[msg.name.index(name)])
		self.mutex.release()


if __name__ == '__main__':
	rospy.init_node('cartesian_control_and_IK', anonymous=True)
	CCIK()
	print("hi")
	rospy.spin()
