#!/usr/bin/env python  
import rospy

import numpy 
import math
import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():
	t1 = geometry_msgs.msg.TransformStamped()
	t1.header.stamp = rospy.Time.now()
	t1.header.frame_id = "base_frame"
	t1.child_frame_id = "object_frame"
	
	q1 = tf.transformations.quaternion_from_euler(0.64, 0.64, 0.0)
	t1.transform.rotation.x = q1[0]
	t1.transform.rotation.y = q1[1]
	t1.transform.rotation.z = q1[2]
	t1.transform.rotation.w = q1[3]
	q12 = tf.transformations.translation_from_matrix(numpy.dot(tf.transformations.euler_matrix(0.64,0.64,0.0),tf.transformations.translation_matrix((1.5,0.8,0.0))))
	t1.transform.translation.x = q12[0]
	t1.transform.translation.y = q12[1]
	t1.transform.translation.z = q12[2]
	br.sendTransform(t1)

	#######--------------------------------
	t2 = geometry_msgs.msg.TransformStamped()
	t2.header.stamp = rospy.Time.now()
	t2.header.frame_id = "base_frame"
	t2.child_frame_id = "robot_frame"
	q2 = tf.transformations.quaternion_about_axis(1.5, (0,1,0))
	t2.transform.rotation.x = q2[0]
	t2.transform.rotation.y = q2[1]
	t2.transform.rotation.z = q2[2]
	t2.transform.rotation.w = q2[3]
	q22 = tf.transformations.translation_from_matrix(numpy.dot(tf.transformations.rotation_matrix(1.5, (0,1,0)),tf.transformations.translation_matrix((0.0,0.0,-2.0))))
	#print(numpy.dot(tf.transformations.rotation_matrix(1.5, (0,1,0)),tf.transformations.translation_matrix((0.0,0.0,-2.0))))
	t2.transform.translation.x = q22[0]
	t2.transform.translation.y = q22[1]
	t2.transform.translation.z = q22[2]
	br.sendTransform(t2)

   	######-----------------------------------
	t3 = geometry_msgs.msg.TransformStamped()
	t3.header.stamp = rospy.Time.now()
	t3.header.frame_id = "robot_frame"
	t3.child_frame_id = "camera_frame"
	t3.transform.translation.x = 0.3
	t3.transform.translation.y = 0.0
	t3.transform.translation.z = 0.3
	####  TODO
	# matrix from base to object
	T1 = numpy.dot(tf.transformations.euler_matrix(0.64,0.64,0.0),tf.transformations.translation_matrix((1.5,0.8,0.0)))
	# Inverse matrix of T1
	T1_inverse = tf.transformations.inverse_matrix(T1)
	# matrix from base to robot
	T2 = numpy.dot(tf.transformations.rotation_matrix(1.5, (0,1,0)),tf.transformations.translation_matrix((0.0,0.0,-2.0)))

	#inverse matrix of T2
	T2_inverse =  tf.transformations.inverse_matrix(T2)
	
	T3 = tf.transformations.translation_matrix((0.3,0.0,0.3))
	T3_inverse = tf.transformations.inverse_matrix(T3)
	# coordinate of object = [0,0,0]
	C_object = numpy.matrix([[0],[0],[0],[1]])
	C_object_in_base_coordinate = numpy.matmul(T1,C_object)
	C_object_in_robot_coordinate = numpy.matmul(T2_inverse,C_object_in_base_coordinate)
	C_object_in_camera_coordinate = numpy.matmul(T3_inverse,C_object_in_robot_coordinate)
	#print(C_object_in_camera_coordinate)
	C_in_C = C_object_in_camera_coordinate
	distance = math.sqrt(pow(C_in_C[0,0],2)+pow(C_in_C[1,0],2)+pow(C_in_C[2,0],2))
	A = numpy.array(C_in_C[0:3])
	print(A)
	B = numpy.array([[distance],[0],[0]])
	print(B)
	#R = numpy.dot(tf.transformations.inverse_matrix(A),B)
	anglez =  math.atan2(A[1,0],A[0,0])
	length = math.sqrt(pow(A[0,0],2)+pow(A[1,0],2))
	angley = -math.atan2(A[2,0],length)
	print(anglez)
	print(length)
	print(angley)
	Tf = numpy.dot(tf.transformations.rotation_matrix(anglez, (0,0,1)),tf.transformations.rotation_matrix(angley, (0,1,0)))
	q3 = tf.transformations.quaternion_from_matrix(Tf)
	print(q3)
	t3.transform.rotation.x = q3[0]
	t3.transform.rotation.y = q3[1]
	t3.transform.rotation.z = q3[2]
	t3.transform.rotation.w = q3[3]
	
	br.sendTransform(t3)

if __name__ == '__main__':
    rospy.init_node('solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.1)
