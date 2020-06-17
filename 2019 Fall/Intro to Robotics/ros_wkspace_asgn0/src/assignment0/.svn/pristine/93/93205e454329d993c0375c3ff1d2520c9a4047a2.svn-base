#!/usr/bin/env python

import rospy
from assignment0.msg import TwoInt
from std_msgs.msg import Int16


rospy.init_node('listener', anonymous=True)


def callback(data):
	#rospy.loginfo(rospy.get_caller_id() + "The two random int is " + str(data.num1) +" and "+str(data.num1))
	pub = rospy.Publisher('sum',Int16,queue_size = 10)
	rate = rospy.Rate(10) # 10hz
	#while not rospy.is_shutdown():
	sum_number = Int16(data.num1 + data.num2)
	rospy.loginfo(str(data.num1)+ " + " + str(data.num2) + " = " + str(sum_number))
	pub.publish(sum_number)
	print("published"+str(type(sum_number)))
	rate.sleep()
	
def listener():
	
	data = rospy.Subscriber("numbers", TwoInt, callback)
	rospy.spin()
	


if __name__ == '__main__':
	while not rospy.is_shutdown():
		try:
			listener()
		except rospy.ROSInterruptException:
			pass
	




