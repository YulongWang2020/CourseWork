#!/usr/bin/env python
# license removed for brevity

#A node called "generator.py". At a frequency of 10Hz, 
#this node must generate two random integers between 0 and 100,
#use them to populate the two fields of a message of type TwoInt, 
#and publish the message on a topic called "/numbers".

import rospy
import random
from assignment0.msg import TwoInt

def talker():
	pub = rospy.Publisher('numbers',TwoInt, queue_size = 10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	
	
	while not rospy.is_shutdown():
		number = TwoInt()
		number.num1 = random.randint(0,100)
		number.num2 = random.randint(0,100)
		rospy.loginfo(number)
		pub.publish(number)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
