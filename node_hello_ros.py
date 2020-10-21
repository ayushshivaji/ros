#!/usr/bin/env python

import rospy


def main():
	rospy.init_node('node_hello_ros', anonymous=True)#initializing the node

	rospy.loginfo("Hello World")#print the message

	rospy.spin()#doesn't stop the node until interrupt is recieved

if __name__ == '__main__':#main def
	try:
		main()
	except rospy.ROSInterruptException:
		pass

