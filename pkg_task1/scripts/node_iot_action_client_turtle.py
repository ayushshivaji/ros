#!/usr/bin/env python
import time
import math
import rospy
import actionlib
from pkg_task1.msg import msgTurtleAction
from pkg_task1.msg import msgTurtleGoal
from pkg_task1.msg import msgTurtleFeedback
from pkg_task1.msg import msgTurtleResult
from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal

class simpleturtleclient:

	def __init__(self):
		
		self.sac=actionlib.SimpleActionClient('/action_turtle',msgTurtleAction)
		self.sac_iot=actionlib.SimpleActionClient('/action_ros_iot',msgRosIotAction)
		self.sac_iot.wait_for_server()
		rospy.loginfo("Iot Server is now up")
		self.sac.wait_for_server()
		rospy.loginfo("Server is up now")
	
	def ros_callback(self,msg):
		if msg.message == 'start' :
			rospy.loginfo('starting to send goals')
			self.goals()

	def goals(self):
		self.send_goal(2,0)
		rospy.sleep(15)
		self.send_goal(2,60)
		rospy.sleep(15)
		self.send_goal(2,60)
		rospy.sleep(15)
		self.send_goal(2,60)
		rospy.sleep(15)
		self.send_goal(2,60)
		rospy.sleep(15)
		self.send_goal(2,60)
		

	def mqtt_send_goal(self,arg_protocol,arg_mode,arg_message):
		msg=msgRosIotGoal()
		msg.protocol=arg_protocol
		msg.mode=arg_mode
		msg.message=arg_message
		self.sac_iot.send_goal(msg)

	def mqtt_sub(self):
		msg=msgRosIotGoal()
		msg.protocol='mqtt'
		msg.mode='sub'
		self.sac_iot.send_goal(msg)

	def send_goal(self,arg_distance,arg_theta):
		goal=msgTurtleGoal(distance=arg_distance, theta=arg_theta)
		self.sac.send_goal(goal,done_cb=self.done_callback)
		rospy.loginfo('Goals sent')

	def done_callback(self,status,result):
		print 'DONE!!!'
		msg=str(result.final_x)+","+str(result.final_y)+","+str(result.final_theta)
		self.mqtt_send_goal('http','',msg)
		rospy.sleep(5)
		self.mqtt_send_goal('mqtt','pub',msg)
		#rospy.sleep(2)
		

def main():

	rospy.init_node('node_iot_action_client_turtle')
	client=simpleturtleclient()
	client.mqtt_sub()
	subscriber=rospy.Subscriber('/ros_iot_bridge/mqtt/sub',msgMqttSub,client.ros_callback)

	rospy.spin()

if __name__ == '__main__':
	main()
