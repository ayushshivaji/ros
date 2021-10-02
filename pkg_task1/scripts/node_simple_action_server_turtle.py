#!/usr/bin/env python

import time
import math
import actionlib
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import rospy
from pkg_task1.msg import msgTurtleAction
from pkg_task1.msg import msgTurtleGoal
from pkg_task1.msg import msgTurtleFeedback
from pkg_task1.msg import msgTurtleResult
from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotActionGoal


class simpleturtleserver:

	def __init__(self):
		self.sas=actionlib.SimpleActionServer('/action_turtle',msgTurtleAction,execute_cb=self.on_recv_goal,auto_start=False)
		self.subscriber=rospy.Subscriber('/turtle1/Pose',Pose,self.update_pose)
		self.velpublisher=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
		self.new_pose=Pose()
		
		self.curr_x=0
		self.curr_y=0
		self.curr_theta=0

		self.sas.start()
		rospy.loginfo("Started Turtle Simple Action Server.")


	def update_pose(self,data):
		self.curr_theta=data.theta
		self.curr_x=data.x
		self.curr_y=data.y

	def on_recv_goal(self,goal):
		rospy.loginfo("Goal Recieved")
		self.func_rotate(goal.theta,10,'a')
		self.move_straight(goal.distance,3,'a')
		result = msgTurtleResult()
		result.final_x=int(self.curr_x)
		result.final_y=int(self.curr_y)
		result.final_theta=int(self.curr_theta)
		self.sas.set_succeeded(result)

	def func_rotate(self, param_degree, param_speed, param_dir):
		vel = Twist()
		new_pose = Pose()
		start_degree = abs(math.degrees(self.curr_theta))
		current_degree = abs(math.degrees(self.curr_theta))

		if(param_dir == 'a'):
			vel.angular.z = math.radians(abs(int(param_speed)))  # Anticlockwise
		else:
			vel.angular.z = (-1) * math.radians(abs(int(param_speed)))
			math.radians(abs(int(param_speed)))

		var_loop_rate = rospy.Rate(75)
		degree_rotated = 0.0
		while not rospy.is_shutdown():
			if(degree_rotated < param_degree):
				self.velpublisher.publish(vel)
				var_loop_rate.sleep()
				current_degree = abs(math.degrees(self.curr_theta))
				degree_rotated = abs(current_degree - start_degree)
				#print('Degree Rotated: {}'.format(degree_rotated))
			else:
				break# Stop the Turtle after the desired angle is reached
		vel.angular.z = 0
		self.velpublisher.publish(vel)
		print('Angle Reached')
		

	def move_straight(self,param_dis,param_speed,param_dir):
		vel=Twist()
		new_pose=Pose()
		start_x=self.curr_x
		start_y=self.curr_y
		dist =0.0		
  
		if(param_dir == 'b'):
			vel.linear.x=(-1)*abs(int(param_speed))
		else:
			vel.linear.x = abs(int(param_speed))

		var_loop_rate = rospy.Rate(75)
		while not rospy.is_shutdown():
			feedback_msg=msgTurtleFeedback()
			feedback_msg.cur_x=self.curr_x
			feedback_msg.cur_y=self.curr_y
			self.sas.publish_feedback(feedback_msg)

			if ((dist < param_dis)):
				self.velpublisher.publish(vel)
				var_loop_rate.sleep()
				dist = abs(math.sqrt(((self.curr_x - start_x) ** 2) + ((self.curr_y - start_y) ** 2)))
			else:
				break    # Stop the Turtle after desired distance is covered
		vel.linear.x = 0
		self.velpublisher.publish(vel)
       # print('Destination Reached in distance')

def main():
	rospy.init_node('node_simple_action_server_turtle')
   	obj=simpleturtleserver()
   	handle_sub_pose = rospy.Subscriber('/turtle1/pose', Pose, obj.update_pose)
   	rospy.spin()

if __name__ == '__main__':
	main()
