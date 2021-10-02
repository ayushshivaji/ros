#!/usr/bin/env python

from __future__ import print_function
import rospy 
import time
import actionlib
from pyiot import iot
import requests
from std_msgs.msg import String
import rospkg
import yaml
import os
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
from datetime import datetime, timedelta
import json

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from hrwros_gazebo.msg import LogicalCameraImage

from std_srvs.srv import Empty
from pkg_task5.msg import colorMsg


package_color=list()
packages = list()

class Task4_node2:
  
    def __init__(self):
        # self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
        param_config=rospy.get_param('config_pyiot')
        self.config_google_sheet_inventory=param_config['google_apps']['spread_sheet_inventory']
        self.config_server_url=param_config['mqtt']['server_url']
        self.config_server_port=param_config['mqtt']['server_port']
        self.config_mqtt_topic_to_sub=param_config['mqtt']['topic_to_sub']
        # self.config_mqtt_topic_to_pub=param_config['mqtt']['topic_to_pub']
        self.config_qos=param_config['mqtt']['qos']

        self._robot_ns = '/'  + "ur5_2"
        self._planning_group = "manipulator"
    
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._group.set_planning_time(10)

        self._computed_plan = ''

        self.box_name = "package"

        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

    def my_variadic(self,url,kwargs):
        msg=iot.http_publish(url,kwargs)        
        print(msg)
    # def my_variadic(self,url,kwa):
    #     msg=iot.http_publish(url,kwa)        
    #     print(msg)

    def update_shipped_orders_sheet(self,Ord_id,city,item,priority,ship_qty,cost,ship_status,ship_date_time,est_del_time):
        param={"id":"OrdersShipped", "Team Id":"VB#1363", "Unique Id":"aYJaAnaB", "Order Id":Ord_id,"City":city,"Item":item,"Priority":priority,"Shipped Quantity":ship_qty,"Cost":cost,"Shipped Status":ship_status,"Shipped Date and Time":ship_date_time,"Estimated Time of Delivery":est_del_time}
        self.my_variadic(self.config_google_sheet_inventory,param)

    def get_time_str(self):
        timestamp = int(time.time())
        value = datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
    
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        self._group.stop()

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
        else:
            pass

        return flag_plan



    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self._scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = self.box_name in self._scene.get_known_object_names()
            if(box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            self._seconds = rospy.get_time()

        return False


    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

  
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False
        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
        return True


    def conveyor_belt_control(self, pwr):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            handle_conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            final_res = handle_conveyor(pwr)
            return final_res
        except rospy.ServiceException as e:
            print("service call failed %s"%e)


    def ur5_2_vacuum_on_off(self, val_tf):   #To activate/deactivate vacuum gripper by calling rosservice
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as err_service:
            print ("Service call failed: %s"%err_service)


    def add_box(self, xc, yc, zc, timeout=4):    #To add the box to the planning scene in RViz
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/world"
        box_pose.pose.position.x = xc
        box_pose.pose.position.y = yc
        box_pose.pose.position.z = zc
        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0
        self._scene.add_box(self.box_name, box_pose, size=(0.15, 0.15, 0.15))
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
        grasping_group = "manipulator"
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, self.box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)



    def detach_box(self, timeout=4):
        self._scene.remove_attached_object(self._eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)



    def remove_box(self, timeout=4):
        self._scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


    def func_callback_topic_my_topic(self,myMsg):
        global package_color
        rospy.loginfo("now %s package is coming.",myMsg.box_color)
        self.conveyor_belt_control(100)
        info_pkg=list(myMsg.box_color.split())
        package_color.append(info_pkg)
        return

    def update_data(self, data):
        global packages   #to store details of packagen1
        del packages[:]
        try:
            if data.models[0].type == 'packagen00' or data.models[0].type == 'packagen01' or data.models[0].type == 'packagen02' or data.models[0].type == 'packagen10' or data.models[0].type == 'packagen11' or data.models[0].type == 'packagen12' or data.models[0].type == 'packagen20' or data.models[0].type == 'packagen21' or data.models[0].type == 'packagen22' or data.models[0].type == 'packagen30' or data.models[0].type == 'packagen31' or data.models[0].type == 'packagen32':
                packages.append(data.models[0].pose.position.x)
                packages.append(data.models[0].pose.position.y)
                packages.append(data.models[0].pose.position.z)
            elif data.models[1].type == 'packagen00' or data.models[1].type == 'packagen01' or data.models[1].type == 'packagen02' or data.models[1].type == 'packagen10' or data.models[1].type == 'packagen11' or data.models[1].type == 'packagen12' or data.models[1].type == 'packagen20' or data.models[1].type == 'packagen21' or data.models[1].type == 'packagen22' or data.models[1].type == 'packagen30' or data.models[1].type == 'packagen31' or data.models[1].type == 'packagen32':
                packages.append(data.models[1].pose.position.x)
                packages.append(data.models[1].pose.position.y)
                packages.append(data.models[1].pose.position.z)
            elif data.models[2].type == 'packagen00' or data.models[2].type == 'packagen01' or data.models[2].type == 'packagen02' or data.models[2].type == 'packagen10' or data.models[2].type == 'packagen11' or data.models[2].type == 'packagen12' or data.models[2].type == 'packagen20' or data.models[2].type == 'packagen21' or data.models[2].type == 'packagen22' or data.models[2].type == 'packagen30' or data.models[2].type == 'packagen31' or data.models[2].type == 'packagen32':
                packages.append(data.models[2].pose.position.x)
                packages.append(data.models[2].pose.position.y)
                packages.append(data.models[2].pose.position.z)
            elif data.models[3].type == 'packagen00' or data.models[3].type == 'packagen01' or data.models[3].type == 'packagen02' or data.models[3].type == 'packagen10' or data.models[3].type == 'packagen11' or data.models[3].type == 'packagen12' or data.models[3].type == 'packagen20' or data.models[3].type == 'packagen21' or data.models[3].type == 'packagen22' or data.models[3].type == 'packagen30' or data.models[3].type == 'packagen31' or data.models[3].type == 'packagen32':
                packages.append(data.models[3].pose.position.x)
                packages.append(data.models[3].pose.position.y)
                packages.append(data.models[3].pose.position.z)
        except:
            '''
            package has not yet reached to the logical camera2
            '''
    

    #To pick a red color package and place it to red bin
    def do_red(self):
        global package_color
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'red_pkg_to_redbin.yaml', 10)
        rospy.sleep(1)
        self.detach_box()
        self.ur5_2_vacuum_on_off(False)
        self.remove_box()
        value=datetime.today()+timedelta(1)
        date_now = value.strftime('%Y-%m-%d %H:%M:%S')
        # date_now+=timedelta(days=1)
        tnow=self.get_time_str()
        self.update_shipped_orders_sheet(int(package_color[0][1]),package_color[0][2],package_color[0][3],package_color[0][4],int(package_color[0][5]),int(package_color[0][6]),"Yes",tnow,date_now)
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'return_from_redbin.yaml', 10)
        rospy.sleep(1)
        package_color.pop(0)
        return

# def update_shipped_orders_sheet(self,Ord_id,city,item,priority,ship_qty,cost,ship_status,ship_date_time,est_del_time):

     #To pick a green color package and place it to green bin
    def do_green(self):
        global package_color
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'green_pkg_to_greenbin.yaml', 10)
        rospy.sleep(1)
        self.detach_box()
        self.ur5_2_vacuum_on_off(False)
        self.remove_box()
        value=datetime.today()+timedelta(5)
        date_now = value.strftime('%Y-%m-%d %H:%M:%S')
        # date_now+=timedelta(days=1)
        tnow=self.get_time_str()
        self.update_shipped_orders_sheet(int(package_color[0][1]),package_color[0][2],package_color[0][3],package_color[0][4],int(package_color[0][5]),int(package_color[0][6]),"Yes",tnow,date_now)
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'return_from_greenbin.yaml', 10)
        rospy.sleep(1)
        package_color.pop(0)
        return

     #To pick a yellow color package and place it to yellow bin
    def do_yellow(self):
        global package_color
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'yellow_pkg_to_yellowbin.yaml', 10)
        rospy.sleep(1)
        self.detach_box()
        self.ur5_2_vacuum_on_off(False)
        self.remove_box()
        value=datetime.today()+timedelta(3)
        date_now = value.strftime('%Y-%m-%d %H:%M:%S')
        # date_now+=timedelta(days=1)
        tnow=self.get_time_str()
        self.update_shipped_orders_sheet(int(package_color[0][1]),package_color[0][2],package_color[0][3],package_color[0][4],int(package_color[0][5]),int(package_color[0][6]),"Yes",tnow,date_now)
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'return_from_yellowbin.yaml', 10)
        rospy.sleep(1)
        package_color.pop(0)
        return

     
    def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def main(args):

    global packages
    rospy.init_node('node_task5_2', anonymous=True)
    ur5_2=Task4_node2()
   
    handle_sub_mymsg = rospy.Subscriber("my_topic", colorMsg, ur5_2.func_callback_topic_my_topic)
    rospy.sleep(1)
    handle_sub_cam = rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage, ur5_2.update_data)
    rospy.sleep(1)
    

    try:

        intial_joint_angles = [math.radians(7.7848814851),
                             math.radians(-140.015566396),
                             math.radians(-58.0424957695),
                             math.radians(-71.7379508013),
                             math.radians(90.3676987576),
                             math.radians(-84.2097855739)]

        red_bin_joint_angles = [math.radians(75),
                                math.radians(-60),
                                math.radians(70),
                                math.radians(-107),
                                math.radians(-91),
                                math.radians(-100)]

        yellow_bin_joint_angles = [math.radians(-1),
                                  math.radians(-60),
                                  math.radians(70),
                                  math.radians(-107),
                                  math.radians(-91),
                                  math.radians(-100)]

        green_bin_joint_angles = [math.radians(100),
                                 math.radians(-125),
                                 math.radians(-79),
                                 math.radians(-65),
                                 math.radians(87),
                                 math.radians(11)]

        ur5_2.conveyor_belt_control(0)
        ur5_2.hard_set_joint_angles(intial_joint_angles,5)


        for count in range(9):   #this loop run 9 times to pick 9 packages
            while len(packages) == 0:
                '''when a package is not in visible range of logical camera2 '''
                '''As soon as a packages comes under the logical camera2, this loop terminates'''
            while packages[1] > 0.080080:
                ur5_2.conveyor_belt_control(80)

            ur5_2.conveyor_belt_control(0)
            ur5_2.add_box(-0.800369, -0.001243, 0.995000)
            rospy.sleep(0.5)
            ur5_2.ur5_2_vacuum_on_off(True)
            rospy.sleep(0.5)
            ur5_2.attach_box()
            if package_color[0][0]== 'red':
                ur5_2.do_red()
            elif package_color[0][0]== 'yellow':
                ur5_2.do_yellow()
            else:
                ur5_2.do_green()

        # rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    del ur5_2
# Python Main
if __name__ == '__main__':
    main(sys.argv)


