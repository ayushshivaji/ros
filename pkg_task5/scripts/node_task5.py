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
import datetime
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



incoming_order_list=list()
packages_priority_map=dict()

color_map=list()

class task5:

    def __init__(self):
        param_config=rospy.get_param('config_pyiot')
        self.config_google_sheet_inventory=param_config['google_apps']['spread_sheet_inventory']
        self.config_server_url=param_config['mqtt']['server_url']
        self.config_server_port=param_config['mqtt']['server_port']
        self.config_mqtt_topic_to_sub=param_config['mqtt']['topic_to_sub']
        # self.config_mqtt_topic_to_pub=param_config['mqtt']['topic_to_pub']
        self.config_qos=param_config['mqtt']['qos']
        self.bridge = CvBridge()
        
        self._robot_ns = '/'  + "ur5_1"
        self._planning_group = "manipulator"
    
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self.box_name = 'package'
        self._group.set_planning_time(10)

        self._computed_plan = ''

        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')
        
    def get_time_str(self):
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time

    # def my_variadic(self,url,**kwargs):
    #     msg=iot.http_publish(url,kwargs)    	
    #     print(msg)
    def my_variadic(self,url,kwargs):
        msg=iot.http_publish(url,kwargs)        
        print(msg)

    def update_inventory_sheet(self,sku,item,priority,storage,cost):
        param={"id":"Inventory", "Team Id":"VB#1363", "Unique Id":"aYJaAnaB", "SKU":sku,"Item":item,"Priority":priority,"Storage Number":storage,"Cost":cost,"Quantity":1}
        self.my_variadic(self.config_google_sheet_inventory,param)

    def update_incoming_orders_sheet(self,Ord_id,ord_date_time,item,priority,ord_qty,city,lon,lat,cost):
        param={"id":"IncomingOrders", "Team Id":"VB#1363", "Unique Id":"aYJaAnaB", "Order Id":Ord_id,"Order Date and Time":ord_date_time,"Item":item,"Priority":priority,"Order Quantity":ord_qty,"City":city,"Longitude":lon,"Latitude":lat,"Cost":cost}
        self.my_variadic(self.config_google_sheet_inventory,param)

    def update_dispatch_orders_sheet(self,Ord_id,city,item,priority,disp_qty,cost,disp_status,disp_date_time):
        param={"id":"OrdersDispatched", "Team Id":"VB#1363", "Unique Id":"aYJaAnaB", "Order Id":Ord_id,"City":city,"Item":item,"Priority":priority,"Dispatch Quantity":disp_qty,"Cost":cost,"Dispatch Status":disp_status,"Dispatch Date and Time":disp_date_time}
        self.my_variadic(self.config_google_sheet_inventory,param)

    def camera2D_callback_func(self, data):
        global color_map     #a dictonary to hold color mapping of each package
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        (rows, cols, channels) = cv_image.shape
        resized_image = cv2.resize(cv_image, (720/2, 1280/2))

        color_map = self.get_packages_color(resized_image)

        cv2.waitKey(3)


      #This function processes the input image and return a dictonary containing package name as key and their respective color as value
    def get_packages_color(self, arg_image):
        color_list = list()  #local variable to store color of all the packages

        color_list.append(self.get_dominant_colour(arg_image[150:220, 50:130],'0','0'))
        color_list.append(self.get_dominant_colour(arg_image[150:220, 150:210],'0','1'))
        color_list.append(self.get_dominant_colour(arg_image[150:220, 230:310],'0','2'))
        color_list.append(self.get_dominant_colour(arg_image[240:300, 50:130],'1','0'))
        color_list.append(self.get_dominant_colour(arg_image[240:300, 150:210],'1','1'))
        color_list.append(self.get_dominant_colour(arg_image[240:300, 230:310],'1','2'))
        color_list.append(self.get_dominant_colour(arg_image[310:370, 50:130],'2','0'))
        color_list.append(self.get_dominant_colour(arg_image[310:370, 150:210],'2','1'))
        color_list.append(self.get_dominant_colour(arg_image[310:370, 230:310],'2','2'))
        color_list.append(self.get_dominant_colour(arg_image[380:450, 50:130],'3','0'))
        color_list.append(self.get_dominant_colour(arg_image[380:450, 150:210],'3','1'))
        color_list.append(self.get_dominant_colour(arg_image[380:450, 230:310],'3','2'))
        # cv2.imshow("/eyrc/vb/camera_1/image_raw",arg_image)
        return color_list   #returnig color_list to camera2D_callback_func that will store this data in a global variable color_map             


    def get_dominant_colour(self, arg_img,row,col):
        global packages_priority_map
        b = arg_img[:, :, :1]
        g = arg_img[:, :, 1:2]
        r = arg_img[:, :, 2:]

        b_mean = np.mean(b)
        g_mean = np.mean(g)
        r_mean = np.mean(r)

        if g_mean > r_mean and g_mean > b_mean:
            packages_priority_map["Clothes"].append(row+col)
            return ['G','G'+row+col,'Clothes','LP','R'+row+' C'+col,150]
        elif r_mean > g_mean and r_mean > b_mean:
            packages_priority_map["Medicine"].append(row+col)
            return ['R','R'+row+col,'Medicine','HP','R'+row+' C'+col,450]
        else:
            packages_priority_map["Food"].append(row+col)
            return ['Y','Y'+row+col,'Food','MP','R'+row+' C'+col,250]

    	# def update_incoming_orders_sheet(self,Ord_id,ord_date_time,item,priority,ord_qty,city,lon,lat,cost):
    def on_recv_msg_mqtt(self, client, userdata, message):
        global incoming_order_list
        # payload = message.payload.decode("utf-8")
        payload= json.loads(message.payload)
        if payload["item"]=="Clothes":
            pri="LP"
            price=150
        elif payload["item"]=="Food":
            pri="MP"
            price=250
        elif payload["item"]=="Medicine":
            pri="HP"
            price=450
        incoming_order_list.append(payload)	
        self.update_incoming_orders_sheet(payload["order_id"],payload["order_time"],payload["item"],pri,payload["qty"],payload["city"], payload["lon"],payload["lat"],price)
        print(payload)




    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
           	 # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = waypoints[0].orientation.x
        wpose.orientation.y = waypoints[0].orientation.y
        wpose.orientation.z = waypoints[0].orientation.z
        wpose.orientation.w = waypoints[0].orientation.w

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
                waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
        	del plan.joint_trajectory.points[0]
        	del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)


    def hard_go_to_pose(self, arg_pose, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
        	number_attempts += 1
        	flag_success = self.go_to_pose(arg_pose)
        	rospy.logwarn("attempts: {}".format(number_attempts))
        	# self.clear_octomap()


    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        self._group.set_pose_target(arg_pose)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        list_joint_values = self._group.get_current_joint_values()

        if flag_plan == True:
        	pass
        else:
        	pass

        return flag_plan


    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
        	number_attempts += 1
        	flag_success = self.set_joint_angles(arg_list_joint_angles)
        	rospy.logwarn("attempts: {}".format(number_attempts))
        # self.clear_octomap()


    def set_joint_angles(self, arg_list_joint_angles):
	
        list_joint_values = self._group.get_current_joint_values()
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)
        self._group.stop()

        list_joint_values = self._group.get_current_joint_values()
        pose_values = self._group.get_current_pose().pose

        if (flag_plan == True):
        	pass
        else:
        	pass

        return flag_plan


    def wait_for_state_update(self, box_nam, box_is_known=False, box_is_attached=False, timeout=4):

        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self._scene.get_attached_objects([box_nam])

            is_attached = len(attached_objects.keys()) > 0

            is_known = box_nam in self._scene.get_known_object_names()

            if(box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            self._seconds = rospy.get_time()

        return False


    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # # self.clear_octomap()

        return True

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)

        return ret


    def conveyor_belt_control(self, pwr):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            handle_conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            final_res = handle_conveyor(pwr)
            return final_res
        except rospy.ServiceException as e:
            print("service call failed %s"%e)


    def ur5_1_vacuum_on_off(self, val_tf):  
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as err_service:
            print ("Service call failed: %s" %err_service)

    def add_box(self, x, y, z, box_nam, timeout=4):    
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/world"
        box_pose.pose.position.x = x 
        box_pose.pose.position.y = y 
        box_pose.pose.position.z = z 
        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0

        self._scene.add_box(box_nam, box_pose, size=(0.15, 0.15, 0.15))
        return self.wait_for_state_update(box_nam, box_is_known=True, timeout=timeout)


    def attach_box(self, box_nam, timeout=5):   #to attach the package to the end effector in planning scene
        grasping_group = 'manipulator'
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, box_nam, touch_links=touch_links)
        return self.wait_for_state_update(box_nam, box_is_attached=True, box_is_known=False, timeout=timeout)


    def detach_box(self, box_nam, timeout=5):  #To detach the package from the end effector
        self._scene.remove_attached_object(self._eef_link, name=box_nam)
        return self.wait_for_state_update(box_nam, box_is_known=True, box_is_attached=False,timeout=timeout)


    def remove_box(self, box_nam, timeout=5):    #to remove package from the planning scene
        self._scene.remove_world_object(box_nam)
        return self.wait_for_state_update(box_nam, box_is_attached=False, box_is_known=False, timeout=timeout)


    def pick_place(self, pkg_num):

        pkg_key = 'packagen' + pkg_num
        pick_pkg_filename = 'go_to_package' + pkg_num + '.yaml'
        place_pkg_filename = 'package' + pkg_num + '_to_belt.yaml'

        self.moveit_hard_play_planned_path_from_file(self._file_path, pick_pkg_filename, 10)    
        rospy.sleep(1)
        # self.attach_box(pkg_key)
        # rospy.sleep(1)
        self.ur5_1_vacuum_on_off(True)
        rospy.sleep(1)
        self.moveit_hard_play_planned_path_from_file(self._file_path, place_pkg_filename, 10)   
        rospy.sleep(1)
        self.ur5_1_vacuum_on_off(False)
        rospy.sleep(1)
        # self.detach_box(pkg_key)
        # rospy.sleep(1)
        # self.remove_box(pkg_key)
        # rospy.sleep(1)

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5_1Moveit Deleted." + '\033[0m')



	# def __del__(self):
	# 	moveit_commander.roscpp_shutdown()
	# 	rospy.loginfo('\033[94m' + "Object of class task5 Deleted." + '\033[0m')

def main():
    global packages_priority_map
    global incoming_order_list
    packages_priority_map["Clothes"]=list()
    packages_priority_map["Food"]=list()
    packages_priority_map["Medicine"]=list()
    intial_joint_angles = [math.radians(7.7848814851),
                        math.radians(-138.015566396),
                        math.radians(-58.0424957695),
                        math.radians(-71.7379508013),
                        math.radians(90.3676987576),
                        math.radians(-84.2097855739)]
    rospy.init_node('node_task5')
    obj_task5=task5()
    obj_color = colorMsg()
    var_handle_pub = rospy.Publisher('my_topic', colorMsg, queue_size=10)
    image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, obj_task5.camera2D_callback_func)
    rospy.sleep(2)
    image_sub.unregister()
    obj_task5.hard_set_joint_angles(intial_joint_angles, 10)

    try:
        
        #getting incoming orders
    	ret = iot.mqtt_subscribe_thread_start(obj_task5.on_recv_msg_mqtt,obj_task5.config_server_url,obj_task5.config_server_port,obj_task5.config_mqtt_topic_to_sub,obj_task5.config_qos)
    	if ret == 0 :
    		rospy.loginfo('MQTT subscribe thread started')
    	else:
    		rospy.loginfo('MQTT subscribe failed')

        # print(color_map)
    	#updating inventory sheet
    	for val in color_map:
            obj_task5.update_inventory_sheet(val[1]+"0221",val[2],val[3],val[4],val[5])
            # print("Done")
            rospy.sleep(0.5)
    	package_picked=1
        # print(obj_task5.get_time_str())
        # to_pick=""
        # new_order=dict()
    	while package_picked<=9:
            if len(incoming_order_list)>0:
                new_order=incoming_order_list[0]
                to_pick=new_order["item"]
                # print(incoming_order_list)
                # print(type(incoming_order_list))

                tnow=obj_task5.get_time_str()
                if to_pick=="Clothes":
                	obj_task5.pick_place(packages_priority_map["Clothes"][0])
                	obj_task5.update_dispatch_orders_sheet(new_order["order_id"],new_order["city"],new_order["item"],"LP",new_order["qty"],150,"Yes",tnow)
                	obj_color.box_color = "green "+str(new_order["order_id"])+" "+str(new_order["city"])+" "+str(new_order["item"])+" "+"LP"+" "+str(new_order["qty"])+" "+str(150)
                    	var_handle_pub.publish(obj_color)
                    	packages_priority_map["Clothes"].pop(0)
                elif to_pick =="Food":
                	obj_task5.pick_place(packages_priority_map["Food"][0])
                	obj_task5.update_dispatch_orders_sheet(new_order["order_id"],new_order["city"],new_order["item"],"MP",new_order["qty"],250,"Yes",tnow)
                	obj_color.box_color = "yellow "+str(new_order["order_id"])+" "+str(new_order["city"])+" "+str(new_order["item"])+" "+"MP"+" "+str(new_order["qty"])+" "+str(250)
                    	var_handle_pub.publish(obj_color)
                    	packages_priority_map["Food"].pop(0)
                elif to_pick=="Medicine":
                    obj_task5.pick_place(packages_priority_map["Medicine"][0])
                    obj_task5.update_dispatch_orders_sheet(new_order["order_id"],new_order["city"],new_order["item"],"HP",new_order["qty"],450,"Yes",tnow)
                    obj_color.box_color = "red "+str(new_order["order_id"])+" "+str(new_order["city"])+" "+str(new_order["item"])+" "+"HP"+" "+str(new_order["qty"])+" "+str(450)
                    var_handle_pub.publish(obj_color)
                    packages_priority_map["Medicine"].pop(0)

                incoming_order_list.pop(0)
                package_picked+=1

    		
    	rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

    cv2.destroyAllWindows()
    # del obj_task5

if __name__ == '__main__':
    main()



