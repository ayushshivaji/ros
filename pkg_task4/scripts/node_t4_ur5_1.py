#!/usr/bin/env python
from __future__ import print_function
import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import rospkg
import yaml
import os
import math
import time
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import numpy as np

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from hrwros_gazebo.msg import LogicalCameraImage

from std_srvs.srv import Empty
from pkg_task4.msg import colorMsg


color_map = dict()  #To store color of each packages detected using color detection

class Task4_node1:
    def __init__(self):
        global color_map

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
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo("Package Path: {}".format(self._file_path))


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

   
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

  
     #2D camera callback function
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
        color_list = dict()  #local variable to store color of all the packages

        #cropping the image to get each package's image and then detecting the color  
        color_list['packagen00'] = self.get_dominant_colour(arg_image[150:220, 50:130])  
        color_list['packagen01'] = self.get_dominant_colour(arg_image[150:220, 150:210])
        color_list['packagen02'] = self.get_dominant_colour(arg_image[150:220, 230:310])
        color_list['packagen10'] = self.get_dominant_colour(arg_image[240:300, 50:130])
        color_list['packagen11'] = self.get_dominant_colour(arg_image[240:300, 150:210])
        color_list['packagen12'] = self.get_dominant_colour(arg_image[240:300, 230:310])
        color_list['packagen20'] = self.get_dominant_colour(arg_image[310:370, 50:130])
        color_list['packagen21'] = self.get_dominant_colour(arg_image[310:370, 150:210])
        color_list['packagen22'] = self.get_dominant_colour(arg_image[310:370, 230:310])
        color_list['packagen30'] = self.get_dominant_colour(arg_image[380:450, 50:130])
        color_list['packagen31'] = self.get_dominant_colour(arg_image[380:450, 150:210])
        color_list['packagen32'] = self.get_dominant_colour(arg_image[380:450, 230:310])
        # print(color_list)
        # cv2.imshow("/eyrc/vb/camera_1/image_raw",arg_image)
        # cv2.imshow("/eyrc/vb/camera_1/image_raw",arg_image[x:x+h,y:y+w)
    
        return color_list   #returnig color_list to camera2D_callback_func that will store this data in a global variable - color_map             


    def get_dominant_colour(self, arg_img):
        b = arg_img[:, :, :1]
        g = arg_img[:, :, 1:2]
        r = arg_img[:, :, 2:]

        b_mean = np.mean(b)
        g_mean = np.mean(g)
        r_mean = np.mean(r)

        if g_mean > r_mean and g_mean > b_mean:
            return 'green'
        elif r_mean > g_mean and r_mean > b_mean:
            return 'red'
        else:
            return 'yellow'


    #To activate/deactivate vacuum gripper of ur5_1 by calling rosservice
    def ur5_1_vacuum_on_off(self, val_tf):  
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as err_service:
            print ("Service call failed: %s" %err_service)


    #To activate/deactivate vacuum gripper of ur5_2 by calling rosservice
    def ur5_2_vacuum_on_off(self, val_tf):   
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as err_service:
            print ("Service call failed: %s"%err_service)


    #To add the box to the planning scene in RViz
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
        global color_map

        pkg_key = 'packagen' + pkg_num
        pick_pkg_filename = 'go_to_package' + pkg_num + '.yaml'
        place_pkg_filename = 'package' + pkg_num + '_to_belt.yaml'

        self.moveit_hard_play_planned_path_from_file(self._file_path, pick_pkg_filename, 10)    
        rospy.sleep(1)
        self.attach_box(pkg_key)
        rospy.sleep(1)
        self.ur5_1_vacuum_on_off(True)
        rospy.sleep(1)
        self.moveit_hard_play_planned_path_from_file(self._file_path, place_pkg_filename, 10)   
        rospy.sleep(1)
        self.ur5_1_vacuum_on_off(False)
        rospy.sleep(1)
        self.detach_box(pkg_key)
        rospy.sleep(1)
        self.remove_box(pkg_key)
        rospy.sleep(1)

    def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo('\033[94m' + "Object of class Ur5_1Moveit Deleted." + '\033[0m')

def main(args):
  
    global color_map
    var_handle_pub = rospy.Publisher('my_topic', colorMsg, queue_size=10)  #ROS Topic to communicate with node_t4_ur5_2 regarding color of the packages
 
    rospy.init_node('node_t4_ur5_1', anonymous=True)
    var_loop_rate = rospy.Rate(1)
    obj_color = colorMsg()
  
    ur5_1 = Task4_node1()   #ur5_1 is object of class Task4


    intial_joint_angles = [math.radians(7.7848814851),
                          math.radians(-138.015566396),
                            math.radians(-58.0424957695),
                             math.radians(-71.7379508013),
                             math.radians(90.3676987576),
                             math.radians(-84.2097855739)]

    go_to_package30 = [math.radians(-57.2871594674),  
                         math.radians(-97.5399620727),  
                         math.radians(106.906360596),  
                         math.radians(-99.504221891),  
                         math.radians(-90.7821683281),   
                         math.radians(40.3563745024)]

    try:

        image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, ur5_1.camera2D_callback_func)

        #Adding packages to the planning scene
        ur5_1.add_box(0.26, -0.42, 1.90, 'packagen00')
        ur5_1.add_box(0.0, -0.42, 1.90, 'packagen01')
        ur5_1.add_box(-0.26, -0.42, 1.90, 'packagen02')
        ur5_1.add_box(0.26, -0.42, 1.64, 'packagen10')
        ur5_1.add_box(0.0, -0.42, 1.64, 'packagen11')
        ur5_1.add_box(-0.26, -0.42, 1.64, 'packagen12')
        ur5_1.add_box(0.26, -0.42, 1.41, 'packagen20')
        ur5_1.add_box(0, -0.42, 1.41, 'packagen21')
        ur5_1.add_box(-0.28, -0.42, 1.43, 'packagen22')
        ur5_1.add_box(0.28, -0.42, 1.19, 'packagen30')
        ur5_1.add_box(0, -0.42, 1.18, 'packagen31')
        ur5_1.add_box(-0.26, -0.42, 1.18, 'packagen32')
    


        ur5_1.hard_set_joint_angles(intial_joint_angles, 10)
        rospy.sleep(2)
        image_sub.unregister()


        #for picking 8 packages using saved trajectories
        pkg_suffix_list = ['00', '01', '02', '10', '11', '12', '20', '21']
        for suffix in pkg_suffix_list:
            pkg_name = 'packagen' + suffix   #example: packagen00,packagen01 etc
            obj_color.box_color = color_map[pkg_name]    #example: color_map['packagen00']  
            ur5_1.pick_place(suffix)
            var_handle_pub.publish(obj_color)
    

        #for picking and placing packagen30
        obj_color.box_color = color_map['packagen30']
        ur5_1.hard_set_joint_angles(go_to_package30, 10)
        rospy.sleep(1)
        ur5_1.attach_box('packagen30')
        rospy.sleep(1)
        ur5_1.ur5_1_vacuum_on_off(True)
        rospy.sleep(1)
        ur5_1.ee_cartesian_translation(0, 0.15, 0)
        ur5_1.hard_set_joint_angles(intial_joint_angles, 10)
        rospy.sleep(1)
        ur5_1.ur5_1_vacuum_on_off(False)
        rospy.sleep(1)
        ur5_1.detach_box('packagen30')
        rospy.sleep(1)
        ur5_1.remove_box('packagen30')
        rospy.sleep(1)
        var_handle_pub.publish(obj_color)
    
    
        # rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
  
    cv2.destroyAllWindows()
    del ur5_1

if __name__ == '__main__':
    main(sys.argv)




