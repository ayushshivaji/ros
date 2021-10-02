#! /usr/bin/env python

import rospy
import sys
import copy
import time
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse


from hrwros_gazebo.msg import LogicalCameraImage
pkg1=[]
pkg2=[]
pkg3=[]
class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg5_waypoints', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

        self.box_name = 'package1'
        

    def set_joint_angles(self, arg_list_joint_angles):  #to set the joint angles of the ur5
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan:
            rospy.loginfo('\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

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
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


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
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = self._scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = self.box_name in self._scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False


    def add_box(self, xc,yc,zc,timeout=4):    #To add the box to the planning scene in RViz
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


    def attach_box(self, timeout=4):   #to attach the package to the end effector in planning scene
        grasping_group = 'ur5_1_planning_group'
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, self.box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):  #To detach the package from the end effector
        self._scene.remove_attached_object(self._eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):    #to remove package from the planning scene
        self._scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def vacuum_on_off(self, val_tf):   #To activate/deactivate vacuum gripper by calling rosservice
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as err_service:
            print "Service call failed: %s"%err_service


    def conveyor_belt_power(self, pwr):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            handle_conveyor_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            final_belt_res = handle_conveyor_belt(pwr)
            return final_belt_res
        except rospy.ServiceException as err_service:
            print "Service call failed: %s"%err_service

    def __del__(self):    #destructor
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
    

    def update_data(self,data):
        # print "PRINTING IN UPDATE DATA"
        global pkg1
        global pkg2
        global pkg3
        del pkg1[:]
        del pkg2[:]
        del pkg3[:]
        try:
            if data.models[0].type == 'packagen1':
                pkg1.append(data.models[0].pose.position.x)
                pkg1.append(data.models[0].pose.position.y)
                pkg1.append(data.models[0].pose.position.z)
            elif data.models[1].type == 'packagen1':
                pkg1.append(data.models[1].pose.position.x)
                pkg1.append(data.models[1].pose.position.y)
                pkg1.append(data.models[1].pose.position.z)
            elif data.models[2].type == 'packagen1':
                pkg1.append(data.models[2].pose.position.x)
                pkg1.append(data.models[2].pose.position.y)
                pkg1.append(data.models[2].pose.position.z)
            elif data.models[3].type == 'packagen1':
                pkg1.append(data.models[3].pose.position.x)
                pkg1.append(data.models[3].pose.position.y)
                pkg1.append(data.models[3].pose.position.z)
        except:
            '''
            packagen1 is not yet reached to the logical camera
            '''
        try:
            if data.models[0].type == 'packagen2':
                pkg2.append(data.models[0].pose.position.x)
                pkg2.append(data.models[0].pose.position.y)
                pkg2.append(data.models[0].pose.position.z)
            elif data.models[1].type == 'packagen2':
                pkg2.append(data.models[1].pose.position.x)
                pkg2.append(data.models[1].pose.position.y)
                pkg2.append(data.models[1].pose.position.z)
            elif data.models[2].type == 'packagen2':
                pkg2.append(data.models[2].pose.position.x)
                pkg2.append(data.models[2].pose.position.y)
                pkg2.append(data.models[2].pose.position.z)
            elif data.models[3].type == 'packagen2':
                pkg2.append(data.models[3].pose.position.x)
                pkg2.append(data.models[3].pose.position.y)
                pkg2.append(data.models[3].pose.position.z)
        except:
            '''
            packagen1 is not yet reached to the logical camera
            '''  
        try:  
            if data.models[0].type == 'packagen3':
                pkg3.append(data.models[0].pose.position.x)
                pkg3.append(data.models[0].pose.position.y)
                pkg3.append(data.models[0].pose.position.z)
            elif data.models[1].type == 'packagen3':
                pkg3.append(data.models[1].pose.position.x)
                pkg3.append(data.models[1].pose.position.y)
                pkg3.append(data.models[1].pose.position.z)
            elif data.models[2].type == 'packagen3':
                pkg3.append(data.models[2].pose.position.x)
                pkg3.append(data.models[2].pose.position.y)
                pkg3.append(data.models[2].pose.position.z)
            elif data.models[3].type == 'packagen3':
                pkg3.append(data.models[3].pose.position.x)
                pkg3.append(data.models[3].pose.position.y)
                pkg3.append(data.models[3].pose.position.z)
        except:
            '''
            packagen1 is not yet reached to the logical camera
            '''

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


def main():
    global pkg1
    global pkg2
    global pkg3
    ur5 = CartesianPath()

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    # Teams may use this info in Tasks

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1.008 + vacuum_gripper_width + (box_length/2)
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    handle_sub_cam = rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,ur5.update_data)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        ini_to_bin = [math.radians(172),
                      math.radians(-40),
                      math.radians(58),
                      math.radians(-108),
                      math.radians(-89),
                      math.radians(-100)]
        green_to_bin = [math.radians(-1),
                      math.radians(-52),
                      math.radians(70),
                      math.radians(-107),
                      math.radians(-91),
                      math.radians(88)]

        red_to_bin = [math.radians(60),
                      math.radians(-52),
                      math.radians(70),
                      math.radians(-107),
                      math.radians(-91),
                      math.radians(-100)]

        blue_to_bin = [math.radians(100),
                      math.radians(-125),
                      math.radians(-79),
                      math.radians(-65),
                      math.radians(87),
                      math.radians(11)]


        
        # ur5.go_to_pose(ur5_2_home_pose)
        ur5.set_joint_angles(ini_to_bin)
        rospy.sleep(1)
        while len(pkg1)==0:
            ur5.conveyor_belt_power(50)
        while pkg1[1] > 0.0101213055:
            ur5.conveyor_belt_power(45)
        ur5.conveyor_belt_power(0)
        ur5.add_box(-0.800369,-0.011243,0.995000)
        ur5.attach_box()
        ur5.vacuum_on_off(True)
        ur5.set_joint_angles(red_to_bin)
        ur5.vacuum_on_off(False)
        rospy.sleep(1)
        ur5.detach_box()
        ur5.remove_box()
        # ur5.go_to_pose(ur5_2_home_pose)
        ur5.set_joint_angles(ini_to_bin)
        rospy.sleep(1)

        


        while len(pkg2)==0:
            ur5.conveyor_belt_power(50)
        while pkg2[1] > 0.081213055:
            ur5.conveyor_belt_power(45)
        ur5.conveyor_belt_power(0)
        ur5.ee_cartesian_translation(0.15, 0, 0)
        ur5.add_box(-0.650369,-0.011243,0.995000)
        ur5.attach_box()
        ur5.vacuum_on_off(True)
        ur5.conveyor_belt_power(11)
        ur5.set_joint_angles(green_to_bin)
        ur5.vacuum_on_off(False)
        rospy.sleep(1)
        ur5.detach_box()
        ur5.remove_box()
        # ur5.conveyor_belt_power(0)
        ur5.go_to_pose(ur5_2_home_pose)

        rospy.sleep(1)

      


        while len(pkg3)==0:
            ur5.conveyor_belt_power(50)
        while pkg3[1] > 0.081213055:
            ur5.conveyor_belt_power(45)
        ur5.conveyor_belt_power(0)
        ur5.ee_cartesian_translation(-0.15, 0, 0)
        ur5.add_box(-0.950369,-0.011243,0.995000)
        ur5.attach_box()
        ur5.vacuum_on_off(True)
        ur5.conveyor_belt_power(0)
        ur5.set_joint_angles(blue_to_bin)
        ur5.vacuum_on_off(False)
        rospy.sleep(1)
        ur5.detach_box()
        ur5.remove_box()
        ur5.go_to_pose(ur5_2_home_pose)
        break
        # print "Packagen3",pkg3
       



            




        # print "Press any key to stop the belt"
        # inp=raw_input()
        # ur5.conveyor_belt_power(0)
    


        # ur5.add_box()
        # break
        # ur5.remove_box()
        # y = -0.5
        # x = -0.5
        # z = -0.5
        # rospy.loginfo('\033[94m' + "Translating EE to the package from current position." + '\033[0m')
        # ur5.ee_cartesian_translation(x, y, z)
        # ur5.vacuum_on_off(True)
        # rospy.loginfo('\033[94m' + "Translating current position." + '\033[0m')
        # ur5.ee_cartesian_translation(0.5, 0.5, 0.5)
        # inp = raw_input()
        # break
        
    del ur5


if __name__ == '__main__':
    main()

    #rostopic echo /eyrc/vb/logical_camera_2
