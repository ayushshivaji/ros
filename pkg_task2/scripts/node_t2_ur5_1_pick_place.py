#!/usr/bin/env python

import sys
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse


class PickPlace():

    def __init__(self):   #constructor
        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        self.box_name = 'package1'

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5 init done." + '\033[0m')


    #To get the state update of package
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


    def add_box(self, timeout=4):    #To add the box to the planning scene in RViz
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/world"
        box_pose.pose.position.x = 0.010000
        box_pose.pose.position.y = 0.454034
        box_pose.pose.position.z = 1.905428
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


    def vacuum_on_off(self, val_tf):   #To activate/deactivate vacuum gripper by calling rosservice
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as err_service:
            print "Service call failed: %s"%err_service

    def __del__(self):    #destructor
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')



def main():
    try:
        all_zeroes = [math.radians(0),
                      math.radians(0),
                      math.radians(0),
                      math.radians(0),
                      math.radians(0),
                      math.radians(0)]


        all_straight = [math.radians(0),
                        math.radians(-90),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0),
                        math.radians(0)]


        go_to_package = [math.radians(60),
                         math.radians(-80),
                         math.radians(14),
                         math.radians(-120),
                         math.radians(-61),
                         math.radians(80)]


        go_to_bin = [math.radians(25),
                     math.radians(-150),
                     math.radians(-50),
                     math.radians(-170),
                     math.radians(0),
                     math.radians(90)]
        
        



        myobj = PickPlace()
        rospy.sleep(2)

        #adding package to the planning scene
        myobj.add_box()

        #setting joint angles for EE to get the package from the shelf
        myobj.set_joint_angles(go_to_package)
        rospy.sleep(2)

       # attaching package to the EE
        myobj.attach_box()

        #Activating vacuum gripper
        myobj.vacuum_on_off(True)

        #setting joint angles for EE to go to the bin
        myobj.set_joint_angles(go_to_bin)
        rospy.sleep(2)

        #Deactivating vacuum gripper
        myobj.vacuum_on_off(False)

        #detaching package from the EE
        myobj.detach_box()

        #removing package from the planning scene
        myobj.remove_box()

	    
        myobj.set_joint_angles(all_straight)
        rospy.sleep(1)
        
        #setting all joint angles to zero
        myobj.set_joint_angles(all_zeroes)
        rospy.sleep(2)

        rospy.loginfo('\033[94m' + "DONE" + '\033[0m')

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()