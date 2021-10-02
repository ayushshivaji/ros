#! /usr/bin/env python


from __future__ import print_function

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse


from hrwros_gazebo.msg import LogicalCameraImage
pkg1 = []
pkg2 = []
pkg3 = []

class task3:

    def __init__(self):

        rospy.init_node('node_task3', anonymous=True)

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


        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')


        self.box_name = "package"


    def set_joint_angles(self, arg_list_joint_angles):

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

        if flag_plan == True:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

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


    def update_data(self, data):
        global pkg1   #to store details of packagen1
        global pkg2   #to store details of packagen2
        global pkg3   #to store details of packagen3
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
            packagen2 is not yet reached to the logical camera
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
            packagen3 is not yet reached to the logical camera
            '''


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
        # # This to keep EE parallel to Ground Plane
        # wpose.orientation.x = -0.5
        # wpose.orientation.y = -0.5
        # wpose.orientation.z = 0.5
        # wpose.orientation.w = 0.5


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
        grasping_group = "ur5_1_planning_group"
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, self.box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)



    def detach_box(self, timeout=4):
        self._scene.remove_attached_object(self._eef_link, name=self.box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)



    def remove_box(self, timeout=4):
        self._scene.remove_world_object(self.box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)



    def vacuum_on_off(self, val_tf):
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def conveyor_belt_control(self, pwr):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            handle_conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            final_res = handle_conveyor(pwr)
            return final_res
        except rospy.ServiceException as e:
            print("service call failed %s"%e)


    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')



def main():

    t3 = task3()

    global pkg1
    global pkg2
    global pkg3
    handle_sub_cam = rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage, t3.update_data)
    rospy.sleep(1)

    try:
        intial_joint_angles = [math.radians(172),
                               math.radians(-40),
                               math.radians(58),
                               math.radians(-108),
                               math.radians(-89),
                               math.radians(-100)]

        green_box_joint_angles = [math.radians(169),
                                  math.radians(-58),
                                  math.radians(90),
                                  math.radians(-121),
                                  math.radians(-87),
                                  math.radians(-100)]

        blue_box_joint_angles = [math.radians(174),
                                 math.radians(-24),
                                 math.radians(27),
                                 math.radians(-93),
                                 math.radians(-87),
                                 math.radians(-100)]

        red_bin_joint_angles = [math.radians(75),
                                math.radians(-60),
                                math.radians(70),
                                math.radians(-107),
                                math.radians(-91),
                                math.radians(-100)]

        green_bin_joint_angles = [math.radians(-1),
                                  math.radians(-60),
                                  math.radians(70),
                                  math.radians(-107),
                                  math.radians(-91),
                                  math.radians(-100)]

        blue_bin_joint_angles = [math.radians(100),
                                 math.radians(-125),
                                 math.radians(-79),
                                 math.radians(-65),
                                 math.radians(87),
                                 math.radians(11)]

        rospy.sleep(4)
        t3.set_joint_angles(intial_joint_angles)

        #pkg1,pkg2,pkg3 is a list containing x,y,z coordinates of packagen1,packagen2,and packagen3 respectively as seen in logical camera.
        #till package1 is not seen by logical camera
        while len(pkg1) == 0:
            t3.conveyor_belt_control(100)
            #when packagen1 is in visible range of logical camera
        while pkg1[1] > 0.080080:
            t3.conveyor_belt_control(80)
        t3.conveyor_belt_control(0)
        t3.add_box(-0.800369, -0.001243, 0.995000)
        t3.vacuum_on_off(True)
        t3.attach_box()
        t3.set_joint_angles(red_bin_joint_angles)
        t3.detach_box()
        t3.vacuum_on_off(False)
        t3.remove_box()
        #packagen1 DONE


        t3.conveyor_belt_control(50)
        t3.set_joint_angles(intial_joint_angles)
        #till packagen2 is not seen by logical camera
        while len(pkg2) == 0:
            t3.conveyor_belt_control(100)
        #when packagen1 is in visible range of logical camera
        while pkg2[1] > 0.080080:
            t3.conveyor_belt_control(80)
        t3.conveyor_belt_control(0)
        t3.set_joint_angles(green_box_joint_angles)
        t3.add_box(-0.660569, 0.008008, 0.995000)
        t3.vacuum_on_off(True)
        t3.attach_box()
        t3.set_joint_angles(green_bin_joint_angles)
        t3.detach_box()
        t3.vacuum_on_off(False)
        t3.remove_box()
        #packagen2 DONE


        t3.conveyor_belt_control(50)
        t3.set_joint_angles(intial_joint_angles)
        #till packagen3 is not seen by logical camera
        while len(pkg3) == 0:
            t3.conveyor_belt_control(100)
        #when packagen1 is in visible range of logical camera
        while pkg3[1] > 0.07880:
            t3.conveyor_belt_control(80)
        t3.conveyor_belt_control(0)
        t3.set_joint_angles(blue_box__joint_angles)
        t3.add_box(-0.900779, -0.022788, 0.995000)
        t3.vacuum_on_off(True)
        t3.attach_box()
        t3.set_joint_angles(blue_bin_joint_angles)
        t3.detach_box()
        t3.vacuum_on_off(False)
        t3.remove_box()
        t3.set_joint_angles(intial_joint_angles)
        #packagen3 DONE

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
