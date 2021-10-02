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
import math

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse


from hrwros_gazebo.msg import LogicalCameraImage

from std_srvs.srv import Empty
from pkg_task4.msg import colorMsg


color_map =dict()

class Camera1:

  def __init__(self):
    global color_map
    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
    self._robot_ns = '/'  + "ur5_1"
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
    self.box_name = 'package'
    self._group.set_planning_time(10)

    self._computed_plan = ''

    rospy.loginfo(
      '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
    rospy.loginfo(
      '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
    rospy.loginfo(
      '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


    rp = rospkg.RosPack()
    self._pkg_path = rp.get_path('pkg_task4')
    self._file_path = self._pkg_path + '/config/saved_trajectories/'
    rospy.loginfo( "Package Path: {}".format(self._file_path) )


    rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')



  def clear_octomap(self):
    clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
    return clear_octomap_service_proxy()

  # def go_to_pose(self, arg_pose):

  #   pose_values = self._group.get_current_pose().pose
  #   rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
  #   rospy.loginfo(pose_values)

  #   self._group.set_pose_target(arg_pose)
  #   flag_plan = self._group.go(wait=True)  # wait=False for Async Move

  #   pose_values = self._group.get_current_pose().pose
  #   rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
  #   rospy.loginfo(pose_values)

  #   list_joint_values = self._group.get_current_joint_values()
  #   rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
  #   rospy.loginfo(list_joint_values)

  #   if (flag_plan == True):
  #       rospy.loginfo(
  #           '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
  #   else:
  #       rospy.logerr(
  #           '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

  #   return flag_plan

  def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        # state= self._robot.get_current_state()
        # self._group.set_start_state(state)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        self._group.stop()
        self._group.clear_pose_targets()

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

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
      # state= self._robot.get_current_state()
      # self._group.set_start_state(state)
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
          # rospy.loginfo(
          #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
      else:
          pass
          # rospy.logerr(
          #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

      return flag_plan

  def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
    number_attempts = 0
    flag_success = False

    while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
      number_attempts += 1
      flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
      rospy.logwarn("attempts: {}".format(number_attempts) )
      # # self.clear_octomap()
    
    return True


  def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
    file_path = arg_file_path + arg_file_name
    
    with open(file_path, 'r') as file_open:
      loaded_plan = yaml.load(file_open)
    ret = self._group.execute(loaded_plan)
    # rospy.logerr(ret)
    return ret
  
  def hard_go_to_pose(self, arg_pose, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

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

  def conveyor_belt_control(self, pwr):
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            handle_conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            final_res = handle_conveyor(pwr)
            return final_res
        except rospy.ServiceException as e:
            print("service call failed %s"%e)

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
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)



  def get_package_name(self,left,top):

    c0_min=75
    c0_max=175

    c1_min=275
    c1_max=375

    c2_min=475
    c2_max=575

    r0_min=275
    r0_max=375

    r1_min=450
    r1_max=550

    r2_min=600
    r2_max=700

    r3_min=750
    r3_max=850

    suffix=None
    
    if c0_min < left <c0_max:
      if r0_min<top<r0_max:
        suffix ="00"
      elif r1_min<top<r1_max:
        suffix ="10"
      elif r2_min<top<r2_max:
        suffix ="20"
      elif r3_min<top<r3_max:
        suffix ="30"
    elif c1_min < left <c1_max:
      if r0_min<top<r0_max:
        suffix ="01"
      elif r1_min<top<r1_max:
        suffix ="11"
      elif r2_min<top<r2_max:
        suffix ="21"
      elif r3_min<top<r3_max:
        suffix ="31"
    elif c2_min < left <c2_max:
      if r0_min<top<r0_max:
        suffix ="02"
      elif r1_min<top<r1_max:
        suffix ="12"
      elif r2_min<top<r2_max:
        suffix ="22"
      elif r3_min<top<r3_max:
        suffix ="32"
    package_name = "packagen"+ suffix

    return package_name


  def get_qr_data(self, arg_image):
    color_list= dict()
    img2=arg_image
    alpha=6
    beta=2
    arg_image= cv2.convertScaleAbs(img2, alpha=alpha, beta=beta)

    qr_result = decode(arg_image)
    for barcode in qr_result:
      (x,y,w,h)=barcode.rect
      cv2.rectangle(img2,(x,y),(x+w,y+h),(255,255,255),3)
      barcodeData = barcode.data.encode("ascii")
      color_list[self.get_package_name(x,y)] = barcodeData
      # color_map[self.get_package_name(x,y)] = barcodeData
      cv2.putText(img2,barcodeData,(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,01,(255,255,255),2)

    resized_image = cv2.resize(img2, (720/2, 1280/2)) 
    # print(color_map)
    # cv2.imshow("/eyrc/vb/camera_1/image_raw",resized_image)
    # color_map = color_list
    return color_list

  def ur5_1_vacuum_on_off(self, val_tf):   #To activate/deactivate vacuum gripper by calling rosservice
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as err_service:
            print ("Service call failed: %s" %err_service)

  def ur5_2_vacuum_on_off(self, val_tf):   #To activate/deactivate vacuum gripper by calling rosservice
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            handle_vacuum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            final_res = handle_vacuum(val_tf)
            return final_res.result
        except rospy.ServiceException as err_service:
            print ("Service call failed: %s"%err_service)


  def add_box(self, x,y,z,box_nam,timeout=4):    #To add the box to the planning scene in RViz
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "/world"
        box_pose.pose.position.x = x #0.26
        box_pose.pose.position.y = y #0.42
        box_pose.pose.position.z = z #1.89617872908
        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0

        self._scene.add_box(box_nam, box_pose, size=(0.15, 0.15, 0.15))
        return self.wait_for_state_update(box_nam, box_is_known=True, timeout=timeout)


  def attach_box(self, box_nam ,timeout=5):   #to attach the package to the end effector in planning scene
        grasping_group = 'manipulator'
        touch_links = self._robot.get_link_names(group=grasping_group)
        self._scene.attach_box(self._eef_link, box_nam, touch_links=touch_links)
        return self.wait_for_state_update( box_nam,box_is_attached=True, box_is_known=False, timeout=timeout)



  def detach_box(self, box_nam ,timeout=5):  #To detach the package from the end effector
      self._scene.remove_attached_object(self._eef_link, name=box_nam)
      return self.wait_for_state_update(box_nam,box_is_known=True, box_is_attached=False,timeout=timeout)



  def remove_box(self, box_nam,timeout=5):    #to remove package from the planning scene
      self._scene.remove_world_object(box_nam)
      return self.wait_for_state_update(box_nam, box_is_attached=False, box_is_known=False, timeout=timeout)

  def callback(self,data):
    global color_map
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    (rows,cols,channels) = cv_image.shape
    image = cv_image
    color_map = self.get_qr_data(image)
    cv2.waitKey(3)



def main(args):
  
  global color_map
  var_handle_pub = rospy.Publisher('my_topic', colorMsg, queue_size=10)
 
  rospy.init_node('node_eg1_read_camera', anonymous=True)
  var_loop_rate = rospy.Rate(1)
  obj_color= colorMsg()
  
  ic = Camera1()

  ur5_pose_initial = geometry_msgs.msg.Pose()
  ur5_pose_initial.position.x = -0.795027565398
  ur5_pose_initial.position.y = 0.00145411382599
  ur5_pose_initial.position.z = 1.21865400088
  ur5_pose_initial.orientation.x = 0.0256327213055
  ur5_pose_initial.orientation.y = -0.7026892846
  ur5_pose_initial.orientation.z = 0.711034668081
  ur5_pose_initial.orientation.w = 0.00065855447333

  ur5_pose_01 = geometry_msgs.msg.Pose()
  ur5_pose_01.position.x = 0.00692127667095
  ur5_pose_01.position.y = -0.206297118897
  ur5_pose_01.position.z = 1.93126164427
  ur5_pose_01.orientation.x = -0.642681429017
  ur5_pose_01.orientation.y = 0.00567497548158
  ur5_pose_01.orientation.z = 0.766084064208
  ur5_pose_01.orientation.w = 0.00660166767116

  ur5_pose_00 = geometry_msgs.msg.Pose()
  ur5_pose_00.position.x = 0.213568427436
  ur5_pose_00.position.y = -0.203999210204
  ur5_pose_00.position.z = 1.89617872908
  ur5_pose_00.orientation.x = 0.999848241993
  ur5_pose_00.orientation.y = 0.0174206432703
  ur5_pose_00.orientation.z = 0.000114674467333
  ur5_pose_00.orientation.w = 3.19626008815e-05

  ur5_pose_02 = geometry_msgs.msg.Pose()
  ur5_pose_02.position.x = -0.288474876792
  ur5_pose_02.position.y = -0.20705310277
  ur5_pose_02.position.z = 1.88204753505 
  ur5_pose_02.orientation.x = -0.903499028653
  ur5_pose_02.orientation.y = -0.0287406680258
  ur5_pose_02.orientation.z = 0.425784448074
  ur5_pose_02.orientation.w = 0.0396368894202


  ur5_pose_12 = geometry_msgs.msg.Pose()
  ur5_pose_12.position.x = -0.317580249554
  ur5_pose_12.position.y = -0.205523791424
  ur5_pose_12.position.z = 1.66940022497
  ur5_pose_12.orientation.x = -0.840895431103
  ur5_pose_12.orientation.y = -0.0335670684005
  ur5_pose_12.orientation.z = 0.539017791292
  ur5_pose_12.orientation.w = 0.0350420681508

  ur5_pose_11 = geometry_msgs.msg.Pose()
  ur5_pose_11.position.x = -0.0171672590917
  ur5_pose_11.position.y = -0.214312486122
  ur5_pose_11.position.z = 1.66104688749
  ur5_pose_11.orientation.x = -0.499572171718
  ur5_pose_11.orientation.y = -0.018543522129
  ur5_pose_11.orientation.z = 0.866058013482
  ur5_pose_11.orientation.w = 0.00522497040592
  


  ur5_pose_10 = geometry_msgs.msg.Pose()
  ur5_pose_10.position.x = 0.226948216174
  ur5_pose_10.position.y = -0.20762705217
  ur5_pose_10.position.z = 1.6582607292
  ur5_pose_10.orientation.x = 0.909905301931
  ur5_pose_10.orientation.y = 0.0875979502209
  ur5_pose_10.orientation.z = -0.405097430296
  ur5_pose_10.orientation.w = 0.0171759309075



  ur5_pose_20 = geometry_msgs.msg.Pose()
  ur5_pose_20.position.x = 0.292560822421
  ur5_pose_20.position.y = -0.205637417758
  ur5_pose_20.position.z = 1.42385192256
  ur5_pose_20.orientation.x = -0.499475489222
  ur5_pose_20.orientation.y = -0.0134301447194
  ur5_pose_20.orientation.z = 0.866192937145
  ur5_pose_20.orientation.w = 0.00732547058173


  ur5_pose_21 = geometry_msgs.msg.Pose()
  ur5_pose_21.position.x = -0.0139276176655
  ur5_pose_21.position.y = -0.197395555153
  ur5_pose_21.position.z = 1.42415487132
  ur5_pose_21.orientation.x = -0.499512857657
  ur5_pose_21.orientation.y = -0.0134601864809
  ur5_pose_21.orientation.z = 0.866171423442
  ur5_pose_21.orientation.w = 0.0072659223556



  ur5_pose_22 = geometry_msgs.msg.Pose()
  ur5_pose_22.position.x = -0.268
  ur5_pose_22.position.y = -0.211
  ur5_pose_22.position.z = 1.441
  ur5_pose_22.orientation.x = -0.499467963867
  ur5_pose_22.orientation.y = -0.0132218111111
  ur5_pose_22.orientation.z = 0.866201338085
  ur5_pose_22.orientation.w = 0.00722348124199

  ur5_pose_32 = geometry_msgs.msg.Pose()
  ur5_pose_32.position.x = -0.288072558761
  ur5_pose_32.position.y = -0.214558975946
  ur5_pose_32.position.z = 1.18480905922
  ur5_pose_32.orientation.x = -0.499440885603
  ur5_pose_32.orientation.y = -0.0128390430599
  ur5_pose_32.orientation.z = 0.866225066125
  ur5_pose_32.orientation.w = 0.00693509754717


  ur5_pose_31 = geometry_msgs.msg.Pose()
  ur5_pose_31.position.x = -0.0142338387769
  ur5_pose_31.position.y = -0.201259234614
  ur5_pose_31.position.z = 1.20403993831
  ur5_pose_31.orientation.x = -0.499487795434
  ur5_pose_31.orientation.y = -0.012817136629
  ur5_pose_31.orientation.z = 0.866197733883
  ur5_pose_31.orientation.w = 0.00701063742099


  ur5_pose_30 = geometry_msgs.msg.Pose()
  ur5_pose_30.position.x = 0.281357246892
  ur5_pose_30.position.y = -0.205758084245
  ur5_pose_30.position.z = 1.2038397181
  ur5_pose_30.orientation.x = -0.866411594748
  ur5_pose_30.orientation.y = 0.0328583301602
  ur5_pose_30.orientation.z = -0.496738914898
  ur5_pose_30.orientation.w = 0.0387521489898
  
  

  try:
    image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,ic.callback)
    intial_joint_angles = [math.radians(7.7848814851),
                               math.radians(-138.015566396),
                               math.radians(-58.0424957695),
                               math.radians(-71.7379508013),
                               math.radians(90.3676987576),
                               math.radians(-84.2097855739)]


    go_to_package00 = [math.radians(158.01402839),  
                         math.radians(-102.632117494),  
                         math.radians(-14.9648235344),  
                         math.radians(-62.2556466022),  
                         math.radians(25.0184750591),   
                         math.radians(90.864031701)]   


    go_to_package01 = [math.radians(123.827291163),  #117
                         math.radians(-104.818924694),  #-104.5
                         math.radians(5.04754302775),  #4.23
                         math.radians(-80.1796413986),  #-79.74
                         math.radians(55.1538749567),   #61.84
                         math.radians(79.9189223976)]   #80


    go_to_package02 = [math.radians(53.5746838252),  
                         math.radians(-114.616318597),  
                         math.radians(2.20319311999),  
                         math.radians(-74.4851548351),  
                         math.radians(127.245235993),   
                         math.radians(125.346973192)]  
  

    go_to_package10 = [math.radians(-63.2385702353),  
                         math.radians(-102.62792675),  
                         math.radians(86.3701885443),  
                         math.radians(-161.36517947),  
                         math.radians(-106.758499889),   
                         math.radians(132.886652558)]  


    go_to_package11 = [math.radians(-125.089044245),  
                         math.radians(-115.639843953),  
                         math.radians(94.0728520083),  
                         math.radians(-155.800762457),  
                         math.radians(-54.3961891714),   
                         math.radians(58.4326689919)] 

    go_to_package12 = [math.radians(49.6757118557),  
                         math.radians(-113.246425966),  
                         math.radians(-8.49648636915),  
                         math.radians(114.499530114),  
                         math.radians(-131.142326344),   
                         math.radians(-70.0076469175)]


    go_to_package20 = [math.radians(-52.8748147843),  
                         math.radians(-95.9407620208),  
                         math.radians(87.6507779595),  
                         math.radians(10.4865113179),  
                         math.radians(127.063340713),   
                         math.radians(-118.736767412)]  

    # go_to_package21 = [math.radians(-127.511357569),  
    #                      math.radians(-119.315684623),  
    #                      math.radians(103.181275214),  
    #                      math.radians(18.344370367),  
    #                      math.radians(52.4599435851),   
    #                      math.radians(-121.402947086)]



    go_to_package21_1 = [math.radians(-119.983335628),  
                         math.radians(-118.609216933),  
                         math.radians(130.357805145),  
                         math.radians(165.718837837),  
                         math.radians(-60.0100449578),   
                         math.radians(1.24827507969)]

    go_to_package21 = [math.radians(119.454241311),  
                         math.radians(-60.6608504759),  
                         math.radians(-103.190086373),  
                         math.radians(161.782135765),  
                         math.radians(-60.6016603576),   
                         math.radians(-118.986869596)]  


    # go_to_package22 = [math.radians(61.3231746994),  
    #                      math.radians(-75.4481787887),  
    #                      math.radians(-117.971830892),  
    #                      math.radians(8.88601828579),  
    #                      math.radians(115.076554063),   
    #                      math.radians(0.164470890745)]


    go_to_package22 = [math.radians(54.8491069738),  
                         math.radians(-84.9217576757),  
                         math.radians(-115.025645681),  
                         math.radians(18.5894796906),  
                         math.radians(123.985973313),   
                         math.radians(179.236980703)]

# ur5_shoulder_pan_joint: 54.8491069738
# ur5_shoulder_lift_joint: -84.9217576757
# ur5_elbow_joint: -115.025645681
# ur5_wrist_1_joint: 18.5894796906
# ur5_wrist_2_joint: 123.985973313
# ur5_wrist_3_joint: 179.236980703



# go_to_package30 = [math.radians(),  
#                          math.radians(),  
#                          math.radians(),  
#                          math.radians(),  
#                          math.radians(),   
#                          math.radians()]


    go_to_package30_1 = [math.radians(-57.2871594674),  
                         math.radians(-97.5399620727),  
                         math.radians(106.906360596),  
                         math.radians(-99.504221891),  
                         math.radians(-90.7821683281),   
                         math.radians(40.3563745024)]
 
    go_to_package30 = [math.radians(-54.4254235325),  
                         math.radians(-73.7440112084),  
                         math.radians(133.035760733),  
                         math.radians(127.838844276),  
                         math.radians(-126.425506987),   
                         math.radians(-116.151250323)]

    # go_to_package30 = [math.radians(-57.6176435523),  
    #                      math.radians(-74.8402900322),  
    #                      math.radians(135.955258867),  
    #                      math.radians(125.738215221),  
    #                      math.radians(-123.249940991),   
    #                      math.radians(-116.628871147)]

    go_to_package31 = [math.radians(-126.79610012),  
                         math.radians(-118.385367098),  
                         math.radians(135.347418811),  
                         math.radians(-14.8712187506),  
                         math.radians(53.184443983),   
                         math.radians(-121.313428754)]


    go_to_package32 = [math.radians(-161.011343607),  
                         math.radians(-89.3488482791),  
                         math.radians(117.498248004),  
                         math.radians(-23.0123347486),  
                         math.radians(19.0131435052),   
                         math.radians(-124.924442024)]



    pkg00='packagen00'
    pkg01='packagen01'
    pkg02='packagen02'
    pkg10='packagen10'
    pkg11='packagen11'
    pkg12='packagen12'
    pkg20='packagen20'
    pkg21='packagen21'
    pkg22='packagen22'
    pkg30='packagen30'
    pkg31='packagen31'
    pkg32='packagen32'

    # ic.add_box(0.26,-0.42,1.90,pkg00)
    # ic.add_box(0.0,-0.42,1.90,pkg01)
    # ic.add_box(-0.26,-0.42,1.90,pkg02)
    # ic.add_box(0.26,-0.42,1.64,pkg10)
    # ic.add_box(0.0,-0.42,1.64,pkg11)
    # ic.add_box(-0.26,-0.42,1.64,pkg12)
    # ic.add_box(0.26,-0.42,1.41,pkg20)
    # ic.add_box(0,-0.42,1.44,pkg21)
    ic.add_box(-0.28,-0.41,1.43,pkg22)
    ic.add_box(0.28,-0.42,1.19,pkg30)
    ic.add_box(0,-0.42,1.18,pkg31)
    ic.add_box(-0.26,-0.42,1.18,pkg32)

    # ic.remove_box(pkg00)
    # ic.remove_box(pkg01)
    # ic.remove_box(pkg02)
    # ic.remove_box(pkg10)
    # ic.remove_box(pkg11)
    # ic.remove_box(pkg12)
    # ic.remove_box(pkg20)
    # ic.remove_box(pkg21)

    # ic.detach_box(pkg22)
    # ic.remove_box(pkg22)
    ic.ur5_1_vacuum_on_off(False)
    ic.hard_set_joint_angles(intial_joint_angles,10)
    rospy.sleep(1)
    rospy.sleep(2)
    # # print(color_map)
    # obj_color.box_color = color_map['packagen00']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen00'])
    image_sub.unregister()

   
    # ic.hard_set_joint_angles(go_to_package00,10)
    
    # file_name = 'go_to_package00.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.attach_box(pkg00)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)
    
    # file_name = 'package00_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg00)
    # rospy.sleep(1)
    # ic.remove_box(pkg00)
    # var_handle_pub.publish(obj_color)
   



    # obj_color.box_color = color_map['packagen01']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen01'])
    # ic.hard_set_joint_angles(go_to_package01,10)

    # file_name = 'go_to_package01.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.conveyor_belt_control(100)
    # rospy.sleep(1)
    # ic.attach_box(pkg01)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)

    # file_name = 'package01_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg01)
    # rospy.sleep(1)
    # ic.remove_box(pkg01)
    # var_handle_pub.publish(obj_color)


    # obj_color.box_color = color_map['packagen02']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen02'])
    # ic.hard_set_joint_angles(go_to_package02,10)

    # file_name = 'go_to_package02.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )  

    # rospy.sleep(1)
    # ic.conveyor_belt_control(100)
    # rospy.sleep(1)
    # ic.attach_box(pkg02)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)

    # file_name = 'package02_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg02)
    # rospy.sleep(1)
    # ic.remove_box(pkg02)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)

    
    # obj_color.box_color = color_map['packagen10']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen10'])
    # ic.hard_set_joint_angles(go_to_package10,10)

    # file_name = 'go_to_package10.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )  

    # rospy.sleep(1)
    # ic.conveyor_belt_control(100)
    # rospy.sleep(1)
    # ic.attach_box(pkg10)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)

    # file_name = 'package10_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg10)
    # rospy.sleep(1)
    # ic.remove_box(pkg10)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)

    
    # obj_color.box_color = color_map['packagen11']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen11'])
    # ic.hard_set_joint_angles(go_to_package11,10)

    # file_name = 'go_to_package11.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.conveyor_belt_control(100)
    # rospy.sleep(1)
    # ic.attach_box(pkg11)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)

    # file_name = 'package11_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg11)
    # rospy.sleep(1)
    # ic.remove_box(pkg11)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)



    # obj_color.box_color = color_map['packagen12']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen12'])
    # ic.hard_set_joint_angles(go_to_package12,10)

    # file_name = 'go_to_package12.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.conveyor_belt_control(100)
    # rospy.sleep(1)
    # ic.attach_box(pkg12)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)

    # file_name = 'package12_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg12)
    # rospy.sleep(1)
    # ic.remove_box(pkg12)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)
    

    # obj_color.box_color = color_map['packagen20']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen20'])
    # ic.hard_set_joint_angles(go_to_package20,10)

    # file_name = 'go_to_package20.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.conveyor_belt_control(100)
    # rospy.sleep(1)
    # ic.attach_box(pkg20)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)

    # file_name = 'package20_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg20)
    # rospy.sleep(1)
    # ic.remove_box(pkg20)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)


    # # obj_color.box_color = color_map['packagen21']
    # # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen21'])
    # ic.hard_set_joint_angles(go_to_package21_1,10)
    # rospy.sleep(1)
    # # ic.ee_cartesian_translation(0,-0.005,0)
    # # # file_name = 'go_to_package21.yaml'
    # # file_path = ic._file_path + file_name
    # # with open(file_path, 'w') as file_save:
    # #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # # rospy.loginfo( "File saved at: {}".format(file_path) )

    # # rospy.sleep(1)
    # # ic.conveyor_belt_control(100)
    # # rospy.sleep(1)
    # ic.attach_box(pkg21)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # # ic.conveyor_belt_control(0)
    # # rospy.sleep(1)
   
    # ic.ee_cartesian_translation(0,0.37,0)
    # ic.hard_set_joint_angles(intial_joint_angles,10)
    # # file_name = 'package21_to_belt.yaml'
    # # file_path = ic._file_path + file_name
    # # with open(file_path, 'w') as file_save:
    # #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # # rospy.loginfo( "File saved at: {}".format(file_path) )

    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg21)
    # rospy.sleep(1)
    # ic.remove_box(pkg21)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)



    # obj_color.box_color = color_map['packagen22']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen22'])
    ic.hard_set_joint_angles(go_to_package22,15)
    # ic.hard_go_to_pose(ur5_pose_22,12)

    file_name = 'go_to_package22.yaml'
    file_path = ic._file_path + file_name
    with open(file_path, 'w') as file_save:
      yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    rospy.sleep(1)
    ic.conveyor_belt_control(100)
    rospy.sleep(1)
    ic.attach_box(pkg22)
    rospy.sleep(1)
    ic.ur5_1_vacuum_on_off(True)
    rospy.sleep(1)
    ic.conveyor_belt_control(0)
    rospy.sleep(1)
    ic.hard_set_joint_angles(intial_joint_angles,15)
    # ic.hard_go_to_pose(ur5_pose_initial,12)

    file_name = 'package22_to_belt.yaml'
    file_path = ic._file_path + file_name
    with open(file_path, 'w') as file_save:
      yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    rospy.loginfo( "File saved at: {}".format(file_path) )

    rospy.sleep(1)
    ic.ur5_1_vacuum_on_off(False)
    rospy.sleep(1)
    ic.detach_box(pkg22)
    rospy.sleep(1)
    ic.remove_box(pkg22)
    rospy.sleep(1)
    # # var_handle_pub.publish(obj_color)


    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # # obj_color.box_color = color_map['packagen30']
    # # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen30'])
    # ic.hard_set_joint_angles(go_to_package30_1,10)
    # # ic.hard_go_to_pose(ur5_pose_30,5)
    # # # file_name = 'go_to_package30.yaml'
    # # # file_path = ic._file_path + file_name
    # # # with open(file_path, 'w') as file_save:
    # # #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # # # rospy.loginfo( "File saved at: {}".format(file_path) )
    # # # rospy.sleep(1)
    # # # ic.conveyor_belt_control(100)
    # # # rospy.sleep(1)
    # # # ic.attach_box(pkg30)
    # # # rospy.sleep(1)
   
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # # ic.conveyor_belt_control(0)
    # # rospy.sleep(1)
    # # ic.hard_set_joint_angles(intial_joint_angles,10)
    # # ic.hard_go_to_pose(ur5_pose_initial,5)
    # # file_name = 'package30_to_belt.yaml'
    # # file_path = ic._file_path + file_name
    # # with open(file_path, 'w') as file_save:
    # #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # # rospy.loginfo( "File saved at: {}".format(file_path) )
    # # ic.hard_set_joint_angles(go_to_package30_1,5)
    # ic.ee_cartesian_translation(0,0.3,0)
    # # ic.ur5_1_vacuum_on_off(False)
    # # ic.ee_cartesian_translation(0,-0.01,0)
    # # ic.ur5_1_vacuum_on_off(True)
    # # ic.ee_cartesian_translation(0,0.15,0)
    # # ic.ur5_1_vacuum_on_off(False)
    # # ic.ee_cartesian_translation(0,-0.15,0)
    # # ic.ur5_1_vacuum_on_off(True)
    # # ic.ee_cartesian_translation(0,0.2,0)
    # ic.hard_set_joint_angles(intial_joint_angles,10)




    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # # ic.detach_box(pkg30)
    # # rospy.sleep(1)
    # ic.remove_box(pkg30)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)
    

    # obj_color.box_color = color_map['packagen31']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen31'])
    # ic.hard_set_joint_angles(go_to_package31,10)
    # # ic.hard_go_to_pose(ur5_pose_31,10)
    # file_name = 'go_to_package31.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )
    # rospy.sleep(1)
    # ic.conveyor_belt_control(100)
    # rospy.sleep(1)
    # ic.attach_box(pkg31)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)
    # # ic.hard_go_to_pose(ur5_pose_initial,10)
    # file_name = 'package31_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg31)
    # rospy.sleep(1)
    # ic.remove_box(pkg31)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)


    # obj_color.box_color = color_map['packagen32']
    # print("GOING TO PICK PACKAGE HAVING COLOR :", color_map['packagen32'])
    # ic.hard_set_joint_angles(go_to_package32,10)
    # file_name = 'go_to_package32.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)    
    # rospy.loginfo( "File saved at: {}".format(file_path) )
    # rospy.sleep(1)
    # ic.conveyor_belt_control(100)
    # rospy.sleep(1)
    # ic.attach_box(pkg32)
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(True)
    # rospy.sleep(1)
    # ic.conveyor_belt_control(0)
    # rospy.sleep(1)
    # ic.hard_set_joint_angles(intial_joint_angles,10)
    # file_name = 'package32_to_belt.yaml'
    # file_path = ic._file_path + file_name
    # with open(file_path, 'w') as file_save:
    #   yaml.dump(ic._computed_plan, file_save, default_flow_style=True)
    # rospy.loginfo( "File saved at: {}".format(file_path) )
    # rospy.sleep(1)
    # ic.ur5_1_vacuum_on_off(False)
    # rospy.sleep(1)
    # ic.detach_box(pkg32)
    # rospy.sleep(1)
    # ic.remove_box(pkg32)
    # rospy.sleep(1)
    # var_handle_pub.publish(obj_color)

    
    # rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



#     [Decoded(data='yellow', type='QRCODE', rect=Rect(left=129, top=643, width=89, height=90), polygon=[Point(x=129, y=644), Point(x=129, y=732), Point(x=218, y=733), Point(x=218, y=643)]),
#      Decoded(data='green', type='QRCODE', rect=Rect(left=128, top=496, width=90, height=91), polygon=[Point(x=128, y=496), Point(x=128, y=587), Point(x=218, y=585), Point(x=218, y=496)]), 
#      Decoded(data='red', type='QRCODE', rect=Rect(left=128, top=315, width=90, height=91), polygon=[Point(x=128, y=315), Point(x=128, y=406), Point(x=218, y=406), Point(x=218, y=315)]), 
#      Decoded(data='red', type='QRCODE', rect=Rect(left=130, top=796, width=90, height=90), polygon=[Point(x=130, y=796), Point(x=130, y=886), Point(x=220, y=886), Point(x=220, y=796)]), 


#      Decoded(data='yellow', type='QRCODE', rect=Rect(left=316, top=496, width=90, height=91), polygon=[Point(x=316, y=497), Point(x=316, y=585), Point(x=406, y=587), Point(x=405, y=496)]), 
#      Decoded(data='yellow', type='QRCODE', rect=Rect(left=318, top=797, width=89, height=89), polygon=[Point(x=318, y=797), Point(x=318, y=886), Point(x=407, y=886), Point(x=406, y=797)]), 
#      Decoded(data='green', type='QRCODE', rect=Rect(left=315, top=315, width=90, height=91), polygon=[Point(x=315, y=315), Point(x=315, y=406), Point(x=405, y=406), Point(x=405, y=315)]), 
#      Decoded(data='red', type='QRCODE', rect=Rect(left=315, top=643, width=90, height=90), polygon=[Point(x=315, y=643), Point(x=315, y=733), Point(x=405, y=733), Point(x=405, y=643)]), 
    

#      Decoded(data='yellow', type='QRCODE', rect=Rect(left=503, top=316, width=89, height=90), polygon=[Point(x=503, y=316), Point(x=503, y=405), Point(x=592, y=406), Point(x=592, y=316)]), 
#      Decoded(data='green', type='QRCODE', rect=Rect(left=501, top=796, width=91, height=91), polygon=[Point(x=501, y=796), Point(x=501, y=887), Point(x=590, y=887), Point(x=592, y=796)]), 
#      Decoded(data='green', type='QRCODE', rect=Rect(left=502, top=643, width=91, height=91), polygon=[Point(x=502, y=643), Point(x=502, y=734), Point(x=593, y=734), Point(x=593, y=643)]), 
#      Decoded(data='red', type='QRCODE', rect=Rect(left=502, top=495, width=90, height=91), polygon=[Point(x=502, y=495), Point(x=502, y=586), Point(x=591, y=586), Point(x=592, y=495)])]
# ['yellow', 'yellow', 'yellow', 'yellow', 'green', 'green', 'red', 'green', 'red', 'green', 'red', 'red']


#! /usr/bin/env python

# import rospy
# import sys
# import copy

# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# import actionlib


# class CartesianPath:

#     # Constructor
#     def __init__(self):

#         rospy.init_node('node_eg5_waypoints', anonymous=True)

#         self._planning_group = "ur5_1_planning_group"
#         self._commander = moveit_commander.roscpp_initialize(sys.argv)
#         self._robot = moveit_commander.RobotCommander()
#         self._scene = moveit_commander.PlanningSceneInterface()
#         self._group = moveit_commander.MoveGroupCommander(self._planning_group)
#         self._display_trajectory_publisher = rospy.Publisher(
#             '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#         self._exectute_trajectory_client = actionlib.SimpleActionClient(
#             'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
#         self._exectute_trajectory_client.wait_for_server()

#         self._planning_frame = self._group.get_planning_frame()
#         self._eef_link = self._group.get_end_effector_link()
#         self._group_names = self._robot.get_group_names()

#         rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


#     def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
#         # 1. Create a empty list to hold waypoints
#         waypoints = []

#         # 2. Add Current Pose to the list of waypoints
#         waypoints.append(self._group.get_current_pose().pose)

#         # 3. Create a New waypoint
#         wpose = geometry_msgs.msg.Pose()
#         wpose.position.x = waypoints[0].position.x + (trans_x)  
#         wpose.position.y = waypoints[0].position.y + (trans_y)  
#         wpose.position.z = waypoints[0].position.z + (trans_z)
#         # This to keep EE parallel to Ground Plane
#         wpose.orientation.x = -0.5
#         wpose.orientation.y = -0.5
#         wpose.orientation.z = 0.5
#         wpose.orientation.w = 0.5


#         # 4. Add the new waypoint to the list of waypoints
#         waypoints.append(copy.deepcopy(wpose))


#         # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
#         (plan, fraction) = self._group.compute_cartesian_path(
#             waypoints,   # waypoints to follow
#             0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
#             0.0)         # Jump Threshold
#         rospy.loginfo("Path computed successfully. Moving the arm.")

#         # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
#         # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
#         num_pts = len(plan.joint_trajectory.points)
#         if (num_pts >= 3):
#             del plan.joint_trajectory.points[0]
#             del plan.joint_trajectory.points[1]

#         # 6. Make the arm follow the Computed Cartesian Path
#         self._group.execute(plan)

    
#     def go_to_pose(self, arg_pose):

#         pose_values = self._group.get_current_pose().pose
#         rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
#         rospy.loginfo(pose_values)

#         self._group.set_pose_target(arg_pose)
#         flag_plan = self._group.go(wait=True)  # wait=False for Async Move

#         pose_values = self._group.get_current_pose().pose
#         rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
#         rospy.loginfo(pose_values)

#         list_joint_values = self._group.get_current_joint_values()
#         rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
#         rospy.loginfo(list_joint_values)

#         if (flag_plan == True):
#             rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
#         else:
#             rospy.logerr(
#                 '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

#         return flag_plan


#     # Destructor
#     def __del__(self):
#         moveit_commander.roscpp_shutdown()
#         rospy.loginfo(
#             '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


# def main():
#     ur5 = CartesianPath()

#     box_length = 0.15               # Length of the Package
#     vacuum_gripper_width = 0.115    # Vacuum Gripper Width
#     delta = vacuum_gripper_width + (box_length/2)  # 0.19
#     # Teams may use this info in Tasks

#     ur5_2_home_pose = geometry_msgs.msg.Pose()
#     ur5_2_home_pose.position.x = -0.8
#     ur5_2_home_pose.position.y = 0
#     ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
#     # This to keep EE parallel to Ground Plane
#     ur5_2_home_pose.orientation.x = -0.5
#     ur5_2_home_pose.orientation.y = -0.5
#     ur5_2_home_pose.orientation.z = 0.5
#     ur5_2_home_pose.orientation.w = 0.5

#     while not rospy.is_shutdown():
#         # 1. Go to Home Position
#         ur5.go_to_pose(ur5_2_home_pose)
#         rospy.loginfo('\033[96m' + "Enter 'n' to go to next pose." + '\033[0m')
#         inp = raw_input()

#         # 2. Translate EE by 0.5m  in x
#         rospy.loginfo('\033[94m' + "Translating EE by 0.5m in x from current position." + '\033[0m')
#         ur5.ee_cartesian_translation(0.5, 0, 0)

#         rospy.loginfo('\033[96m' + "Enter 'n' to go to next pose." + '\033[0m')
#         inp = raw_input()
        
#         # 3. Translate EE by 0.5m  in y
#         rospy.loginfo('\033[94m' + "Translating EE by 0.5m in y from current position." + '\033[0m')
#         ur5.ee_cartesian_translation(0, 0.5, 0)

#         rospy.loginfo('\033[96m' + "Enter 'n' to go to next pose." + '\033[0m')
#         inp = raw_input()
        
#         # 4. Translate EE by 0.5m  in z
#         rospy.loginfo('\033[94m' + "Translating EE by 0.5m in z from current position." + '\033[0m')
#         ur5.ee_cartesian_translation(0, 0, 0.5)

#         rospy.loginfo('\033[96m' + "Enter 'n' to go to home pose." + '\033[0m')
#         inp = raw_input()
        
#     del ur5


# if __name__ == '__main__':
#     main()
