#!/usr/bin/env python3
import rospy
import time
# from cobot_msgs.msg import *
# from cobot_msgs.srv import *
from opendr_control.srv import SetJointState, SetPoseTarget2D, MoveGripper
from geometry_msgs.msg import Pose
from opendr_control.msg import PickGoal
from opendr_control.pick_and_place_client import PickAndPlaceClient
from pathlib import Path
import yaml
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, VisionInfo
from std_msgs.msg import String, Bool
# from moveit_commander.conversions import pose_to_list
from opendr_control.detections import Detections



class Babole:

    def __init__(self, input_image__command_topic='/opendr/commands',human_pose_topic='/opendr/human_pose', input_voice__command_topic='/text_commands',input_object__command_topic='/opendr/grasp_detected', another ='/opendr/object_categories'):

        # self.robot_publisher = rospy.Publisher('robot_commands', String, queue_size=10) #if we dont use 
        self.input_image__command_topic = input_image__command_topic
        self.input_voice__command_topic = input_voice__command_topic
        self.input_object__command_topic = input_object__command_topic
        self.human_pose_topic = human_pose_topic
        self.another = another

        self.move_joint_space = rospy.ServiceProxy("/opendr/set_joint_state", SetJointState)
        self.move_cartesian_space_2D = rospy.ServiceProxy('/opendr/set_pose_target_2D', SetPoseTarget2D)
        self.engine_pos = Pose()
        self.rod_pose_right = PickGoal()
        self.rod_pose_left = PickGoal()
        self.rocker_pose = PickGoal()
        self.hand_over = Pose()
        self.bolt_pose = PickGoal()
        self.pushrod_pose = PickGoal()
        self.pushrod_pose_right = PickGoal()
        self.rocker_pose_right = PickGoal()
        self.rod_hole_0 = Pose()
        self.rod_hole_1 = Pose()
        self.rod_hole_2 = Pose()
        self.rod_hole_3 = Pose()
        self.bolt_hole_0 = Pose()
        self.move_gripper = rospy.ServiceProxy('/opendr/move_gripper', MoveGripper)

        self.id_List = {}
        self.ctbolt = 0
        self.rocker_ct = 0
        self.rod_ct = 0
        self.target = ''
        self.hole_count = 0

        self.ctrod = 0
        self.side = 'RIGHT'
        self.placement = 0

        filename = "data_poses"
        with open(str(Path.home() / filename)+'.yml', "r") as stream:
            self.poses = yaml.safe_load(stream)
        # print(self.poses)

        
        #--------------PRERECORDED POSSES------------------#,

        """
        self.rod_pose_left.pose.position.x = self.poses[0][0]
        self.rod_pose_left.pose.position.y = self.poses[0][1]
        self.rod_pose_left.pose.position.z = self.poses[0][2]
        self.rod_pose_left.pose.orientation.x = self.poses[0][3]
        self.rod_pose_left.pose.orientation.y = self.poses[0][4]
        self.rod_pose_left.pose.orientation.z = self.poses[0][5]
        self.rod_pose_left.pose.orientation.w = self.poses[0][6]
        self.rod_pose_left.force = 20.0
        self.rod_pose_left.width = 0.008

        self.rod_pose_right.pose.position.x = self.poses[1][0]
        self.rod_pose_right.pose.position.y = self.poses[1][1]
        self.rod_pose_right.pose.position.z = self.poses[1][2]
        self.rod_pose_right.pose.orientation.x = self.poses[1][3]
        self.rod_pose_right.pose.orientation.y = self.poses[1][4]
        self.rod_pose_right.pose.orientation.z = self.poses[1][5]
        self.rod_pose_right.pose.orientation.w = self.poses[1][6]
        self.rod_pose_right.force = 20.0
        self.rod_pose_right.width = 0.008

        self.bolt_pose.pose.position.x = self.poses[2][0]
        self.bolt_pose.pose.position.y = self.poses[2][1]
        self.bolt_pose.pose.position.z = self.poses[2][2]
        self.bolt_pose.pose.orientation.x = self.poses[2][3]
        self.bolt_pose.pose.orientation.y = self.poses[2][4]
        self.bolt_pose.pose.orientation.z = self.poses[2][5]
        self.bolt_pose.pose.orientation.w = self.poses[2][6]
        self.bolt_pose.force = 20.0
        self.bolt_pose.width = 0.01
        """

        self.hand_over.position.x = 0.7335193772726805
        self.hand_over.position.y = -0.16851981119747114
        self.hand_over.position.z = 0.25836742430589843
        self.hand_over.orientation.x = 0.9997962098991813
        self.hand_over.orientation.y = 0.016963925381664604
        self.hand_over.orientation.z = 0.010720903250329676
        self.hand_over.orientation.w = 0.0021968478269389297

        self.rod_hole_0.position.x = self.poses[3][0]
        self.rod_hole_0.position.y = self.poses[3][1]
        self.rod_hole_0.position.z = self.poses[3][2]
        self.rod_hole_0.orientation.x = self.poses[3][3]
        self.rod_hole_0.orientation.y = self.poses[3][4]
        self.rod_hole_0.orientation.z = self.poses[3][5]
        self.rod_hole_0.orientation.w = self.poses[3][6]


        self.rod_hole_1.position.x = self.poses[4][0]
        self.rod_hole_1.position.y = self.poses[4][1]
        self.rod_hole_1.position.z = self.poses[4][2]
        self.rod_hole_1.orientation.x = self.poses[4][3]
        self.rod_hole_1.orientation.y = self.poses[4][4]
        self.rod_hole_1.orientation.z = self.poses[4][5]
        self.rod_hole_1.orientation.w = self.poses[4][6]

        self.rod_hole_2.position.x = self.poses[5][0]
        self.rod_hole_2.position.y = self.poses[5][1]
        self.rod_hole_2.position.z = self.poses[5][2]
        self.rod_hole_2.orientation.x = self.poses[5][3]
        self.rod_hole_2.orientation.y = self.poses[5][4]
        self.rod_hole_2.orientation.z = self.poses[5][5]
        self.rod_hole_2.orientation.w = self.poses[5][6]


        self.rod_hole_3.position.x = self.poses[6][0]
        self.rod_hole_3.position.y = self.poses[6][1]
        self.rod_hole_3.position.z = self.poses[6][2]
        self.rod_hole_3.orientation.x = self.poses[6][3]
        self.rod_hole_3.orientation.y = self.poses[6][4]
        self.rod_hole_3.orientation.z = self.poses[6][5]
        self.rod_hole_3.orientation.w = self.poses[6][6]


        self.detections = Detections()

        self.sub1 = rospy.Subscriber(self.input_image__command_topic, String, self.visual_callback)
        print("Pose detection has started.")

        self.sub2 = rospy.Subscriber(self.input_voice__command_topic, String, self.voice_callback)
        print("Speech recognition has started.")

        self.sub3 = rospy.Subscriber(self.input_object__command_topic, ObjectHypothesisWithPose, self.detections.process_detection)
        self.sub4 = rospy.Subscriber(self.another, VisionInfo, self.detections.save_categories)
        self.sub5 = rospy.Subscriber(self.human_pose_topic, String, self.handover_callback)
        self.pick_and_place_client = PickAndPlaceClient()
        self.pick_and_place_client.start()

        rospy.loginfo("Sensor fusion node started!")
        time.sleep(5)

    def listen(self):
        """
        Start the node and begin processing input data
        """

        self.pushrod_id = self.detections.find_object_by_category("pushrod")
        self.pushrod_poses = self.detections.get_object_pose(self.pushrod_id)
        self.pushrod_pose.width = 0.008
        self.pushrod_pose.force = 20.0



        self.rocker_id = self.detections.find_object_by_category("rocker arm object")
        self.rocker_poses= self.detections.get_object_pose(self.rocker_id)

        """
        # -------------FOR POINTING SCENARIO----------------#

        self.rocker_pose.width = 0.015
        self.rocker_pose.force = 20.0
        self.rocker_pose_right.width = 0.015
        self.rocker_pose_right.force = 20.0

        self.rocker_pose_right.pose = self.rocker_poses[0]
        self.rocker_pose_right.pose.position.y = self.rocker_pose.pose.position.y + 0.07
        self.rocker_pose.pose = self.rocker_poses[1]
        self.rocker_pose.pose.position.y = self.rocker_pose.pose.position.y + 0.07
        self.pushrod_pose_right.width = 0.008
        self.pushrod_pose_right.force = 20.0

        """


    def object_callback(self, data):
        if data.id in self.id_List.keys():
            self.id_List[data.id] = [data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.pose.pose.orientation.x,
                                        data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        else:

            self.id_List[data.id] = [data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.pose.pose.orientation.x,
                                        data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]


    def visual_callback(self, vis):
        self.side = vis.data
        #print(self.side)

    def handover_callback(self,loc):
        if loc.data == "Left":
            self.hand_over.position.x = 0.6107444565186262
            self.hand_over.position.y = -0.32890051409813253
            self.hand_over.position.z = 0.3257611130107832
            self.hand_over.orientation.x = 0.9952321089953179
            self.hand_over.orientation.y = -0.09688094802682137
            self.hand_over.orientation.z = 0.002799389883800204
            self.hand_over.orientation.w = -0.010922204467702522



        elif loc.data == "Middle":
            self.hand_over.position.x = 0.6407055463467419
            self.hand_over.position.y = -0.09343268176791157
            self.hand_over.position.z = 0.34802447797333846
            self.hand_over.orientation.x = 0.9997995282015122
            self.hand_over.orientation.y = 0.012293520961086771
            self.hand_over.orientation.z = -0.015121398691397982
            self.hand_over.orientation.w = 0.004595220563669775




        elif loc.data == "Right":

            self.hand_over.position.x = 0.7108390356486403
            self.hand_over.position.y = 0.20088097919557663
            self.hand_over.position.z = 0.3839867653724264
            self.hand_over.orientation.x =0.9862947195974229
            self.hand_over.orientation.y = 0.16482374894039678
            self.hand_over.orientation.z = -0.0008416270180809999
            self.hand_over.orientation.w = 0.007426273859527397



    def voice_callback(self, msg):
        msg = msg.data.split(" ")
        print(msg)

        if msg[0] == 'ANOTHER':
            if msg[1] == 'ONE':
                if self.target == 'ROD':
                    if self.hole_count == 1:
                        self.pushrod_pose.pose = self.pushrod_poses[self.rod_ct]
                        self.pushrod_pose.pose.position.y = self.pushrod_pose.pose.position.y + 0.07
                        self.pick_and_place_client.pick_and_place(self.pushrod_pose, self.rod_hole_1)
                        self.rod_ct += 1
                        self.placement = self.rod_ct
                        self.hole_count += 1
                    if self.hole_count == 2:
                        self.pushrod_pose.pose = self.pushrod_poses[self.rod_ct]
                        self.pushrod_pose.pose.position.y = self.pushrod_pose.pose.position.y + 0.07
                        self.pick_and_place_client.pick_and_place(self.pushrod_pose, self.rod_hole_2 )
                        self.rod_ct += 1
                        self.placement = self.rod_ct
                        self.hole_count += 1

                    if self.hole_count == 3:
                        self.pushrod_pose.pose = self.pushrod_poses[self.rod_ct]
                        self.pushrod_pose.pose.position.y = self.pushrod_pose.pose.position.y + 0.07
                        self.pick_and_place_client.pick_and_place(self.pushrod_pose, self.rod_hole_3 )
                        self.rod_ct += 1
                        self.placement = self.rod_ct
                        self.hole_count += 1

        if msg[0] == 'PICK':
            if msg[1] == 'BOLT':
                print("picking bolt")
                self.pick_and_place_client.pick(self.bolt_pose)
                self.placement = 2

            elif msg[1] == 'ROD':

                print("picking rod")
                self.pushrod_pose.pose = self.pushrod_poses[self.rod_ct]
                self.pushrod_pose.pose.position.y = self.pushrod_pose.pose.position.y + 0.07
                self.pick_and_place_client.pick(self.pushrod_pose)
                self.rod_ct += 1
                self.placement = self.rod_ct
              

                """
                # -------------FOR POINTING SCENARIO----------------#

                if self.side == 'RIGHT':

                    self.pushrod_pose.pose = self.pushrod_poses[0]
                    self.pushrod_pose.pose.position.y = self.pushrod_pose.pose.position.y + 0.07
                    self.pick_and_place_client.pick(self.pushrod_pose)
                    self.rod_ct += 1
                    self.hole_count = 1
                    self.placement = self.rod_ct

                elif self.side == 'LEFT':

                    self.pushrod_pose.pose = self.pushrod_poses[1]
                    self.pushrod_pose.pose.position.y = self.pushrod_pose.pose.position.y + 0.07
                    self.pick_and_place_client.pick(self.pushrod_pose)
                    self.rod_ct += 1
                    self.hole_count = 1
                    self.placement = self.rod_ct
                else:
                    print("which  one")
                """
        elif msg[0] == 'GIVE':
            if msg[1] == 'ROCKER':

                print("handing over a rocker arm")
                self.rocker_pose.pose = self.rocker_poses[self.rocker_ct]
                self.rocker_pose.pose.position.y = self.rocker_pose.pose.position.y + 0.07

                self.pick_and_place_client.pick_and_give(self.rocker_pose, self.hand_over)
                self.rocker_ct += 1
                """
                # -------------FOR POINTING SCENARIO----------------#
                print(self.side)

                if self.side == 'RIGHT':
                    print("picking rod on the right")
                    self.pick_and_place_client.pick_and_give(self.rocker_pose_right, self.hand_over)
                    self.placement = 0

                elif self.side == 'LEFT':
                    print("picking rod on the left")
                    self.pick_and_place_client.pick_and_give(self.rocker_pose, self.hand_over)
                    self.placement = 1
                else:
                    print("Which rocker?")
                """
            else :

                print("handing over")
                self.pick_and_place_client.give(self.hand_over)

        elif msg[0] == 'PLACE':
            #place_pose = [0.41382180771283933, 0.3007492642198627, -0.47489533258320993,-2.590046687455205, 1.523898492696461, 1.51826741027832, 1.0674265426469474]

            if self.placement == 0:
                print("placing the rod0")
                #self.move_joint_space(place_pose)
                self.pick_and_place_client.place(self.rod_hole_0)

            elif self.placement == 1:
                #self.move_joint_space(place_pose)
                print("placing the rod1")
                self.pick_and_place_client.place(self.rod_hole_1)
            elif self.placement == 2:
                #self.move_joint_space(place_pose)
                print("placing the bolt")
                self.pick_and_place_client.place(self.rod_hole_2)

            elif self.placement == 3:
                #self.move_joint_space(place_pose)
                print("placing the bolt")
                self.pick_and_place_client.place(self.rod_hole_3)

        elif msg[0] == 'MOVE' and msg[1] == 'HOME':

            joint_pose = [-2.8298893005650064e-05, -0.7850717113980075, 0.0002637194674237419, -2.3563226722248816, 0.0012791172673718796, 1.569120704009106, 0.7847318894341841]
            self.move_joint_space(joint_pose)
            self.move_gripper(0.04)

    def stop_pick_and_place_client(self):
        self.pick_and_place_client.stop()


if __name__ == '__main__':
    rospy.init_node('sensor_fusion', anonymous=True)
    opendr_fusion_node = Babole()
    opendr_fusion_node.listen()

    rospy.spin()
