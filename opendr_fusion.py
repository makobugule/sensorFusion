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


class Babole:

    def __init__(self, input_image__command_topic='/opendr/directions', input_voice__command_topic='/text_commands',input_object__command_topic='/opendr/grasp_detected'):

        # self.robot_publisher = rospy.Publisher('robot_commands', String, queue_size=10) #if we dont use 
        self.input_image__command_topic = input_image__command_topic
        self.input_voice__command_topic = input_voice__command_topic
        self.input_object__command_topic = input_object__command_topic

        self.pick_and_place_client = PickAndPlaceClient()
        self.pick_and_place_client.start()
        self.move_joint_space = rospy.ServiceProxy("/opendr/set_joint_state", SetJointState)
        self.move_cartesian_space_2D = rospy.ServiceProxy('/opendr/set_pose_target_2D', SetPoseTarget2D)
        self.engine_pos = Pose()
        self.rod_pose_right = PickGoal()
        self.rod_pose_left = PickGoal()
        self.hand_over = PickGoal()
        self.bolt_pose = PickGoal()
        self.rod_hole_0 = Pose()
        self.rod_hole_1 = Pose()
        self.bolt_hole_0 = Pose()
        self.move_gripper = rospy.ServiceProxy('/opendr/move_gripper', MoveGripper)

        self.id_List = {}
        self.ctbolt = 0
        self.ctrod = 0
        self.side = 'Right'
        self.placement = 0

        filename = "data_poses"
        with open(str(Path.home() / filename)+'.yml', "r") as stream:
            self.poses = yaml.safe_load(stream)
        # print(self.poses)
        self.rod_pose_left.pose.position.x = self.poses[0][0]
        self.rod_pose_left.pose.position.y = self.poses[0][1]
        self.rod_pose_left.pose.position.z = self.poses[0][2]
        self.rod_pose_left.pose.orientation.x = self.poses[0][3]
        self.rod_pose_left.pose.orientation.y = self.poses[0][4]
        self.rod_pose_left.pose.orientation.z = self.poses[0][5]
        self.rod_pose_left.pose.orientation.w = self.poses[0][6]
        self.rod_pose_left.force = 20.0
        self.rod_pose_left.width = 0.008

        self.hand_over.pose.position.x = 0.7335193772726805
        self.hand_over.pose.position.y = -0.16851981119747114
        self.hand_over.pose.position.z = 0.25836742430589843
        self.hand_over.pose.orientation.x = 0.9997962098991813
        self.hand_over.pose.orientation.y = 0.016963925381664604
        self.hand_over.pose.orientation.z = 0.010720903250329676
        self.hand_over.pose.orientation.w = 0.0021968478269389297

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

        self.bolt_hole_0.position.x = self.poses[5][0]
        self.bolt_hole_0.position.y = self.poses[5][1]
        self.bolt_hole_0.position.z = self.poses[5][2]
        self.bolt_hole_0.orientation.x = self.poses[5][3]
        self.bolt_hole_0.orientation.y = self.poses[5][4]
        self.bolt_hole_0.orientation.z = self.poses[5][5]
        self.bolt_hole_0.orientation.w = self.poses[5][6]

    def listen(self):
        """
        Start the node and begin processing input data
        """
        rospy.Subscriber(self.input_object__command_topic, ObjectHypothesisWithPose, self.object_callback)
        print("Object detection has started")

        rospy.Subscriber(self.input_image__command_topic, Bool, self.visual_callback)
        print("Pose detection has started.")

        rospy.Subscriber(self.input_voice__command_topic, String, self.voice_callback)
        print("Speech recognition has started.")

        rospy.loginfo("Sensor fusion node started!")
        rospy.spin()

    def object_callback(self, data):
        if data.id in self.id_List.keys():
            self.id_List[data.id] = [data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.pose.pose.orientation.x,
                                        data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
        else:

            self.id_List[data.id] = [data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,data.pose.pose.orientation.x,
                                        data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w] 
        if data.id == 6:
            if self.ctbolt < 1:
                print("Rod is detected")
                self.ctbolt += 1
            y_modified = self.id_List[6][1] + 0.070
            self.rod_pose = PickGoal()
            self.rod_pose.pose.position.x = self.id_List[6][0]
            self.rod_pose.pose.position.y = y_modified
            self.rod_pose.pose.position.z = self.id_List[6][2]
            self.rod_pose.pose.orientation.x = self.id_List[6][3]
            self.rod_pose.pose.orientation.y = self.id_List[6][4]
            self.rod_pose.pose.orientation.z = self.id_List[6][5]
            self.rod_pose.pose.orientation.w = self.id_List[6][6]
            self.rod_pose.force = 20.0
            self.rod_pose.width = 0.008

        if data.id == 5:
            if self.ctrod < 1:
                print("Bolt is detected")
                self.ctrod += 1
            y_modified = self.id_List[5][1] + 0.070
            self.bolt_pose = PickGoal()
            self.bolt_pose.pose.position.x = self.id_List[5][0]
            self.bolt_pose.pose.position.y = y_modified
            self.bolt_pose.pose.position.z = self.id_List[5][2]
            self.bolt_pose.pose.orientation.x = self.id_List[5][3]
            self.bolt_pose.pose.orientation.y = self.id_List[5][4]
            self.bolt_pose.pose.orientation.z = self.id_List[5][5]
            self.bolt_pose.pose.orientation.w = self.id_List[5][6]
            self.bolt_pose.force = 20.0
            self.bolt_pose.width = 0.01
        if data.id == 7:
            self.rocker_pose = PickGoal()
            self.rocker_pose.pose.position.x = self.id_List[7][0]
            self.rocker_pose.pose.position.y = y_modified
            self.rocker_pose.pose.position.z = self.id_List[7][2]
            self.rocker_pose.pose.orientation.x = self.id_List[7][3]
            self.rocker_pose.pose.orientation.y = self.id_List[7][4]
            self.rocker_pose.pose.orientation.z = self.id_List[7][5]
            self.rocker_pose.pose.orientation.w = self.id_List[7][6]
            self.rocker_pose.force = 20.0  # change those
            self.rocker_pose.width = 0.01
        # print(self.id_List)

    def visual_callback(self, vis):

        if vis.data == 'RIGHT' or vis.data == 'LEFT':
            self.side = vis.data

    def voice_callback(self, msg):
        msg = msg.data.split(" ")
        print(msg)
        if msg[0] == 'PICK':
            if msg[1] == 'BOLT':
                print("picking the bolt")
                self.pick_and_place_client.pick(self.bolt_pose)
                self.placement = 2

            elif msg[1] == 'ROD':
                if self.side == 'RIGHT':
                    print("picking rod on the right")
                    self.pick_and_place_client.pick(self.rod_pose_right)
                    self.placement = 0

                elif self.side == 'LEFT':
                    print("picking rod on the left")
                    self.pick_and_place_client.pick(self.rod_pose_left)
                    self.placement = 1
                else:
                    print("Which rod?")
        elif msg[0] == 'GIVE' and msg[1] == 'ROCKER':
            self.pick_and_place_client.pick(self.rocker_pose)
            self.pick_and_place_client.place(self.hand_over)

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
                self.pick_and_place_client.place(self.bolt_hole_0)
        elif msg[0] == 'MOVE' and msg[1] == 'HOME':
            joint_pose = [-2.8298893005650064e-05, -0.7850717113980075, 0.0002637194674237419, -2.3563226722248816, 0.0012791172673718796, 1.569120704009106, 0.7847318894341841]

            # joint_pose = [0.00, -0.25*math.pi, 0.00, -0.75 * math.pi, 0.00, 0.50 * math.pi, -0.673]
            self.move_joint_space(joint_pose)
            self.move_gripper(0.04)

            # self.move_cartesian_space_2D([0.38, 0.00], False)
            self.move_joint_space()

    def stop_pick_and_place_client(self):
        self.pick_and_place_client.stop()


if __name__ == '__main__':
    rospy.init_node('sensor_fusion', anonymous=True)
    opendr_fusion_node = Babole()
    opendr_fusion_node.listen()

    time.sleep(5)
    rospy.spin()
