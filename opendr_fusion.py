#!/usr/bin/env python

import sys
import copy
import rospy
#from cobot_msgs.msg import *
#from cobot_msgs.srv import *
#import geometry_msgs.msg
from std_msgs.msg import String
#from moveit_commander.conversions import pose_to_list

class Babole:

    def __init__(self,input_image__command_topic='/opendr/commands'):
        
        self.robot_publisher = rospy.Publisher('robot_commands', String, queue_size=10) #if we dont use 
                                                                                         #this script to move the robot
        #self.cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)
        #self.cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
        self.rod_pose = []
        self.rocker_pose = []
        self.input_image__command_topic = input_image__command_topic

        self.engine_pos = Pose()
        self.rod_pose_right = Pose()
        self.rod_pose_left = Pose()
        self.bolt_pose= Pose()
        self.rod_hole_0 = Pose()
        self.rod_hole_1 = Pose()
        self.bolt_hole_0 =Pose()

        self.side = ' '
        self.placement = 0




    def listen(self):
        """
        Start the node and begin processing input data
        """
        rospy.init_node('sensor_fusion', anonymous=True)
        rospy.Subscriber(self.input_image__command_topic,String,self.visual_callback)
        #rospy.Subscriber('opendr__voice_commands',self.voice_callback)
        rospy.loginfo("Sensor fusion node started!")
        rospy.spin()

    def visual_callback(self,data):

        print(data)
        if data.data == True:
            self.side = 'Right'
        else:
            self.side = 'Left'
    

    def voice_callback(self,msg):

        if msg[0] == 'PICK':
            if msg[1] == 'BOLT':
                if self.side == 'Right':
                    self.pick_and_place_client.pick(self.bolt_pose)
                    self.placement= 0

            elif msg[1] == 'ROD':
                if self.side == 'Right':
                    self.pick_and_place_client.pick(self.rod_pose_right)
                    self.placement= 1
                else:
                    self.pick_and_place_client.pick(self.rod_pose_left)
                    self.placement= 2  
        elif msg[0] == 'PLACE':
            if self.placement == 0:
                self.pick_and_place_client.place(self.rod_hole_0)
            elif self.placement == 1:
                self.pick_and_place_client.place(self.rod_hole_1)
            elif self.placement == 2:
                self.pick_and_place_client.place(self.bolt_hole_0)

        elif msg[0] == 'GO HOME':
            # GO HOME COMMAND IN MOVEIT PICK_AND_PLACE.PY
    
if __name__ == '__main__':
# Select the device for running the
    opendr_fusion_node = Babole()
    opendr_fusion_node.listen()
