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
            self.rod_pose = [0,0,0]
            #rocker_pose = [1,1,1]
        else:
            self.rod_pose = [2,2,2]

        print(self.rod_pose)

    """

    def voice_callback(self,msg):

        if msg == 'GRASP':
            self.cartesian_action_service_2D(self.rod_pose)
    """

if __name__ == '__main__':
# Select the device for running the
    opendr_fusion_node = Babole()
    opendr_fusion_node.listen()
