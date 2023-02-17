import sys
import copy
import rospy
from cobot_msgs.msg import *
from cobot_msgs.srv import *
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class Babole():

    def __init__(self) :
        
        self.robot_publisher = rospy.Publisher('robot_commands', String, queue_size=10)
        self.cartesian_action_service_2D = rospy.ServiceProxy('/take_2D_cartesian_action', Take2DCartesianAction)
        self.cartesian_action_service_1D = rospy.ServiceProxy('/take_1D_cartesian_action', Take1DCartesianAction)
        self.rod_pose = []
        self.rocker_pose = []


    def listen(self):
        """
        Start the node and begin processing input data
        """
        rospy.init_node('sensor_fusion', anonymous=True)
        rospy.Subscriber('opendr_visual_commands',self.visual_callback)
        rospy.Subscriber('opendr__voice_commands',self.voice_callback)
        rospy.loginfo("Sensor fusion node started!")
        rospy.spin()

    def visual_callback(self,data):

        if data == True:
            self.rod_pose = [0,0,0]
            #rocker_pose = [1,1,1]
        else:
            self.rod_pose = [2,2,2]

        

    def voice_callback(self,msg):

        if msg == 'GRASP':
            self.cartesian_action_service_2D(self.rod_pose)




