#!/usr/bin/env python
# Copyright 2020-2022 OpenDR European Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
import torch
import numpy as np
import cv2 
import time
from std_msgs.msg import String
from std_msgs.msg import Bool

from vision_msgs.msg import ObjectHypothesis
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image as ROS_Image
from opendr_bridge import ROSBridge
from opendr.perception.pose_estimation import draw
from opendr.perception.pose_estimation import LightweightOpenPoseLearner
from opendr.engine.data import Image

import pandas as pd 

class WristDetectionNode:

    def __init__(self, input_image_topic="/camera/color/image_raw", output_image_topic="/opendr/image_pose_annotated",
                 pose_annotations_topic="/opendr/poses",
                 device="cuda", output_command_topic="/opendr/commands"):


        if output_image_topic is not None:
            self.image_publisher = rospy.Publisher(output_image_topic, ROS_Image, queue_size=10)
        else:
            self.image_publisher = None

        if pose_annotations_topic is not None:
            self.pose_publisher = rospy.Publisher(pose_annotations_topic, Detection2DArray, queue_size=10)
        else:
            self.pose_publisher = None

        if output_command_topic is not None: 
            self.command_publisher =  rospy.Publisher(output_command_topic,Bool, queue_size=10)
        else:
            self.command_publisher = None



        self.input_image_topic = input_image_topic
        self.bridge = ROSBridge()

        # Initialize the pose estimation
        self.pose_estimator = LightweightOpenPoseLearner(device=device, num_refinement_stages=0,
                                                         mobilenet_use_stride=False,
                                                         half_precision=False)
     
        self.pose_estimator.download(path=".", verbose=True)
        self.pose_estimator.load("openpose_default")
        self.cdArray_x = []
        self.cdArray_y= []

        self.counter = 0
        self.Lock = True
        self.prev_frame_time = 0

        self.fps_array = np.array([])
        self.counter = 0
        self.hadi = np.zeros((500,2))

    def listen(self):
        """
        Start the node and begin processing input data
        """
        rospy.init_node('wrist_detection', anonymous=True)
        rospy.Subscriber(self.input_image_topic, ROS_Image, self.callback)
        rospy.loginfo("wrist detection node started!")
        rospy.spin()



    def callback(self, data):


        # Convert sensor_msgs.msg.Image into OpenDR Image
        image = self.bridge.from_ros_image(data, encoding='bgr8')

        # Run pose estimation
        poses,coordinates = self.pose_estimator.infer(image)

        """
        POSE KEYPOINTS DEFINITIONS

        2:right shoulder
        3:right elbow
        4:right wrist

        5:left shoulder
        6:left elbow
        7:left wrist

        """

        r = coordinates[4] # right wrist coordinates
        l = coordinates[7] # left wrist coordinates

        image = image.opencv()
        width = image.shape
        green_color = (0, 255, 0)
        black_color = (0, 0, 0)
        white_color= (255, 255, 255)
        red_color = (0, 0, 255)
        thickness = 6
        fontScale = 2
        font = cv2.FONT_HERSHEY_DUPLEX

        image = cv2.rectangle(image, (0, 415), (400, 500), (0, 0, 0), -1)
        image = cv2.rectangle(image, (1520,415), (1920,500), (0, 0, 0), -1)

        # In case we dont like the coloring locks : 
        """
        image = cv2.rectangle(image, (0, 0), (400, 400), color, 6)
        image = cv2.rectangle(image, (1520,0), (1920,400), color, 6)

        cv2.putText(image, 'UNLOCKED', (15, 480), font, fontScale, (255, 255, 255), thickness, cv2.LINE_AA)
        cv2.putText(image, 'LOCKED', (1600, 480), font, fontScale, (255, 255, 255), thickness, cv2.LINE_AA)
        """
        
        self.toc = time.perf_counter()
        cv2.putText(image,  str(self.toc), (10, 890), font, fontScale, (255, 255, 255), thickness, cv2.LINE_AA)


        if self.Lock == True:
            cv2.putText(image, 'UNLOCKED', (15, 480), font, fontScale, white_color, thickness, cv2.LINE_AA)
            cv2.putText(image, 'LOCKED', (1600, 480), font, fontScale, green_color, thickness, cv2.LINE_AA)
            image = cv2.rectangle(image, (0, 0), (400, 400), white_color, 6)
            image = cv2.rectangle(image, (1520,0), (1920,400), green_color, 6)

        else:
            cv2.putText(image, 'LOCKED', (1600, 480), font, fontScale, white_color, thickness, cv2.LINE_AA)
            cv2.putText(image, 'UNLOCKED', (15, 480), font, fontScale, green_color, thickness, cv2.LINE_AA)
            image = cv2.rectangle(image, (0, 0), (400, 400), green_color, 6)
            image = cv2.rectangle(image, (1520,0), (1920,400), white_color, 6)


        if r[0] != -1:
            cv2.circle(image,(r[0],r[1]), 15, red_color, -1)
            cv2.putText(image, 'handRIGHT', (r[0],r[1]), font, fontScale, green_color, thickness, cv2.LINE_AA)
            #print(r)

     
            if r[0] < 415 and r[1] < 415:
                self.Lock = False
                self.hadi[self.counter][0] = 1 # right
                self.hadi[self.counter][1] = self.toc 

            if l[0] > 1520 and l[1] < 415:
                self.Lock = True      
                self.hadi[self.counter][0] = 0 #left
                self.hadi[self.counter][1] = self.toc 
        """
        if l[0] != -1:
            cv2.circle(image,(l[0],l[1]), 15, red_color, -1)
            cv2.putText(image, 'handLEFT', (l[0],l[1]), font, fontScale, green_color, thickness, cv2.LINE_AA) 
            
            if r[0] < 415 and r[1] < 415:
                self.Lock = False
                self.hadi[self.counter][0] = 1 # right
                self.hadi[self.counter][1] = self.toc 

            if l[0] > 1520 and l[1] < 415:
                self.Lock = True      
                self.hadi[self.counter][0] = 0 #left
                self.hadi[self.counter][1] = self.toc 
        """
               
        df = pd.DataFrame(self.hadi).T   
        df.to_excel(excel_writer = "/home/mako/Desktop/output.xlsx") 



        #FPS AND DELAY COUNT

        new_frame_time = time.time()        
        seconds = new_frame_time - self.prev_frame_time
        fps = 1/(new_frame_time-self.prev_frame_time)
        self.prev_frame_time = new_frame_time


        self.fps_array = np.append(self.fps_array,fps)
        average_fps = np.mean(self.fps_array)
        framde_delay = 1000/average_fps
        print("Current Fps: ",fps)
        print("Average Fps: ",average_fps)
        print("Frame delay(ms): ",framde_delay)
        print("----------------")


        if self.command_publisher is not None:

            #print(self.Lock)
            self.command_publisher.publish(self.Lock)

        if self.image_publisher is not None:
            message = self.bridge.to_ros_image(Image(image), encoding='bgr8')
            self.image_publisher.publish(message)

        

    def most_frequent(self,List):
        
        ct = 0
        num = List[0]
         
        for i in List:
            curr_frequency = List.count(i)
            if(curr_frequency> ct):
                ct = curr_frequency
                num = i
 
        return num      


if __name__ == '__main__':
    # Select the device for running the
    try:
        if torch.cuda.is_available():
            print("GPU found.")
            device = 'cuda'
        else:
            print("GPU not found. Using CPU instead.")
            device = 'cpu'
    except:
        device = 'cpu'

    pose_estimation_node = WristDetectionNode(device=device)
    pose_estimation_node.listen()

