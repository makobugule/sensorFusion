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
        self.Lock = 'CONTINUE'
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
        poses, coordinates= self.pose_estimator.infer(image)

        #print(poses[0])
        #r = poses[0][4]
        #l = poses[0][7]
        r = coordinates[4] # right wrist coordinates
        l = coordinates[7] # left wrist coordinates


        """
        print("l_wrist_x = {}, l_wrist_y = {}, r_wrist_x = {}, r_wrist_y  = {}"
              .format(poses[0][4][0], poses[0][4][1],
                      poses[0][7][0],poses[0][7][1]))
        """

        #r = coordinates[4] # right wrist coordinates
        #l = coordinates[7] # left wrist coordinates

        image = image.opencv()
        width = image.shape
        green_color = (0, 255, 0)
        black_color = (0, 0, 0)
        white_color= (255, 255, 255)
        red_color = (0, 0, 255)
        thickness = 5
        fontScale = 1.5
        font = cv2.FONT_HERSHEY_DUPLEX


        right_box = [550 ,750]
        yLimit= [500 ,700]
        left_box = [950,1150]

        rup_box = [550 ,750] # [1000 ,1400]
        yupLimit = [0 ,400]
        lup_box =  [950,1150] # [1520 ,1920]

        #image = cv2.rectangle(image, (0, 0), (800, 800), (0, 0, 0), -1)  #black boxes
        #image = cv2.rectangle(image, (1100,0), (1920,800), (0, 0, 0), -1)


        self.toc = time.perf_counter()
 



        if self.Lock == 'RIGHT':

                cv2.putText(image, 'UNLOCKED', (lup_box[0], yupLimit[1] + 50), font, fontScale, white_color, thickness, cv2.LINE_AA)
                cv2.putText(image, 'LOCKED', (rup_box[0], yupLimit[1]+ 50), font, fontScale, green_color, thickness, cv2.LINE_AA)
                image = cv2.rectangle(image, (right_box[0], yLimit[0]), (right_box[1], yLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (left_box[0],yLimit[0]), (left_box[1],yLimit[1]), green_color, 6)
                image = cv2.rectangle(image, (rup_box[0], yupLimit[0]), (rup_box[1], yupLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (lup_box[0],yupLimit[0]), (lup_box[1],yupLimit[1]), white_color, 6)
        elif self.Lock == 'LEFT':
                cv2.putText(image, 'UNLOCKED', (lup_box[0], yupLimit[1] + 50), font, fontScale, green_color, thickness, cv2.LINE_AA)
                cv2.putText(image, 'LOCKED', (rup_box[0], yupLimit[1]+ 50), font, fontScale, white_color, thickness, cv2.LINE_AA)
                image = cv2.rectangle(image, (right_box[0], yLimit[0]), (right_box[1], yLimit[1]), green_color, 6)
                image = cv2.rectangle(image, (left_box[0],yLimit[0]), (left_box[1],yLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (rup_box[0], yupLimit[0]), (rup_box[1], yupLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (lup_box[0],yupLimit[0]), (lup_box[1],yupLimit[1]), white_color, 6)

        elif self.Lock == 'STOP':

                cv2.putText(image, 'UNLOCKED', (lup_box[0], yupLimit[1] + 50), font, fontScale, green_color, thickness, cv2.LINE_AA)
                cv2.putText(image, 'LOCKED', (rup_box[0], yupLimit[1]+ 50), font, fontScale, white_color, thickness, cv2.LINE_AA)
                image = cv2.rectangle(image, (right_box[0], yLimit[0]), (right_box[1], yLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (left_box[0],yLimit[0]), (left_box[1],yLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (rup_box[0], yupLimit[0]), (rup_box[1], yupLimit[1]), green_color, 6)
                image = cv2.rectangle(image, (lup_box[0],yupLimit[0]), (lup_box[1],yupLimit[1]), white_color, 6)


        elif self.Lock == 'CONTINUE':

                cv2.putText(image, 'UNLOCKED', (lup_box[0], yupLimit[1] + 50), font, fontScale, green_color, thickness, cv2.LINE_AA)
                cv2.putText(image, 'LOCKED', (rup_box[0], yupLimit[1]+ 50), font, fontScale, white_color, thickness, cv2.LINE_AA)
                image = cv2.rectangle(image, (right_box[0], yLimit[0]), (right_box[1], yLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (left_box[0],yLimit[0]), (left_box[1],yLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (rup_box[0], yupLimit[0]), (rup_box[1], yupLimit[1]), white_color, 6)
                image = cv2.rectangle(image, (lup_box[0],yupLimit[0]), (lup_box[1],yupLimit[1]), green_color, 6)




        if r[0] != -1 and l[0] != -1:
        
            cv2.circle(image,(r[0],r[1]), 15, red_color, -1)
            cv2.putText(image, 'handRIGHT', (r[0],r[1]), font, fontScale, green_color, thickness, cv2.LINE_AA)
            cv2.circle(image,(l[0],l[1]), 15, red_color, -1)
            cv2.putText(image, 'handLEFT', (l[0],l[1]), font, fontScale, green_color, thickness, cv2.LINE_AA) 
            #print(r)

     
            if r[0] > right_box[0] and r[0] < right_box[1]:
                if r[1] > yLimit[0] and r[1] < yLimit[1]:
                    self.Lock = False

            if l[0] > left_box[0] and l[0] < left_box[1]:
                if l[1] > yLimit[0] and l[1] < yLimit[1]:  
                    self.Lock = True      





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
        cv2.putText(image,  str(average_fps), (10, 890), font, fontScale, (255, 255, 255), thickness, cv2.LINE_AA)

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


