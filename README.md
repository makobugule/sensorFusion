# openposeWrist


<br />This branch includes speech recognition, gesture recogntion and sensor fusion scripts. 

<br />Speech recognition scripts: microphone_input.py  and commandCreator.py 
        microphone_input.py: Main script to run on terminal. Vosk is inside here. Publishes string messeage such as PICK ROD, PLACE ROD, GIVE ROCKER, GO HOME etc. 
        commandCreator.py: Includes definitions of usefull words and possible misunderstood words, command functions such as pick, place, go home. 
<br />Gesture recognition script: wrist_detection.py
        wrist detection.py: Includes left, right wrist and neck keypoint detections. Publishes string message such as LEFT(wrist), RIGHT(wrist), PAUSE, CONTINUE, LEFT(neck), RIGHT(neck), MIDDLE(neck).

<br />Object grasp pose detection script: grasp_pose_detection_detectron2.py
        grasp_pose_detection_detectron2.py: Includes object detections according to camera link and a function to convert to robot frame. Publishes pose message of detected individual objects 
        
<br />Sensor fusion script: opendr_fusion.py    
        opendr_fusion.py: Subscribes to speech, gesture and object detection script outputs. Main actions are occuring in the voice callback while gesture recognition is constantly updating human pose data. In the test scenario, object detection is decided to do object tracking for the first 10 seconds. Detections are ongoing afterwards but object poses are not being updated. 
 

 
