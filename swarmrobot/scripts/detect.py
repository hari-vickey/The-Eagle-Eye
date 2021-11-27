#!/usr/bin/env python3
"""
ROS Node - detect.py - V1 - Tested
This Node will detect the aruco markers of the bot then publishes them 
to other ROS Nodes using ROS publisher.

This Node uses rospy_message_converter to convert dictionary to 
ROS Messages.

ROS Publisher  - /bot_position
ROS Subscriber - /image_raw

"""
import sys
import cv2
import json
import rospy
import numpy as np
import cv2.aruco as aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy_message_converter import message_converter

# Class Detect
class Detect():
    """
    This class will detect the aruco markers of the bot
    and publish them using ros publihsers to other nodes
    """

    # Constructor
    # Initializing the variables of this class
    def __init__(self):

        # Creating a object to CvBridge() class
        self.bridge = CvBridge()

        # Subscribing to the ROS Image topic
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback, 
                                          queue_size = 1)

        # Publishing Bot Positions
        self.publisher = rospy.Publisher("/bot_position", String, 
                                         queue_size=1)

        # Creating msg variable of String Datatype
        self.msg = String()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        image = cv_image

        self.aruco_detect_bot(image)

    def aruco_detect_bot(self, frame):
        try:

            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            frame = aruco.drawDetectedMarkers(frame, markerCorners)
            print(markerIds, len(markerCorners))
            converted = np.int_(markerCorners)
            bot_name, points, cpts = [], [], []

            for i in markerIds:
                name = 'bot' + str(int(i))
                bot_name.append(name)
                pts = [converted[0][0][2].tolist(), converted[0][0][0].tolist()]
                points.append(pts)

            bot = dict(zip(bot_name, points))

            for i in bot:
                x1 = int((bot[i][0][0]+bot[i][1][0])/2)
                y1 = int((bot[i][0][1]+bot[i][1][1])/2)
                point = (x1, y1)
                cpts.append(point)

            bot = dict(zip(bot_name, cpts))
            self.msg = json.dumps(bot)
            self.publisher.publish(self.msg)
            print(bot)

        except Exception as e:
            pass

def main(args):
  
    rospy.init_node('node_bot_detect', anonymous=True)
    det = Detect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        rospy.on_shutdown(det.clean_shutdown)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)