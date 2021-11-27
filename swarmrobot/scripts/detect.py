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

        # Publishing Bot Positions
        self.publisher = rospy.Publisher("/bot_position", String, 
                                         queue_size=1)

        # Subscribing to the ROS Image topic
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback, 
                                          queue_size = 1)

        # Subscribing to Data Visualizer
        self.viz_sub = rospy.Subscriber("/data_visualize", String, self.viz_callback)
        # Creating msg variable of String Datatype
        self.msg = String()

    # Function for ROS Camera Subscription Callback
    def callback(self,data):
        """
        This functions gets all the image published in the subscribed topic and
        sends the image to the next function to process them
        """
        try:
            # Convert img msg to an data readable by cv2 library,
            # to process them further
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        self.image = cv_image

        # Detect Bots using Aruco Markers
        self.aruco_detect_bot(self.image)

    def aruco_detect_bot(self, frame):
        try:

            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            frame = aruco.drawDetectedMarkers(frame, markerCorners)
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

    # Function for Data Visualization Callback
    def viz_callback(self, data):
        """
        This Function gets all the published data visualization messages
        and process them to show the path
        """
        msg = message_converter.convert_ros_message_to_dictionary(data)
        temp = msg['data']
        bot = json.loads(temp)
        for i in bot:
            self.mark_points(self.image, i[0], i[1], i[2])

    # Function to Mark Points on the Image
    def mark_points(self, img, start, goal, ls):
        """
        Marking Points in the input image
        Also draw the lines of the path estimated
        """
        # Marking the Start Point and Goal point
        img = cv2.circle(img, start, 2, (255, 0, 0), 8)
        img = cv2.circle(img, goal, 2, (0, 0, 255), 8)
        ls.insert(0, start)
        ls.insert(len(ls), goal)
        # Marking the Minimized set of goalpoints
        for point1, point2 in zip(ls, ls[1:]):
            cv2.line(img, point1, point2, [0, 255, 0], 2)

        return img

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