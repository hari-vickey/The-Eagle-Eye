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
# Importing Required Modules
import sys
import cv2
import math
import json
import rospy
import numpy as np
import cv2.aruco as aruco
from helper import function
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

        # Creating Variables to store necessary values
        self.location = ['Mumbai', 'Delhi', 'Kolkata', 
                         'Chennai', 'Bengaluru', 'Hyderabad', 
                         'Pune', 'Ahmedabad', 'Jaipur']

        # Global Varibales for Aruco Marker Detections
        self.completed, self.graphc = 0, 0
        self.dict, self.destination, self.inductzone = {}, {}, {}
        self.aruco1, self.ind1, self.aruco2, self.ind2 = 0, 0, 0, 0
        self.temp1, self.temp2, self.temp3, self.temp4 = [], [], [], []

        # Publishing Bot Positions
        self.publisher = rospy.Publisher("/bot_position", String, 
                                         queue_size=1)

        # Subscribing to the ROS Image topic
        img_sub = rospy.Subscriber("/image_raw", Image, 
                                          self.callback, queue_size = 1)

        # Subscribing to Data Visualizer
        self.viz_sub = rospy.Subscriber("/data_visualize", String, self.viz_callback, queue_size=1)

        # Creating msg variable of String Datatype
        self.msg = String()

    # Function for ROS Camera Subscription Callback
    def callback(self, data):
        """
        This functions gets all the image published in the subscribed topic and
        sends the image to the next function to process them
        """
        try:
            # Convert img msg to an data readable by cv2 library,
            # to process them further
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = cv_image[0:720, 115:1100]

            # If Aruco markers of inductzone and destination detected
            # Also, if the graph is plotted then bot detection starts
            if self.completed == 1:
                # Detect Bots using Aruco Markers
                bot = self.aruco_detect_bot(cv_image)
                img = self.mark_points(cv_image, bot)
                img = cv2.resize(img, (800, 600))
                cv2.imshow("usb_cam_stream", img)
                cv2.waitKey(1)
            elif self.completed == 2:
                function.write_graph()
                self.destination.update(self.inductzone)
                function.write_location(self.destination)

                print('\033[93m' + "Induct Zone Aruco Markers Detected")
                print(self.inductzone)
                print(".....")
                print('\033[92m' + "Completed Graph Plotting")
                print(".....")
                print('\033[94m' + "Destination Markers Detected")
                print(self.destination)
                print("....." + '\033[0m')
                self.completed = 1

            # Execute the arena_config function only one time
            elif self.graphc == 0:
                self.arena_config(cv_image)

            # Execute the aruco_detection_destination untill it detects,
            # all the destination points
            elif self.aruco1 == 0:
                self.aruco_detect_inductpoint(cv_image)

            # Execute the aruco_detection_inductstation untill it detects,
            # all the inductstation points
            elif self.aruco2 == 0:
                self.aruco_detect_destination(cv_image)

        except Exception as e:
            pass

    # Function to detect aruco markers Bot
    def aruco_detect_bot(self, frame):
        """
        This function will detect the aruco markers of the bot
        using dictionary of size 4x4 50
        """
        try:
            # Creating Detector Parameters
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            frame = aruco.drawDetectedMarkers(frame, markerCorners)
            converted = np.int_(markerCorners)
            bot_name, points, cpts = [], [], []
            l = 0

            for i in markerIds:
                name = 'bot' + str(int(i))
                bot_name.append(name)
                pts = [converted[l][0][2].tolist(), 
                       converted[l][0][0].tolist(),
                       converted[l][0][1].tolist(),
                       converted[l][0][3].tolist()]
                points.append(pts)
                l += 1

            bot = dict(zip(bot_name, points))

            for i in bot:
                try:
                    deg = self.bot_angle(bot[i][2], bot[i][0], bot[i][1], bot[i][3])
                except ZeroDivisionError:
                    deg = 0

                x1 = int((bot[i][1][0]+bot[i][2][0])/2)
                y1 = int((bot[i][1][1]+bot[i][2][1])/2)
                point = (x1, y1, int(deg))
                cpts.append(point)

            bot = dict(zip(bot_name, cpts))
            self.msg = json.dumps(bot)
            self.publisher.publish(self.msg)
            print(bot)

            return bot

        except Exception as e:
            # print(e)
            pass

    # Function to Angle of the Bot
    def bot_angle(self, pt2, pt0, pt1, pt3):
        """
        This function is to calculate the angle of the bot
        using the third and four point of the aruco marker
        """
        x = abs(pt3[0] - pt0[0])
        y = abs(pt3[1] - pt0[1])
        p = abs(pt0[0] - pt1[0])
        q = abs(pt0[1] - pt1[1])
        if pt0[1] < pt3[1] and pt0[0] > pt3[0] and pt0[0] > pt1[0]:
            rad = math.atan(x/y)
            d = rad *(180/(math.pi))
        elif pt0[1] > pt3[1] and pt0[0] > pt3[0] and pt0[1] > pt1[1] and pt0[0] > pt1[0]:
            rad = math.atan(q/p)
            d = rad *(180/(math.pi))
            d = d + 45
        elif pt0[1] > pt3[1] and pt0[0] > pt3[0] and pt0[1] > pt1[1] and pt0[0] < pt1[0] :
            rad = math.atan(p/q)
            d = rad *(180/(math.pi))
            d = d + 130    
            # Left Turn
        elif pt0[1] < pt3[1] and pt0[0] < pt3[0] and pt0[1] < pt1[1]:
            rad = math.atan(x/y)
            d = -(rad *(180/(math.pi)))
        elif pt0[0] < pt3[0] and pt0[0] < pt1[0] and pt0[1] > pt3[1] and pt0[1] < pt1[1]:
            rad = math.atan(p/q)
            d = rad *(180/(math.pi))
            d = -(d + 45)   
        elif pt0[0] < pt3[0] and pt0[0] < pt1[0] and pt0[1] > pt3[1] and pt0[1] > pt1[1]:
            rad = math.atan(q/p)
            d = rad *(180/(math.pi))
            d = -(d + 130)      
        elif pt0[1] == pt3[1]:
            d = 90
        elif pt0[0] == pt3[0] and pt0[1] == pt1[1]:
            d = 180
        elif pt0[0] == pt1[0]:
            d = 90     

        return d

    # Function to have spatial awareness of the arena
    def arena_config(self, image):
        """
        This function defines the bot radius and calls the next function to
        create the arena as graph and generates the points of the obstacle
        """
        radius, clearance = 0, 0
        height, width, _ = image.shape
        function.init_graph(height, width)
        self.graphc = 1

    # Function to detect aruco Markers of Induct Station Points
    def aruco_detect_inductpoint(self, frame):
        try:
            """
            This Function will detect inductstation aruco markers
            """
            self.ind1 = 0
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the Induct Point markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            for i in markerIds:
                try:
                    self.temp1.index(i)
                    self.ind1 += 1

                except ValueError:
                    temp = int(i)
                    self.temp1.append(temp)
                    self.temp2.append(markerCorners[self.ind1].tolist())
                    self.ind1 += 1

            if len(self.temp1) == 2:
                self.inductzone = self.extract_induct_point(self.temp1, self.temp2)
                self.aruco1 = 1

        except Exception as e:
            pass

    # Function to Extract Induct Points
    def extract_induct_point(self, ids, bbox):
        """
        Extract the induct points from the list and convert
        it to dictionary and returns it.
        """
        l_ind = 0
        temp_ls, dest_ls = [], []

        for i in ids:
            name = int(i)
            cen_x = int((bbox[l_ind][0][0][0] + bbox[l_ind][0][2][0])/2)
            cen_y = int((bbox[l_ind][0][0][1] + bbox[l_ind][0][2][1])/2)
            ctp = (cen_x, cen_y)
            temp_ls.append(ctp)
            dest_ls.append(name)
            l_ind += 1

        dest = dict(zip(dest_ls, temp_ls))

        return dest

    # Function to detect aruco Markers of Destination Points
    def aruco_detect_destination(self, frame):
        """
        This Function is responsible to detect all the destination markers
        to identify all possible location around the chute.
        """
        try:
            self.ind2 = 0
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            for i in markerIds:
                indices = [j for j, x in enumerate(self.temp1) if x == i]
                if len(indices) == 1:
                    temp = int(i)
                    self.temp3.append(temp)
                    self.temp4.append(markerCorners[self.ind2])
                elif len(indices) == 0:
                    temp = int(i)
                    self.temp3.append(temp)
                    self.temp4.append(markerCorners[self.ind2])
                self.ind2 += 1

            if len(self.temp3) == 18:
                self.destination = self.extract_goal_point(self.temp3, self.temp4)
                self.aruco2, self.completed = 1, 2

        except:
            pass

    # Function to Extract Goal Points
    def extract_goal_point(self, ids, bbox):
        """
        This function will extract the co-ords of the goal point and
        returns a dictionary
        """
        ind = 0
        ids_ls, cor_ls = [], []
        temp_ls, sum_ls, pts_ls = [], [], []

        for i in ids:
            ids_ls.append(ids[ind])
            t = bbox[ind].tolist()
            cor_ls.append(t[0])
            ind += 1

        for k in range(1, 10):
            indices = [i for i, x in enumerate(ids_ls) if x == k]
            for l in indices:
                sum_ls = sum_ls + cor_ls[l]
            temp_ls.append(sum_ls)
            sum_ls = []

        for i in range(0,9):
            x1 = int((temp_ls[i][0][0] + temp_ls[i][2][0])/2)
            y1 = int((temp_ls[i][0][1] + temp_ls[i][2][1])/2)
            x2 = int((temp_ls[i][4][0] + temp_ls[i][6][0])/2)
            y2 = int((temp_ls[i][4][1] + temp_ls[i][6][1])/2)

            if (y1<y2):
                x1, x2 = x2, x1
                y1, y2 = y2, y1

            d1 = (x1, y1)
            d2 = (x2, y2)
            i1 = (x1, y2)
            i2 = (x2, y1)

            function.render_graph((x1+25, y1-25), (x2-25, y2+25))
            pts = [d1, d2, i1, i2]
            pts_ls.append(pts)

        dest = dict(zip(self.location, pts_ls))

        return dest

    # Function for Data Visualization Callback
    def viz_callback(self, data):
        """
        This Function gets all the published data visualization messages
        and process them to show the path
        """
        msg = message_converter.convert_ros_message_to_dictionary(data)
        temp = msg['data']
        bot = json.loads(temp)
        print(bot)
        for i in bot:
            if i == "bot2":
                self.dict[i] = bot[i]
            if i == "bot4":
                self.dict[i] = bot[i]

    # Function to Mark Points on the Image
    def mark_points(self, img, bot):
        """
        Marking Points in the input image
        Also draw the lines of the path estimated
        """
        for i in bot:
            img = cv2.circle(img, (bot[i][0], bot[i][1]), 2, (255, 255, 0), 8)

        # Marking the Start Point and Goal point
        for i in self.dict:
            # Marking the Start Point and Goal point
            img = cv2.circle(img, self.dict[i][0], 2, (255, 0, 0), 8)
            img = cv2.circle(img, self.dict[i][1], 2, (0, 0, 255), 8)
            ls = self.dict[i][2]
            ls.insert(0, self.dict[i][0])
            # Marking the Minimized set of goalpoints
            for point1, point2 in zip(ls, ls[1:]):
                cv2.line(img, point1, point2, [0, 255, 0], 2)

        return img

# Main Function
def main(args):
    # Initializing ROS Node
    rospy.init_node('node_bot_detect', anonymous=True)
    # Creating instance for the class
    det = Detect()
    try:
        # Spin the Node
        rospy.spin()

    except KeyboardInterrupt:
        # ShutDown the Node
        rospy.loginfo("Shutting down")
        rospy.on_shutdown(det.clean_shutdown)

    # Destroying CV2 Windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)