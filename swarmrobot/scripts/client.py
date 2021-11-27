#!/usr/bin/env python3
"""
Flipkart - Grid 3.0 - Robotics Competition
ROS Node - server.py (yet to be completed)
This node will do the following :
    1. Subscribe to the images published by the ros
    2. Create a graph of obstacles to avoid collision with them
    3. Detect the bots using Aruco Markers
    4. Decode the aruco marker in the arena to get the destination and induct 
       station points
    5. Plan a path using a custom planner or A* Algorithm
    6. Publish the commands to move the bot using real time comparison between
       goal and the aruco_detect_bot ouput
"""
##################            Code V1 - Stage 1                #################
# Importing Required Modules
import os
import cv2
import math
import json
import rospy
import actionlib
import threading
import numpy as np
import pandas as pd
import cv2.aruco as aruco
from helper import function
from sensor_msgs.msg import Image
from heapq import heappush, heappop
from cv_bridge import CvBridge, CvBridgeError
from swarmrobot.msg import msgBot1Action, msgBot1Goal, msgBot1Result

# Class Server
class Server():

    """
    This class has all the required functions to complete the task as
    mentioned in the doc string of this file
    """

    # Constructor
    # Initializing the variables of this class
    def __init__(self):

        #Initialize SimpleActionClientTurtle
        self._ac1 = actionlib.ActionClient('/action_bot1', msgBot1Action)
        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Creating a object to CvBridge() class
        self.bridge = CvBridge()

        # Creating Variables to store necessary values
        self.location = ['Mumbai', 'Delhi', 'Kolkata', 
                         'Chennai', 'Bengaluru', 'Hyderabad', 
                         'Pune', 'Ahemdabad', 'Jaipur']

        # Global Varibales for Aruco Marker Detections
        self.destination, self.inductzone = {}, {}
        self.aruco1, self.ind1, self.aruco2, self.ind2 = 0, 0, 0, 0
        self.temp1, self.temp2, self.temp3, self.temp4 = [], [], [], []

        self.bot_obs, self.points = [], []

        # Global Varibales for Execution
        self.good, self.Lock = 0, 1
        self.n, self.rotate = 0, 0
        self.flag1, self.flag2, self.flag3 = 0, 0, 0
        self.goal, self.goalr, self.pnt = (0, 0), (0, 0), (0, 0)
        self.exec, self.reverse, self.graphc, self.path = 0, 0, 0, 1

        # Subscribing to the ROS Image topic
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback,
                                          queue_size = 1)
        # Wait for UR5_2 Action Server
        self._ac1.wait_for_server()
        print("Action Server Up")
        self.read_sheet()

    # Function to read Excel Sheet
    def read_sheet(self):
        """
        This Function will read the excel sheet and sort 
        induct stations
        """
        ## Work Needed ##
        # Send Goal to Action Client
        while True:
            if self.good == 1 and self.Lock == 1:
                self.Lock = 0
                start = self.inductzone[1]
                goal = self.closest_point(self.destination['Hyderabad'], start)
                # print(start, goal)
                self._goal_handles[0] = self.send_goal_1(1, start[0]-60, start[1], 
                                                         goal[0], goal[1])

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

            if self.aruco1 == 1 and self.aruco2 == 1 and self.graphc == 1:
                # Threaded the algorithm function to process real time images
                # thread = threading.Thread(name="worker", target=self.algorithm, args=(cv_image, ))
                # thread.start()
                # self.algorithm(cv_image)
                self.good = 1
                self.image_sub.unregister()

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
            # print(e)
            pass

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
        print("Completed Basic Graph Plotting")

    # Function to detect aruco Markers of Induct Station Points
    def aruco_detect_inductpoint(self, frame):
        try:
            self.ind1 = 0
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the Induct Point markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

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

                print('\033[93m' + "Induct Zone Aruco Markers Detected" + '\033[93m')
                print(self.inductzone)

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
                print('\033[92m' + "Destination Markers Detected" + '\033[92m')
                print(self.destination)
                print('\033[0m')
                self.aruco2 = 1

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

    # Function On_Transition
    def on_transition(self, goal_handle):
        """
        This function will be called when there is a change of state in the 
        Action Client State Machine
        """

        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        result = msgBot1Result()
        ind = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                ind = i + 1
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(ind))

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(ind) + ": Goal just went active.")

        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(ind) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            result = goal_handle.get_result()
            rospy.loginfo(result.flag)
            if result.flag == True:
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(ind))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(ind))

    # Function Send Goal
    def send_goal_1(self, *args):
        """
        This function is used to send Goals to Action Server
        """
        # Create a Goal Message object
        goal = msgBot1Goal()
        goal.induct_station = args[0]
        goal.induct_x = args[1]
        goal.induct_y = args[2]
        goal.goal_x = args[3]
        goal.goal_y = args[4]
        rospy.loginfo("Send goal.")
        # self.on_transition - It is a function pointer to a function,
        # which will be called when there is a change of state in the 
        # Action Client State Machine
        goal_handle = self._ac1.send_goal(goal, self.on_transition, None)

        return goal_handle

    # Function Algorithm
    def algorithm(self, image):
        """
        This function is responsible to complete the entire task.
        It is also combined with multiple control statements to
        get optimal results
        """
        self.send_goal()
        ###############################
        #                             #
        ##                           ##
        ###                         ###
        # Lot of Work need to be done #
        ###                         ###
        ##                           ##
        #                             #
        ###############################


    # Function to identify the closest point
    def closest_point(self, ls, pos):
        """
        This Function will choose the closest point of the destination
        from the current position of the bot.
        """
        x, y = pos
        d = list(map(lambda t: math.sqrt(pow(t[0]-x,2)+pow(t[1]-y,2)),ls))
        min_res = min(d)
        i = d.index(min_res)

        return ls[i]

    # Function to validate point
    def validate_point(self, coord, points):
        """
        This function will validate the point with obstacles of
        its presence and returns the result as 1 or 0
        """
        try:
            points.index(coord)
            flag = 0
        except ValueError:
            flag = 1

        return flag

    # Function to Publish direction command
    def publish_command_direction(self, bot, goal, cur):
        """
        This function will publish command as direction forward
        based on certain goal commands
        """
        #Creating ROS Publisher
        direction = rospy.Publisher('/direction_control', Int16, queue_size=1)
        direct = Int16()

        # Declaring Variables with Initial Value
        x_follow, y_follow, rotate, value = 0, 0, 0, 1

        # Choosing the Axis to make the bot to follow in one direction
        # This is done by comparing the current position with goal position
        if cur[0] in range((goal[0]-30), (goal[0]+30)):
            y_follow = 1
            # On condition of the following axis
            # Subtracting x co-ordinates of the bot with goal,
            # To get the direction of rotatation of the bot
            # If angle is positive then rotate clockwise
            # else rotate anti-clockwise
            value = cur[0] - self.goal[0]

        elif cur[1] in range((goal[1]-30), (goal[1]+30)):
            x_follow = 1
            # On condition of the following axis
            # Subtracting y co-ordinates of the bot with goal,
            # To get the direction of rotatation of the bot
            # If angle is positive then rotate anti-clockwise
            # else rotate clockwise
            value = self.goal[1] - cur[1]

        # Vertical Movement
        # Comparing the current y co-ordinates of the bot with goal
        # If the current position is in the range of the goal position or
        # withing the limit then, Stopping the Bot
        # Note: Value=30 is the tolerance added to compensate the tf delay
        if cur[1] in range((goal[1]-30), (goal[1]+30)) and y_follow == 1:
            direct = 4
            direction.publish(direct)
            rotate = 1

        # If the bot is not in the range of the goal
        # then Bot is moved forward
        # Note: Value=30 is the tolerance added to compensate the tf delay
        elif cur[1] not in range((goal[1]-30), (goal[1]+30)) and y_follow == 1:
            direct = 1
            direction.publish(direct)

        # Horizontal Movement
        # Comparing the current x co-ordinates of the bot with goal
        # If the current position is in the range of the goal position or
        # withing the limit then, Stopping the Bot
        # Note: Value=30 is the tolerance added to compensate the tf delay
        if cur[0] in range((goal[0]-30), (goal[0]+30)) and x_follow == 1:
            direct = 4
            direction.publish(direct)
            rotate = 1

        # If the bot is not in the range of the goal
        # then Bot is moved forward
        # Note: Value=30 is the tolerance added to compensate the tf delay
        elif cur[0] not in range((goal[0]-30), (goal[0]+30)) and x_follow == 1:
            direct = 1
            direction.publish(direct)

        self.publish_command_rotation(bot, value, rotate)

    # Function to publish rotational command
    def publish_command_rotation(self, bot, val, rot):
        """
        This functions get the angle and publish the commands to
        rotate the bot accordingly
        """
        ##

        # Need to Work in this Function

        ##

        # Creating a ROS Publisher
        direction = rospy.Publisher('/direction_control', Twist, queue_size=1)
        direct = Int16()

        # Defining Constants
        angular_speed = 1

        # If the value is less than 0, then rotate clockwise
        # else rotate anti-clockwise
        if val<=0:
            # Work Here
            i = 1
        else:
            # Work Here
            i = 0

        # If rotate is made to 1 then rotate the bot
        # Based On the path that the bot is executing
        # added +55 degree because the base link,
        # Not at exact center of the bot
        if rot == 1:
            if self.path == 1:
                angle = 180
                self.path = 2
            elif self.path == 2 and self.rotate == 0:
                self.actuate_servo(bot)
                angle = 360
                self.exec, self.path, self.reverse = 0, 1, 2
                self.rotate = 1

    # Function to actuate servo motor
    def actuate_servo(self, bot):

        # Creating a ROS Publisher
        servo = rospy.Publisher('/servo_control', Int16, queue_size=1)
        servo = Int16()

        # Defining angle for servo actuatuion,
        # To drop the package from bot
        servo = 1
        pub1.publish(servo)

        # Wait till the package is dropped
        rospy.sleep(2)

        # Defined angle as 180 to actuate the
        # servo back to initial position
        servo = 0
        pub1.publish(servo)

    # Destructor of the Class
    def __del__(self):
        rospy.loginfo('\033[94m' + "Shutting Down" + '\033[0m')

# Main Function
def main():
    """
    This is the start of execution of this node
    """
    # Initializing the Node
    rospy.init_node('node_server', anonymous=True)

    # Creating Object for the Class Server
    ser = Server()

    try:
        # Spinning the Main Function
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

        # Deleting the Created Object if Interrupted
        del ser

        # Destroying all the cv2 Windows Created
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()