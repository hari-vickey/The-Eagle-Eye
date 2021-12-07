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
import math
import json
import rospy
import actionlib
import threading
import numpy as np
import pandas as pd
from helper import function
from sensor_msgs.msg import Image
from heapq import heappush, heappop
from cv_bridge import CvBridge, CvBridgeError
from swarmrobot.msg import msgBot1Action, msgBot1Goal, msgBot1Result

# Class Client
class Client():

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

        self.bot_obs, self.points = [], []

        # Global Varibales for Execution
        self.Lock = 1
        self.n, self.rotate = 0, 0
        self.flag1, self.flag2, self.flag3 = 0, 0, 0
        self.goal, self.goalr, self.pnt = (0, 0), (0, 0), (0, 0)
        self.exec, self.reverse, self.graphc, self.path = 0, 0, 0, 1

        # Wait for UR5_2 Action Server
        self._ac1.wait_for_server()
        print("Action Server Up")
        print("waiting for Aruco Markers Detection")
        # rospy.sleep(3)
        self.location = function.read_location()
        print(self.location)
        self.read_sheet()

    # Function to read Excel Sheet
    def read_sheet(self):
        """
        This Function will read the excel sheet and sort 
        induct stations
        """
        ## Work Needed ##
        # Send Goal to Action Client
        # Threaded the algorithm function to process real time images
        # thread = threading.Thread(name="worker", target=self.algorithm, args=(cv_image, ))
        # thread.start()
        # self.algorithm(cv_image)
        start = (self.location[2][0], self.location[2][1])
        goal = self.closest_point(self.location['Kolkata'], start)
        print(start, goal)
        self._goal_handles[0] = self.send_goal_1(2, start[0]-60, 
                                                 start[1], goal[0], 
                                                 goal[1])

        # for i in self.location:
        #     if str(i) == "2" or str(i) == "1":
        #         continue
        #     start = (self.location[1][0], self.location[1][1])
        #     goal = self.closest_point(self.location[i], start)
        #     print(start, goal)
        #     self._goal_handles[0] = self.send_goal_1(1, start[0]-60, 
        #                                              start[1], goal[0], 
        #                                              goal[1])

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
        rospy.loginfo("Goal Sent")
        print("Induct Station - " + str(args[0]))
        print("Goal Point - " + "(" + str(args[3]) +", " + str(args[4]) + ")")
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

    # Creating Object for the Class Client
    cli = Client()

    try:
        # Spinning the Main Function
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

        # Deleting the Created Object if Interrupted
        del cli

        # Destroying all the cv2 Windows Created
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()