#!/usr/bin/env python3
# ROS Node - Simple Action Server - Bot1
"""
This ROS Node responsible for Bot1 Actions to move the bot,
To the destination location and drop the package using ROS
publisher with the help of rosserial_socket_server.

Action Server - /action_bot1
ROS Publisher - bot1/control_signal
ROS Subscriber - /incoming_goal
"""
import cv2
import json
import rospy
import client
import actionlib
import numpy as np
from helper import function
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
from rospy_message_converter import message_converter
from swarmrobot.msg import msgBot1Action, msgBot1Goal, msgBot1Result

class Bot1():

    # Constructor
    # Initializing the variables of this class
    def __init__(self):
        # Initialize Simple Action Server
        self._as = actionlib.ActionServer('/action_bot1', msgBot1Action, 
                                          self.on_goal, self.on_cancel, 
                                          auto_start=False)
        '''
            * self.on_goal - It is the fuction pointer which points to 
            a function which will be called when should  Action Server 
            receives a Goal.

            * self.on_cancel - It is the fuction pointer which points 
            to a function which will be called when the Action Server 
            receives a Cancel Request.
        '''
        # Defining Variables for this Class
        self.flag, self.next, self.done, self.indstn = 0, 0, 0, 0
        self.start, self.dest = (0, 0), (0, 0)
        self.path, self.angle, self.points, self.pt = [], [], [], []
        # self.pos = {}

        # Creating a object to CvBridge() class
        self.bridge = CvBridge()

        # Defining ROS Publisher
        # To Communicate with bots
        self.pub = rospy.Publisher("bot1/control_signal", Int16MultiArray, 
                                   queue_size=1, tcp_nodelay=True)

        # Defining ROS Publisher
        # To Visualize Path
        self.viz = rospy.Publisher("/data_visualize", String, queue_size=1)
        self.str = String()
        self.msg = Int16MultiArray()

        # Subscribing to the ROS String Topic
        self.botpos_sub = rospy.Subscriber("/bot_position", String, 
                                           self.pos_callback, 
                                           queue_size=1)

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started Bot1 Action Server.")

    # Callback Function for Bot Position Subscriber
    def pos_callback(self, data):
        """
        This Function gets all the published bot position as String and 
        convert the data to dictionary for processing.
        """
        try:
            if self.flag == 1:
                msg = message_converter.convert_ros_message_to_dictionary(data)
                temp = msg['data']
                bot = json.loads(temp)
                pos = bot['bot1']
                self.path_execute(pos)
                value = {'bot1': [pos, self.dest, self.path]}
                print(value)
                msg = json.dumps(value)
                self.viz.publish(msg)
            else:
                pass
        except Exception as e:
            print(e)

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):       
        """
        This function will receive the goal from the action server and
        process them to acheive the goal
        """
        try:
            goal = goal_handle.get_goal()
            rospy.loginfo("Received new goal from Client")

            self.start = (goal.induct_x, goal.induct_y)
            print(self.start)
            self.dest = (goal.goal_x, goal.goal_y)
            print(self.dest)
            self.indstn = goal.induct_station

            # Accept the Goal
            goal_handle.set_accepted()

            # Processing the Goal
            goal_id = goal_handle.get_goal_id()
            rospy.loginfo("Processing goal : " + str(goal_id.id))
            self.process_goal(self.start, self.dest)

            # Check whether the Goal is Processed
            while True:
                if self.flag == 2:
                    rospy.loginfo("Goal Completed")
                    self.flag = 0
                    result = msgBot1Result()
                    result.flag = True
                    goal_handle.set_succeeded(result)
                    if self.done == 2:
                        break

        except:
            print(e)
            goal_handle.set_rejected()

    # Function to process
    def process_goal(self, start, goal):
        """
        This Function to plan a path and execute them to move 
        the bot
        """
        try:
            graph = function.read_graph()
            points, self.angle = function.path_plan(graph, start, goal)
            # Add Start and Goal to the path List
            # print(points)
            if len(points) == 0:
                points.append(start)
                points.append(goal)
            else:
                points.insert(0, start)
                points.insert(len(points), goal)
            self.path = points
            # print(self.next, len(self.path), self.path)
            self.flag = 1
        except Exception as e:
            print("processing")
            print(e)

    # Function to Monitor Bot
    def path_execute(self, cur):
        """
        This function will excute the path planned and monitor its
        status of operation
        """
        try:
            # Get the position of the bot
            if self.next == len(self.path)-1:
                self.done = 2
            if self.next == len(self.angle) and self.done != 2:
                self.goal = self.path[self.next+1]
                self.done = 1

            # self.goal = self.dest
            # print(self.goal)
            # If self.done is 0 then turn the bot to appropriate angle
            if self.done == 0:
                # If next value is equal to the length of the
                # path or angle list, then skip the statements.
                if self.indstn == 1:
                    self.goal = self.path[self.next+1]
                    ang = self.angle[self.next]
                    if ang > 0:
                        print("Rotate ClockWise")
                        direct = 2
                    else:
                        print("Rotate AntiClockWise")
                        direct = 3

                elif self.indstn == 2:
                    if ang > 0:
                        print("Rotate AntiClockWise")
                        direct = 3
                    else:
                        print("Rotate ClockWise")
                        direct = 2

                self.msg.data = [direct, ang, 0]
                self.pub.publish(self.msg)
                self.done = 1

            # If self.done is 1 then move the bot to the waypoint
            elif self.done == 1:
                # If the bot is within the range of goal,
                # then stop the bot else move forward
                if cur[0] in range(self.goal[0]-15, self.goal[0]+15):
                    if cur[1] in range(self.goal[1]-15, self.goal[1]+15):
                        print("Stop")
                        self.msg.data = [0, 0, 0]
                        self.next += 1
                        self.done = 0
                else:
                    print("Move Forward")
                    self.msg.data = [1, 0, 0]
                    
                self.pub.publish(self.msg)

            elif self.done == 2:
                print("self.done is 2")
                self.done = 0
                self.flag = 2

        except Exception as e:
            print(e)

    # Function to Cancel Incoming Goal
    def on_cancel(self, goal_handle):
        """
        This function will be called when Goal Cancel request is send to the Action Server
        """
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()

# Main Function
def main():
    """
    This is the start of execution of this node
    """
    # Initializing the Node
    rospy.init_node('node_bot_1', anonymous=True)

    # Creating Object for the Class Bot1
    bot1 = Bot1()

    try:
        # Spinning the Main Function
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        # Deleting the Created Object if Interrupted
        del bot1

if __name__ == '__main__':
    main()