#!/usr/bin/env python3
# Flipkart - Grid 3.0 - Robotics Competition
# ROS Node - Action Server - Bot1
"""
This ROS Node responsible for Bot1 Actions to move the bot,
To the destination location and drop the package using ROS
publisher with the help of rosserial_socket_server.
Action Server - /action_bot1
ROS Publisher - bot1/control_signal
              - /data_visualize
"""
# Importing Required Modules
import cv2
import json
import rospy
import client
import actionlib
import threading
import numpy as np
from helper import function
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
from rospy_message_converter import message_converter
from swarmrobot.msg import msgBot1Action, msgBot1Goal, msgBot1Result

"""
This Class Bot1 consists of all members and modules which is used
to control the bot1 based on the goal Received from the Client
"""
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
        self.init, self.reverse = True, False
        self.goal, self.start = (0, 0), (0, 0)
        self.use, self.flag, self.next, self.done, self.verify = 0, 0, 0, 0, 0
        self.rotate, self.ang, self.indsn, self.first, self.task = 0, 0, 0, 1, 1

        self.path, self.angle, self.points = [], [], []

        # Creating a object to CvBridge() class
        self.bridge = CvBridge()

        # Defining ROS Publisher
        # To Communicate with bots
        self.pub = rospy.Publisher("bot1/control_signal", Int16MultiArray, 
                                   queue_size=1, tcp_nodelay=True)

        # Defining ROS Publisher
        # To Visualize Path
        self.viz = rospy.Publisher("/data_visualize", String, latch=True,
                                   queue_size=4)
        self.str = String()
        self.msg = Int16MultiArray()

        # Subscribing to the ROS String Topic
        self.botpos_sub = rospy.Subscriber("/bot_position", String, 
                                           self.pos_callback, 
                                           queue_size=1)

        # self.bot_init()
        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started Bot1 Action Server.")

    # Function Bot Initialize
    def bot_init(self):
        """
        This Function will move the bot from it's starting position
        to inside the induct zone
        """
        try:
            # print("Getting Into Induct Station")
            # self.publish_command(1)
            # rospy.sleep(1)

            # self.rotate_bot(90)
            # self.publish_command(4)
            # rospy.sleep(1)
            print("In Induct Station")
            print("Waiting For Package")
            self.publish_command(0)
            rospy.sleep(1)

            print("Package Obtained")
            self.publish_command(1)
            rospy.sleep(1)
            self.publish_command(0)

        except Exception as e:
            print("Exception In Bot Initialize Function")
            print(e)

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
                self.pos = bot['bot1']
                self.path_execute(self.pos)
                value = {'bot1': [self.pos, self.goal, self.path]}
                msg = json.dumps(value)
                self.viz.publish(msg)
            else:
                pass

        except Exception as e:
            print("Exception in Position Callback Function")
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
            self.goal = (goal.goal_x, goal.goal_y)
            print(self.goal)
            self.indsn = goal.induct_station

            # Accept the Goal
            goal_handle.set_accepted()

            # Processing the Goal
            goal_id = goal_handle.get_goal_id()
            rospy.loginfo("Processing goal : " + str(goal_id.id))
            self.process_goal(self.start, self.goal)

            # Check whether the Goal is Processed
            while True:
                if self.flag == 2:
                    rospy.loginfo("Goal Completed")
                    self.flag = 0
                    result = msgBot1Result()
                    result.flag = True
                    goal_handle.set_succeeded(result)
                    # if self.done == 2:
                    self.done = 0
                    break

        except:
            print("Exception in On Goal Function")
            print(e)
            goal_handle.set_rejected()

    # Function to process
    def process_goal(self, start, goal):
        """
        This Function to plan a path and execute them to move 
        the bot
        """
        try:
            # graph = function.read_graph()
            # points, self.angle = function.path_plan(graph, start, goal)
            points, turns = function.path_plan_custom(start, goal, 
                                                           self.indsn, 
                                                           self.reverse)
            self.path = points
            if self.reverse == True:
                for i in range(len(turns)):
                    turns[i] = 170 - turns[i]
                print(turns)
            self.angle = turns

            print("Final Path List with Start and Goal Points")
            print(self.path)
            self.flag = 1

        except Exception as e:
            print("Exception in Process Goal Function")
            print(e)

    # Function to Monitor Bot
    def path_execute(self, cur):
        """
        This function will excute the path planned and monitor its
        status of operation
        """
        try:
            if self.next == len(self.path) and self.done != 3:
                self.done = 2

            # If self.done is 0 then turn the bot to appropriate angle
            elif self.done == 0:
                # If next value is equal to the length of the
                # path or angle list, then skip the statements.
                if self.rotate == 0:
                    self.goal = self.path[self.next]
                    if len(self.angle) == 0:
                        self.ang = 0
                    else:
                        self.ang = self.angle[self.next]
                    # self.rotate_bot(cur[2], self.ang, 1)
                    self.rotate = 1
                elif self.rotate == 1:
                    self.rotate_bot(cur[2], self.ang)

            # If self.done is 1 then move the bot to the waypoint
            elif self.done == 1:
                # If the bot is within the range of goal,
                # then stop the bot else move forward
                if cur[1] in range(self.goal[1]-65, self.goal[1]+35) \
                and cur[0] in range(self.goal[0]-35, self.goal[0]+35):
                        print("Reached the Point")
                        for i in range(0, 5):
                            self.publish_command(0)
                        self.rotate, self.verify, self.done = 0, 0, 1
                        self.next += 1
                        print(self.next, len(self.path))
                else:
                    # print("Move Forward")
                    self.move_bot(cur)

            if self.done == 2 and self.reverse == False:
                if self.task == 1:
                    self.drop_package(cur)
                elif self.task == 2:
                    self.align_bot(cur)
                elif self.task == 0:
                    print("Reverse Path is tracking")
                    self.done, self.rotate, self.next = 0, 0, 0
                    self.task = 1
                    self.reverse = True
                    self.process_goal(self.goal, self.start)

            elif self.done == 2 and self.reverse == True:
                if self.task == 1:
                    self.align_bot(cur)
                elif self.task == 0:
                    # print("self.done == 3")
                    # self.induct_zone(cur)
                    self.done = 3
                    print(self.done)
            elif self.done == 3:
                print("Stop")
                # self.induct_zone(self.ang)
                self.done, self.next, self.flag = 0, 0, 2
                self.reverse = False

        except Exception as e:
            print("Exception in Path Execute Function")
            print(e)

    # Function to Move Bot
    def move_bot(self, pos):
        """
        This Function to move the bot in desired direction
        It may be either forward or backward
        """
        if pos[2] in range(self.ang-2, self.ang+2):
            direct = 1
            # print("Forward")
            if self.reverse == True:
                direct = 4
        else:
            direct = function.publish_offset(pos[2], self.ang)
        # direct = 1
        self.publish_command(direct)

    # Function to Stop Bot
    def publish_command(self, direct, angle=0, servo=0):
        """
        This function will send command to the bots using 
        rosserial socket server through ROS Publisher
        """
        self.msg.data = [direct, angle, servo]
        self.pub.publish(self.msg)

    # Function to Rotate Bot
    def rotate_bot(self, current, angle, once=0, offset=0):
        """
        This function is to rotate bot to specific angle
        based on offset
        """
        if self.reverse == True:
            if angle in range(abs(angle)-5, abs(angle)+5): 
                direct, turn, self.rotate, self.done = 0, 0, 0, 1
        if angle in range(-2, 2):
            direct, turn, self.rotate, self.done = 0, 0, 0, 1
        elif once == 1:
            print("Rotating Bot using Gyro")
            temp = angle - offset
            direct = function.rotate_direction(-temp)
            turn = abs(angle - offset)
            # print("Rotating Bot to the angle " + str(angle))
        elif once == 0:
            # This statement is to check the bot that 
            # it is rotated to the specified angle or not
            if current in range(angle-2, angle+2):
                # print("Current Angle : " + str(angle))
                # print("Angle in List : " + str(current))
                direct, turn = 0, 0
                # self.verify += 1
                # if self.verify >= 2:
                print("Obtained Angle")
                # rospy.sleep(2)
                direct, turn, self.rotate, self.done = 0, 0, 0, 1
                if self.use == 1:
                    self.first = 0
            else:
                direct = function.publish_offset(current, angle)
                # print("Obtaining Angle")
                print(current, angle)
                turn = 0
        self.publish_command(direct, -turn)

    # Function Drop Package
    def drop_package(self, cur):
        """
        This function will make the bot to drop the package from the surface
        of the bot by actuating the servo and then align the bot to the 
        horizontal axis of the camera
        """
        if self.first == 1:
            if cur[1] < self.goal[1]:
                angle = 45
            else:
                angle = -45
            self.rotate_bot(self.pos[2], angle, 0)
            self.use = 1
        elif self.first == 0:
            print("Turned Bot 45 deg to chute")
            print("Actuating Servo")
            self.actuate_servo()
            self.use, self.first, self.task = 0, 1, 2

    # Function to Actuate Servo
    def actuate_servo(self):
        """
        This function will publish signals to the bot 
        to actuate the servo
        1 - 180 degree
        0 - 0 degree
        """
        # Actuate Servo to 180 degree
        print("Actuate Servo to 180 Degree")
        self.publish_command(0, 0, 1)
        # Sleep For 2 Seconds
        # So, that the package falls from the bot
        rospy.sleep(2)

        # Revert the Servo to Normal Position
        print("Actuate Servo to 0 Degree")
        self.publish_command(0)

    # Function to align Bot
    def align_bot(self, cur):
        """
        This function will try to align the bot to the Axis from the 
        current position
        """
        ang = cur[2]
        if self.first == 1:
            self.rotate_bot(cur[2], 0, 0)
            self.use = 1
        if self.first == 0:
            print("Aligned the Bot to Axis")
            self.use, self.task, self.first = 0, 0, 1

    # Function Induct Zone
    def induct_zone(self, ang):
        """
        This Function will move the bot inside the induct zone
        and make the robot wait for next package afterwards 
        move the bot outside the induct station
        """
        print("Getting into Induct Station")
        self.publish_command(1)
        rospy.sleep(1)

        print("Bot has returned to Induct zone")
        print("Ready to Get the Next Package")
        self.publish_command(0)
        rospy.sleep(2)

        print("Getting Out of Induct Station")
        self.publish_command(4)
        rospy.sleep(1)

        self.publish_command(0)
        rospy.sleep(1)

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