#!/usr/bin/env python3
# ROS Node - Action Server - Bot2
"""
This ROS Node responsible for Bot2 Actions to move the bot,
To the destination location and drop the package using ROS
publisher with the help of rosserial_socket_server.

Action Server - /action_bot2
ROS Publisher - bot1/control_signal
              - /data_visualize
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

"""
This Class Bot2 consists of all members and modules which is used
to control the bot1 based on the goal Received from the Client
"""
class Bot2():

    # Constructor
    # Initializing the variables of this class
    def __init__(self):
        # Initialize Simple Action Server
        self._as = actionlib.ActionServer('/action_bot2', msgBot1Action, 
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
        self.graph = {}
        self.reverse, self.init = False, True
        self.start, self.dest = (0, 0), (0, 0)
        self.flag, self.next, self.ang, self.done, self.indstn = 0, 0, 0, 0, 0
        self.path, self.angle, self.points, self.pt, self.bot_obs = [], [], [], [], []

        # Creating a object to CvBridge() class
        self.bridge = CvBridge()

        # Defining ROS Publisher
        # To Communicate with bots
        self.pub = rospy.Publisher("bot2/control_signal", Int16MultiArray, 
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

        self.bot_init()
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
            print("Getting Into Induct Station")
            self.move_bot(1)
            rospy.sleep(1)

            self.rotate_bot(90)
            self.move_bot(4)
            rospy.sleep(1)

            print("Induct Station Reached")
            print("Waiting For Package")
            self.move_bot(0)
            rospy.sleep(1)

            print("Package Obtained")
            self.move_bot(1)
            rospy.sleep(1)
            self.move_bot(0)

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
                pos = bot['bot2']
                # graph = function.bot_in_graph(self.graph, self.bot_obs, pos, 'bot2')
                # points, self.angle = function.path_plan(self.graph, start, goal)
                self.path_execute(pos)
                value = {'bot2': [pos, self.dest, self.path]}
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
                    result = msgBot2Result()
                    result.flag = True
                    goal_handle.set_succeeded(result)
                    if self.done == 2:
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
            self.graph = function.read_graph()
            points, self.angle = function.path_plan(self.graph, start, goal)
            # Add Start and Goal to the path List
            print(points)
            if len(points) == 0:
                points.append(start)
                points.append(goal)
            else:
                points.insert(0, start)
                points.insert(len(points), goal)

            self.start = start
            self.path = points
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
            # Get the position of the bot
            if self.next == len(self.path)-1:
                self.done = 2
            if self.next == len(self.angle) and self.done != 2:
                self.goal = self.path[self.next+1]
                self.done = 1
                if len(self.angle) == 0:
                    self.ang = 0
                else:
                    self.ang = self.angle[-1]

            # If self.done is 0 then turn the bot to appropriate angle
            if self.done == 0:
                # If next value is equal to the length of the
                # path or angle list, then skip the statements.
                self.goal = self.path[self.next+1]
                ang = self.angle[self.next]
                self.rotate_bot(ang)
                self.done = 1

            # If self.done is 1 then move the bot to the waypoint
            elif self.done == 1:
                # If the bot is within the range of goal,
                # then stop the bot else move forward
                if cur[0] in range(self.goal[0]-15, self.goal[0]+15):
                    if cur[1] in range(self.goal[1]-15, self.goal[1]+15):
                        print("Reached the Point")
                        self.move_bot(0)
                        self.next += 1
                        self.done = 0
                else:
                    # angle = function.dynamic_angle(cur, self.goal)
                    self.move_bot(1)

            elif self.done == 2 and self.reverse == 0:
                print("Turning Bot 45 deg to chute")
                self.rotate_bot(self.ang, 45)

                print("Actuating Servo")
                self.actuate_servo()

                print("Aligning the Bot to Axis")
                self.rotate_bot(90)
                self.rotate_bot(45)

                print("Reverse Path is tracking")
                self.done, self.next = 0, 0
                self.reverse = True
                self.process_goal(self.dest, self.start)

            elif self.done == 2:
                self.inductzone(self.ang)
                self.done, self.next, self.flag = 0, 0, 2
                self.reverse = False

        except Exception as e:
            print("Exception in Path Execute Function")
            print(e)

    # Function to Move Bot
    def move_bot(self, direct):
        """
        This Function to move the bot in desired direction
        It may be either forward or backward
        """
        self.msg.data = [direct, 0, 0]
        self.pub.publish(self.msg)

    # Function to Rotate Bot
    def rotate_bot(self, angle, offset=0):
        """
        This function is to rotate bot to specific angle
        based on offset
        """
        direct = function.rotate_direction(self.indstn, angle, 
                                           self.reverse)
        turn = abs(angle - offset)
        self.msg.data = [direct, turn, 0]
        self.pub.publish(self.msg)

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
        self.msg.data = [0, 0, 1]
        self.pub.publish(self.msg)

        # Sleep For 2 Seconds
        # So, that the package falls from the bot
        rospy.sleep(2)

        # Revert the Servo to Normal Position
        print("Actuate Servo to 0 Degree")
        self.msg.data = [0, 0, 0]
        self.pub.publish(self.msg)

    # Function Induct Zone
    def induct_zone(self, ang):
        """
        This Function will move the bot inside the induct zone
        and make the robot wait for next package afterwards 
        move the bot outside the induct station
        """
        print("Aligning the Bot to Axis")
        self.rotate_bot(ang, 90)
        self.rotate_bot(90)

        print("Getting into Induct Station")
        self.move_bot(4)
        rospy.sleep(2)

        print("Bot has returned to Induct zone")
        print("Ready to Get the Next Package")
        self.move_bot(0)
        rospy.sleep(2)

        print("Getting Out of Induct Station")
        self.move_bot(1)
        rospy.sleep(1)

        self.move_bot(0)
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
    rospy.init_node('node_bot_2', anonymous=True)

    # Creating Object for the Class Bot1
    bot2 = Bot2()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        # Deleting the Created Object if Interrupted
        del bot2

if __name__ == '__main__':
    main()