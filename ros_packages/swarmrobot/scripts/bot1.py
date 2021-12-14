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
        self.reverse, self.init = False, True
        self.start, self.dest = (0, 0), (0, 0)
        self.flag, self.next, self.done = 0, 0, 0
        self.rotate, self.ang, self.i_ang , self.indstn = 0, 0, 0, 0
        self.path, self.angle, self.points, self.pt = [], [], [], []

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
                self.pos = bot['bot1']
                # Threading Applied
                thread = threading.Thread(name="worker", target=self.path_execute, 
                                          args=(self.pos, ))
                thread.start()
                # self.path_execute(self.pos)
                value = {'bot1': [self.pos, self.dest, self.path]}
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
                    result = msgBot1Result()
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
            graph = function.read_graph()
            # points, self.angle = function.path_plan(graph, start, goal)
            points, self.angle = function.path_plan_custom(start, goal, self.reverse)
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
                if self.rotate == 0:
                    self.goal = self.path[self.next+1]
                    self.ang = self.angle[self.next]
                    # self.ang = int(function.dynamic_angle(cur, self.goal))
                    self.rotate_bot(self.ang)
                    self.rotate = 1
                elif self.rotate == 1:
                    self.rotate_bot_check(self.ang, self.pos[2])
                else:
                    self.done = 1
                    # c=0
                    self.i_ang = self.ang

            # If self.done is 1 then move the bot to the waypoint
            elif self.done == 1:
                # If the bot is within the range of goal,
                # then stop the bot else move forward
                if cur[0] in range(self.goal[0]-20, self.goal[0]+20) and \
                cur[1] in range(self.goal[1]-20, self.goal[1]+20):
                        print("Reached the Point")
                        self.move_bot(0)
                        self.rotate, self.done = 0, 0
                        self.next += 1
                else:
                    # angle = function.dynamic_angle(cur, self.goal)
                    # print("Move Forward")
                    # if c == 0:
                    #     pos = self.pos[2]
                    self.move_bot(1, self.i_ang)
                    # c = c + 1

            elif self.done == 2 and self.reverse == False:
                print("Turning Bot 45 deg to chute")
                self.rotate_bot(self.ang, 45)

                print("Actuating Servo")
                self.actuate_servo()

                print("Aligning the Bot to Axis")
                self.rotate_bot(135)
                # self.rotate_bot(90)
                # self.rotate_bot(45)

                print("Reverse Path is tracking")
                self.done, self.next = 0, 0
                self.reverse = True
                self.process_goal(self.dest, self.start)

            elif self.done == 2:
                print("Stop")
                self.induct_zone(self.ang)
                self.done, self.next, self.flag = 0, 0, 2
                self.reverse = False

        except Exception as e:
            print("Exception in Path Execute Function")
            print(e)

    # Function to Move Bot
    def move_bot(self, direct, pos):
        """
        This Function to move the bot in desired direction
        It may be either forward or backward
        """
        dif =  pos - self.pos[2] 

        if direct == 0:
            self.msg.data = [0, 0, 0]   
        elif self.pos[2] != pos and direct == 1:
            direct = function.rotate_direction(self.indstn, dif, 
                                               self.reverse, 1)
            self.msg.data = [direct, 0, 0]
        

        self.pub.publish(self.msg)

    # Function to Rotate Bot
    def rotate_bot(self, angle, offset=0):
        """
        This function is to rotate bot to specific angle
        based on offset
        """
        temp = angle - offset
        direct = function.rotate_direction(self.indstn, temp, 
                                           self.reverse)
        turn = abs(angle - offset)
        self.msg.data = [direct, turn, 0]
        self.pub.publish(self.msg)
        print("turning")

    # Function Check Rotate Bot
    def rotate_bot_check(self, angle, current):
        """
        This Function is to check the bot that it is rotated 
        to the specified angle or not
        """
        ang = abs(angle)
        if current in range(ang-10, ang+10):
            print(ang, current)
            print("Angle Obtained")
            self.msg.data = [0, 0, 0]
            self.rotate = 2

        else:
            if current <= ang:
                direct = 6
            elif current >= ang:
                direct = 5
            if self.reverse == True:
                if direct == 5:
                    direct = 6
                else:
                    direct = 5
            print("Obtaining Angle")
            print(current)
            
            self.msg.data = [direct, 0, 0]

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