#!/usr/bin/env python3
# Importing Required Modules
import json
import rospy
from helper import function
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from rospy_message_converter import message_converter

"""
This Class Bot1 consists of all members and modules which is used
to control the bot1 based on the goal Received from the Client
"""
class Test():

    # Constructor
    # Initializing the variables of this class
    def __init__(self):
        # Defining ROS Publisher
        # To Communicate with bot 1
        self.pub = rospy.Publisher("bot1/control_signal", Int16MultiArray, 
                                   queue_size=1, tcp_nodelay=True)

        self.msg = Int16MultiArray()
        self.indstn = 1
        self.reverse = False
        # Subscribing to the ROS String Topic
        self.botpos_sub = rospy.Subscriber("/bot_position", String, 
                                           self.pos_callback, 
                                           queue_size=1)

        print("1. Rotate Bot to specific Angle\n2. Move Straight\n \
              3. Actuate Servo")
        self.choice = int(input("Enter a Number : "))
        if self.choice == 1:
            self.ang = int(input("Enter a Angle"))
            self.indstn = int(input("Enter Induct Station"))

    # Callback Function for Bot Position Subscriber
    def pos_callback(self, data):
        """
        This Function gets all the published bot position as String and 
        convert the data to dictionary for processing.
        """
        try:
            msg = message_converter.convert_ros_message_to_dictionary(data)
            temp = msg['data']
            bot = json.loads(temp)
            self.pos = bot['bot1']
            if self.choice == 1:
                self.rotate_bot(self.ang)
            elif self.choice == 2:
                print("Try Again Later")
                self.choice = 0
                # self.move_straight()
            value = {'bot1': [self.pos, self.dest, []]}
            msg = json.dumps(value)
            self.viz.publish(msg)
        except Exception as e:
            print("Exception in Position Callback Function")
            print(e)

    # Function to Move Bot
    def move_straight(self, direct, pos):
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
        if self.flag == 1:
            temp = angle - offset
            direct = function.rotate_direction(self.indstn, temp, 
                                               self.reverse)
            turn = abs(angle - offset)
            self.msg.data = [direct, turn, 0]
            self.pub.publish(self.msg)
            print("Bot Rotate Function")
            self.flag = 2
        elif self.flag == 2:
            rotate_bot_check(abs(angle), self.pos[2])

    # Function Check Rotate Bot
    def rotate_bot_check(self, ang, current):
        """
        This Function is to check the bot that it is rotated 
        to the specified angle or not
        """
        if current in range(ang-2, ang+2):
            print(ang, current)
            print("Angle Obtained")
            self.msg.data = [0, 0, 0]
            # self.rotate = 2
            self.flag = 3
            self.choice = 0

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
            print(current, ang)

            self.msg.data = [direct, 0, 0]

        self.pub.publish(self.msg)

# Main Function
def main():
    # Initializing the Node
    rospy.init_node('node_test', anonymous=True)

    # Creating Object for the Class Bot1
    test = Test()

    try:
        # Spinning the Main Function
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        # Deleting the Created Object if Interrupted
        del test

if __name__ == '__main__':
    main()