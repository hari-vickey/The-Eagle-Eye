#!/usr/bin/env python3
# Importing Required Modules
import json
import rospy
from helper import function
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from rospy_message_converter import message_converter

class Test():
    # Constructor
    # Initializing the variables of this class
    def __init__(self):
        # Defining ROS Publisher
        # To Communicate with bot 1
        print("1. Rotate Bot to specific Angle\n2. Move Straight\n \
              3. Actuate Servo")
        self.choice = int(input("Enter a Number : "))
        if self.choice == 1:
            self.ang = int(input("Enter a Angle"))
            self.indstn = int(input("Enter Induct Station"))
        self.pub = rospy.Publisher("bot1/control_signal", Int16MultiArray, 
                                   queue_size=1, tcp_nodelay=True)

        self.msg = Int16MultiArray()
        self.indstn, self.first, self.flag, self.c = 1, 1, 1, 0
        self.reverse = False
        # Subscribing to the ROS String Topic
        self.botpos_sub = rospy.Subscriber("/bot_position", String, 
                                           self.pos_callback, 
                                           queue_size=1)

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
                # print("Try Again Later")
                # self.choice = 0
                self.move_straight(self.pos)
            # value = {'bot1': [self.pos, (0, 0), []]}
            # msg = json.dumps(value)
            # self.viz.publish(msg)
        except Exception as e:
            print("Exception in Position Callback Function")
            print(e)

    # Function to Move Bot
    def move_straight(self, pos):
        """
        This Function to move the bot in desired direction
        It may be either forward or backward
        """
        if self.first == 1:
            self.cang = pos[2]
            self.first = 2
        else:
            if pos[2] in range(self.cang-2, self.cang+2):
                direct = 1
                print("Forward")
            elif pos[2] > (self.cang-2):
                direct = 6
                print("Left")
            elif pos[2] < (self.cang+2):
                direct = 5
                print("Right")
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
            self.rotate_bot_check(abs(angle), self.pos[2])

    # Function Check Rotate Bot
    def rotate_bot_check(self, ang, current):
        """
        This Function is to check the bot that it is rotated 
        to the specified angle or not
        """
        if current in range(ang-5, ang+5):
            print(ang, current)
            print("Angle Obtained")
            self.c = self.c + 1
            if self.c >= 3:
                self.msg.data = [0, 0, 0]
                # self.rotate = 2
                self.flag = 3
                self.choice = 0
                self.first = 1

        else:
            if current <= ang:
                direct = 7
            elif current >= ang:
                direct = 8
            if self.reverse == True:
                if direct == 7:
                    direct = 8
                else:
                    direct = 7
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