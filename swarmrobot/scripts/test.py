#!/usr/bin/env python3
import json
import rospy
from std_msgs.msg import String
from rospy_message_converter import message_converter

def callback(data):
    msg = message_converter.convert_ros_message_to_dictionary(data)
    temp = msg['data']
    bot = json.loads(temp)
    print(bot)

rospy.init_node('node_test', anonymous=True)
sub = rospy.Subscriber('/bot', String, callback, queue_size=1)
rospy.spin()