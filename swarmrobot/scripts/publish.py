#!/usr/bin/env python3
import json
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from rospy_message_converter import message_converter
def destruct():
    direct = 0
    angle = 0
    servo = 0
    msg.data = [direct, angle, servo]
    # print(msg)
    pubs.publish(msg)
    rospy.logwarn("Shutdown Request Received")

rospy.init_node("Data_Publisher_Node")
rospy.loginfo("Data Publisher Node")
pubs = rospy.Publisher("bot2/control_signal", Int16MultiArray, queue_size=1)
# pubd = rospy.Publisher("/bot", String, queue_size=1)
msg = Int16MultiArray()
# st = String()
rospy.loginfo("Publishing to the topic: /control_signal")
# des = {'bot1': (1024, 431)}
# st = json.dumps(des)
# st = message_converter.convert_dictionary_to_ros_message('std_msgs/String', des)
# print(msg.data[0])
try:
    while not rospy.is_shutdown():
        direct = int(input("Enter Direction : "))
        angle = int(input(" Enter Angle : "))
        servo = int(input("Enter Servo : "))
        if direct == 1:
            for i in range(0, 10):
                msg.data = [direct, angle, servo]
                pubs.publish(msg)
            msg.data = [0, 0, 0]
            pubs.publish(msg)
        else:
            msg.data = [direct, angle, servo]
            pubs.publish(msg)

except KeyboardInterrupt:
    destruct()