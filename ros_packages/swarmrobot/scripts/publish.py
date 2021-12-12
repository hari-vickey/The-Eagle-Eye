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
    msg1.data = [direct, angle, servo]
    msg2.data = [direct, angle, servo]
    msg3.data = [direct, angle, servo]
    msg4.data = [direct, angle, servo]
    pub1.publish(msg1)
    pub2.publish(msg2)
    pub3.publish(msg3)
    pub4.publish(msg4)
    rospy.logwarn("Shutdown Request Received")

rospy.init_node("Data_Publisher_Node")
rospy.loginfo("Data Publisher Node")
pub1 = rospy.Publisher("bot1/control_signal", Int16MultiArray, queue_size=1)
pub3 = rospy.Publisher("bot3/control_signal", Int16MultiArray, queue_size=1)
pub4 = rospy.Publisher("bot4/control_signal", Int16MultiArray, queue_size=1)
pub2 = rospy.Publisher("bot2/control_signal", Int16MultiArray, queue_size=1)
msg1 = Int16MultiArray()
msg2 = Int16MultiArray()
msg3 = Int16MultiArray()
msg4 = Int16MultiArray()

rospy.loginfo("Publishing to the topic: bot1/control_signal")
rospy.loginfo("Publishing to the topic: bot2/control_signal")
rospy.loginfo("Publishing to the topic: bot3/control_signal")
rospy.loginfo("Publishing to the topic: bot4/control_signal")

try:
    while not rospy.is_shutdown():
        direct = int(input("Enter Direction : "))
        angle = int(input(" Enter Angle : "))
        servo = int(input("Enter Servo : "))
        # direct, angle, servo = 1, 0, 0
        msg1.data = [direct, angle, servo]
        msg2.data = [direct, angle, servo]
        msg3.data = [direct, angle, servo]
        msg4.data = [direct, angle, servo]
        pub1.publish(msg1)
        pub2.publish(msg2)
        pub3.publish(msg3)
        pub4.publish(msg4)

except Exception as e:
    destruct()