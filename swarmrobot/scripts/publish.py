#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray
def destruct():
    rospy.logwarn("Shutdown Request Received")

rospy.init_node("Data_Publisher_Node")
rospy.loginfo("Data Publisher Node")
pubs = rospy.Publisher("/control_signal", Int16MultiArray, queue_size=1)
msg = Int16MultiArray()
rospy.loginfo("Publishing to the topic: /control_signal")

# print(msg.data[0])
try:
    while not rospy.is_shutdown():
        direct = int(input("Enter Direction : "))
        angle = int(input(" Enter Angle : "))
        servo = int(input("Enter Servo : "))
        msg.data = [direct, angle, servo]
        print(msg)
        pubs.publish(msg)
except KeyboardInterrupt:
    destruct()