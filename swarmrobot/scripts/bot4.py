#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def destruct():
    rospy.logwarn("Shutdown Request Received")
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    value.data = 0.0
    pub1.publish(twist)
    pub2.publish(value)
rospy.init_node("swarm_bot_4_node")
pub1 = rospy.Publisher("/bot_4/cmd_vel", Twist, queue_size=10)
pub2 = rospy.Publisher("/bot_4/bot_4_controller/command", Float64, queue_size=10)
rate = rospy.Rate(1)
value = Float64()
twist = Twist()
try:
    rospy.on_shutdown(destruct)
    while not rospy.is_shutdown():
        arg = input('1. Actuate Servo\n2. Move the Bot\nEnter a Number: ')
        if arg == '1':
            print("To block the package Enter 1 to retrive Enter 2 : ")
            mv = input()
            if mv == '1':
                value.data = 3.5
            else:
                value.data = 0
            pub2.publish(value)
            rate.sleep()
        elif arg == '2':
            print("Example to see ")
            ip1 = input('Linear Velocity : ')
            ip2 = input('Angular Velocity : ')
            twist.linear.x = float(ip1)
            twist.angular.z = float(ip2)
            pub1.publish(twist)
            rate.sleep()
        rospy.on_shutdown(destruct)
except KeyboardInterrupt:
    pass