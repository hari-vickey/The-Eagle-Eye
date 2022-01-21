#!/usr/bin/env python3
import sys
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Camera1:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        image = cv_image
        image = image[0:720, 115:1100]
        cv2.imshow("live", image)
        # cv2.imwrite("img.png", image)
        cv2.waitKey(1)

def main(args):
  
    rospy.init_node('node_eg1_read_camera', anonymous=True)
    ic = Camera1()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        rospy.on_shutdown(ic.clean_shutdown)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)