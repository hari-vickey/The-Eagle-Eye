#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
import sys
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Camera1:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback, queue_size = 1)

        #self.video_writer = None
        #self.file_name = 'data.avi'
        self.location = ['Mumbai', 'Delhi', 'Kolkata', 
                         'Chennai', 'Bengaluru', 'Hyderabad', 
                         'Pune', 'Ahemdabad', 'Jaipur']

        self.marker_count = 0
        self.aruco1, self.aruco2, self.ind1, self.ind2 = 0, 0, 0, 0
        self.temp1, self.temp2, self.temp3, self.temp4 = [], [], [], []
        self.destination, self.inductzone = {}, {}

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        image = cv_image

        # self.aruco_detect_inductpoint(image)
        self.aruco_detect_destination(image)
        # self.aruco_detect_bot(image)

    def aruco_detect_inductpoint(self, frame):
        try:
            if self.aruco2 == 0:

                self.ind2 = 0

                parameters =  cv2.aruco.DetectorParameters_create()

                # Detect the Induct Point markers in the image
                dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
                markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

                for i in markerIds:
                    try:
                        self.temp3.index(i)
                        self.ind2 += 1

                    except ValueError:
                        temp = int(i)
                        self.temp3.append(temp)
                        self.temp4.append(markerCorners[self.ind2].tolist())
                        self.ind2 += 1

                if len(self.temp3) == 2:
                    self.inductzone = self.extract_induct_point(self.temp3, self.temp4)

                    print("Induct Zone Aruco Markers Detected")
                    print(self.inductzone)

                    self.aruco2 = 1

                #cv2.imshow("img", frame)
                #cv2.waitKey(1)

        except Exception as e:
            pass

    def aruco_detect_destination(self, frame):
        try:
            if self.aruco1 == 0:

                self.ind1 = 0
                parameters =  cv2.aruco.DetectorParameters_create()

                # Detect the markers in the image
                dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
                markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
                print("Inside the loop")
                for i in markerIds:
                    try:
                        # self.temp1.index(i)
                        indices = [j for j, x in enumerate(self.temp1) if x == i]
                        if len(indices) == 1:
                            temp = int(i)
                            self.temp1.append(temp)
                            self.temp2.append(markerCorners[self.ind1])    
                        self.ind1 += 1
                    except ValueError:
                        temp = int(i)
                        self.temp1.append(temp)
                        self.temp2.append(markerCorners[self.ind1])
                        self.ind1 += 1

                if len(self.temp2) == 18:
                    self.destination = self.extract_goal_point(self.temp1, self.temp2)
                    print(self.destination)
                    self.aruco1 = 1

                #cv2.imshow("img", frame)
                #cv2.waitKey(1)

        except Exception as e:
            print(e)
            pass

    def aruco_detect_bot(self, frame):
        try:

            parameters =  cv2.aruco.DetectorParameters_create()

            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            converted = np.int_(markerCorners)
            bot_name, points = [], []

            for i in markerIds:
                name = 'bot' + str(i)
                bot_name.append(name)
                pts = [converted[0][0][2].tolist(), converted[0][0][0].tolist()]
                points.append(pts)

            bot = dict(zip(bot_name, points))
            print(bot)

            cv2.imshow("img", frame)
            cv2.waitKey(1)

        except Exception as e:
            # print(e)
            cv2.imshow("img", frame)
            cv2.waitKey(1)
            pass

    def extract_induct_point(self, ids, bbox):
        l_ind = 0
        temp_ls, dest_ls = [], []

        for i in ids:
            name = 'induct_station_' + str(i)
            cen_x = int((bbox[l_ind][0][0][0] + bbox[l_ind][0][2][0])/2)
            cen_y = int((bbox[l_ind][0][0][1] + bbox[l_ind][0][2][1])/2)
            ctp = (cen_x, cen_y)
            temp_ls.append(ctp)
            dest_ls.append(name)
            l_ind += 1

        dest = dict(zip(dest_ls, temp_ls))

        return dest

    def extract_goal_point(self, ids, bbox):
        ind = 0
        ids_ls, cor_ls = [], []
        temp_ls, sum_ls, pts_ls = [], [], []

        for i in markerIds:
            ids_ls.append(ids[ind][0])
            t = markerCorners[ind].tolist()
            cor_ls.append(t[0])
            ind += 1

        for k in range(1, 10):
            try:
                indices = [i for i, x in enumerate(ids_ls) if x == k]
                for l in indices:
                    sum_ls = sum_ls + cor_ls[l]
                temp_ls.append(sum_ls)
                sum_ls = []
            except:
                pass

        for i in range(0,9):
            try:
                x1 = int((temp_ls[i][0][0] + temp_ls[i][2][0])/2)
                y1 = int((temp_ls[i][0][1] + temp_ls[i][2][1])/2)
                x2 = int((temp_ls[i][4][0] + temp_ls[i][6][0])/2)
                y2 = int((temp_ls[i][4][1] + temp_ls[i][6][1])/2)
                i1 = (x1, y2)
                i2 = (x2, y1)
                # cv2.line(frame, (x1, y1), i1, (255, 0, 0), thickness=2)
                # cv2.line(frame, i1, (x2, y2), (255, 0, 0), thickness=2)
                # cv2.line(frame, (x2, y2), i2, (255, 0, 0), thickness=2)
                # cv2.line(frame, i2, (x1, y1), (255, 0, 0), thickness=2)
                pts = [[x1, y1], [x2, y2], i1, i2]
                pts_ls.append(pts)

        dest = dict(zip(self.location, pts_ls))

        return dest

    def generate_imaginary_points(self, frame, dest):
        pts_ls = []

        for i in range(0, 9): 
            v1 = dest[place[i]]
            int1 = (v1[0][0], v1[1][1])
            int2 = (v1[1][0], v1[0][1])

            # cv2.line(frame, v1[0], int1, (255, 0, 0), thickness=2)
            # cv2.line(frame, int1, v1[1], (255, 0, 0), thickness=2)
            # cv2.line(frame, v1[1], int2, (255, 0, 0), thickness=2)
            # cv2.line(frame, int2, v1[0], (255, 0, 0), thickness=2)

            pts = [v1[0], int1, int2, v1[1]]
            pts_ls.append(pts)

        return frame, pts_ls

    def closest_point(self, ls, pos):
        x, y = pt
        idx = -1
        smallest = float("inf")

        for p in pts:

            if p[0] == x or p[1] == y:
                dist = abs(x - p[0]) + abs(y - p[1])

            if dist < smallest:
                idx = pts.index(p)
                smallest = dist

            elif dist == smallest:

                if pts.index(p) < idx:
                    idx = pts.index(p)
                    smallest = dist

        return idx

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