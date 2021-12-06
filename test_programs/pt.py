import time
import math
import cv2
import numpy as np
import cv2.aruco as aruco
from heapq import heappush, heappop
from helper import function

def aruco_detect_inductpoint(frame):

    inductzone = {}
    parameters =  cv2.aruco.DetectorParameters_create()
    # Detect the Induct Point markers in the image
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    frame = aruco.drawDetectedMarkers(frame, markerCorners)
    inductzone = extract_induct_point(markerIds, markerCorners)

    return frame, inductzone

# Function to Extract Induct Points
def extract_induct_point(ids, bbox):
    """
    Extract the induct points from the list and convert
    it to dictionary and returns it.
    """
    l_ind = 0
    temp_ls, dest_ls = [], []

    for i in ids:
        name = int(i)
        cen_x = int((bbox[l_ind][0][0][0] + bbox[l_ind][0][2][0])/2)
        cen_y = int((bbox[l_ind][0][0][1] + bbox[l_ind][0][2][1])/2)
        ctp = (cen_x-60, cen_y)
        temp_ls.append(ctp)
        dest_ls.append(name)
        l_ind += 1

    dest = dict(zip(dest_ls, temp_ls))

    return dest

def get_destination(frame):
    parameters =  cv2.aruco.DetectorParameters_create()
    # Detect the markers in the image
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

    frame = aruco.drawDetectedMarkers(frame, markerCorners)

    ind = 0
    ids_ls, cor_ls = [], []
    dic = {}

    for i in markerIds:
        ids_ls.append(markerIds[ind][0])
        t = markerCorners[ind].tolist()
        cor_ls.append(t[0])
        ind += 1

    temp_ls, sum_ls = [], []
    for k in range(1, 10):
        try:
            indices = [i for i, x in enumerate(ids_ls) if x == k]
            for l in indices:
                sum_ls = sum_ls + cor_ls[l]
            temp_ls.append(sum_ls)
            sum_ls = []
        except:
            pass

    pts_ls = []
    for i in range(0,9):
        try:
            x1 = int((temp_ls[i][0][0] + temp_ls[i][2][0])/2)
            y1 = int((temp_ls[i][0][1] + temp_ls[i][2][1])/2)
            x2 = int((temp_ls[i][4][0] + temp_ls[i][6][0])/2)
            y2 = int((temp_ls[i][4][1] + temp_ls[i][6][1])/2)
            
            if( y1<y2 ):
                x1,x2 = x2,x1
                y1,y2 = y2,y1

            d1 = (x1, y1)
            d2 = (x2, y2)
            i1 = (x1, y2)
            i2 = (x2, y1)
            
            cv2.line(frame, d1, i1, (0, 0, 0), thickness=2)
            cv2.line(frame, i1, d2, (0, 0, 0), thickness=2)
            cv2.line(frame, d2, i2, (0, 0, 0), thickness=2)
            cv2.line(frame, i2, d1, (0, 0, 0), thickness=2)

            function.render_graph((x1+25, y1-25), (x2-25, y2+25))

            pts = [d1, d2, i1, i2]
            pts_ls.append(pts)

        except Exception as e:
            # print(e)
            pass
    
    dest = dict(zip(place, pts_ls))

    return frame, dest

def closest_point(pts, pt):
    x, y = pt
    d = list(map(lambda t: math.sqrt(pow(t[0]-x,2)+pow(t[1]-y,2)),pts))
    min_res = min(d)
    i_m = d.index(min_res)
    return pts[i_m]

# Function to detect aruco markers Bot
def aruco_detect_bot(frame):
    """
    This function will detect the aruco markers of the bot
    using dictionary of size 4x4 50
    """
    try:
        # Creating Detector Parameters
        parameters =  cv2.aruco.DetectorParameters_create()
        # Detect the markers in the image
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
        frame = aruco.drawDetectedMarkers(frame, markerCorners)
        converted = np.int_(markerCorners)
        bot_name, points, cpts = [], [], []
        l = 0

        for i in markerIds:
            name = 'bot' + str(int(i))
            bot_name.append(name)
            pts = [converted[l][0][2].tolist(), 
                   converted[l][0][0].tolist(), 
                   converted[l][0][1].tolist(), 
                   converted[l][0][3].tolist()]
            points.append(pts)
            l += 1

        bot = dict(zip(bot_name, points))
        # print(bot)
        for i in bot:
            x1 = int((bot[i][0][0]+bot[i][1][0])/2)
            y1 = int((bot[i][0][1]+bot[i][1][1])/2)
            deg = bot_angle(bot[i][0], bot[i][3])
            point = (x1, y1, int(deg))
            cpts.append(point)

        bot = dict(zip(bot_name, cpts))
        print(bot)

    except Exception as e:
        pass

# Function to Angle of the Bot
def bot_angle(pt1, pt2):
    """
    This function is to calculate the angle of the bot
    using the third and four point of the aruco marker
    """
    x = abs(pt2[0] - pt1[0])
    y = abs(pt2[1] - pt1[1])
    rad = math.atan(y/x)
    d = rad *(180/(math.pi))
    deg = 90 - d
    return deg

place = ['Mumbai', 'Delhi', 'Kolkata', 
         'Chennai', 'Bengaluru', 'Hyderabad', 
         'Pune', 'Ahemdabad', 'Jaipur']

# graph = {}

frame = cv2.imread("images/img.png")

function.init_graph(720, 1280)

# frame, destination = get_destination(frame)
# print('\033[92m' + "Destination Markers Detected" + '\033[92m')
# print(destination)

ang = aruco_detect_bot(frame)
# frame, inductzone = aruco_detect_inductpoint(frame)
# print('\033[93m' + "Induct Zone Aruco Markers Detected" + '\033[93m')
# print(inductzone)

# start = inductzone[1]
# goal = closest_point(destination['Jaipur'], start)
# path, angle = function.path_plan(start, goal)
# frame = function.mark_points(frame, start, goal, path)
# print(path)
# print(angle)

# start = inductzone[2]
# goal = closest_point(destination['Delhi'], start)
# path, angle = function.path_plan(graph, start, goal)
# frame = function.mark_points(frame, start, goal, path)
# print(path)
# print(angle)

# print('\033[94m' + "Induct Station 1 - Destination" + '\033[0m')
# start = inductzone[1]
# for i in destination:
#     goal = closest_point(destination[i], start)
#     print(i, start, goal)
#     path, angle = function.path_plan(start, goal)
#     frame = function.mark_points(frame, start, goal, path)
#     print(path)
#     print(angle)

# print('\033[95m'"Induct Station 2 - Induct Station" + '\033[0m')
# start = inductzone[2]
# for i in destination:
#     goal = closest_point(destination[i], start)
#     print(i, start, goal)
#     path, angle = function.path_plan(graph, start, goal)
#     frame = function.mark_points(frame, start, goal, path)
#     print(path)
#     print(angle)

cv2.imshow("frame", frame)
cv2.waitKey()