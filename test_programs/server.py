#!/usr/bin/env python3
"""
Flipkart - Grid 3.0 - Robotics Competition
ROS Node - server.py - Stage1 Code
This node will do the following :
    1. Subscribe to the images published by the ros
    2. Create a graph of obstacles to avoid collision with them
    3. Detect the bots using tensorflow
    4. Decode the aruco marker in the arena to get the destination points
    5. Check the current state of the bot and goal of the bot to be reached is
       not inside the obstacle
    6. Plan a path using a custom planner or A* Algorithm
    7. Publish the commands to move the bot using real time comparison between
       goal and the tensorflow ouput
    8. Loops 6 and 7 to complete the task

ROS Publications                            ROS Subscriptions

    bot1/servo_control                          /image_raw
    bot1/direction_control
    bot2/servo_control
    bot2/direction_control
    bot3/servo_control
    bot3/direction_control
    bot4/servo_control
    bot4/direction_control

Yet to Complete:

  1.  Verify the working in real time
  2.  Think of rotating the bot in precise way
  3.  How to avoid dynamic obstacles
        a. Replanning Multiple Times
        b. Having a subset of goals
  4.  Publish Commands for A star Algorithm

"""
################## Code V5 - Complete Code for Task Execution ##################

# Importing Required Modules
import time
import math
import threading
from heapq import heappush, heappop
import cv2
import cv2.aruco as aruco
import rospy
import rospkg
import numpy as np
import tensorflow as tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Class Server
class Server():

    """
    This class has all the required functions to complete the task as
    mentioned in the doc string of this file
    """

    # Constructor
    # Initializing the variables of this class
    def __init__(self):

        # Creating a object to CvBridge() class
        self.bridge = CvBridge()

        # Creating Variables to store necessary values
        self.bot_name = ('bot1', 'bot2', 'bot3', 'bot4')
        self.location = ['Mumbai', 'Delhi', 'Kolkata', 
                         'Chennai', 'Bengaluru', 'Hyderabad', 
                         'Pune', 'Ahemdabad', 'Jaipur']

        # Global Varibales for Aruco Marker Detections
                self.destination, self.inductzone = {}, {}
        self.aruco1, self.ind1, self.aruco2, self.ind2 = 0, 0, 0, 0
        self.temp1, self.temp2, self.temp3, self.temp4 = [], [], [], [], []

        # Global Variables for Graph Creation
        self.graph = {}
        self.bot_obs, self.points = [], []

        # Global Varibales for Execution
        self.n, self.rotate = 0, 0
        self.flag1, self.flag2, self.flag3 = 0, 0, 0
        self.goal, self.goalr, self.pnt = (0, 0), (0, 0), (0, 0)
        self.exec, self.path, self.reverse, self.graphc = 0, 1, 0, 1

        # Subscribing to the ROS Image topic
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback,
                                          queue_size = 1)

        # Reading data from Config_iot.yaml file using ROS Parameter server
        # print("Starting Mqtt Server")
        # param_config_iot = rospy.get_param('config_iot')
        # self.mqtt_server_url = param_config_iot['mqtt']['server_url']
        # self.mqtt_server_port = param_config_iot['mqtt']['server_port']
        # self.mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        # self.mqtt_qos = param_config_iot['mqtt']['qos']
        # print("Mqtt Thread has been Started")

    # Function for ROS Camera Subscription Callback
    def callback(self,data):
        """
        This functions gets all the image published in the subscribed topic and
        sends the image to the next function to process them
        """
        try:
            # Convert img msg to an data readable by cv2 library,
            # to process them further
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Execute the aruco_detection_destination untill it detects,
            # all the destination points
            if self.aruco1 == 0:
                self.aruco_detect_destination(cv_image)

            # Execute the aruco_detection_inductstation untill it detects,
            # all the inductstation points
            elif self.aruco2 == 0:
                self.aruco_detect_inductstation(cv_image)

            # Execute the arena_config function only one time
            elif self.graphc == 1:
                self.arena_config(cv_image)
                self.graphc = 0

            if self.aruco1 == 1 and self.aruco2 == 1 and self.graphc == 0:
                # Threaded the algorithm function to process real time images
                thread = threading.Thread(name="worker", target=self.algorithm, args=(cv_image, ))
                thread.start()
                #self.algorithm(cv_image)

            # img, co = self.detect_bot(cv_image)
            # print(co)
            # cv2.imshow("img", img)
            # cv2.waitKey(1)

        except Exception as e:
            print(e)
            pass

    # Function to have spatial awareness of the arena
    def arena_config(self, image):
        """
        This function defines the bot radius and calls the next function to
        create the arena as graph and generates the points of the obstacle
        """
        radius, clearance = 0, 0
        height, width, _ = image.shape
        self.g = self.create_graph(image, clearance, radius, height, width)
        self.points = [x for x in self.g.keys() if not (self.g[x]['valid'])]
        print("Completed Graph Plotting")

    # Function to detect aruco Markers of Destination Points
    def aruco_detect_destination(self, frame):
        """
        This Function is responsible to detect all the destination markers
        to identify all possible location around the chute.
        """
        try:
            self.ind1 = 0
            parameters =  cv2.aruco.DetectorParameters_create()

            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            frame = aruco.drawDetectedMarkers(frame, markerCorners)

            for i in markerIds:
                indices = [j for j, x in enumerate(self.temp1) if x == i]
                if len(indices) == 1:
                    temp = int(i)
                    self.temp1.append(temp)
                    self.temp2.append(markerCorners[self.ind1])
                elif len(indices) == 0:
                    temp = int(i)
                    self.temp1.append(temp)
                    self.temp2.append(markerCorners[self.ind1])
                self.ind1 += 1

            if len(self.temp2) == 18:
                self.destination = self.extract_goal_point(frame, self.temp1, self.temp2)
                print(self.destination)
                self.aruco1 = 1
                cv2.imshow("img", frame)
                cv2.waitKey(1)

        except Exception as e:
            pass

    # Function to Detect bots using aruco markers
    def aruco_detect_bot(self, frame):
        """
        This Function is to detect the bots in order to guide them.
        """
        try:
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            frame = aruco.drawDetectedMarkers(frame, markerCorners)

            converted = np.int_(markerCorners)
            bot_name, points = [], []

            for i in markerIds:
                name = 'bot' + str(i)
                bot_name.append(name)
                pts = [converted[0][0][2].tolist(), converted[0][0][0].tolist()]
                points.append(pts)

            bot = dict(zip(bot_name, points))
            return frame, bot

        except Exception:
            pass

    # Function to Extract Goal Points
    def extract_goal_point(self, frame, ids, bbox):
        """
        This function will extract the co-ords of the goal point and
        returns a dictionary
        """
        ind = 0
        ids_ls, cor_ls = [], []
        temp_ls, sum_ls, pts_ls = [], [], []

        for i in ids:
            ids_ls.append(ids[ind])
            t = bbox[ind].tolist()
            cor_ls.append(t[0])
            ind += 1

        for k in range(1, 10):
            indices = [i for i, x in enumerate(ids_ls) if x == k]
            for l in indices:
                sum_ls = sum_ls + cor_ls[l]
            temp_ls.append(sum_ls)
            sum_ls = []

        for i in range(0,9):
            x1 = int((temp_ls[i][0][0] + temp_ls[i][2][0])/2)
            y1 = int((temp_ls[i][0][1] + temp_ls[i][2][1])/2)
            x2 = int((temp_ls[i][4][0] + temp_ls[i][6][0])/2)
            y2 = int((temp_ls[i][4][1] + temp_ls[i][6][1])/2)
            i1 = (x1, y2)
            i2 = (x2, y1)
            cv2.line(frame, (x1, y1), i1, (255, 0, 0), thickness=2)
            cv2.line(frame, i1, (x2, y2), (255, 0, 0), thickness=2)
            cv2.line(frame, (x2, y2), i2, (255, 0, 0), thickness=2)
            cv2.line(frame, i2, (x1, y1), (255, 0, 0), thickness=2)
            pts = [(x1, y1), (x2, y2), i1, i2]
            pts_ls.append(pts)

        dest = dict(zip(self.location, pts_ls))

        return dest

    # Function to Extract Induct Points
    def extract_induct_point(self, ids, bbox):
        """
        Extract the induct points from the list and convert
        it to dictionary and returns it.
        """
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

    # Function to identify the closest point
    def closest_point(self, ls, pos):
        """
        This Function will choose the closest point of the destination
        from the current position of the bot.
        """
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

    # Function to create Graph
    def create_graph(self, image, clearance, radius, height, width):
        """
        This function defines the obstacles with the bot radius and
        also generates the obstacle map and path map
        """
        graph = {}
        # Work here
        pt1, pt2 = self.get_obstacle(image, height, width)

        for i in range(width):
            for j in range(height):
                left_obs = (i-pt1[1],pt1[0]-j)
                right_obs = (pt2[1]-i,pt2[0]-j)

                aug_left_obs = [(i-radius-clearance) - pt1[1], (pt1[0]) - (j+radius+clearance)]
                aug_right_obs = [pt2[1]-(i+radius+clearance), (pt2[0]) - (j+radius+clearance)]

                graph[(i,j)] = {'visited':False, 'distance':np.inf, 'valid':True, 'parent': (0, 0), 'id':'blank'}
                if left_obs[0]<=0 and left_obs[1]<=0:
                    graph[(i,j)]['valid'] = False
                    graph[(i,j)]['id'] = 'obs'
                elif aug_left_obs[0]<=0 and aug_left_obs[1]<=0:
                    graph[(i,j)]['valid'] = False
                    graph[(i,j)]['id'] = 'aug'
                elif right_obs[0]<=0 and right_obs[1]<=0:
                    graph[(i,j)]['valid'] = False
                    graph[(i,j)]['id'] = 'obs'
                elif aug_right_obs[0]<=0 and aug_right_obs[1]<=0:
                    graph[(i,j)]['valid'] = False
                    graph[(i,j)]['id'] = 'aug'

        return graph

    # Function to add bots as dynamic obstacles
    def create_clear_bot_in_graph(self, graph, bot_des, exempt_bot_name):
        """
        This function will add the current bot position to base graph
        to recreate the path planned.
        """
        for i in self.bot_obs:
            min1, max1 = int(min(i[0][0],i[1][0])), int(max(i[0][0],i[1][0]))
            min2, max2 = int(min(i[0][1],i[1][1])), int(max(i[0][1],i[1][1]))
            for j in range(min1,max1):
                for k in range(min2,max2):
                    graph[(j,k)]['valid'] = True
                    graph[(j,k)]['id'] = 'blank'
        self.bot_obs.clear()

        for i in bot_des:
            if i != exempt_bot_name:
                z=bot_des[i]
                min1, max1 = int(min(z[0][0],z[1][0])), int(max(z[0][0],z[1][0]))
                min2, max2 = int(min(z[0][1],z[1][1])), int(max(z[0][1],z[1][1]))
                for j in range(min1,max1):
                    for k in range(min2,max2):
                        graph[(j,k)]['valid'] = False
                        graph[(j,k)]['id'] = 'obs'
                self.bot_obs.append(z)

        return graph

    # Function Algorithm
    def algorithm(self, image):
        """
        This function is responsible to complete the entire task.
        It is also combined with multiple control statements to
        get optimal results
        """
        if self.n >= 4:
                rospy.loginfo('\033[94m' + "Run is Complete" + '\033[0m')
                rospy.loginfo('\033[92m' + "Thank You" + '\033[0m')
                return

        try:
            # Detecting the bot
            img, coor = self.aruco_detect_bot(image)
            # print(coor)
            # Get the center point of the bot
            bot = self.bot_name[self.n]
            current = self.center_point(coor, bot)

            if self.exec == 0:
                if self.reverse == 0:
                    self.goalr = current
                    self.reverse = 1

                dest = self.dest_name[self.n]
                self.goal = (self.destination[dest][0], self.destination[dest][1])
                print("Current" + str(current), "Goal" + str(self.goal))
                self.flag1 = self.validate_point(current, self.points)
                self.flag2 = self.validate_point(self.goal, self.points)

                if  self.flag1 == 1 and self.flag2 == 1:
                    if self.reverse == 2:
                        self.goal = self.goalr
                    print("Path Plan")
                    img, self.pnt = self.path_plan_custom(img, current, self.goal)
                    self.flag3 = self.validate_point(self.pnt, self.points)

                    if self.flag3 == 0:
                        self.path_plan(img, current, self.goal)

                    self.exec = 1

            if  self.flag1 == 1 and self.flag2 == 1:

                img, pnt = self.path_plan_custom(img, current, self.goal)

                if self.path == 1:
                    self.publish_command_direction(bot, pnt, current)

                elif self.path == 2:
                    self.publish_command_direction(bot, self.goal, current)

            elif self.flag1 == 0 or self.flag2 == 0:
                rospy.logwarn("Current Point or Goal Point is inside the obstacle Retrying..")

            # Re-Initializing all the values,
            # On each bot completing it's task
            if self.path == 2:

                if current[0] in range(self.goalr[0]-20, self.goalr[0]+20):
                    if current[1] in range(self.goalr[1]-20, self.goalr[1]+20):
                        self.exec, self.reverse, self.rotate, self.path = 0, 0, 0, 1
                        self.n += 1

            cv2.imshow("result", img)
            cv2.waitKey(1)

        except Exception:
            cv2.imshow("result", image)
            cv2.waitKey(1)
            pass

    # Function Center Point
    def center_point(self, cord, bot):
        """
        This function will use the upper and lower boundary
        of the bbox to get the center point
        """
        x1 = int((cord[bot][0][0]+cord[bot][1][0])/2)
        y1 = int((cord[bot][0][1]+cord[bot][1][1])/2)
        point = (x1, y1)

        return point

    # Function to validate point
    def validate_point(self, coord, points):
        """
        This function will validate the point with obstacles of
        its presence and returns the result as 1 or 0
        """
        try:
            points.index(coord)
            flag = 0

        except ValueError:
            flag = 1

        return flag

    # Function for Custom Path Planning
    def path_plan_custom(self, img, start, end):
        """
        This function will generate the custom path using the
        start and end points
        Also draw the lines of the path estimated
        """
        # If the goal is collinear with the current axis is
        # then there is no need of waypoint
        if start[0] == end[0] or start[1] == end[1]:
            way_point = (end[0],end[1])
            cv2.line(img, start, end, (255, 0, 0), cv2.LINE_4, 1)

        # If the start or goal point is not in the same axis,
        # then resolving the path to horizontal and vertical paths
        elif start[0] != end[0] or start[1] != end[1]:
            way_point = (start[0], end[1])

            if self.reverse == 2:
                way_point = (end[0], start[1])
            cv2.line(img, start, way_point, (255, 0, 0), cv2.LINE_4, 1)
            cv2.line(img, way_point, end, (255, 0, 0), cv2.LINE_4, 1)

        return img, way_point

    # Function to plan a Path
    def path_plan(self, img, start, goal):

        # Marking the Start Point and Goal point
        img = cv2.circle(img, start, 2, (0, 255, 0), 8)
        img = cv2.circle(img, goal, 2, (0, 0, 255), 8)

        # Calling Astar Algorithm to compute the shortest path
        min_distance, path = self.astar(self.g, start, goal)
        path = np.array(path)

        # Defining Constant
        k=1

        # Marking the Minimized set of goalpoints
        for n in path:
            if k%4==0:
                image = cv2.circle(img, tuple(n), 2, (255, 0, 255), 4)
            k+=1

    # Function For A-star Algorithm
    def astar(self, graph, source, goal):
        """
        This function will generate the path using the graph generated.
        Returns the minimum path and number of nodes visited.
        """
        count = 0
        row = []
        (goal_x, goal_y) = goal

        graph[source]['visited'] = True
        num_nodes_visited = 1
        graph[source]['distance'] = 0

        queue = []
        queue_distance = self.calculate_distance(goal, source)+graph[source]['distance']
        heappush(queue, (queue_distance, source))

        while (len(queue) != 0):

            current = heappop(queue)[1]
            if current[0] <= goal[0]+5 and current[0] >= goal[0] and current[1] <= goal[1]+5 and current[1] >= goal[1] :
                print("Goal reached")
                (goal_x,goal_y)=(current[0],current[1])

                if row:
                    x,y = zip(*row)
                break

            for i in [-5, 0, 5]:
                for j in [-5, 0, 5]:
                    if i != 0 or j != 0:
                        neighbour = (abs(current[0]+i), abs(current[1]+j))
                        lst = list(neighbour)

                        if lst[0] >=1280:
                            lst[0] = 1279
                        if lst[1] >=720:
                            lst[1] = 719

                        neighbour = tuple(lst)
                        if graph[neighbour]['valid'] == True:

                            if abs(i)+abs(j) == 2:
                                distance = math.sqrt(2)

                            else:
                                distance = 1

                            if graph[neighbour]['visited'] == False:
                                graph[neighbour]['visited'] = True
                                row.append([abs(current[0]+i), abs(current[1]+j)])
                                x,y = zip(*row)

                                num_nodes_visited += 1

                                graph[neighbour]['parent'] = current
                                graph[neighbour]['distance'] = graph[current]['distance'] + distance

                                queue_distance = self.calculate_distance(goal, neighbour)+graph[neighbour]['distance']
                                heappush(queue, (queue_distance, neighbour))

        path = [(goal_x, goal_y)]
        parent = (goal_x, goal_y)

        while parent != source:
            parent = graph[path[len(path)-1]]['parent']
            path.append(parent)

        min_distance = (graph[(goal_x,goal_y)]['distance'])
        print("Total Number of Nodes Visited:", num_nodes_visited)

        return(min_distance, path)

    # Function to calculate distance
    def calculate_distance(self, goal, current):
        """
        This function will calculate the distance between the current and
        goal point
        """
        d = math.sqrt(((goal[0]-current[0])*(goal[0]-current[0]))+((goal[1]-current[1])*(goal[1]-current[1])))

        return d

    # Function to Publish direction command
    def publish_command_direction(self, bot, goal, cur):
        """
        This function will publish command as direction forward
        based on certain goal commands
        """

        # Declaring Variables with Initial Value
        x_follow, y_follow, rotate, value = 0, 0, 0, 1

        # Choosing the Axis to make the bot to follow in one direction
        # This is done by comparing the current position with goal position
        if cur[0] in range((goal[0]-30), (goal[0]+30)):
            y_follow = 1
            # On condition of the following axis
            # Subtracting x co-ordinates of the bot with goal,
            # To get the direction of rotatation of the bot
            # If angle is positive then rotate clockwise
            # else rotate anti-clockwise
            value = cur[0] - self.goal[0]

        elif cur[1] in range((goal[1]-30), (goal[1]+30)):
            x_follow = 1
            # On condition of the following axis
            # Subtracting y co-ordinates of the bot with goal,
            # To get the direction of rotatation of the bot
            # If angle is positive then rotate anti-clockwise
            # else rotate clockwise
            value = self.goal[1] - cur[1]

        # Vertical Movement
        # Comparing the current y co-ordinates of the bot with goal
        # If the current position is in the range of the goal position or
        # withing the limit then, Stopping the Bot
        # Note: Value=30 is the tolerance added to compensate the tf delay
        if cur[1] in range((goal[1]-30), (goal[1]+30)) and y_follow == 1:
            direct = 0
            print("Stop")
            msg = bot[4] + str(direct) + "0"
            _ = self.mqtt_publish(str(msg))
            rotate = 1

        # If the bot is not in the range of the goal
        # then Bot is moved forward
        # Note: Value=30 is the tolerance added to compensate the tf delay
        elif cur[1] not in range((goal[1]), (goal[1])) and y_follow == 1:
            direct = 1
            msg = bot[4] + str(direct) + "0"
            _ = self.mqtt_publish(str(msg))

            # If the bot deviates in x direction while following y-axis
            # then rotate the bot in opposite direction
            if cur[0] != self.pnt[0]:
                print("Monitoring Deviation")
                if cur[0] <= self.pnt[0]:
                    print("Fine Right")
                    direct = 5
                elif cur[0] >= self.pnt[0]:
                    print("Fine Left")
                    direct = 6
                if self.reverse == 2:
                    if cur[0] <= self.pnt[0]:
                        direct = 6
                    elif cur[0] >= self.pnt[0]:
                        direct = 5
                msg = bot[4] + str(direct) + "0"
                _ = self.mqtt_publish(str(msg))


        # Horizontal Movement
        # Comparing the current x co-ordinates of the bot with goal
        # If the current position is in the range of the goal position or
        # withing the limit then, Stopping the Bot
        # Note: Value=30 is the tolerance added to compensate the tf delay
        if cur[0] in range((goal[0]-30), (goal[0]+30)) and x_follow == 1:
            direct = 0
            msg = bot[4] + str(direct) + "0"
            _ = self.mqtt_publish(str(msg))
            rotate = 1

        # If the bot is not in the range of the goal
        # then Bot is moved forward
        # Note: Value=30 is the tolerance added to compensate the tf delay
        elif cur[0] not in range((goal[0]), (goal[0])) and x_follow == 1:
            direct = 1
            msg = bot[4] + str(direct) + "0"
            _ = self.mqtt_publish(str(msg))

            # If the bot deviates in x direction while following y-axis
            # then rotate the bot in opposite direction
            if cur[1] != self.pnt[1]:
                if cur[1] <= self.pnt[1]:
                    print("Fine Right")
                    direct = 5
                elif cur[1] >= self.pnt[1]:
                    print("Fine Left")
                    direct = 6
                if self.reverse == 2:
                    if cur[1] <= self.pnt[1]:
                        direct = 6
                    elif cur[1] >= self.pnt[1]:
                        direct = 5
                msg = bot[4] + str(direct) + "0"
                _ = self.mqtt_publish(str(msg))

        self.publish_command_rotation(bot, value, rotate)

    # Function to publish rotational command
    def publish_command_rotation(self, bot, val, rot):
        """
        This functions get the angle and publish the commands to
        rotate the bot accordingly
        """

        # Defining Constants
        # 83.33
        angular_speed = 1

        # If the value is less than 0, then rotate clockwise
        # else rotate anti-clockwise
        if val<=0:
            direct = 2
        else:
            direct = 3

        # If rotate is made to 1 then rotate the bot
        # Based On the path that the bot is executing
        # added +55 degree because the base link,
        # Not at exact center of the bot
        if rot == 1:
            if self.path == 1:
                msg = bot[4] + str(direct) + "0"
                self.mqtt_publish(str(msg))
                self.path = 2
            elif self.path == 2 and self.rotate == 0:
                #self.actuate_servo(bot)
                direct = 7
                msg = bot[4] + str(direct) + "0"
                self.mqtt_publish(str(msg))
                servo = 1
                msg = bot[4] + str(servo) + "0"
                self.mqtt_publish(str(msg))
                self.exec, self.path, self.reverse = 0, 1, 2
                self.rotate = 1

            # Stop the rotation
            direct = 0
            msg = bot[4] + str(direct) + "0"
            self.mqtt_publish(str(msg))
            servo = 0
            msg = bot[4] + str(servo) + "0"
            self.mqtt_publish(str(msg))

    # Function to actuate servo motor
    def actuate_servo(self, bot):

        # Creating a ROS Publisher
        topic = bot + '/servo_control'
        servo = rospy.Publisher(topic, Int16, queue_size=1)
        value = Int16()

        # Defining value for servo actuatuion,
        # To drop the package from bot
        value = 0
        servo.publish(value)

        # Wait till the package is dropped
        #rospy.sleep(2)

        # Defined angle as 180 to actuate the
        # servo back to initial position
        value = 1
        servo.publish(value)

    # Destructor of the Class
    def __del__(self):
        rospy.loginfo('\033[94m' + "Shutting Down" + '\033[0m')

# Main Function
def main():
    """
    This is the start of execution of this node
    """
    # Initializing the Node
    rospy.init_node('stage_1_server_node', anonymous=True)

    # Creating Object for the Class Server
    ser = Server()

    try:
        # Spinning the Main Function
        rospy.spin()

    # Deleting the Created Object if Interrupted
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        del ser
        # Destroying all the cv2 Windows Created
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()