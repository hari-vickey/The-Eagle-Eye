#!/usr/bin/env python3
"""
Flipkart - Grid 3.0 - Robotics Competition
ROS Node - server.py (yet to be completed)
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

    /servo_control                              /usb_camera/image_raw
    /direction_control
"""
##################            Code V1 - Stage 1                #################
# Importing Required Modules
import math
import threading
from heapq import heappush, heappop
import cv2
import cv2.aruco as aruco
import rospy
import rospkg
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge, CvBridgeError
from rospy_message_converter import message_converter

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
        self.exec, self.reverse, self.graphc, self.path = 0, 0, 0, 1

        # Publishing Bot Positions
        self.publisher = rospy.Publisher("/bot_position", String, queue_size=1)
        # Subscribing to the ROS Image topic
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback,
                                          queue_size = 1)

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

            if self.aruco1 == 1 and self.aruco2 == 1 and self.graphc == 1:
                # Threaded the algorithm function to process real time images
                # thread = threading.Thread(name="worker", target=self.algorithm, args=(cv_image, ))
                # thread.start()
                self.aruco_detect_bot(cv_image)
                # self.algorithm(cv_image)

            # Execute the aruco_detection_destination untill it detects,
            # all the destination points
            elif self.aruco1 == 0:
                self.aruco_detect_inductstation(cv_image)

            # Execute the aruco_detection_inductstation untill it detects,
            # all the inductstation points
            elif self.aruco2 == 0:
                self.aruco_detect_destination(cv_image)

            # Execute the arena_config function only one time
            elif self.graphc == 0:
                self.arena_config(cv_image)

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
        self.graph = self.init_graph(clearance, radius, height, width)
        print("Completed Graph Plotting")

    # Function to Initialize Graph
    def init_graph(self, height, width):
        graph = {}
        for i in range(width):
            for j in range(height):
                graph[(i,j)] = {'visited':False, 'distance':np.inf, 'valid':True, 'parent': (0, 0), 'id':'blank'}

        return graph

    # Function to Render Graph
    def render_graph(self, graph, pnt1, pnt2):
        """
        This function defines the obstacles with the bot radius and
        also generates the obstacle map and path map
        """
        clearance, radius = 20, 15
        for i in range(pnt1[0],pnt2[0]+1):
            for j in range(pnt2[1],pnt1[1]+1):
                    graph[(i,j)]['valid'] = False
                    graph[(i,j)]['id'] = 'obs'
        for i in range(pnt1[0]-clearance,pnt2[0]+1+clearance):
            for j in range(pnt2[1]-clearance,pnt1[1]+1+clearance):
                    if graph[(i,j)]['id'] != 'obs':
                        graph[(i,j)]['valid'] = False
                        graph[(i,j)]['id'] = 'aug'

        return graph

    # Function to detect aruco Markers of Induct Station Points
    def aruco_detect_inductpoint(self, frame):
        try:
            self.ind1 = 0
            parameters =  cv2.aruco.DetectorParameters_create()
            # Detect the Induct Point markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            for i in markerIds:
                try:
                    self.temp1.index(i)
                    self.ind1 += 1

                except ValueError:
                    temp = int(i)
                    self.temp1.append(temp)
                    self.temp2.append(markerCorners[self.ind1].tolist())
                    self.ind1 += 1

            if len(self.temp1) == 2:
                self.inductzone = self.extract_induct_point(self.temp1, self.temp2)

                print("Induct Zone Aruco Markers Detected")
                print(self.inductzone)

                self.aruco1 = 1

        except Exception as e:
            pass

    # Function to Extract Induct Points
    def extract_induct_point(self, ids, bbox):
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
            ctp = (cen_x, cen_y)
            temp_ls.append(ctp)
            dest_ls.append(name)
            l_ind += 1

        dest = dict(zip(dest_ls, temp_ls))

        return dest

    # Function to detect aruco Markers of Destination Points
    def aruco_detect_destination(self, frame):
        """
        This Function is responsible to detect all the destination markers
        to identify all possible location around the chute.
        """
        try:
            self.ind2 = 0
            parameters =  cv2.aruco.DetectorParameters_create()

            # Detect the markers in the image
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

            for i in markerIds:
                indices = [j for j, x in enumerate(self.temp1) if x == i]
                if len(indices) == 1:
                    temp = int(i)
                    self.temp3.append(temp)
                    self.temp4.append(markerCorners[self.ind1])
                elif len(indices) == 0:
                    temp = int(i)
                    self.temp3.append(temp)
                    self.temp4.append(markerCorners[self.ind1])
                self.ind2 += 1

            if len(self.temp3) == 18:
                self.destination = self.extract_goal_point(frame, self.temp3, self.temp4)
                print("Destination Aruco Markers Detected")
                print(self.destination)
                self.aruco2 = 1
                self.graphc = 1

        except Exception as e:
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

            if (y1<y2):
                x1, x2 = x2, x1
                y1, y2 = y2, y1

            d1 = (x1, y1)
            d2 = (x2, y2)
            i1 = (x1, y2)
            i2 = (x2, y1)

            cv2.line(frame, d1, i1, (255, 0, 0), thickness=2)
            cv2.line(frame, i1, d2, (255, 0, 0), thickness=2)
            cv2.line(frame, d2, i2, (255, 0, 0), thickness=2)
            cv2.line(frame, i2, d1, (255, 0, 0), thickness=2)

            self.graph = self.render_graph(self.graph, (x1+25, y1-25), (x2-25, y2+25))
            pts = [d1, d2, i1, i2]
            pts_ls.append(pts)

        dest = dict(zip(self.location, pts_ls))

        return dest

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
            message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', bot)
            self.publisher(message)

            return frame, bot

        except Exception:
            pass

    # Function Algorithm
    def algorithm(self, image):
        """
        This function is responsible to complete the entire task.
        It is also combined with multiple control statements to
        get optimal results
        """

        ###############################
        #                             #
        ##                           ##
        ###                         ###
        # Lot of Work need to be done #
        ###                         ###
        ##                           ##
        #                             #
        ###############################


    # Function to identify the closest point
    def closest_point(self, ls, pos):
        """
        This Function will choose the closest point of the destination
        from the current position of the bot.
        """
        x, y = pos
        d = list(map(lambda t: math.sqrt(pow(t[0]-x,2)+pow(t[1]-y,2)),ls))
        min_res = min(d)
        i = d.index(min_res)

        return pts[i]

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
        """
        This Function will plan a path to reach the Goal Point with
        or without waypoints using A star Algorithm
        """
        # Marking the Start Point and Goal point
        img = cv2.circle(img, start, 2, (255, 0, 0), 8)
        img = cv2.circle(img, goal, 2, (0, 0, 255), 8)

        dif_1 = abs(start[0]-goal[0])
        dif_2 = abs(start[1]-goal[1])
        # If the Difference between two points is less than 50
        # Then, No waypoints needed
        if dif_1 <= 50 or dif_2 <= 50:
            path = [start, goal]
            re = path
        else:
            # Calling Astar Algorithm to compute the shortest path
            _, path = self.astar(self.graph, start, goal)
            path = np.array(path)
            # Reducing No of way Points from the above list
            re = self.reduce_points(path, start, goal)
            re.reverse()

        # Defining Constant
        k=1

        # Calculating Necessary Angles to reach Waypoint
        reps, angs = self.calc_angle(re)

        # Marking the Minimized set of goalpoints
        for point1, point2 in zip(reps, reps[1:]):
            cv2.line(img, point1, point2, [0, 255, 0], 2)

        # Removing Start and Goal Points from the waypoint list
        del reps[0], reps[len(reps)-1]

        for i in self.graph:
            if (graph[i]['visited']==True):
                graph[i]['visited'] = False

        return img, path

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

    # Function to Reduce Points from the list of Points
    def reduce_points(self, points, start, end):
        """
        This Function to reduce the large set of points to minimalized points.
        """
        dif = [3, 3]
        temp1, temp2, rep = [], [], []
        rep.append((end[0], end[1]))
        for i in range(1, len(points)-1):

            temp1 = [abs(points[i-1][0] - points[i][0]), abs(points[i-1][1] - points[i][1])]
            temp2 = [abs(points[i+1][0] - points[i][0]), abs(points[i+1][1] - points[i][1])]

            if ((temp1==dif) and ((temp2[0]==dif[0]) or (temp2[1]==dif[1]))):
                if temp2 == dif:
                    continue

                rep.append((points[i][0], points[i][1]))

        rep.append((start[0], start[1]))

        t1 = abs(rep[0][0] - rep[1][0])
        t2 = abs(rep[0][1] - rep[1][1])
        if t1 == 3 or t2 == 3:
            del rep[1]

        t3 = abs(rep[len(rep)-1][0] - rep[len(rep)-2][0])
        t4 = abs(rep[len(rep)-1][1] - rep[len(rep)-2][1])
        if t3 == 3 or t4 == 3:
            del rep[len(rep)-2]

        return rep

    # Function to Calaculate Angle between two successive points
    def calc_angle(self, points):
        """
        This Function will calculate angle between two adjacent points
        from the reduced no of points to turn the bot
        """
        l = 0
        temp_ls, ang_ls, red_ls = [], [], []
        for i in range(len(points)-1):
            a = abs(points[i][1]-points[i+1][1])
            b = abs(points[i][0]-points[i+1][0])
            rad = math.atan(a/b)
            deg = rad *(180/(math.pi))
            temp_ls.append(deg)

        red_ls = points
        for i in temp_ls:
            if i <= 5:
                continue
            ang_ls.append(i)

        return red_ls, ang_ls

    # Function to Publish direction command
    def publish_command_direction(self, bot, goal, cur):
        """
        This function will publish command as direction forward
        based on certain goal commands
        """
        #Creating ROS Publisher
        direction = rospy.Publisher('/direction_control', Int16, queue_size=1)
        direct = Int16()

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
            direct = 4
            direction.publish(direct)
            rotate = 1

        # If the bot is not in the range of the goal
        # then Bot is moved forward
        # Note: Value=30 is the tolerance added to compensate the tf delay
        elif cur[1] not in range((goal[1]-30), (goal[1]+30)) and y_follow == 1:
            direct = 1
            direction.publish(direct)

        # Horizontal Movement
        # Comparing the current x co-ordinates of the bot with goal
        # If the current position is in the range of the goal position or
        # withing the limit then, Stopping the Bot
        # Note: Value=30 is the tolerance added to compensate the tf delay
        if cur[0] in range((goal[0]-30), (goal[0]+30)) and x_follow == 1:
            direct = 4
            direction.publish(direct)
            rotate = 1

        # If the bot is not in the range of the goal
        # then Bot is moved forward
        # Note: Value=30 is the tolerance added to compensate the tf delay
        elif cur[0] not in range((goal[0]-30), (goal[0]+30)) and x_follow == 1:
            direct = 1
            direction.publish(direct)

        self.publish_command_rotation(bot, value, rotate)

    # Function to publish rotational command
    def publish_command_rotation(self, bot, val, rot):
        """
        This functions get the angle and publish the commands to
        rotate the bot accordingly
        """
        ##

        # Need to Work in this Function

        ##

        # Creating a ROS Publisher
        direction = rospy.Publisher('/direction_control', Twist, queue_size=1)
        direct = Int16()

        # Defining Constants
        angular_speed = 1

        # If the value is less than 0, then rotate clockwise
        # else rotate anti-clockwise
        if val<=0:
            # Work Here
            i = 1
        else:
            # Work Here
            i = 0

        # If rotate is made to 1 then rotate the bot
        # Based On the path that the bot is executing
        # added +55 degree because the base link,
        # Not at exact center of the bot
        if rot == 1:
            if self.path == 1:
                angle = 180
                self.path = 2
            elif self.path == 2 and self.rotate == 0:
                self.actuate_servo(bot)
                angle = 360
                self.exec, self.path, self.reverse = 0, 1, 2
                self.rotate = 1

    # Function to actuate servo motor
    def actuate_servo(self, bot):

        # Creating a ROS Publisher
        servo = rospy.Publisher('/servo_control', Int16, queue_size=1)
        servo = Int16()

        # Defining angle for servo actuatuion,
        # To drop the package from bot
        servo = 1
        pub1.publish(servo)

        # Wait till the package is dropped
        rospy.sleep(2)

        # Defined angle as 180 to actuate the
        # servo back to initial position
        servo = 0
        pub1.publish(servo)

    # Destructor of the Class
    def __del__(self):
        rospy.loginfo('\033[94m' + "Shutting Down" + '\033[0m')

# Main Function
def main():
    """
    This is the start of execution of this node
    """
    # Initializing the Node
    rospy.init_node('node_eg1_read_camera', anonymous=True)
    # Creating Object for the Class Server
    ser = Server()
    try:
        # Spinning the Main Function
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        # Deleting the Created Object if Interrupted
        del ser
        # Destroying all the cv2 Windows Created
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()