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

    bot1/servo_control                          /usb_camera/image_raw
    bot1/direction_control
"""
################## Code V2 - Half way stage of path execution of Bot 1 #################
# Importing Required Modules
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
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int64

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
        self.graph, self.exec, self.reverse, self.path = 1, 0, 0, 1
        self.aruco, self.ind = 1, 0
        self.n, self.rotate = 0, 0
        self.temp1, self.temp2 = [], []
        self.flag1, self.flag2, self.flag3 = 0, 0, 0

        self.goal, self.goalr, self.pnt = (0, 0), (0, 0), (0, 0)
        self.destination = ((1170, 140), (1170, 80), (110, 80), (110, 140))

        # Subscribing to the ROS Image topic
        self.image_sub = rospy.Subscriber("/grid/camera_1/image_raw", Image, self.callback, queue_size = 1)
        #self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)

    # Function for ROS Camera Subscription Callback
    def callback(self,data):
        """
        This functions gets all the image published in the subscribed topic and
        sends the image to the next function to process them
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Threaded the algorithm function to process real time images
            thread = threading.Thread(name="worker", target=self.algorithm, args=(cv_image, ))
            thread.start()

            # Execute the aruco_detection untill it detects all
            # the destination points
            # if self.aruco == 1:
            #     self.aruco_detect(cv_image)

            # Execute the arena_config function only one time
            if self.graph == 1:
                self.arena_config(cv_image)
                self.graph = 0

        except Exception as e:
            rospy.logwarn(e)

    # Function to have spatial awareness of the arena
    def arena_config(self, image):
        """
        This function defines the bot radius and calls the next function to
        create the arena as graph and generates the points of the obstacle
        """
        radius, clearance = 10, 0
        height, width, _ = image.shape
        self.g = self.create_graph(image, clearance, radius, height, width)
        self.points = [x for x in self.g.keys() if not (self.g[x]['valid'])]

    # Function to detect Aruco Markers
    def aruco_detect(self, frame):
        """
        This function is created to detect aruco markers present in the
        arena
        """
        try:
            if self.aruco == 0:
                self.ind = 0
                parameters =  cv2.aruco.DetectorParameters_create()
                # Detect the markers in the image
                dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
                markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

                for i in markerIds:
                    try:
                        self.temp1.index(i)
                        self.ind += 1
                    except ValueError:
                        temp = int(i)
                        self.temp1.append(temp)
                        self.temp2.append(markerCorners[self.ind].tolist())
                        self.ind += 1

                if len(self.temp1) == 4:
                    self.destination = self.extract_goal_point(self.temp1, self.temp2)
                    rospy.loginfo('\033[94m' + "Destinations Detected" + '\033[0m')
                    print(self.destination)
                    self.aruco = 0
        except Exception as e:
            pass

    # Function to Extract Goal Points
    def extract_goal_point(self, ids, bbox):
        """
        This function will extract the goal from the boundary box points
        using the marker id and create a center point as result
        Stores the result as dictionary
        """
        l_ind = 0
        temp_ls = []

        for i in ids:
            cen_x = int((bbox[l_ind][0][0][0] + bbox[l_ind][0][2][0])/2)
            cen_y = int((bbox[l_ind][0][0][1] + bbox[l_ind][0][2][1])/2)
            ctp = (cen_x, cen_y)
            temp_ls.append(ctp)
            l_ind += 1

        dest = dict(zip(ids, temp_ls))

        return dest

    # Funtion to get the obstacle position
    def get_obstacle(self, im, h, w):
        """
        This function will compare the black colored pixels and
        comparison techniques to get the obstacle points
        """
        p=int(w/4)
        pp=p*3
        lh=0
        lr=0

        for i in range(h-5):
            fl=1
            for j in range(8):
                if im[i+j][p][0]>60 or im[i+j][p][1]>60 or im[i+j][p][2]>60:
                    fl=0
                    break
            if fl==1:
                lh=i
                break

        for i in range(5,h-5):
            fl=1
            for j in range(8):
                if im[i+j][pp][0]>60 or im[i+j][pp][1]>60 or im[i+j][pp][2]>60:
                    fl=0
                    break
            if fl==1:
                lr=i
                break

        p=300
        g=320
        rs1,rs2=0,0
        ls1,ls2=0,0
        half=int(w/2)

        for i in range(half):
            fl=1
            for j in range(5):
                if im[p][half+i+j][0]>60 or im[p][half+i+j][1]>60 or im[p][half+i+j][2]>60:
                    fl=0
                    break
            if fl==1:
                rs1=half+i
                break

        for i in range(320):
            fl=1
            for j in range(5):
                if im[g][half+i+j][0]>60 or im[g][half+i+j][1]>60 or im[g][half+i+j][2]>60:
                    fl=0
                    break
            if fl==1:
                rs2=half+i
                break

        for i in range(half):
            fl=1
            for j in range(5):
                if im[p][half-i-j][0]>60 or im[p][half-i-j][1]>60 or im[p][half-i-j][2]>60:
                    fl=0
                    break
            if fl==1:
                ls1=half-i
                break

        for i in range(half):
            fl=1
            for j in range(5):
                if im[g][half-i-j][0]>60 or im[g][half-i-j][1]>60 or im[g][half-i-j][2]>60:
                    fl=0
                    break
            if fl==1:
                ls2=half-i
                break
        p1 = (int((lh+lr)/2),int((ls1+ls2)/2))
        p2 = (int((lh+lr)/2),int((rs1+rs2)/2))

        return p1, p2

    # Function to create Graph
    def create_graph(self, image, clearance, radius, height, width):
        """
        This function defines the obstacles with the bot radius and
        also generates the obstacle map and path map
        """
        graph = {}
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

    # Function to filter the boxes
    def filter_boxes(self, box_xywh, scores, score_threshold=0.4, input_shape = tf.constant([512, 512])):
        """
        This function will filter the boxes which are appropriate to the
        detection result and removes the unwanted ones
        """
        scores_max = tf.math.reduce_max(scores, axis=-1)
        mask = scores_max >= score_threshold
        class_boxes = tf.boolean_mask(box_xywh, mask)
        pred_conf = tf.boolean_mask(scores, mask)

        class_boxes = tf.reshape(class_boxes, [tf.shape(scores)[0], -1, tf.shape(class_boxes)[-1]])
        pred_conf = tf.reshape(pred_conf, [tf.shape(scores)[0], -1, tf.shape(pred_conf)[-1]])

        box_xy, box_wh = tf.split(class_boxes, (2, 2), axis=-1)
        input_shape = tf.cast(input_shape, dtype=tf.float32)

        box_yx = box_xy[..., ::-1]
        box_hw = box_wh[..., ::-1]
        box_mins = (box_yx - (box_hw / 2.)) / input_shape
        box_maxes = (box_yx + (box_hw / 2.)) / input_shape

        boxes = tf.concat([
            box_mins[..., 0:1],  # y_min
            box_mins[..., 1:2],  # x_min
            box_maxes[..., 0:1],  # y_max
            box_maxes[..., 1:2]  # x_max
        ], axis=-1)

        return (boxes, pred_conf)

    # Function to Draw Boundary Boxes
    def draw_bbox(self, image, bboxes, classes):
        """
        This function will draw the boundary boxes around the detected bots
        Also create the coords of the bot in dictionary format
        """
        num_classes = len(classes)
        image_h, image_w, _ = image.shape
        classes_to_coor={}
        out_boxes, out_scores, out_classes, num_boxes = bboxes

        for i in range(num_boxes[0]):
            if int(out_classes[0][i]) < 0 or int(out_classes[0][i]) > num_classes: continue
            coor = out_boxes[0][i]
            coor[0] = int(coor[0] * image_h)
            coor[2] = int(coor[2] * image_h)
            coor[1] = int(coor[1] * image_w)
            coor[3] = int(coor[3] * image_w)

            class_ind = int(out_classes[0][i])
            bbox_color = (0, 0, 255)
            bbox_thick = int(0.6 * (image_h + image_w) / 600)
            c1, c2 = (int(coor[1]), int(coor[0])), (int(coor[3]), int(coor[2]))
            cv2.rectangle(image, c1, c2, bbox_color, bbox_thick)
            classes_to_coor[classes[class_ind]]=[(coor[1], coor[0]), (coor[3], coor[2])]

        return image,classes_to_coor

    # Function to Detect Bots
    def detect_bot(self, arg_image):
        """
        This Function will detect the bots using tensorflow model generated
         as tflite and returns the image and co-ordinates of the bot
        """
        rp = rospkg.RosPack()
        str_pkg_path = rp.get_path('swarmrobot')
        input_size=512
        original_image = cv2.cvtColor(arg_image, cv2.COLOR_BGR2RGB)
        image_data = cv2.resize(original_image, (input_size, input_size))
        image_data = image_data / 255.
        images_data = []
        for i in range(1):
            images_data.append(image_data)
            images_data = np.asarray(images_data).astype(np.float32)
        interpreter = tf.lite.Interpreter(model_path="{}/tf/detect.tflite".format(str_pkg_path))
        interpreter.allocate_tensors()
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        interpreter.set_tensor(input_details[0]['index'], images_data)
        interpreter.invoke()
        pred = [interpreter.get_tensor(output_details[i]['index']) for i in range(len(output_details))]
        boxes, pred_conf = self.filter_boxes(pred[0], pred[1], score_threshold=0.35, input_shape=tf.constant([input_size, input_size]))
        boxes, scores, classes, valid_detections = tf.image.combined_non_max_suppression(
            boxes=tf.reshape(boxes, (tf.shape(boxes)[0], -1, 1, 4)),
            scores=tf.reshape(
                pred_conf, (tf.shape(pred_conf)[0], -1, tf.shape(pred_conf)[-1])),
            max_output_size_per_class=1,
            max_total_size=4,
            iou_threshold=0.35,
            score_threshold=0.35
            )
        pred_bbox = [boxes.numpy(), scores.numpy(), classes.numpy(), valid_detections.numpy()]
        image, coor = self.draw_bbox(original_image, pred_bbox , ['bot1','bot2','bot3','bot4'])
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        return image, coor

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
            img, coor = self.detect_bot(image)
            # Get the center point of the bot
            bot = self.bot_name[self.n]
            current = self.center_point(coor, bot)

            if self.exec == 0:
                if self.reverse == 0:
                    self.goalr = current
                    self.reverse = 1
                self.goal = (self.destination[self.n][0], self.destination[self.n][1])
                self.flag1 = self.validate_point(current, self.points)
                self.flag2 = self.validate_point(self.goal, self.points)
                if  self.flag1 == 1 and self.flag2 == 1:
                    if self.reverse == 2:
                        self.goal = self.goalr
                    img, self.pnt = self.path_plan_custom(img, current, self.goal)
                    self.flag3 = self.validate_point(self.pnt, self.points)
                    if self.flag3 == 0:
                        self.path_plan(img, current, self.goal)
                    self.exec = 1

            if  self.flag1 == 1 and self.flag2 == 1:
                img, _ = self.path_plan_custom(img, current, self.goal)
                if self.path == 1:
                    self.publish_command_direction(bot, self.pnt, current)
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
            img = cv2.resize(img, (640 ,360))
            cv2.imshow("result", img)
            cv2.waitKey(1)

        except KeyError:
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
        #Creating ROS Publisher
        topic = '/' + bot + '/cmd_vel'
        #topic = bot + '/direction_control'
        #direction = rospy.Publisher(topic, Int16, queue_size=1)
        direction = rospy.Publisher(topic, Twist, queue_size=1)
        twist = Twist()
        # direct = Int16()

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
            #direct = 4
            #direction.publish(direct)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            direction.publish(twist)
            rotate = 1

        # If the bot is not in the range of the goal
        # then Bot is moved forward
        # Note: Value=30 is the tolerance added to compensate the tf delay
        elif cur[1] not in range((goal[1]-30), (goal[1]+30)) and y_follow == 1:
            #direct = 1
            #direction.publish(direct)
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            direction.publish(twist)

        # Horizontal Movement
        # Comparing the current x co-ordinates of the bot with goal
        # If the current position is in the range of the goal position or
        # withing the limit then, Stopping the Bot
        # Note: Value=30 is the tolerance added to compensate the tf delay
        if cur[0] in range((goal[0]-30), (goal[0]+30)) and x_follow == 1:
            #direct = 4
            #direction.publish(direct)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            direction.publish(twist)
            rotate = 1

        # If the bot is not in the range of the goal
        # then Bot is moved forward
        # Note: Value=30 is the tolerance added to compensate the tf delay
        elif cur[0] not in range((goal[0]-30), (goal[0]+30)) and x_follow == 1:
            #direct = 1
            #direction.publish(direct)
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            direction.publish(twist)

        self.publish_command_rotation(bot, value, rotate)

    # Function to publish rotational command
    def publish_command_rotation(self, bot, val, rot):
        """
        This functions get the angle and publish the commands to
        rotate the bot accordingly
        """
        # Creating a ROS Publisher
        topic = '/' + bot + '/cmd_vel'
        #topic = bot + '/servo_control'
        #servo = rospy.Publisher(topic, Int16, queue_size=1)
        direction = rospy.Publisher(topic, Twist, queue_size=1)
        twist = Twist()
        # direct = Int16()

        # Defining Constants
        angular_speed = 1

        # If the value is less than 0, then rotate clockwise
        # else rotate anti-clockwise
        if val<=0:
            twist.angular.z = -abs(angular_speed)
        else:
            twist.angular.z = abs(angular_speed)

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

            try:
                # Calculate the Relative angle
                relative_angle = int(angle)*3.14/180
                # Setting the current time for distance calculus
                t0 = rospy.get_time()
                # Define the current angle
                current_angle = 0

                # Publish the twist value based on the given condition
                while(current_angle < relative_angle):
                    direction.publish(twist)
                    t1 = rospy.get_time()
                    current_angle = angular_speed*(t1-t0)

                # Stop the rotation
                twist.angular.z = 0
                direction.publish(twist)
            except Exception:
                pass

    # Function to actuate servo motor
    def actuate_servo(self, bot):

        # Creating a ROS Publisher
        topic = '/' + bot + '/controller/command'
        servo = rospy.Publisher(topic, Float64, queue_size=1)
        value = Float64()

        # Defining angle for servo actuatuion,
        # To drop the package from bot
        angle = 0
        servo.publish(angle)
        # servo = 1
        # pub1.publish(servo)

        # Wait till the package is dropped
        #rospy.sleep(2)

        # Defined angle as 180 to actuate the
        # servo back to initial position
        angle = 180
        servo.publish(angle)

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