import sys
import time
import math
import cv2
import numpy as np
from heapq import heappush, heappop
from multiprocessing.dummy import Pool

graph = {}

# Function to Init Image as Graph
def init_graph(height, width):
    """
    This Function will initialize the graph which is
    in the size of the input image
    """
    global graph
    for i in range(width):
        for j in range(height):
            graph[(i,j)] = {'visited':False, 'distance':np.inf, 'valid':True, 'parent': (0, 0), 'id':'blank'}

# Function to Render Graph
def render_graph(pt1, pt2):
    """
    This Function will update the graph value of destination
    induct points.
    """
    global graph

    clearance = 20
    radius = 20

    for i in range(pt1[0],pt2[0]+1):
        for j in range(pt2[1],pt1[1]+1):
                graph[(i,j)]['valid'] = False
                graph[(i,j)]['id'] = 'obs'
    for i in range(pt1[0]-clearance,pt2[0]+1+clearance):
        for j in range(pt2[1]-clearance,pt1[1]+1+clearance):
                if graph[(i,j)]['id'] != 'obs':
                    graph[(i,j)]['valid'] = False
                    graph[(i,j)]['id'] = 'aug'

# Function to return Graph
def return_graph():
    """
    This Function will return the global Graph
    """
    global graph

    return graph

# Function for Custom Path Planning
def path_plan_custom(img, start, end):
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
        cv2.line(img, start, way_point, (255, 0, 0), cv2.LINE_4, 1)
        cv2.line(img, way_point, end, (255, 0, 0), cv2.LINE_4, 1)

    return img, way_point

# Function to plan a Path
def path_plan(start, goal):
    """
    This function will generate the custom path using the
    start and end points
    """
    global graph

    dif_1 = abs(start[0]-goal[0])
    dif_2 = abs(start[1]-goal[1])

    # If the Difference between two points is less than 50
    # Then, No waypoints needed
    if dif_1 <= 80 or dif_2 <= 80:
        path = [start, goal]
        re = path
    else:
        # Calling Astar Algorithm to compute the shortest path
        _, path = astar(graph, start, goal)
        path = np.array(path)
        # Reducing No of way Points from the above list
        re = reduce_points(path, start, goal)
        re.reverse()

    # Defining Constant
    k=1

    # Calculating Necessary Angles to reach Waypoint
    reps, angs = calc_angle(re)

    # Removing Start and Goal Points from the waypoint list
    del reps[0], reps[len(reps)-1]

    for i in graph:
        if (graph[i]['visited']==True):
            graph[i]['visited'] = False

    return reps, angs

# Function to Reduce Points from the list of Points
def reduce_points(points, start, end):
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
def calc_angle(points):
    """
    This Function will calculate angle between two adjacent points
    from the reduced no of points to turn the bot
    """
    l, t = 0, 0
    temp_ls, ang_ls, red_ls = [], [], []
    for i in range(len(points)-1):
        a = abs(points[i][1]-points[i+1][1])
        b = abs(points[i][0]-points[i+1][0])
        rad = math.atan(a/b)
        d = rad *(180/(math.pi))
        deg = (d - t)
        temp_ls.append(int(deg))
        red_ls.append(d)
        t = red_ls[l]
        l += 1

    # red_ls = points
    for i in temp_ls:
        if i in range (-3, 3):
            continue
        ang_ls.append(i)

    return points, ang_ls

# Function For A-star Algorithm
def astar(graph, source, goal):
    """
    This function will generate the path using the graph generated.
    Returns the minimum path and number of nodes visited.
    """
    temp = graph
    count, queue_distance = 0, 0
    row, queue, path = [], [], []
    neighbour, parent = (0, 0), (0, 0)
    (goal_x, goal_y) = goal
    temp[source]['visited'] = True
    num_nodes_visited = 1
    temp[source]['distance'] = 0
    queue_distance = calculate_distance(goal, source)+temp[source]['distance']
    heappush(queue, (queue_distance, source))
    while (len(queue) != 0):

        current = heappop(queue)[1]
        if current[0] <= goal[0]+5 and current[0] >= goal[0] and current[1] <= goal[1]+5 and current[1] >= goal[1] :
            #print("Goal reached")
            (goal_x,goal_y)=(current[0],current[1])
            if row:
                x,y = zip(*row)
            break
        for i in [-3, 0, 3]:
            for j in [-3, 0, 3]:
                if i != 0 or j != 0:
                    neighbour = (abs(current[0]+i), abs(current[1]+j))
                    lst = list(neighbour)
                    if lst[0] >=1280:
                        lst[0] = 1279
                    if lst[1] >=720:
                        lst[1] = 719
                    neighbour = tuple(lst)
                    if temp[neighbour]['valid'] == True:

                        if abs(i)+abs(j) == 2:
                            distance = math.sqrt(2)
                        else:
                            distance = 1

                        if temp[neighbour]['visited'] == False:
                            temp[neighbour]['visited'] = True
                            row.append([abs(current[0]+i), abs(current[1]+j)])
                            x,y = zip(*row)

                            num_nodes_visited += 1
                            temp[neighbour]['parent'] = current
                            temp[neighbour]['distance'] = temp[current]['distance'] + distance
                            queue_distance = calculate_distance(goal, neighbour)+temp[neighbour]['distance']
                            heappush(queue, (queue_distance, neighbour))
    path = [(goal_x, goal_y)]
    parent = (goal_x, goal_y)
    while parent != source:
        parent = temp[path[len(path)-1]]['parent']
        path.append(parent)
    min_distance = (temp[(goal_x,goal_y)]['distance'])
    
    return(min_distance, path)

# Function to calculate distance
def calculate_distance(goal, current):
    """
    This function will calculate the distance between the current and
    goal point
    """
    d = math.sqrt(((goal[0]-current[0])*(goal[0]-current[0]))+((goal[1]-current[1])*(goal[1]-current[1])))
    return d