import cv2
import time
import math
import json
import numpy as np
from heapq import heappush, heappop
import cv2.aruco as aruco

def init_graph(graph,height,width):
    for i in range(width):
        for j in range(height):
            graph[(i,j)] = {'visited':False, 'distance':np.inf, 'valid':True, 'parent': (0, 0), 'id':'blank'}

    return graph

def render_graph(graph,pt1,pt2):
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

    return graph

def aruco_detect_inductpoint(frame):

    inductzone = {}
    parameters =  cv2.aruco.DetectorParameters_create()
    # Detect the Induct Point markers in the image
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
    frame = aruco.drawDetectedMarkers(frame, markerCorners)
    inductzone = extract_induct_point(markerIds, markerCorners)

    # print("Induct Zone Aruco Markers Detected")
    # print(inductzone)

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

def get_destination(frame, graph):
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
            # .append()
            for l in indices:
                sum_ls = sum_ls + cor_ls[l]
                # temp_ls.append(cor_ls[l])
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
            # i1 = (x1, y2)
            # i2 = (x2, y1)
            # print((x1,y1),(x2,y2),"\n")
            
            d1 = (x1, y1)
            d2 = (x2, y2)
            i1 = (x1, y2)
            i2 = (x2, y1)
            
            cv2.line(frame, d1, i1, (0, 0, 0), thickness=2)
            cv2.line(frame, i1, d2, (0, 0, 0), thickness=2)
            cv2.line(frame, d2, i2, (0, 0, 0), thickness=2)
            cv2.line(frame, i2, d1, (0, 0, 0), thickness=2)

            render_graph(graph, (x1+25, y1-25), (x2-25, y2+25))
            # cv2.circle(frame, (x1+30,y1-30), 3, (255,0,0), 2)
            # cv2.circle(frame, (x2-30,y2+30), 3, (255,0,0), 2)
            # cv2.line(frame, (x1+30, y1-30), (i1[0]+30,i1[1]+30), (255, 0, 0), thickness=2)
            # cv2.line(frame, (i1[0]+30,i1[1]+30), (x2-30, y2+30), (255, 0, 0), thickness=2)
            # cv2.line(frame, (x2-30, y2+30), (i2[0]-30,i2[1]-30), (255, 0, 0), thickness=2)
            # cv2.line(frame, (i2[0]-30,i2[1]-30), (x1+30, y1-30), (255, 0, 0), thickness=2)
            pts = [d1, d2, i1, i2]
            pts_ls.append(pts)

        except Exception as e:
            # print(e)
            pass
    
    dest = dict(zip(place, pts_ls))
    
    #For checking obstacles
    # for i in range(1280):
    #     for j in range(720):
    #         if graph[(i,j)]['id'] == 'obs':
    #             cv2.circle(frame, (i,j), 1, (255,255,0), 1)
    #         elif graph[(i,j)]['id'] == 'aug':
    #             cv2.circle(frame, (i,j), 1, (255,0,255), 1)
    
    return frame, dest

def closest_point(pts, pt):
    x, y = pt
    d = list(map(lambda t: math.sqrt(pow(t[0]-x,2)+pow(t[1]-y,2)),pts))
    min_res = min(d)
    i_m = d.index(min_res)
    return pts[i_m]

# Function to plan a Path
def path_plan(graph, start, goal):

    dif_1 = abs(start[0]-goal[0])
    dif_2 = abs(start[1]-goal[1])

    # If the Difference between two points is less than 50
    # Then, No waypoints needed
    if dif_1 <= 50 or dif_2 <= 50:
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

def mark_points(img, start, goal, ls):
    # Marking the Start Point and Goal point
    img = cv2.circle(img, start, 2, (255, 0, 0), 8)
    img = cv2.circle(img, goal, 2, (0, 0, 255), 8)
    ls.insert(0, start)
    ls.insert(len(ls), goal)
    # Marking the Minimized set of goalpoints
    for point1, point2 in zip(ls, ls[1:]):
        cv2.line(img, point1, point2, [0, 255, 0], 2)

    return img

def reduce_points(points, start, end):
    dif = [3, 3]
    temp1, temp2, rep = [], [], []
    rep.append((end[0], end[1]))
    for i in range(1, len(points)-1):
        # print(i)
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

    # if rep[0] in range(rep[1]+5, rep[1]-5)
    return rep

def calc_angle(points):
    l = 0
    temp_ls, ang_ls, red_ls = [], [], []
    for i in range(len(points)-1):
        a = abs(points[i][1]-points[i+1][1])
        b = abs(points[i][0]-points[i+1][0])
        rad = math.atan(a/b)
        deg = rad *(180/(math.pi))
        # print(deg)
        temp_ls.append(deg)

    # ang_ls = temp_ls
    red_ls = points
    for i in temp_ls:
        if i <= 5:
            continue
        ang_ls.append(i)

    return red_ls, ang_ls

def astar(graph, source, goal):
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
    # print("Total Number of Nodes Visited:", num_nodes_visited)
    
    return(min_distance, path)

def calculate_distance(goal, current):
    d = math.sqrt(((goal[0]-current[0])*(goal[0]-current[0]))+((goal[1]-current[1])*(goal[1]-current[1])))
    return d

place = ['Mumbai', 'Delhi', 'Kolkata', 
         'Chennai', 'Bengaluru', 'Hyderabad', 
         'Pune', 'Ahemdabad', 'Jaipur']
temp1, temp2 = [], []
ind1, count = 0, 0
destination = {}
graph = {}

frame = cv2.imread("images/img.png")
# frame = cv2.imread("test.jpg")
# frame = cv2.resize(frame, (640, 360))

graph = init_graph(graph,720,1280)

frame, destination = get_destination(frame, graph)
frame, inductzone = aruco_detect_inductpoint(frame)
ins = json.dumps(destination)
print(ins)
des = {'bot1': (1024, 431)}
ins = json.dumps(des)
print(ins)
# print(graph)
# print(destination['Mumbai'][0])
# print(inductzone[1][0])
# start = inductzone[1]
# goal = closest_point(destination['Hyderabad'], start)
# frame, path, angle = path_plan(graph, frame, start, goal)
# print(path)
# print(angle)

# start = inductzone[2]
# goal = closest_point(destination['Delhi'], start)
# frame, path, angle = path_plan(graph, frame, start, goal)
# print(path)
# print(angle)

# start = inductzone[1]
# for i in destination:
#     goal = closest_point(destination[i], start)
#     print(i, start, goal)
#     path, angle = path_plan(graph, start, goal)
#     frame = mark_points(frame, start, goal, path)
#     print(path)
#     print(angle)

# start = inductzone[2]
# for i in destination:
#     goal = closest_point(destination[i], start)
#     print(i, start, goal)
#     path, angle = path_plan(graph, start, goal)
#     frame = mark_points(frame, start, goal, path)
#     print(path)
#     print(angle)

cv2.imshow("frame", frame)
cv2.waitKey()