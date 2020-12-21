import numpy as np
import matplotlib.pyplot as plt
import math
import random
import pickle
import time
# from Intersection import intersection
from Jiin2 import intersection, checkIntersection


class RoadSegment:
    def __init__(self, backline, frontline, centerline, right_edge, left_edge, isarc=False, radius=0, angle=0, length=0):
        self.backline = backline # [left, center, right]
        self.frontline = frontline # [left, center, right]
        self.centerline = centerline # np.array([x_list, y_list])
        self.right_edge = right_edge # np.array([x_list, y_list])
        self.left_edge = left_edge # np.array([x_list, y_list])
        self.width = math.sqrt((backline[0,0] - backline[0,2])**2 + (backline[1,0] - backline[1,2])**2)
        self.npoints = centerline.shape[1]
        self.isarc = isarc
        self.radius = radius
        self.angle = angle
        self.length = length

    def getFront(self):
        return self.frontline

    def getBack(self):
        return self.backline

    def getCenter(self):
        return self.centerline

    def getRight(self):
        return self.right_edge

    def getLeft(self):
        return self.left_edge

    def drawPlot(self):
        plt.plot(self.backline[0], self.backline[1], color='black')
        plt.plot(self.frontline[0], self.frontline[1], color='black')
        plt.plot(self.left_edge[0], self.left_edge[1], color='black')
        plt.plot(self.right_edge[0], self.right_edge[1], color='black')
        plt.plot(self.centerline[0], self.centerline[1], color='black')
        # plt.plot(self.centerline[0], self.centerline[1], color='black', marker='o')

class Road:
    def __init__(self, inital_segment):
        self.segmentNumber = 1
        self.path = [inital_segment]
        self.NodeList = []
        self.NodeNum = 0

    def clean_NodeList(self):
        self.NodeList = []
        self.NodeNum = 0

    def add_NodeList(self, segment_idx):
        self.NodeList.append(segment_idx)
        self.NodeList.sort()
        self.NodeNum = self.NodeNum + 1

    def find_ceil_segment(self, segment_idx):
        ceil_idx = 0
        for seg_idx in self.NodeList:
            ceil_idx = seg_idx
            if (seg_idx > segment_idx):
                break
        return ceil_idx

    def lastSegment(self):
        return self.path[-1]

    def pushRoadSegment(self, new_segment):
        result = None
        if np.array_equal(self.path[-1].getFront(), new_segment.getBack()):
            # print('Successfully connected')
            self.path.append(new_segment)
            self.segmentNumber += 1
            result = self.path[:]
        #else:
        #    print('Fail: This road segment cause gap')
        return result

    def popRoadSegment(self):
        if self.segmentNumber > 1:
            self.segmentNumber -= 1
            self.path.pop()
            return True
        return False
        

    def drawPlot(self):
        for i in range(self.segmentNumber):
            self.path[i].drawPlot()

    def getNpoints(self):
        return sum(list(map(lambda x: x.npoints, self.path))) - (self.segmentNumber-1)

    def getCenterline(self):
        total_points = sum(list(map(lambda x: x.npoints, self.path))) - (self.segmentNumber-1)
        result = np.zeros((2, total_points), dtype=np.float16)
        result[0, 0] = self.path[0].getCenter()[0,0]
        result[1, 0] = self.path[0].getCenter()[1,0]
        curr = 1
        for i in range(self.segmentNumber):
            curr_segment = self.path[i]
            for j in range(curr_segment.npoints-1):
                result[0, curr] = curr_segment.getCenter()[0, j+1]
                result[1, curr] = curr_segment.getCenter()[1, j+1]
                curr += 1
        return result

    def getRightEdge(self):
        total_points = sum(list(map(lambda x: x.npoints, self.path))) - (self.segmentNumber-1)
        result = np.zeros((2, total_points), dtype=np.float16)
        result[0, 0] = self.path[0].getRight()[0,0]
        result[1, 0] = self.path[0].getRight()[1,0]
        curr = 1
        for i in range(self.segmentNumber):
            curr_segment = self.path[i]
            for j in range(curr_segment.npoints-1):
                result[0, curr] = curr_segment.getRight()[0, j+1]
                result[1, curr] = curr_segment.getRight()[1, j+1]
                curr += 1
        return result

    def getLeftEdge(self):
        total_points = sum(list(map(lambda x: x.npoints, self.path))) - (self.segmentNumber-1)
        result = np.zeros((2, total_points), dtype=np.float16)
        result[0, 0] = self.path[0].getLeft()[0,0]
        result[1, 0] = self.path[0].getLeft()[1,0]
        curr = 1
        for i in range(self.segmentNumber):
            curr_segment = self.path[i]
            for j in range(curr_segment.npoints-1):
                result[0, curr] = curr_segment.getLeft()[0, j+1]
                result[1, curr] = curr_segment.getLeft()[1, j+1]
                curr += 1
        return result

class RoadNetwork:
    def __init__(self, x_size, y_size):
        self.roadNumber = 0
        self.roads = []
        self.mapsize = [x_size, y_size]
        self.waypoints = None

    def addRoad(self, new_road):
        self.roads.append(new_road)
        self.roadNumber += 1
        return self.roads[:]

    def deleteLastRoad(self):
        self.roads.pop()
        self.roadNumber -= 1

    def drawPlot(self):
        for i in range(self.roadNumber):
            self.roads[i].drawPlot()

    def saveNetwork(self, path):
        with open(path, 'wb') as f:
            pickle.dump(self, f)
            f.close()
        return True

    def loadNetwork(self, path):
        with open(path, 'rb') as f:
            data = pickle.load(f)
            self.roadNumber = data.roadNumber
            self.roads = data.roads
            self.mapsize = data.mapsize
            self.waypoints = data.waypoints
            f.close()
        return True

    def getWayPoints(self):
        pass

def CreateStraigthRoadSegment(backline, length, orient):
    ''' If oreint is True, ratate 90 angle, else -90 angle'''
    angle = -90
    if orient == True:
        angle = 90

    ''' Get unit vector of backline '''
    v1 = backline[0][-1] - backline[0][0]
    v2 = backline[1][-1] - backline[1][0]
    vectorLen = math.sqrt(v1**2 + v2**2)
    unitVector = np.array([[v1/vectorLen, v2/vectorLen]], dtype=np.float16).T

    ''' Get orthogonal unit vector '''
    theta = np.radians(angle)
    c, s = np.cos(theta), np.sin(theta)
    orthogonal = np.array(((c, -s), (s, c)), dtype=np.float16)
    orthogonalVector = np.dot(orthogonal, unitVector)

    ''' Get frontline '''
    frontline = backline + orthogonalVector*length

    ''' Get left_edge, centerline, right_edge '''
    n = int(length)
    left_edge = np.zeros((2, n+1), dtype=np.float16)
    for i in range(n+1):
        left_edge[0, i] = backline[0, 0] + (length*i/n)*orthogonalVector[0, 0]
        left_edge[1, i] = backline[1, 0] + (length*i/n)*orthogonalVector[1, 0]

    centerline = np.zeros((2, n+1), dtype=np.float16)
    for i in range(n+1):
        centerline[0, i] = backline[0, 1] + (length*i/n)*orthogonalVector[0, 0]
        centerline[1, i] = backline[1, 1] + (length*i/n)*orthogonalVector[1, 0]

    right_edge = np.zeros((2, n+1), dtype=np.float16)
    for i in range(n+1):
        right_edge[0, i] = backline[0, 2] + (length*i/n)*orthogonalVector[0, 0]
        right_edge[1, i] = backline[1, 2] + (length*i/n)*orthogonalVector[1, 0]

    return RoadSegment(backline, frontline, centerline, right_edge, left_edge, False, 0 ,0, length)

def CreateArcRoadSegment(backline, radius, angle):
    theta = np.radians(angle)
    length = abs(theta*radius)
    road_width = np.linalg.norm(backline.T[2] - backline.T[1])
    n = max(2, int(length))
    theta_seg = theta/n
    c, s = np.cos(theta_seg), np.sin(theta_seg)
    ''' rotation matrix '''
    rotation = np.array(((c, -s), (s, c)))
    # generating inter-points
    left_edge = np.zeros((2, n), dtype=np.float16)
    centerline = np.zeros((2, n), dtype=np.float16)
    right_edge = np.zeros((2, n), dtype=np.float16)
    templine = backline
    left_edge[0, 0] = backline[0, 0]
    left_edge[1, 0] = backline[1, 0]
    centerline[0, 0] = backline[0, 1]
    centerline[1, 0] = backline[1, 1]
    right_edge[0, 0] = backline[0, 2]
    right_edge[1, 0] = backline[1, 2]
    for i in range(n-1):
        ''' translation matrix '''
        left_point = templine.T[0]
        center_point = templine.T[1]
        right_point = templine.T[2]
        width_vector = right_point - center_point
        # find center of arc
        if angle < 0:
            arc_point = center_point + width_vector * radius / road_width
        else:
            arc_point = center_point - width_vector * radius / road_width
        translation = np.matmul(np.identity(2) - rotation, arc_point)
        lp = np.matmul(rotation, left_point)+translation
        left_edge[0, i+1] = lp[0]
        left_edge[1, i+1] = lp[1]
        cp = np.matmul(rotation, center_point) + translation
        centerline[0, i+1] = cp[0]
        centerline[1, i+1] = cp[1]
        rp = np.matmul(rotation, right_point) + translation
        right_edge[0, i+1] = rp[0]
        right_edge[1, i+1] = rp[1]
        templine = np.concatenate([lp, cp, rp]).reshape(3,2).T
    frontline = templine
    return RoadSegment(backline, frontline, centerline, right_edge, left_edge, True, radius, angle)

def CreateRandomRoadSegment(backline, map_x, map_y):
    avg_map_size = (map_x + map_y)//2
    ''' Randomly select the road type - Straight road vs Arc road => percentage = 2: 1'''
    road_type = random.choice([0, 1])
    if road_type == 0:
        ''' Generate Straight Road '''
        length = random.randint(3, 6)*avg_map_size//100
        segment = CreateStraigthRoadSegment(backline, length, True)
    else:
        ''' Generate Arc Road '''
        angle_bound = [40, 60]
        width = math.sqrt((backline[0,0] - backline[0,1])**2 + (backline[1,0] - backline[1,1])**2)
        radius = random.uniform(7, 9)*width
        angle = random.choice([-1,1])*random.uniform(angle_bound[0], angle_bound[1])
        segment = CreateArcRoadSegment(backline, radius, angle)

    return segment

def CreateRandomRoad(map_x, map_y, width):
    ''' Get random start point from map size '''
    width = width/2
    map_size = [map_x, map_y]
    x_bound = [0, map_size[0]]
    y_bound = [0, map_size[1]]
    start_axis = random.choice([0, 1])
    start_point = [0, 0]
    if start_axis == 0:
        start_point = [random.uniform(width*2, map_size[0]-width*2), 0]
        backline = np.array(([start_point[0]-width, start_point[0], start_point[0]+width], [start_point[1], start_point[1], start_point[1]]), dtype=np.float16)
    else:
        start_point = [0, random.uniform(width*2, map_size[1]-width*2)]
        backline = np.array(([start_point[0], start_point[0], start_point[0]], [start_point[1]+width, start_point[1], start_point[1]-width]), dtype=np.float16)

    ''' Create first straight road segment '''
    start_len = map_size[start_axis]//20 # Len of first straight road  = map size / 20 (5%)
    result_road = Road(CreateStraigthRoadSegment(backline, start_len, True))

    ''' Autonomously generate road segment until reach the edge of map '''
    backline = result_road.lastSegment().getFront()
    invalid_count = 0
    success = True
    while not checkOutOfMap(result_road, x_bound, y_bound):
        #print(backline)
        new_segment = CreateRandomRoadSegment(backline, map_x, map_y)
        result_road.pushRoadSegment(new_segment)

        ''' Check whether road is a valid road '''
        if not isValidRoad(result_road):
            # print('Invalid road segement')
            result_road.popRoadSegment() # remove last segment
            #result_road.popRoadSegment()  # remove last segment
            #result_road.popRoadSegment()  # remove last segment
            #result_road.popRoadSegment()  # remove last segment
            #result_road.popRoadSegment()  # remove last segment
            invalid_count += 1
        else:
            invalid_count = 0

        if invalid_count > 5:
            #print('Road Generation Failed')
            return False, None

        ''' Update the next backline'''
        backline = result_road.lastSegment().getFront()

    #print('Road Generation success')
    return success, result_road

def CreateRandomRoadNetwork(map_x, map_y, width):
    ''' Randomly choice the number of road in road network '''
    road_number = random.choice([2,3])
    # road_number = 1
    
    print(f'Generate {road_number} roads')
    result_network = RoadNetwork(map_x, map_y)
    i = 0
    while i < road_number:
        print(f'road {i}')
        start = time.time()
        sucess, new_road = CreateRandomRoad(map_x, map_y, width)
        exe_time = time.time() - start
        print(f'CreateRandomRoad time : {exe_time}')
        if sucess == False:
            continue
        road_len = new_road.getCenterline().shape[1]
        if road_len < (map_x + map_y)//3:
            #print('Too much short road')
            continue
        # ''' Check new road is valid '''
        # if not isValidRoad(new_road):
        #     print(f'[{i}] Not valid road')
        #     continue
        result_network.addRoad(new_road)

        ''' Check network is valid '''
        if not isValidNetworkLast(result_network):
            result_network.deleteLastRoad()
            continue
        i += 1
    #print('done')
    return result_network

def isValidRoad(road):
    return not checkSelfIntersect(road)

def checkSelfIntersect(road):
    for i in range(road.segmentNumber-1):
        curr_segment = road.path[i]
        curr_right, curr_left = curr_segment.getRight(), curr_segment.getLeft()
        for j in range(i+1, road.segmentNumber):
            target_segment = road.path[j]
            target_right, target_left = target_segment.getRight(), target_segment.getLeft()
            ''' Test right_edge & right_edge, left_edge & right_edge, right_edge & left_edge, left_edge & left_edge '''
            test_list = [(curr_right, target_right), (curr_left, target_right), (curr_right, target_left), (curr_left, target_left)]
            for test in test_list:
                ''' Check there are some intersection between two segments '''
                # intersects, _ = intersection(test[0][0, 1:], test[0][1, 1:], test[1][0, 1:], test[1][1, 1:])
                # if len(intersects) > 0:
                #     return True
                if checkIntersection(test[0][0, 1:], test[0][1, 1:], test[1][0, 1:], test[1][1, 1:]):
                    return True
    return False

def checkOutOfMap(road, x_bound, y_bound):
    last_segment = road.lastSegment()
    frontline = last_segment.getFront()
    result = False
    for i in range(3):
        if frontline[0, i] < x_bound[0] or frontline[0, i] > x_bound[1]:
            result = True
            break
        if frontline[1, i] < y_bound[0] or frontline[1, i] > y_bound[1]:
            result = True
            break
    return result

def isValidNetworkLast(road_network: RoadNetwork):
    print("isValidNetworkLast")
    road_number = road_network.roadNumber
    ''' Road network that have only one road is valid '''
    if road_number < 2:
        return True
    intersect_count = 0
    start = time.time()
    for i in range(0, road_number-1):
        partial_result, intersects = checkPartialOverlap(road_network.roads[-1], road_network.roads[i])
        intersect_count += len(intersects[0])
        ''' if partial overlap, return False '''
        if partial_result:
            return False
    exe_time = time.time() - start
    print(f'isValidNetworkLast time : {exe_time}')
    #print(f'No intersect test: {intersect_count == 0}')
    ''' if partial overlap, return False '''
    if intersect_count == 0:
        return False
    return True

def isValidNetwork(road_network: RoadNetwork):
    print("isValidNetwork")
    road_number = road_network.roadNumber
    ''' Road network that have only one road is valid '''
    if road_number < 2:
        return True
    for i in range(road_number):
        intersect_count = 0
        for j in range(road_number):
            if i == j:
                continue
            partial_result, intersects = checkPartialOverlap(road_network.roads[i], road_network.roads[j])
            intersect_count += len(intersects[0])
            ''' if partial overlap, return False '''
            if partial_result:
                return False
        #print(f'No intersect test: {intersect_count == 0}')
        ''' if partial overlap, return False '''
        if intersect_count == 0:
            return False
    return True

def checkPartialOverlap(road1, road2):
    ''' Get each road element (Left Edge, Center Line, Right Edge)'''
    road1_lines = [road1.getLeftEdge(), road1.getCenterline(), road1.getRightEdge()]
    road2_lines = [road2.getLeftEdge(), road2.getCenterline(), road2.getRightEdge()]

    intersects = [[], [], []]
    for r in range(3):
        road_len = road1_lines[r].shape[1]
        for i in range(road_len-1):
            for j in range(3):
                ''' Get intersection point between two line element '''
                curr_intersect = intersection(road1_lines[r][0, i:i+2], road1_lines[r][1, i:i+2], road2_lines[j][0, :], road2_lines[j][1, :])
                if len(curr_intersect[0]) > 0:
                    if len(intersects[r]) > 0:
                        road_type, prev_intersect = intersects[r][-1]
                        try:
                            ''' if detect the same intersection point, ignore it '''
                            if road_type == j and getDistance(prev_intersect, curr_intersect) < 0.5:
                                #print(f'invalid intersect [{i}] \n{curr_intersect}')
                                continue
                        except:
                            return True, [[], [], []]
                    ''' Save (line type, intersection point) '''
                    intersects[r].append((j, curr_intersect))
                    break
    ''' Remain only line type '''
    for i in range(3):
        intersects[i] = list(map(lambda x: x[0], intersects[i]))

    #print(intersects)
    #print(f'Partial overlap test: {not (intersects[0] == intersects[1] and intersects[1] == intersects[2])}')

    ''' Determine whether partial overlap occur '''
    result = not (intersects[0] == intersects[1] and intersects[1] == intersects[2])
    return result, intersects

def getDistance(v1, v2):
    x1, y1 = v1[0], v1[1]
    x2, y2 = v2[0], v2[1]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)