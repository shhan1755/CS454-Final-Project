import math
import random
import copy
import numpy as np
from Road import *

def crossover_method(roadnetwork_list, method):
    invalid_count = 0
    while invalid_count < 30:
        if method == 'join':
            valid, result_road_network = join(roadnetwork_list[0], roadnetwork_list[1])
        elif method == 'merge':
            valid, result_road_network = merge(roadnetwork_list[0], roadnetwork_list[1])
        elif method == 'mutation':
            valid, result_road_network = mutation(roadnetwork_list[0])

        if valid and isValidNetwork(result_road_network):
            #print("Make successful crossover road network")
            return True, result_road_network
        invalid_count += 1
    #print("Fail to make crossover road network")
    return False, None

def join(roadNetwork_1, roadNetwork_2):
    roadNum_1 = roadNetwork_1.roadNumber
    roadNum_2 = roadNetwork_2.roadNumber
    map_x, map_y = roadNetwork_1.mapsize
    roads_1 = copy.deepcopy(roadNetwork_1.roads)
    random.shuffle(roads_1)
    roads_2 = copy.deepcopy(roadNetwork_2.roads)
    random.shuffle(roads_2)

    def join_init(road_1,road_2):
        segmentNumber_1 = road_1.segmentNumber
        segmentNumber_2 = road_2.segmentNumber

        join_pos_1 = random.randint(1,segmentNumber_1-2)
        join_pos_2 = random.randint(1,segmentNumber_2-2)

        result_road = copy.deepcopy(road_1)
        result_road.segmentNumber = join_pos_1
        result_road.path = result_road.path[:join_pos_1]

        join_road = copy.deepcopy(road_2)
        join_road.segmentNumber = segmentNumber_2 - join_pos_2
        join_road.path = join_road.path[join_pos_2:]

        backline = result_road.lastSegment().getFront()
        while (not checkOutOfMap(result_road,[0,map_x], [0,map_y])):
            if(len(join_road.path) > 0):
                join_segment = join_road.path.pop(0)
                if (join_segment.isarc):
                    new_segment = CreateArcRoadSegment(backline, join_segment.radius, join_segment.angle)
                else:
                    new_segment = CreateStraigthRoadSegment(backline, join_segment.length, True)
            else:
                new_segment = CreateRandomRoadSegment(backline, map_x, map_y)
            result_road.pushRoadSegment(new_segment)
            backline = result_road.lastSegment().getFront()

        return result_road

    result_roadNum = min(roadNum_1,roadNum_2)
    result_roads = []

    for i in range(0, result_roadNum):
        invalid_count = 0
        while True:
            if (random.randint(1,2)==1):
                tmp_result_road = join_init(roads_1[i],roads_2[i])
            else:
                tmp_result_road = join_init(roads_2[i],roads_1[i])
            
            if isValidRoad(tmp_result_road):
                result_roads.append(tmp_result_road)
                break
            else:
                invalid_count+=1
                
            if invalid_count > 10:
                return False, None

    result_network = RoadNetwork(map_x, map_y)
    for i in result_roads:
        result_network.addRoad(i)

    
    return True, result_network

def merge(roadNetwork_1, roadNetwork_2):
    roadNum_1 = roadNetwork_1.roadNumber
    roadNum_2 = roadNetwork_2.roadNumber
    map_x, map_y = roadNetwork_1.mapsize

    num_1 = random.randint(1,min(roadNum_1,2))
    if (num_1 == 2):
        num_2 = 1
    else:
        num_2 = random.randint(1,min(roadNum_2,2))

    road_1 = copy.deepcopy(roadNetwork_1.roads)
    random.shuffle(road_1)
    road_2 = copy.deepcopy(roadNetwork_2.roads)
    random.shuffle(road_2)

    result_network = RoadNetwork(map_x, map_y)
    for i in range(num_1):
        result_network.addRoad(road_1[i])
    for i in range(num_2):
        result_network.addRoad(road_2[i])
    
    return True, result_network

def mutation(roadNetwork):
    roadNum = roadNetwork.roadNumber
    map_x, map_y = roadNetwork.mapsize
    mut_roads = copy.deepcopy(roadNetwork.roads)

    mut_Num = random.randint(0,roadNum-1)
    mut_road = mut_roads[mut_Num]

    mut_road_num = mut_road.segmentNumber
    mut_road_pos = random.randint(1,mut_road_num-2)

    mut_road.segmentNumber = mut_road_pos
    mut_road.path = mut_road.path[:mut_road_pos]
    
    backline = mut_road.lastSegment().getFront()
    invalid_count = 0

    while not checkOutOfMap(mut_road,[0,map_x], [0,map_y]):
        new_segment = CreateRandomRoadSegment(backline, map_x, map_y)
        mut_road.pushRoadSegment(new_segment)
        ''' Check whether road is a valid road '''
        if not isValidRoad(mut_road):
            mut_road.popRoadSegment() # remove last segment
            invalid_count += 1
        else:
            invalid_count = 0

        if invalid_count > 5:
            return False, None

        ''' Update the next backline'''
        backline = mut_road.lastSegment().getFront()

    mut_roads[mut_Num] = mut_road
    result_network = RoadNetwork(map_x, map_y)
    for i in mut_roads:
        result_network.addRoad(i)
    
    return True, result_network





