import math
import random
import numpy as np
import copy
from matplotlib import pyplot as plt
from Road import *

class Intersection:
    def __init(self, road_idx, segment_idx):
        self.road = road_idx
        self.segment = segment_idx

class Node:
    def __init__(self):
        self.intersect_num = 0
        self.intersection_list = []
        self.node_num = 0
        self.x = 0
        self.y = 0
        self.inter_tar = [-1, -1, -1, -1, -1]
        self.inter_com = [-1, -1, -1, -1, -1]
        self.is_end = False

    def add_intersection(self, intersection):
        self.intersect_num = self.intersect_num + 1
        self.intersection_list.append(intersection)

class Graph:
    def __init__(self):
        self.node_list = []
        self.adjacency_matrix = []
        self.num_nodes = 0
        self.num_edges = 0

    def add_node(self, node):
        node.node_num = self.num_nodes
        self.num_nodes += 1
        self.node_list.append(node)
        self.adjacency_matrix.append([])
    
    def add_edge(self, node_1, node_2):
        self.num_edges += 1

        #idx_1 = self.get_node_index(node_1)
        idx_1 = node_1.node_num
        if (idx_1 == -1):
            print("ERROR: There should exist Node")

        #idx_2 = self.get_node_index(node_2)
        idx_2 = node_2.node_num
        if (idx_2 == -1):
            print("ERROR: There should exist Node")

        road_idx = -1
        weight = 0
        for intersect_1 in node_1.intersection_list:
            for intersect_2 in node_2.intersection_list:
                if (intersect_1.road == intersect_2.road):
                    road_idx = intersect_1.road
                    if (intersect_1.segment > intersect_2.segment):
                        weight = intersect_1.segment - intersect_2.segment
                    else:
                        weight = intersect_2.segment - intersect_1.segment
                    break
        
        if (road_idx == -1):
            print("ERROR: There should exist same road")
        
        self.adjacency_matrix[idx_1].append([idx_2, weight])
        self.adjacency_matrix[idx_2].append([idx_1, weight])

    def get_node_index(self, tar_node):
        for com_node_idx in range(self.num_nodes):
            com_node = self.node_list[com_node_idx]
            for tar_intersect in tar_node.intersection_list:
                tar_road_idx = tar_intersect.road
                tar_segment_idx = tar_intersect.segment
                for com_intersect in com_node.intersection_list:
                    if ((tar_road_idx == com_intersect.road) and (tar_segment_idx == com_intersect.segment)):
                        return com_node_idx
        return -1

    def find_node(self, road_idx, segment_idx):
        for node in self.node_list:
            ret_node = node
            for intersect in node.intersection_list:
                if ((intersect.road == road_idx) and (intersect.segment == segment_idx)):
                    return ret_node
        return None

    def print_nodelist(self):
        print("Node List")
        for node_idx in range(self.num_nodes):
            print("This is Node: ", node_idx)
            node = self.node_list[node_idx]
            for intersect in node.intersection_list:
                print(intersect.road, intersect.segment)
    
    def print_adjacency_matrix(self):
        print("Adjacency Matrix")
        for node_idx in range(self.num_nodes):
            print("This is Node ", node_idx, ": ", self.adjacency_matrix[node_idx])

def GraphHwa_full(Road_net):
    #print("GraphHwa_full start")
    road_num = len(Road_net.roads)
    graph = Graph()

    if (road_num == 0):
        print("There is no road within the network")
        return graph

    # Clean up the road's NodeList
    #print("Clean Up NodeList")
    for road in Road_net.roads:
        road.clean_NodeList()
    
    # Find out the nodes
    #print("Add Nodes")
    for tar_road_idx in range(road_num):
        tar_road = Road_net.roads[tar_road_idx]
        tar_NodeList = tar_road.NodeList
        prev_node_x = -1
        prev_node_y = -1
        for tar_segment_idx in range(len(tar_road.path)):
            if (tar_segment_idx not in tar_NodeList):
                tar_segment = tar_road.path[tar_segment_idx]
                # Initialize Node
                node = Node()
                # Add Intersection
                intersection = Intersection()
                intersection.road = tar_road_idx
                intersection.segment = tar_segment_idx
                node.add_intersection(intersection)
                cur_tar_x_point = tar_segment.centerline[0][0]
                cur_tar_y_point = tar_segment.centerline[1][0]

                if (tar_segment_idx == 0):
                    node.x = tar_segment.centerline[0][0]
                    node.y = tar_segment.centerline[1][0]
                elif (tar_segment_idx == (len(tar_road.path) - 1)):
                    node.x = tar_segment.centerline[0][-1]
                    node.y = tar_segment.centerline[1][-1]
                else:
                    node.x = cur_tar_x_point
                    node.y = cur_tar_y_point

                    end_signal = 1
                    tar_point_idx = 1
                    while ((end_signal == 1) and (tar_point_idx < len(tar_segment.centerline[0]))):
                        prev_tar_x_point = cur_tar_x_point
                        prev_tar_y_point = cur_tar_y_point
                        cur_tar_x_point = tar_segment.centerline[0][tar_point_idx]
                        cur_tar_y_point = tar_segment.centerline[1][tar_point_idx]
                        for com_road_idx in range(tar_road_idx + 1, road_num):
                            com_road = Road_net.roads[com_road_idx]
                            com_NodeList = com_road.NodeList
                            for com_segment_idx in range(1, len(com_road.path) - 1):
                                if (com_segment_idx not in com_NodeList):
                                    com_segment = com_road.path[com_segment_idx]
                                    cur_com_x_point = com_segment.centerline[0][0]
                                    cur_com_y_point = com_segment.centerline[1][0]
                                    for com_point_idx in range(1, len(com_segment.centerline[0])):
                                        prev_com_x_point = cur_com_x_point
                                        prev_com_y_point = cur_com_y_point
                                        cur_com_x_point = com_segment.centerline[0][com_point_idx]
                                        cur_com_y_point = com_segment.centerline[1][com_point_idx]
                                        bool_intersect, x_pos, y_pos = checksegmentIntersection([float(cur_tar_x_point), float(prev_tar_x_point)], [float(cur_tar_y_point), float(prev_tar_y_point)], [float(cur_com_x_point), float(prev_com_x_point)], [float(cur_com_y_point), float(prev_com_y_point)])
                                        if (bool_intersect and ((x_pos != prev_node_x) or (y_pos != prev_node_y))):                                        
                                            #Add Intersection
                                            #print("Add intersection: ", com_road_idx, com_segment_idx)
                                            intersection = Intersection()
                                            intersection.road = com_road_idx
                                            intersection.segment = com_segment_idx
                                            node.add_intersection(intersection)
                                            node.x = x_pos
                                            node.y = y_pos
                                            node.inter_tar = [tar_road_idx, prev_tar_x_point, prev_tar_y_point, cur_tar_x_point, cur_tar_y_point]
                                            node.inter_com = [com_road_idx, prev_com_x_point, prev_com_y_point, cur_com_x_point, cur_com_y_point]
                                            com_road.add_NodeList(com_segment_idx)
                                            end_signal = 1

                                #else:
                                    #if (tar_road_idx==1 and tar_segment_idx==3):
                                        #print("SKIP COM: ", com_road_idx, com_segment_idx)
                        
                        tar_point_idx = tar_point_idx + 1

                if (tar_segment_idx == 0):
                    #print("Add Starting Node: ", node.x, node.y)
                    node.is_end = True
                    prev_node_x = node.x
                    prev_node_y = node.y
                    tar_road.add_NodeList(tar_segment_idx)
                    graph.add_node(node)
                elif (tar_segment_idx == len(tar_road.path) - 1):
                    #print("Add Ending Node: ", node.x, node.y)
                    node.is_end = True
                    prev_node_x = node.x
                    prev_node_y = node.y
                    tar_road.add_NodeList(tar_segment_idx)
                    graph.add_node(node)
                else:
                    if (node.intersect_num >= 2):
                        node.is_end = False
                        for inst in node.intersection_list:
                            if ((inst.segment == 0) or (inst.segment == (len(Road_net.roads[inst.road].path) - 1))):
                                node.is_end = True
                        #print("Add Node: ", node.x, node.y, node.intersect_num)
                        tar_road.add_NodeList(tar_segment_idx)
                        prev_node_x = node.x
                        prev_node_y = node.y
                        graph.add_node(node)
            #else:
                #if (tar_road_idx==1):
                    #print("SKIP TAR: ", tar_road_idx, tar_segment_idx)

    # Node result
    #graph.print_nodelist()

    # Connecting the Node to make Edge
    #print("Adding Edge")
    #graph.print_adjacency_matrix()
    for road_idx in range(Road_net.roadNumber):
        road = Road_net.roads[road_idx]
        #print("Nodes within road ", road_idx, ": ", road.NodeList)
        cur_node_idx = road.NodeList[0]
        cur_node = graph.find_node(road_idx, cur_node_idx)
        if (cur_node == None):
            print("Error: There should be already added Node", road_idx, cur_node_idx)
            return None
        for node_idx in range(1, road.NodeNum):
            prev_node_idx = cur_node_idx
            prev_node = cur_node
            cur_node_idx = road.NodeList[node_idx]
            cur_node = graph.find_node(road_idx, cur_node_idx)
            if (cur_node == None):
                print("Error: There should be already added Node", road_idx, cur_node_idx)
                return None
            else:
                #print("Add Edge: ", cur_node.node_num, prev_node.node_num)
                graph.add_edge(cur_node, prev_node)

    # Edge result
    #`graph.print_adjacency_matrix()

    #print("GraphHwa_full End")
    return graph

def checksegmentIntersection(cur_x, cur_y, tar_x, tar_y):
    #print("tar_1: ", tar_x[0], tar_y[0], "tar_2: ", tar_x[1], tar_y[1])
    a = cur_x[1] - cur_x[0]
    b = cur_y[1] - cur_y[0]
    c = tar_x[1] - tar_x[0]
    d = tar_y[1] - tar_y[0]
    m = a*d - b*c
    if (m == 0):
        # Means parallel line or same point
        return False, 0, 0

    x = (a*d*tar_x[0] - a*c*tar_y[0] - b*c*cur_x[0] + a*c*cur_y[0])/m
    y = (b*d*tar_x[0] - b*c*tar_y[0] - b*d*cur_x[0] + a*d*cur_y[0])/m

    if (cur_x[1] > cur_x[0]):
        if cur_x[0] <= x <= cur_x[1]:
            pass
        else:
            return False, 0, 0
    else:
        if cur_x[1] <= x <= cur_x[0]:
            pass
        else:
            return False, 0, 0

    if (cur_y[1] > cur_y[0]):
        if cur_y[0] <= y <= cur_y[1]:
            pass
        else:
            return False, 0, 0
    else:
        if cur_y[1] <= y <= cur_y[0]:
            pass
        else:
            return False, 0, 0

    if (tar_x[1] > tar_x[0]):
        if tar_x[0] <= x <= tar_x[1]:
            pass
        else:
            return False, 0, 0
    else:
        if tar_x[1] <= x <= tar_x[0]:
            pass
        else:
            return False, 0, 0

    if (tar_y[1] > tar_y[0]):
        if tar_y[0] <= y <= tar_y[1]:
            pass
        else:
            return False, 0, 0
    else:
        if tar_y[1] <= y <= tar_y[0]:
            pass
        else:
            return False, 0, 0

    #print("GOOD", x, y)
    return True, x, y

def Make_path(road_net, graph, iteration):
    iter = 0
    roads = road_net.roads
    am = graph.adjacency_matrix
    max_path = []
    max_path_num = 0
    
    while (iter < iteration):
        remain_node = []
        for idx in range(graph.num_nodes):
            remain_node.append(graph.node_list[idx].node_num)
        path = []
        path_num = 0
        re_start = False

        # Choose random starting point - Can be improved JW with increasing efficency
        start_road_idx = random.choice(list(range(len(roads))))
        start_road = roads[start_road_idx]
        start_segment_idx = random.choice([start_road.NodeList[0], start_road.NodeList[-1]])
        start_node = graph.find_node(start_road_idx, start_segment_idx)
        if (start_node.is_end != True):
            print("This node is not the end")
            re_start = True
        path.append(start_node)
        remain_node.remove(start_node.node_num)

        # Find next node - Can be improved JW with adding probability
        possible_edge_list = copy.deepcopy(am[start_node.node_num])
        next_edge = random.choice(possible_edge_list)
        possible_edge_list.remove(next_edge)
        while ((next_edge[0] not in remain_node) and (re_start != True)):
            if (len(possible_edge_list) == 0):
                print("The path is not terminated within the end")
                re_start = True
                break
            next_edge = random.choice(possible_edge_list)
            possible_edge_list.remove(next_edge)

        if (re_start != True):
            next_node_idx = next_edge[0]
            weight = next_edge[1]
            next_node = graph.node_list[next_node_idx]
            path.append(next_node)
            path_num = path_num + weight
            remain_node.remove(next_node_idx)

            while ((next_node.is_end != True) and (re_start != True)):
                possible_edge_list = copy.deepcopy(am[next_node.node_num])
                next_edge = random.choice(possible_edge_list)
                possible_edge_list.remove(next_edge)
                while ((next_edge[0] not in remain_node) and (re_start != True)):
                    if (len(possible_edge_list) == 0):
                        print("The path is not terminated within the end")
                        re_start = True
                        break
                    next_edge = random.choice(possible_edge_list)
                    possible_edge_list.remove(next_edge)
                if (re_start != True):
                    next_node_idx = next_edge[0]
                    weight = next_edge[1]
                    next_node = graph.node_list[next_node_idx]
                    path.append(next_node)
                    path_num = path_num + weight
                    remain_node.remove(next_node_idx)

            #print("Path ", iter, ": ", path_num)
            #for node in path:
            #    print(node.node_num)

            if (re_start != True):
                if (path_num > max_path_num):
                    max_path = path
                    max_path_num = path_num
                
                if (re_start == False):
                    iter = iter + 1

    return max_path

def Route_path(road_net, graph, path):
    line_x = []
    line_y = []
    cur_node = path[0]
    for node_idx in range(1, len(path)):
        prev_node = cur_node
        cur_node = path[node_idx]
        road_num = -1
        prev_segment_idx = -1
        cur_segment_idx = -1
        for prev_intersect in prev_node.intersection_list:
            for cur_intersect in cur_node.intersection_list:
                if (prev_intersect.road == cur_intersect.road):
                    temp_road_num = prev_intersect.road
                    Node_seq = road_net.roads[temp_road_num].NodeList
                    temp_prev_segment_idx = prev_intersect.segment
                    temp_cur_segment_idx = cur_intersect.segment
                    if (temp_prev_segment_idx < temp_cur_segment_idx):
                        for s_idx in range(len(Node_seq)-1):
                            if ((Node_seq[s_idx] == temp_prev_segment_idx) and (Node_seq[s_idx + 1] == temp_cur_segment_idx)):
                                road_num = temp_road_num
                                prev_segment_idx = temp_prev_segment_idx
                                cur_segment_idx = temp_cur_segment_idx
                    else:
                        for s_idx in range(len(Node_seq)-1):
                            if ((Node_seq[s_idx] == temp_cur_segment_idx) and (Node_seq[s_idx + 1] == temp_prev_segment_idx)):
                                road_num = temp_road_num
                                prev_segment_idx = temp_prev_segment_idx
                                cur_segment_idx = temp_cur_segment_idx

        road = road_net.roads[road_num]

        first_done = True
        second_done = True

        if (node_idx == 1):
            if (road_num == cur_node.inter_tar[0]):
                second_prev_x = cur_node.inter_tar[1]
                second_prev_y = cur_node.inter_tar[2]
                second_cur_x = cur_node.inter_tar[3]
                second_cur_y = cur_node.inter_tar[4]
            elif (road_num == cur_node.inter_com[0]):
                second_prev_x = cur_node.inter_com[1]
                second_prev_y = cur_node.inter_com[2]
                second_cur_x = cur_node.inter_com[3]
                second_cur_y = cur_node.inter_com[4]
            else:
                second_done = False
            
            second_segment = road.path[cur_segment_idx]
            second_x_line = second_segment.centerline[0]
            second_y_line = second_segment.centerline[1]
            second_x = []
            second_y = []
            second_idx = 0

            if (second_done):
                for idx in range(len(second_x_line) - 1):
                    if ((second_x_line[idx] == second_prev_x) and (second_y_line[idx] == second_prev_y) and (second_x_line[idx+1] == second_cur_x) and (second_y_line[idx+1] == second_cur_y)):
                        second_idx = idx + 1
        
            if (prev_segment_idx < cur_segment_idx):
                for segment_idx in range(prev_segment_idx, cur_segment_idx):
                    segment = road.path[segment_idx]
                    line_x.append(segment.centerline[0])
                    line_y.append(segment.centerline[1])
                if (second_done):
                    for idx in range(0, second_idx):
                        second_x.append(second_x_line[idx])
                        second_y.append(second_y_line[idx])
                    line_x.append(second_x)
                    line_y.append(second_y)
            else:
                back_list = list(range(cur_segment_idx + 1, prev_segment_idx + 1))
                back_list.reverse()
                for segment_idx in back_list:
                    segment = road.path[segment_idx]
                    line_x.append(np.flip(segment.centerline[0]))
                    line_y.append(np.flip(segment.centerline[1]))
                if (second_done):
                    back_second = list(range(second_idx, len(second_x_line)))
                    back_second.reverse()
                    for idx in back_second:
                        second_x.append(second_x_line[idx])
                        second_y.append(second_y_line[idx])
                    line_x.append(second_x)
                    line_y.append(second_y)
            cur_line_x = []
            cur_line_y = []
            for i in range(2):
                cur_line_x.append(cur_node.x)
                cur_line_y.append(cur_node.y)
            line_x.append(cur_line_x)
            line_y.append(cur_line_y)
        elif (node_idx == len(path) - 1):
            if (road_num == prev_node.inter_tar[0]):
                first_prev_x = prev_node.inter_tar[1]
                first_prev_y = prev_node.inter_tar[2]
                first_cur_x = prev_node.inter_tar[3]
                first_cur_y = prev_node.inter_tar[4]
            elif (road_num == prev_node.inter_com[0]):
                first_prev_x = prev_node.inter_com[1]
                first_prev_y = prev_node.inter_com[2]
                first_cur_x = prev_node.inter_com[3]
                first_cur_y = prev_node.inter_com[4]
            else:
                first_done = False

            first_segment = road.path[prev_segment_idx]
            first_x_line = first_segment.centerline[0]
            first_y_line = first_segment.centerline[1]
            first_x = []
            first_y = []
            first_idx = 0

            if (first_done):
                for idx in range(len(first_x_line) - 1):
                    if ((first_x_line[idx] == first_prev_x) and (first_y_line[idx] == first_prev_y) and (first_x_line[idx+1] == first_cur_x) and (first_y_line[idx+1] == first_cur_y)):
                        first_idx = idx + 1

            if (prev_segment_idx < cur_segment_idx):
                if (first_done):
                    for idx in range(first_idx, len(first_x_line)):
                        first_x.append(first_x_line[idx])
                        first_y.append(first_y_line[idx])
                    line_x.append(first_x)
                    line_y.append(first_y)
                for segment_idx in range(prev_segment_idx + 1, cur_segment_idx + 1):
                    segment = road.path[segment_idx]
                    line_x.append(segment.centerline[0])
                    line_y.append(segment.centerline[1])
            else:
                if (first_done):
                    back_first = list(range(0, first_idx))
                    back_first.reverse()
                    for idx in back_first:
                        first_x.append(first_x_line[idx])
                        first_y.append(first_y_line[idx])
                    line_x.append(first_x)
                    line_y.append(first_y)
                back_list = list(range(cur_segment_idx, prev_segment_idx))
                back_list.reverse()
                for segment_idx in back_list:
                    segment = road.path[segment_idx]
                    line_x.append(np.flip(segment.centerline[0]))
                    line_y.append(np.flip(segment.centerline[1]))
        else:
            if (road_num == prev_node.inter_tar[0]):
                first_prev_x = prev_node.inter_tar[1]
                first_prev_y = prev_node.inter_tar[2]
                first_cur_x = prev_node.inter_tar[3]
                first_cur_y = prev_node.inter_tar[4]
            elif (road_num == prev_node.inter_com[0]):
                first_prev_x = prev_node.inter_com[1]
                first_prev_y = prev_node.inter_com[2]
                first_cur_x = prev_node.inter_com[3]
                first_cur_y = prev_node.inter_com[4]
            else:
                first_done = False

            if (road_num == cur_node.inter_tar[0]):
                second_prev_x = cur_node.inter_tar[1]
                second_prev_y = cur_node.inter_tar[2]
                second_cur_x = cur_node.inter_tar[3]
                second_cur_y = cur_node.inter_tar[4]
            elif (road_num == cur_node.inter_com[0]):
                second_prev_x = cur_node.inter_com[1]
                second_prev_y = cur_node.inter_com[2]
                second_cur_x = cur_node.inter_com[3]
                second_cur_y = cur_node.inter_com[4]
            else:
                second_done = False

            first_segment = road.path[prev_segment_idx]
            first_x_line = first_segment.centerline[0]
            first_y_line = first_segment.centerline[1]
            first_x = []
            first_y = []
            first_idx = 0

            if (first_done):
                for idx in range(len(first_x_line) - 1):
                    if ((first_x_line[idx] == first_prev_x) and (first_y_line[idx] == first_prev_y) and (first_x_line[idx+1] == first_cur_x) and (first_y_line[idx+1] == first_cur_y)):
                        first_idx = idx + 1
            
            second_segment = road.path[cur_segment_idx]
            second_x_line = second_segment.centerline[0]
            second_y_line = second_segment.centerline[1]
            second_x = []
            second_y = []
            second_idx = 0
            
            if (second_done):
                for idx in range(len(second_x_line) - 1):
                    if ((second_x_line[idx] == second_prev_x) and (second_y_line[idx] == second_prev_y) and (second_x_line[idx+1] == second_cur_x) and (second_y_line[idx+1] == second_cur_y)):
                        second_idx = idx + 1
        
            if (prev_segment_idx < cur_segment_idx):
                if (first_done):
                    for idx in range(first_idx, len(first_x_line)):
                        first_x.append(first_x_line[idx])
                        first_y.append(first_y_line[idx])
                    line_x.append(first_x)
                    line_y.append(first_y)
                for segment_idx in range(prev_segment_idx + 1, cur_segment_idx):
                    segment = road.path[segment_idx]
                    line_x.append(segment.centerline[0])
                    line_y.append(segment.centerline[1])
                if (second_done):
                    for idx in range(0, second_idx):
                        second_x.append(second_x_line[idx])
                        second_y.append(second_y_line[idx])
                    line_x.append(second_x)
                    line_y.append(second_y)
            else:
                if (first_done):
                    back_first = list(range(0, first_idx))
                    back_first.reverse()
                    for idx in back_first:
                        first_x.append(first_x_line[idx])
                        first_y.append(first_y_line[idx])
                    line_x.append(first_x)
                    line_y.append(first_y)
                back_list = list(range(cur_segment_idx + 1, prev_segment_idx))
                back_list.reverse()
                for segment_idx in back_list:
                    segment = road.path[segment_idx]
                    line_x.append(np.flip(segment.centerline[0]))
                    line_y.append(np.flip(segment.centerline[1]))
                back_second = list(range(second_idx, len(second_x_line)))
                back_second.reverse()
                if (second_done):
                    for idx in back_second:
                        second_x.append(second_x_line[idx])
                        second_y.append(second_y_line[idx])
                    line_x.append(second_x)
                    line_y.append(second_y)
            cur_line_x = []
            cur_line_y = []
            for i in range(2):
                cur_line_x.append(cur_node.x)
                cur_line_y.append(cur_node.y)
            line_x.append(cur_line_x)
            line_y.append(cur_line_y)

    return Merge_path([line_x, line_y])

def Route_path_print(road_net, graph, path):
    line_x = []
    line_y = []
    cur_node = path[0]
    for node_idx in range(1, len(path)):
        prev_node = cur_node
        cur_node = path[node_idx]
        road_num = -1
        prev_segment_idx = -1
        cur_segment_idx = -1
        for prev_intersect in prev_node.intersection_list:
            for cur_intersect in cur_node.intersection_list:
                if (prev_intersect.road == cur_intersect.road):
                    temp_road_num = prev_intersect.road
                    Node_seq = road_net.roads[temp_road_num].NodeList
                    temp_prev_segment_idx = prev_intersect.segment
                    temp_cur_segment_idx = cur_intersect.segment
                    if (temp_prev_segment_idx < temp_cur_segment_idx):
                        for s_idx in range(len(Node_seq)-1):
                            if ((Node_seq[s_idx] == temp_prev_segment_idx) and (Node_seq[s_idx + 1] == temp_cur_segment_idx)):
                                road_num = temp_road_num
                                prev_segment_idx = temp_prev_segment_idx
                                cur_segment_idx = temp_cur_segment_idx
                    else:
                        for s_idx in range(len(Node_seq)-1):
                            if ((Node_seq[s_idx] == temp_cur_segment_idx) and (Node_seq[s_idx + 1] == temp_prev_segment_idx)):
                                road_num = temp_road_num
                                prev_segment_idx = temp_prev_segment_idx
                                cur_segment_idx = temp_cur_segment_idx

        road = road_net.roads[road_num]

        first_done = True
        second_done = True

        if (node_idx == 1):
            if (road_num == cur_node.inter_tar[0]):
                second_prev_x = cur_node.inter_tar[1]
                second_prev_y = cur_node.inter_tar[2]
                second_cur_x = cur_node.inter_tar[3]
                second_cur_y = cur_node.inter_tar[4]
            elif (road_num == cur_node.inter_com[0]):
                second_prev_x = cur_node.inter_com[1]
                second_prev_y = cur_node.inter_com[2]
                second_cur_x = cur_node.inter_com[3]
                second_cur_y = cur_node.inter_com[4]
            else:
                second_done = False
            
            second_segment = road.path[cur_segment_idx]
            second_x_line = second_segment.centerline[0]
            second_y_line = second_segment.centerline[1]
            second_x = []
            second_y = []
            second_idx = 0

            if (second_done):
                for idx in range(len(second_x_line) - 1):
                    if ((second_x_line[idx] == second_prev_x) and (second_y_line[idx] == second_prev_y) and (second_x_line[idx+1] == second_cur_x) and (second_y_line[idx+1] == second_cur_y)):
                        second_idx = idx + 1
        
            if (prev_segment_idx < cur_segment_idx):
                for segment_idx in range(prev_segment_idx, cur_segment_idx):
                    segment = road.path[segment_idx]
                    line_x.append(segment.centerline[0])
                    line_y.append(segment.centerline[1])
                if (second_done):
                    for idx in range(0, second_idx):
                        second_x.append(second_x_line[idx])
                        second_y.append(second_y_line[idx])
                    line_x.append(second_x)
                    line_y.append(second_y)
            else:
                back_list = list(range(cur_segment_idx + 1, prev_segment_idx + 1))
                back_list.reverse()
                for segment_idx in back_list:
                    segment = road.path[segment_idx]
                    line_x.append(np.flip(segment.centerline[0]))
                    line_y.append(np.flip(segment.centerline[1]))
                if (second_done):
                    back_second = list(range(second_idx, len(second_x_line)))
                    back_second.reverse()
                    for idx in back_second:
                        second_x.append(second_x_line[idx])
                        second_y.append(second_y_line[idx])
                    line_x.append(second_x)
                    line_y.append(second_y)
            cur_line_x = []
            cur_line_y = []
            for i in range(2):
                cur_line_x.append(cur_node.x)
                cur_line_y.append(cur_node.y)
            line_x.append(cur_line_x)
            line_y.append(cur_line_y)
        elif (node_idx == len(path) - 1):
            if (road_num == prev_node.inter_tar[0]):
                first_prev_x = prev_node.inter_tar[1]
                first_prev_y = prev_node.inter_tar[2]
                first_cur_x = prev_node.inter_tar[3]
                first_cur_y = prev_node.inter_tar[4]
            elif (road_num == prev_node.inter_com[0]):
                first_prev_x = prev_node.inter_com[1]
                first_prev_y = prev_node.inter_com[2]
                first_cur_x = prev_node.inter_com[3]
                first_cur_y = prev_node.inter_com[4]
            else:
                first_done = False

            first_segment = road.path[prev_segment_idx]
            first_x_line = first_segment.centerline[0]
            first_y_line = first_segment.centerline[1]
            first_x = []
            first_y = []
            first_idx = 0

            if (first_done):
                for idx in range(len(first_x_line) - 1):
                    if ((first_x_line[idx] == first_prev_x) and (first_y_line[idx] == first_prev_y) and (first_x_line[idx+1] == first_cur_x) and (first_y_line[idx+1] == first_cur_y)):
                        first_idx = idx + 1

            if (prev_segment_idx < cur_segment_idx):
                if (first_done):
                    for idx in range(first_idx, len(first_x_line)):
                        first_x.append(first_x_line[idx])
                        first_y.append(first_y_line[idx])
                    line_x.append(first_x)
                    line_y.append(first_y)
                for segment_idx in range(prev_segment_idx + 1, cur_segment_idx + 1):
                    segment = road.path[segment_idx]
                    line_x.append(segment.centerline[0])
                    line_y.append(segment.centerline[1])
            else:
                if (first_done):
                    back_first = list(range(0, first_idx))
                    back_first.reverse()
                    for idx in back_first:
                        first_x.append(first_x_line[idx])
                        first_y.append(first_y_line[idx])
                    line_x.append(first_x)
                    line_y.append(first_y)
                back_list = list(range(cur_segment_idx, prev_segment_idx))
                back_list.reverse()
                for segment_idx in back_list:
                    segment = road.path[segment_idx]
                    line_x.append(np.flip(segment.centerline[0]))
                    line_y.append(np.flip(segment.centerline[1]))
        else:
            if (road_num == prev_node.inter_tar[0]):
                first_prev_x = prev_node.inter_tar[1]
                first_prev_y = prev_node.inter_tar[2]
                first_cur_x = prev_node.inter_tar[3]
                first_cur_y = prev_node.inter_tar[4]
            elif (road_num == prev_node.inter_com[0]):
                first_prev_x = prev_node.inter_com[1]
                first_prev_y = prev_node.inter_com[2]
                first_cur_x = prev_node.inter_com[3]
                first_cur_y = prev_node.inter_com[4]
            else:
                first_done = False

            if (road_num == cur_node.inter_tar[0]):
                second_prev_x = cur_node.inter_tar[1]
                second_prev_y = cur_node.inter_tar[2]
                second_cur_x = cur_node.inter_tar[3]
                second_cur_y = cur_node.inter_tar[4]
            elif (road_num == cur_node.inter_com[0]):
                second_prev_x = cur_node.inter_com[1]
                second_prev_y = cur_node.inter_com[2]
                second_cur_x = cur_node.inter_com[3]
                second_cur_y = cur_node.inter_com[4]
            else:
                second_done = False

            first_segment = road.path[prev_segment_idx]
            first_x_line = first_segment.centerline[0]
            first_y_line = first_segment.centerline[1]
            first_x = []
            first_y = []
            first_idx = 0

            if (first_done):
                for idx in range(len(first_x_line) - 1):
                    if ((first_x_line[idx] == first_prev_x) and (first_y_line[idx] == first_prev_y) and (first_x_line[idx+1] == first_cur_x) and (first_y_line[idx+1] == first_cur_y)):
                        first_idx = idx + 1
            
            second_segment = road.path[cur_segment_idx]
            second_x_line = second_segment.centerline[0]
            second_y_line = second_segment.centerline[1]
            second_x = []
            second_y = []
            second_idx = 0
            
            if (second_done):
                for idx in range(len(second_x_line) - 1):
                    if ((second_x_line[idx] == second_prev_x) and (second_y_line[idx] == second_prev_y) and (second_x_line[idx+1] == second_cur_x) and (second_y_line[idx+1] == second_cur_y)):
                        second_idx = idx + 1
        
            if (prev_segment_idx < cur_segment_idx):
                if (first_done):
                    for idx in range(first_idx, len(first_x_line)):
                        first_x.append(first_x_line[idx])
                        first_y.append(first_y_line[idx])
                    line_x.append(first_x)
                    line_y.append(first_y)
                for segment_idx in range(prev_segment_idx + 1, cur_segment_idx):
                    segment = road.path[segment_idx]
                    line_x.append(segment.centerline[0])
                    line_y.append(segment.centerline[1])
                if (second_done):
                    for idx in range(0, second_idx):
                        second_x.append(second_x_line[idx])
                        second_y.append(second_y_line[idx])
                    line_x.append(second_x)
                    line_y.append(second_y)
            else:
                if (first_done):
                    back_first = list(range(0, first_idx))
                    back_first.reverse()
                    for idx in back_first:
                        first_x.append(first_x_line[idx])
                        first_y.append(first_y_line[idx])
                    line_x.append(first_x)
                    line_y.append(first_y)
                back_list = list(range(cur_segment_idx + 1, prev_segment_idx))
                back_list.reverse()
                for segment_idx in back_list:
                    segment = road.path[segment_idx]
                    line_x.append(np.flip(segment.centerline[0]))
                    line_y.append(np.flip(segment.centerline[1]))
                back_second = list(range(second_idx, len(second_x_line)))
                back_second.reverse()
                if (second_done):
                    for idx in back_second:
                        second_x.append(second_x_line[idx])
                        second_y.append(second_y_line[idx])
                    line_x.append(second_x)
                    line_y.append(second_y)
            cur_line_x = []
            cur_line_y = []
            for i in range(2):
                cur_line_x.append(cur_node.x)
                cur_line_y.append(cur_node.y)
            line_x.append(cur_line_x)
            line_y.append(cur_line_y)

    return Merge_path_print([line_x, line_y])


def Merge_path(path_line):
    path_x = path_line[0]
    path_y = path_line[1]
    segment_pt = len(path_x)
    pt_x = []
    pt_y = []
    for path_idx in range(segment_pt-1):
        pt_x = pt_x + list(path_x[path_idx][:-1])
        pt_y = pt_y + list(path_y[path_idx][:-1])
    pt_x = pt_x + list(path_x[-1])
    pt_y = pt_y + list(path_y[-1])
    
    return np.array([pt_x,pt_y])


def Merge_path_print(path_line):
    path_x = path_line[0]
    path_y = path_line[1]
    segment_pt = len(path_x)
    pt_x = []
    pt_y = []
    for path_idx in range(segment_pt-1):
        if (segment_pt == 0):
            print("ZERO: ", path_idx)
        plt.plot(path_x[path_idx][0], path_y[path_idx][0], color = 'black', marker = 'o')
        plt.text(path_x[path_idx][0], path_y[path_idx][0], str(path_idx))
        if (path_idx%3 == 0):
            if (path_idx == 0):
                plt.plot(list(path_x[path_idx]), list(path_y[path_idx]), color = 'Black');
            else: 
                plt.plot(list(path_x[path_idx]), list(path_y[path_idx]), color = 'Red');
        elif (path_idx%3 == 1):
            plt.plot(list(path_x[path_idx]), list(path_y[path_idx]), color = 'Blue');
        else:
            plt.plot(list(path_x[path_idx]), list(path_y[path_idx]), color = 'Green');
        pt_x = pt_x + list(path_x[path_idx][:-1])
        pt_y = pt_y + list(path_y[path_idx][:-1])
    #plt.show()
    pt_x = pt_x + list(path_x[-1])
    pt_y = pt_y + list(path_y[-1])
    
    return np.array([pt_x,pt_y])
    
def GraphHwa(road):
    return

def Graph_Map(road_network, graph, path_line, curr_time):
    # Plot Graph
    for road_idx in range(len(road_network.roads)):
        road = road_network.roads[road_idx]
        right = road.getRightEdge()
        left = road.getLeftEdge()
        center = road.getCenterline()
        for segment in road.path:
            #plt.scatter(segment.backline[0][0], segment.backline[1][0], color = 'gray')
            #plt.scatter(segment.backline[0][1], segment.backline[1][1], color = 'gray')
            #plt.scatter(segment.backline[0][2], segment.backline[1][2], color = 'gray')
            plt.plot(segment.backline[0], segment.backline[1], color = 'black')

        if road_idx == 0:
            plt.plot(right[0], right[1], color='red')
            plt.plot(left[0], left[1], color='red')
            plt.plot(center[0], center[1], color='red')
        
        if road_idx == 1:
            plt.plot(right[0], right[1], color='blue')
            plt.plot(left[0], left[1], color='blue')
            plt.plot(center[0], center[1], color='blue')

        if road_idx == 2:
            plt.plot(right[0], right[1], color='green')
            plt.plot(left[0], left[1], color='green')
            plt.plot(center[0], center[1], color='green')

        if road_idx == 3:
            plt.plot(right[0], right[1], color='brown')
            plt.plot(left[0], left[1], color='brown')
            plt.plot(center[0], center[1], color='brown')

        if road_idx == 4:
            plt.plot(right[0], right[1], color='yellow')
            plt.plot(left[0], left[1], color='yellow')
            plt.plot(center[0], center[1], color='yellow')

    # Plot Node
    for node_idx in range(graph.num_nodes):
        node = graph.node_list[node_idx]
        plt.plot(node.x, node.y, color = 'black', marker = 'o')
        if (node.is_end == True):
            plt.text(node.x, node.y, 'End Node ' + str(node_idx))
        else:
            plt.text(node.x, node.y, 'Node ' + str(node_idx))

    # Plot path
    path_x = path_line[0]
    path_y = path_line[1]
    plt.plot(path_x, path_y, color = 'black', linewidth = 2)

    time_name = f'{curr_time}_map'
    filename = f'AD_Simul\\Map_data\\{time_name}.jpg'
    plt.savefig(filename)
    plt.clf()
    #plt.show()

            