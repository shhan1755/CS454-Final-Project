from Road import *
import numpy as np
import csv


def rotationRoad(startline: np.ndarray, roadlines: list):
    target_vector = [1, 0]
    startline_vector = [startline[0, 1] - startline[0, 0], startline[1, 1] - startline[1, 0]]
    startline_vector = startline_vector / np.linalg.norm(startline_vector)

    ''' Get theta value of rotation '''
    # direction = 1 if (startline_vector[0] >= 0 and startline_vector[1] < 0) or (startline_vector[0] < 0 and startline_vector[1] >= 0) else -1
    theta = np.arccos(np.clip(np.dot(target_vector, startline_vector), -1.0, 1.0))
    # print(f'[theta] {theta}, {np.pi/2}')
    if theta <= np.pi/2:
        theta = - theta

    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))

    translation = [startline[0, 0], startline[1, 0]]
    result = []
    for line in roadlines:
        line_x = line[0, :] - translation[0]
        line_y = line[1, :] - translation[1]
        newline = np.array((line_x, line_y))
        result.append(np.matmul(R, newline))
    # firstline = np.array(tuple(map(lambda x: [x[0, 0], x[1, 0]], result))).T
    # firstline_vector = [firstline[0, 0] - firstline[0, 2], firstline[1, 0] - firstline[1, 2]]
    # print(firstline_vector)
    # print(firstline_vector[0]*target_vector[0] + firstline_vector[1]*target_vector[1])
    return result


def saveMatlabNetwork(road_network: RoadNetwork, startline: np.ndarray, waypoints: np.ndarray = None, time_step = 1, path: str = 'output.csv'):
    if waypoints is None:
        waypoints = road_network.roads[0].getCenterline()

    ''' Rotate all road according to startline '''
    rotated = [] # [[road1's left edge, road1's centerline, road1's right edge], [road2's left edge, road2's centerline, road2's right edge], [road3's left edge, road3's centerline, road3's right edge]]
    for road in road_network.roads:
        lines = [road.getLeftEdge(), road.getCenterline(), road.getRightEdge()]
        rotated.append(rotationRoad(startline, lines))

    rotated_way = rotationRoad(startline, [waypoints])[0]
    # plt.plot(rotated_way[0], rotated_way[1], color='red', linewidth=4)
    way_npoints = rotated_way.shape[1]
    new_way = np.zeros((2, way_npoints//time_step), dtype=np.float16)
    print(len(rotated))
    print(len(rotated[0]))
    new_rotated = []
    for road in rotated:
        road_npoint = road[0].shape[1]
        new_rotated.append([])
        for line in road:
            new_rotated[-1].append(np.zeros((2, road_npoint//time_step + 1), dtype=np.float16))
            for i in range(road_npoint//time_step):
                new_rotated[-1][-1][0, i] = line[0, time_step*i]
                new_rotated[-1][-1][1, i] = line[1, time_step*i]
            new_rotated[-1][-1][0, road_npoint//time_step] = line[0, road_npoint-1]
            new_rotated[-1][-1][1, road_npoint//time_step] = line[1, road_npoint-1]

    new_way = rotated_way


    ''' Save road network to csv file (MATLAB format) '''
    with open(path, 'w', newline = '') as output:
        writer = csv.writer(output, delimiter=',')
        writer.writerow([len(rotated)])
        for road in new_rotated:
            writer.writerow([road[1].shape[1]])
            writer.writerow(list(road[1][0]))
            writer.writerow(list(road[1][1]))
        writer.writerow([new_way.shape[1]])
        writer.writerow(list(new_way[0]))
        writer.writerow(list(new_way[1]))
        output.close()

    return True, rotated
