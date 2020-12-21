import numpy as np
import sys
import random
import math
import copy
import csv

from Road import *
from graph import *
from Crossover import *
import socket
from Rotation import *

from datetime import datetime

def flags_parser(argv):
    flag_list = ["-p", "-f", "-k", "-e", "-cr", "-x", "-y", "-w", "-n"]
    ret = []
    p_list = []
    f_list = []
    k_list = []
    e_list = []
    cr_list = []
    x_list = []
    y_list = []
    w_list = []
    n_list = []

    idx_end = len(argv)
    idx_p = 0
    idx_f = 0
    idx_k = 0
    idx_e = 0
    idx_cr = 0
    idx_x = 0
    idx_y = 0
    idx_w = 0
    idx_n = 0
    temp = ""

    if "-p" in argv:
        idx_p = argv.index("-p")
        idx = idx_p + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                p_list.append(temp)
                idx = idx + 1
    if "-f" in argv:
        idx_f = argv.index("-f")
        idx = idx_f + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                f_list.append(temp)
                idx = idx + 1
    if "-k" in argv:
        idx_k = argv.index("-k")
        idx = idx_k + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                k_list.append(temp)
                idx = idx + 1
    if "-e" in argv:
        idx_e = argv.index("-e")
        idx = idx_e + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                e_list.append(temp)
                idx = idx + 1
    if "-cr" in argv:
        idx_cr = argv.index("-cr")
        idx = idx_cr + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                cr_list.append(temp)
                idx = idx + 1
    if "-x" in argv:
        idx_x = argv.index("-x")
        idx = idx_x + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                x_list.append(temp)
                idx = idx + 1
    if "-y" in argv:
        idx_y = argv.index("-y")
        idx = idx_y + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                y_list.append(temp)
                idx = idx + 1    
    if "-w" in argv:
        idx_w = argv.index("-w")
        idx = idx_w + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                w_list.append(temp)
                idx = idx + 1
    if "-n" in argv:
        idx_n = argv.index("-n")
        idx = idx_n + 1
        while (idx != idx_end):
            temp = argv[idx]
            if (temp in flag_list):
                break
            else:
                n_list.append(temp)
                idx = idx + 1

    ret.append(p_list)
    ret.append(f_list)
    ret.append(k_list)
    ret.append(e_list)
    ret.append(cr_list)
    ret.append(x_list)
    ret.append(y_list)
    ret.append(w_list)
    ret.append(n_list)

    return ret

class RN:
    __slots__ = ['roadnetwork', 'fitness','histogram','name']

    def __init__(self, roadnetwork, fitness, histogram):
        self.roadnetwork = roadnetwork
        self.fitness = fitness
        self.histogram = histogram
        self.name = "temp"

    def set_fitness(self, fitness):
        self.fitness = fitness

    def get_fitness(self):
        return self.fitness

    def set_name(self, name):
        self.name = name
    
    def get_name(self):
        return self.name

class GA:
    __slots__ = ['pop_size', 'gen_num', 'sel_size', 'elitism', 'cr', 'map_x', 'map_y', 'map_w', 'path_num', 'population', 'similarity_threshold'] ############## Jaemin Edited ##########
    
    def __init__(self):
        # parameters
        self.pop_size = 25 # p, population size
        self.gen_num = 40 # f, generation number
        self.sel_size = 5 # k, selection size
        self.elitism = 0.1 # e, elitism
        self.cr = 'join' # cr, crossover method
        self.map_x = 200 # x, map x size
        self.map_y = 200 # y, map y size
        self.map_w = 4 # w, map w size
        self.path_num = 3 # n, path geneartion number
        self.population = []
        self.similarity_threshold = 0.05
        
    def args_setting(self, args):
        if args[0]:
            self.pop_size = int(args[0][0])
        if args[1]:
            self.gen_num = int(args[1][0])
        if args[2]:
            self.sel_size = int(args[2][0])
        if args[3]:
            self.elitism = float(args[3][0])
        if args[4]:
            self.cr = str(args[4][0])
        if args[5]:
            self.map_x = int(args[5][0])
        if args[6]:
            self.map_y = int(args[6][0])
        if args[7]:
            self.map_w = int(args[7][0])
        if args[8]:
            self.path_num = int(args[8][0])

    def random_solution(self):
        print('sol')
        random_network = CreateRandomRoadNetwork(self.map_x, self.map_y, self.map_w)
        rand_RN = RN(random_network, -1, make_histogram(random_network))
        self.fitness(rand_RN)
        while (rand_RN.fitness == -1):
            random_network = CreateRandomRoadNetwork(self.map_x, self.map_y, self.map_w)
            rand_RN.roadnetwork = random_network
            self.fitness(rand_RN)
        return rand_RN
   
    def selection(self, pop, k):
        tournament_pool = random.sample(pop, min(len(pop), k))
        max_val = 0
        max_idx = []
        temp = 0
        for i in range(min(len(pop),k)):
            temp = tournament_pool[i].get_fitness()
            if (temp > max_val):
                max_val = temp
                max_idx = [i]
            elif (temp == max_val):
                max_idx.append(i)
        choice = random.choice(max_idx)
        return tournament_pool[choice]

    def fitness(self, solution):
        print('fit')
        rn = solution.roadnetwork
        rn_graph = GraphHwa_full(rn)
        rn_path = Make_path(rn, rn_graph, self.path_num)
        path_line = Route_path(rn, rn_graph, rn_path)
        
        #print(f'path line: \n{path_line.shape}')
        #print(f'path line: \n{path_line[0]}')

        curr_time = str(datetime.now()).replace(' ', '_').replace(':', '_').split('.')[0]
        time_name = f'{curr_time}_map'
        filename = f'AD_Simul\\CSV_data\\{time_name}.csv'
        Graph_Map(rn, rn_graph, path_line, curr_time)
        saveMatlabNetwork(rn, path_line, path_line, 6, filename)
        
        print('send')

        sock.send(time_name.encode())
        data = sock.recv(1024)
        fitness = float(data.decode())
        # fitness = 0
        

        print('fitness:', fitness)

        fitness = min(fitness, 3)

        print('bounded fitness:', fitness)

        solution.set_name(filename)
        solution.set_fitness(fitness)
        return fitness

    def save_generation(self, generation):
        f = open(f'AD_Simul\\Generation_CSV\\{generation}th_generation.csv','w', encoding = 'utf-8', newline='')
        wr = csv.writer(f)
        for rn in self.population:
            wr.writerow([rn.get_name(), rn.get_fitness()])
        f.close()

    def iter_generation(self):
        print("Iterate Generation")
        
        best = (0, [])
        conv = 0
        genline = []
        timeline = []
        new_list = []
        e_num = int(self.pop_size * self.elitism)

        crossover_operator = ['join', 'merge']
        imp_crossover_operator = ['imp_join', 'imp_merge']

        for gen in range(self.gen_num):
            offspring = []

            new_list.clear()
            new_list = sorted(self.population, key=lambda x: x.fitness, reverse=True)[:e_num]
   
            while len(offspring) < self.pop_size:
                parent1 = self.selection(self.population, self.sel_size)
                parent2 = self.selection(self.population, self.sel_size)

                child1_rn = crossover_method([parent1.roadnetwork, parent2.roadnetwork], random.choice(imp_crossover_operator))

                if (child1_rn[0] == True):
                    child1_rn = crossover_method([child1_rn[1]], 'mutation')
                    if (child1_rn[0]):
                        hist_tmp = make_histogram(child1_rn[1])
                        child1 = RN(child1_rn[1], -1, hist_tmp)
                        if (len(offspring) > 0):
                            similarity_test = True
                            for road_net in offspring:
                                difference = math.sqrt(np.sum(np.square(np.subtract(hist_tmp,road_net.histogram))))
                                if (difference < self.similarity_threshold):
                                    similarity_test = False
                                    break
                            if similarity_test:
                                print("Add population")
                                self.fitness(child1)
                                if (child1.fitness != -1):
                                    offspring.append(child1)
                        else:
                            self.fitness(child1)
                            if (child1.fitness != -1):
                                offspring.append(child1)
            
            while len(new_list) < self.pop_size:
                survival = self.selection(offspring, min(len(offspring), self.sel_size))
                offspring.remove(survival)
                new_list.append(survival)

            temp_list = sorted(new_list, key=lambda x: x.fitness, reverse=True)
            self.population = temp_list[:]
            self.save_generation(gen+1)

            if self.population[0].get_fitness() > best[0]:
                best = [self.population[0].get_fitness(), self.population[0]]
                genline.append(gen)
                timeline.append([gen, best])

            # print(최고)
            #print("Best in generation:", gen, best[0], best[1].fitness)
        #print(best[1].fitnesss)

def ga_initializer():
    temp_argv = sys.argv
    argv = temp_argv[1:]
    args=flags_parser(argv)
    ga = GA()
    ga.args_setting(args)

    # Initial Population
    print("Initialize population")

    ga.population.append(ga.random_solution())
    while len(ga.population) < ga.pop_size :
        print(len(ga.population), ga.pop_size)
        rand_RN = ga.random_solution()
        similarity_test = True
        for road_net in ga.population:
            difference = math.sqrt(np.sum(np.square(np.subtract(rand_RN.histogram,road_net.histogram))))
            if (difference < ga.similarity_threshold):
                similarity_test = False
                break
        if similarity_test:
            print("Add population")
            ga.population.append(rand_RN)
    f = open(f'AD_Simul\\Generation_CSV\\0th_generation.csv','w', encoding = 'utf-8', newline='')
    wr = csv.writer(f)
    for rn in ga.population:
        wr.writerow([rn.get_name(), rn.get_fitness()])
    f.close()

    return ga
    
def make_histogram(road_network):
    width = road_network.roads[0].path[0].width
    bin_0 = [-9.5*width, -8.5*width, -8*width, -7.5*width, -6.5*width, 6.5*width, 7.5*width, 8*width, 8.5*width, 9.5*width]
    radius= []
    length = []
    for i in road_network.roads:
        for j in i.path:
            if j.isarc:
                if j.angle < 0:
                    radius.append(-j.radius*2)
                    length.append(-(math.pi * j.radius * j.angle) /180)
                else:
                    radius.append(j.radius*2)
                    length.append((math.pi * j.radius * j.angle) /180)
            else:
                radius.append(0)
                length.append((j.length)/4)
    #Improvement#
    tmp_num = len(radius)
    bin_1 = [-19*width, -16*width, -10*width, -2.5*width, 0, 2.5*width, 10*width, 16*width, 19*width]
    radius2= []
    length2 = []
    for i in range(1,tmp_num):
        length2.append((length[i-1]+length[i])/2)
        radius2.append(radius[i]-radius[i-1])
    hist, bin_0 = np.histogram(radius, bins=bin_0, weights=length)
    hist2, bin_1 = np.histogram(radius2, bins=bin_1, weights=length2)
    hist = hist / np.sum(hist)
    hist2 = hist2 / np.sum(hist2)
    hist_final = np.concatenate((hist,hist2))

    return hist_final

if __name__ == '__main__':
    HOST = '127.0.0.1'
    PORT = 10004 
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print('init start')
    ga = ga_initializer()
    print('init finish')
    ga.iter_generation()
    finish = 'q'
    sock.send(finish.encode())
    sock.close()
