# -*- coding: utf-8 -*-

from dynamic_planner import DynamicPlanner
from Queue import PriorityQueue

from math import sqrt
import glob

location_to_results = glob.glob('../*/src/*/comp0037_planner_controller/scripts/performance_metrics.txt')[0]

heurstic_list = ["constant", "euclidean", "octile", "manhattan"]

class AStarPlanner(DynamicPlanner):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid, heuristic):
        DynamicPlanner.__init__(self, title, occupancyGrid)

        if(heuristic.lower() not in heurstic_list):
            heuristic = "Dijkstra's (0 Heuristic)"
        
        with open(location_to_results, "w+") as f:
            f.write('A* Algorithm (Heuristic: {}): \n'.format(heuristic))
            self.heuristics = heuristic

    def calc_heuristics(self, cell, parentCell):
        if(self.heuristics.lower() == heurstic_list[0]):
            return 5

        elif(self.heuristics.lower() == heurstic_list[1]):
            del_x = cell.coords[0] - self.goal.coords[0]
            del_y = cell.coords[1] - self.goal.coords[1]
            return sqrt(pow(del_x,2)+pow(del_y,2))
            
        elif(self.heuristics.lower() == heurstic_list[2]):
            x_diff = abs(cell.coords[0]-self.goal.coords[0])
            y_diff = abs(cell.coords[1]-self.goal.coords[1])
            return max(x_diff,y_diff) + (sqrt(2)-1)*min(x_diff,y_diff)

        elif(self.heuristics.lower() == heurstic_list[3]):
            return abs(cell.coords[0]-self.goal.coords[0]) + abs(cell.coords[1]-self.goal.coords[1])

        else:
            return 0
