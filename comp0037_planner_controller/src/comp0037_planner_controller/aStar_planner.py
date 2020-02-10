# -*- coding: utf-8 -*-

from dynamic_planner import DynamicPlanner
from Queue import PriorityQueue

from math import sqrt

class AStarPlanner(DynamicPlanner):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid, heuristic):
        DynamicPlanner.__init__(self, title, occupancyGrid)
        with open("performance_metrics.txt", "a") as f:
            f.write('A* Algorithm (Heuristic: {}): \n'.format(heuristic))
            self.heuristics = heuristic

    def calc_heuristics(self, cell, parentCell):
        if(self.heuristics =='constant'):
            return 5

        elif(self.heuristics =='euclidean'):
            del_x = cell.coords[0] - self.goal.coords[0]
            del_y = cell.coords[1] - self.goal.coords[1]
            return sqrt(pow(del_x,2)+pow(del_y,2))
            
        elif(self.heuristics =='octile'):
            x_diff = abs(cell.coords[0]-self.goal.coords[0])
            y_diff = abs(cell.coords[1]-self.goal.coords[1])
            return max(x_diff,y_diff) + (sqrt(2)-1)*min(x_diff,y_diff)

        elif(self.heuristics =='manhattan'):
            return abs(cell.coords[0]-self.goal.coords[0]) + abs(cell.coords[1]-self.goal.coords[1])

        else:
            return 0
