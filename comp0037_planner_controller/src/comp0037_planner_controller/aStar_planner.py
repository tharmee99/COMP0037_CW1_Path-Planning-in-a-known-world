# -*- coding: utf-8 -*-

from dynamic_planner import DynamicPlanner
from Queue import PriorityQueue

from math import sqrt
import random

heurstic_list = ["constant", "euclidean", "octile", "manhattan"]

class AStarPlanner(DynamicPlanner):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid, heuristic):
        DynamicPlanner.__init__(self, title, occupancyGrid)

        # random.seed(42)
        # self.nonZeroConstant = random.randrange(parentCell.pathCost + 1)

        self.nonZeroConstant = 1

        if(heuristic.lower() not in heurstic_list):
            heuristic = "0"
        
        if(heuristic.lower() == heurstic_list[0]):
            self.plannerName = 'A* Algorithm (Heuristic: {} = {})'.format(heuristic.capitalize(), self.nonZeroConstant)
        else:
            self.plannerName = 'A* Algorithm (Heuristic: {})'.format(heuristic.capitalize())

        self.heuristics = heuristic.lower()

        

    def calc_heuristics(self, cell):
        if(self.heuristics.lower() == heurstic_list[0]):
            return self.nonZeroConstant

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
