# -*- coding: utf-8 -*-

from dynamic_planner import DynamicPlanner
from Queue import PriorityQueue

from math import sqrt

class DijkstraPlanner(DynamicPlanner):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        DynamicPlanner.__init__(self, title, occupancyGrid)
        with open("performance_metrics.txt", "a") as f:
            f.write('Dijkstra\'s Algorithm: \n')

    def calc_heuristics(self, cell, parentCell):
        print("asdadad")
        return 0