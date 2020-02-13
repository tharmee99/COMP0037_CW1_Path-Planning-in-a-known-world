# -*- coding: utf-8 -*-

from dynamic_planner import DynamicPlanner
from Queue import PriorityQueue

from math import sqrt
import glob

location_to_results=glob.glob('../*/src/*/comp0037_planner_controller/scripts/performance_metrics.txt')[0]

class DijkstraPlanner(DynamicPlanner):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        DynamicPlanner.__init__(self, title, occupancyGrid)
        with open(location_to_results, "w+") as f:
            f.write('Dijkstra\'s Algorithm: \n')

    def calc_heuristics(self, cell, parentCell):
        return 0
