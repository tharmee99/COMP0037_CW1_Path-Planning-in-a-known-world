# -*- coding: utf-8 -*-

from dynamic_planner import DynamicPlanner
from Queue import PriorityQueue

from math import sqrt

class DijkstraPlanner(DynamicPlanner):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        DynamicPlanner.__init__(self, title, occupancyGrid)
        self.plannerName = "Dijkstra's"

    def calc_heuristics(self, cell, parentCell):
        return 0
