# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue

from math import sqrt
import glob

location_to_results=glob.glob('../*/src/*/comp0037_planner_controller/scripts/performance_metrics.txt')[0]

class GreedyPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.greedyQueue = PriorityQueue()
        with open(location_to_results, "w+") as f:
            f.write('Greedy Queue Algorithm: \n')

    def compute_euclidean_distance(self, cell1, cell2):
        del_x = cell2.coords[0] - cell1.coords[0]
        del_y = cell2.coords[1] - cell1.coords[1]
        distance = sqrt(pow(del_x,2)+pow(del_y,2))
        return distance

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        dist = self.compute_euclidean_distance(cell, self.goal)
        self.greedyQueue.put((dist, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.greedyQueue.empty()

    # Return the length of the queue
    def getQueueLength(self):
        return self.greedyQueue.qsize()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.greedyQueue.get()[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
