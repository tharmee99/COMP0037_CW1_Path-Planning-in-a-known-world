# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue

from math import sqrt

class DjikstraPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.dijkstraQueue = PriorityQueue()
        with open("performance_metrics.txt", "a") as f:
            f.write('Dijkstra\'s Algorithm: \n')

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        pass
        # dist = self.compute_euclidean_distance(cell, self.goal)
        # self.dijkstraQueue.put((dist, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.dijkstraQueue.empty()

    # Return the length of the queue
    def getQueueLength(self):
        return len(self.dijkstraQueue)

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.dijkstraQueue.get()[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        
        
