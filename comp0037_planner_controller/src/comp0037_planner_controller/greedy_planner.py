# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from queue import PriorityQueue

from math import sqrt

# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.

class FIFOPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.greedyQueue = PriorityQueue()

    def compute_euclidean_distance(cell1, cell2):
        del_x = cell2.coords[0] - cell1.coords[0]
        del_y = cell2.coords[1] - cell1.coords[1]
        distance = sqrt(pow(del_x,2)+pow(del_y,2))
        
        return distance

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        dist = compute_euclidean_distance(cell, self.goal)
        self.greedyQueue.append((dist, cell))
        # self.greedyQueue.sort(reverse=True)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.greedyQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.greedyQueue.get()[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
