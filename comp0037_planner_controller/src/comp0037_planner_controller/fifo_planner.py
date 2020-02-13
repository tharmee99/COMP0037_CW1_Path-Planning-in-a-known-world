# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
import glob

location_to_results=glob.glob('../*/src/*/comp0037_planner_controller/scripts/performance_metrics.txt')[0]


# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.

class FIFOPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.fifoQueue = deque()
        with open(location_to_results, "w+") as f:
            f.write('Breadth First Search Algorithm: \n')

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.fifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.fifoQueue

    # Return the length of the queue
    def getQueueLength(self):
        return len(self.fifoQueue)

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.fifoQueue.popleft()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
