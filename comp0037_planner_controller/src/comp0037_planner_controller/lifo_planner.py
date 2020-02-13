# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
import glob

location_to_results=glob.glob('../*/src/*/comp0037_planner_controller/scripts/performance_metrics.txt')[0]

class LIFOPlanner(CellBasedForwardSearch):

    # This implements a simple LIFO (last in first out or depth first) search algorithm
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.lifoQueue = list()
        with open(location_to_results, "a") as f:
            f.write('Depth First Search Algorithm: \n')

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.lifoQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.lifoQueue

    # Return the length of the queue
    def getQueueLength(self):
        return len(self.lifoQueue)

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.lifoQueue.pop()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass
