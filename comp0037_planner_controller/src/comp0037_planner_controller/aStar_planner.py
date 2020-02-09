# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue

from math import sqrt

class AStarPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid, heuristic):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.aStarQueue = PriorityQueue()
        with open("performance_metrics.txt", "a") as f:
            f.write('A* Algorithm (Heuristic: {}): \n'.format(heuristic))

    

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        cost = cell.path_cost + self.computeLStageAdditiveCost(parentCell, cell)
        self.aStarQueue.put((dist, cell.parent , cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.aStarQueue.empty()

    # Return the length of the queue
    def getQueueLength(self):
        return len(self.aStarQueue)

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.aStarQueue.get()[2]
        return cell

    # Resolve duplicate
    def resolveDuplicate(self, cell, parentCell):
        predicted_path_cost = cell.path_cost + self.computeLStageAdditiveCost(parentCell, cell)
        if(predicted_path_cost < cell.path_cost):
            cell.parent=parentCell
            cell.path_cost=predicted_path_cost
            self.aStarQueue.put((predicted_path_cost, cell.parent, cell))
            