# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue

from math import sqrt

class DynamicPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.plannerQueue = PriorityQueue()
    
    def calc_heuristics(self, cell, parentCell):
        raise NotImplementedError()

    # Simply put on the end of the queue    
    def pushCellOntoQueue(self, cell):
        if cell != self.start:
            cell.pathCost = cell.parent.pathCost + self.computeLStageAdditiveCost(cell.parent, cell) + self.calc_heuristics(cell, cell.parent)
        self.plannerQueue.put((cell.pathCost, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.plannerQueue.empty()

    # Return the length of the queue
    def getQueueLength(self):
        return self.plannerQueue.qsize()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.plannerQueue.get()[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        newPathCost = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell, cell) + self.calc_heuristics(cell, parentCell)

        if(newPathCost < cell.pathCost):
            cell.parent = parentCell
            cell.pathCost = newPathCost
            self.plannerQueue.put((newPathCost, cell))