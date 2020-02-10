# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue

from math import sqrt

class DijkstraPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.djikstraQueue = PriorityQueue()
        with open("performance_metrics.txt", "a") as f:
            f.write('Dijkstra\'s Algorithm: \n')

    # Simply put on the end of the queue    
    def pushCellOntoQueue(self, cell):

        if cell != self.start:
            cell.pathCost = cell.parent.pathCost + self.computeLStageAdditiveCost(cell.parent, cell)

        self.djikstraQueue.put((cell.pathCost, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.djikstraQueue.empty()

    # Return the length of the queue
    def getQueueLength(self):
        return self.djikstraQueue.qsize()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.djikstraQueue.get()[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        newPathCost = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell, cell)

        if(newPathCost < cell.pathCost):
            cell.parent = parentCell
            cell.pathCost = newPathCost
            self.djikstraQueue.put((newPathCost, cell))