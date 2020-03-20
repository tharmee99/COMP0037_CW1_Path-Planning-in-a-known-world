# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue

from math import sqrt

# Implementation of the Dynamic Planner Algorithm which the Dijkstra and A* Algorithm inherit

class DynamicPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.plannerQueue = PriorityQueue()
    
    def calc_heuristics(self, cell):
        raise NotImplementedError()

    # Insert the cell into the priority queue ordered by the cost to go summe with the heuristic  
    def pushCellOntoQueue(self, cell):
        if cell != self.start:
            cell.pathCost = cell.parent.pathCost + self.computeLStageAdditiveCost(cell.parent, cell)
        
        priorityValue = cell.pathCost + self.calc_heuristics(cell)

        self.plannerQueue.put((priorityValue, cell))

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.plannerQueue.empty()

    # Return the length of the queue
    def getQueueLength(self):
        return self.plannerQueue.qsize()

    # Pop the first element of the queue (smallest priority value)
    def popCellFromQueue(self):
        cell = self.plannerQueue.get()[1]
        return cell

    # Resolves revisiting a cell
    def resolveDuplicate(self, cell, parentCell):
        newPathCost = parentCell.pathCost + self.computeLStageAdditiveCost(parentCell, cell)

        if(newPathCost < cell.pathCost):
            cell.parent = parentCell
            cell.pathCost = newPathCost
            priorityValue = newPathCost + self.calc_heuristics(cell)
            self.reorderPriorityQueue()
            # self.plannerQueue.put((priorityValue, cell))

    def reorderPriorityQueue(self):
        newQueue = PriorityQueue()

        while self.priorityQueue.empty() is False:
            tuple = self.priorityQueue.get()
            newQueue.put(tuple)
             
        self.priorityQueue = newQueue
