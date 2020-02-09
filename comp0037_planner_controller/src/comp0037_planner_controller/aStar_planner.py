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

    def calc_heuristics(self, cell, parentCell,heuristics):
        if(heuristics='constant'):
            return 5

        elif(heuristics='euclidean'):
            del_x = cell.coords[0] - self.goal.coords[0]
            del_y = cell.coords[1] - self.goal.coords[1]
            return sqrt(pow(del_x,2)+pow(del_y,2))
            
        elif(heuristics='octile'):
            x_diff = abs(cell.coords[0]-self.goal.coords[0])
            y_diff = abs(cell.coords[1]-self.goal.coords[1])
            return max(x_diff,y_diff) + (sqrt(2)-1)*min(x_diff,y_diff)

        elif(heuristics='manhattan'):
            return abs(cell.coords[0]-self.goal.coords[0]) + abs(cell.coords[1]-self.goal.coords[1])

        else:
            return 0

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        cost = cell.path_cost + self.computeLStageAdditiveCost(parentCell, cell) + calc_heuristics()
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
            