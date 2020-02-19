# -*- coding: utf-8 -*-

from occupancy_grid import OccupancyGrid
from search_grid import SearchGrid
from planner_base import PlannerBase
from collections import deque
from cell import *
from planned_path import PlannedPath
from math import *
import json
import rospy
import os.path
import csv

class GeneralForwardSearchAlgorithm(PlannerBase):

    # This class implements the basic framework for LaValle's general
    # template for forward search. It includes a lot of methods for
    # managing the graphical output as well.
    
    def __init__(self, title, occupancyGrid):
        PlannerBase.__init__(self, title, occupancyGrid)
        self.exportDirectory = ""
        self.mapName = ""
        self.plannerName = ""
        self.performanceMetrics = {
            "numberOfCellsVisited" : None,
            "maximumLengthOfQueue" : None,
            "totalAngleTurned" : None,
            "pathTravelCost" : None,
            "pathCardinality" : None
        }

        # Flag to store if the last plan was successful
        self.goalReached = None

    # These methods manage the queue of cells to be visited.
    def pushCellOntoQueue(self, cell):
        raise NotImplementedError()

    # This method returns a boolean - true if the queue is empty,
    # false if it still has some cells on it.
    def isQueueEmpty(self):
        raise NotImplementedError()

    def getQueueLength(self):
        raise NotImplementedError()

    # This method finds the first cell (at the head of the queue),
    # removes it from the queue, and returns it.
    def popCellFromQueue(self):
        raise NotImplementedError()

    # This method determines if the goal has been reached.
    def hasGoalBeenReached(self, cell):
        raise NotImplementedError()

    # This method gets the list of cells which could be visited next.
    def getNextSetOfCellsToBeVisited(self, cell):
        raise NotImplementedError()

    # This method determines whether a cell has been visited already.
    def hasCellBeenVisitedAlready(self, cell):
        raise NotImplementedError()

    def markCellAsVisitedAndRecordParent(self, cell, parentCell):
        cell.label = CellLabel.ALIVE
        cell.parent = parentCell

    # Mark that a cell is dead. A dead cell is one in which all of its
    # immediate neighbours have been visited.
    def markCellAsDead(self, cell):
        cell.label = CellLabel.DEAD
    
    # Handle the case that a cell has been visited already.
    def resolveDuplicate(self, cell):
        raise NotImplementedError()
    
    # Compute the additive cost of performing a step from the parent to the
    # current cell. This calculation is carried out the same way no matter
    # what heuristics, etc. are used. The cost computed here takes account
    # of the terrain traversability cost using an equation a bit like that
    # presented in the lectures.
    def computeLStageAdditiveCost(self, parentCell, cell):       
        # If the parent is empty, this is the start of the path and the
        # cost is 0.
        if (parentCell is None):
            return 0
        
        # Travel cost is Cartesian distance
        dX = cell.coords[0] - parentCell.coords[0]
        dY = cell.coords[1] - parentCell.coords[1]
        # Terrain cost 
        # Run this in matlab to visualise ro check the image
        # However, basically it builds up extremely quickly
        # x=[1:0.01:2];
        # c=min(1+(.2./((1.7-x).^2)).^2,1000);       
        cost=min(1+(0.2/((1.75-cell.terrainCost)**2))**2, 1000)
        L = sqrt(dX * dX + dY * dY)*cost# Multiplied by the terrain cost of the cell
        
        return L
        
    # The main search routine. The routine searches for a path between a given
    # set of coordinates. These are then converted into start and destination
    # cells in the search grid and the search algorithm is then run.
    def search(self, startCoords, goalCoords):

        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()

        # Create or update the search grid from the occupancy grid and seed
        # unvisited and occupied cells.
        if (self.searchGrid is None):
            self.searchGrid = SearchGrid.fromOccupancyGrid(self.occupancyGrid)
        else:
            self.searchGrid.updateFromOccupancyGrid()

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)
        self.start.label = CellLabel.START
        self.start.pathCost = 0

        # Get the goal cell object and label it.
        self.goal = self.searchGrid.getCellFromCoords(goalCoords)
        self.goal.label = CellLabel.GOAL

        # If the node is being shut down, bail out here.
        if rospy.is_shutdown():
            return False

        # Draw the initial state
        self.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0
        self.maxQueueLength = 0

        # Indicates if we reached the goal or not
        self.goalReached = False

        # Iterate until we have run out of live cells to try or we reached the goal.
        # This is the main computational loop and is the implementation of
        # LaValle's pseudocode
        while (self.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging.
            if rospy.is_shutdown():
                return False
            
            cell = self.popCellFromQueue()
            if (self.hasGoalBeenReached(cell) == True):
                self.goalReached = True
                break
            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False):
                    self.markCellAsVisitedAndRecordParent(nextCell, cell)
                    self.pushCellOntoQueue(nextCell)
                    self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                    self.maxQueueLength = self.getQueueLength() if (self.maxQueueLength < self.getQueueLength()) else self.maxQueueLength
                else:
                    self.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()
        
        print("Number of cells visited = " + str(self.numberOfCellsVisited))
        print("Maximum Queue length = " + str (self.maxQueueLength))

        self.performanceMetrics["numberOfCellsVisited"] = self.numberOfCellsVisited
        self.performanceMetrics["maximumLengthOfQueue"] = self.maxQueueLength

        # with open(self.exportDirectory, "a") as f:
        #     f.write("Number of cells visited = {} \n".format(self.numberOfCellsVisited))
        #     f.write("Maximum queue length = {} \n".format(self.maxQueueLength))
        
        if self.goalReached:
            print("Goal reached")
        else:
            print("Goal not reached")

        return self.goalReached

    # This method extracts a path from the pathEndCell to the start
    # cell. The path is a list actually sorted in the order:
    # cell(x_1), cell(x_2), ... , cell(x_K), cell(x_G). You can use
    # this method to try to find the path from any end cell. However,
    # depending upon the planner used, the results might not be
    # valid. In this case, the path will probably not terminate at the
    # start cell.

    def getAngle(self, parentCell, cell):
        
        del_y = cell.coords[1]-parentCell.coords[1]
        del_x = cell.coords[1]-parentCell.coords[0]

        angle = atan2(del_y,del_x) * (180/pi)

        return angle

    def extractPathEndingAtCell(self, pathEndCell, colour):

        # Construct the path object and mark if the goal was reached
        path = PlannedPath()
        
        path.goalReached = self.goalReached
        
        # Initial condition - the goal cell
        path.waypoints.append(pathEndCell)
               
        # Start at the goal and find the parent. Find the cost associated with the parent
        cell = pathEndCell.parent
        path.travelCost = self.computeLStageAdditiveCost(pathEndCell.parent, pathEndCell)

        # Initialise total angle turned
        self.totalAngleTurned = 0
        perviousTrajectory = self.getAngle(pathEndCell.parent, pathEndCell)

        # Iterate back through and extract each parent in turn and add
        # it to the path. To work out the travel length along the
        # path, you'll also have to add self at self stage.
        while (cell is not None):
            path.waypoints.appendleft(cell)
            path.travelCost += self.computeLStageAdditiveCost(cell.parent, cell)
            if (cell.parent is not None):
                currentTrajectory = self.getAngle(cell.parent, cell)

                turningAngle = abs(currentTrajectory - perviousTrajectory)

                if(turningAngle > 180):
                    turningAngle = 360 - turningAngle

                self.totalAngleTurned += turningAngle

                perviousTrajectory = currentTrajectory
            cell = cell.parent
            
        # Update the stats on the size of the path
        path.numberOfWaypoints = len(path.waypoints)

        # Note that if we failed to reach the goal, the above mechanism computes a path length of 0.
        # Therefore, if we didn't reach the goal, change it to infinity
        if path.goalReached is False:
            path.travelCost = float("inf")

        print("Total angle turned by robot = " + str(self.totalAngleTurned))
        print("Path travel cost = " + str(path.travelCost))
        print("Path cardinality = " + str(path.numberOfWaypoints))

        self.performanceMetrics["totalAngleTurned"] = self.totalAngleTurned
        self.performanceMetrics["pathTravelCost"] = path.travelCost
        self.performanceMetrics["pathCardinality"] = path.numberOfWaypoints

        # with open(self.exportDirectory, "a") as f:
        #     f.write("Total angle turned by robot = {} \n".format())
        #     f.write("Path travel cost = {} \n".format(path.travelCost))
        #     f.write("Path cardinality = {} \n".format(path.numberOfWaypoints))
        
        # Draw the path if requested
        if (self.showGraphics == True):
            self.plannerDrawer.update()
            self.plannerDrawer.drawPathGraphicsWithCustomColour(path, colour)
            self.plannerDrawer.waitForKeyPress()
        
        # Return the path
        return path

    # Extract the path from a specified end cell to the start. This is not
    # necessarily the full path. Rather, it lets us illustrate parts of the
    # path.
    def extractPathEndingAtCoord(self, endCellCoord):
        endCell = self.searchGrid.getCellFromCoords(endCellCoord)
        self.extractPathEndingAtCell(endCell, 'red')

    # Extract the path between the start and goal.
    def extractPathToGoal(self):
        path = self.extractPathEndingAtCell(self.goal, 'yellow')
       
        return path

    def exportMetrics(self):        
        column_headers = ['PlanningAlgorithm','mapName','numberOfCellsVisited',
                            'maximumLengthOfQueue','totalAngleTurned',
                            'pathTravelCost','pathCardinality']
        
        data = [self.plannerName, self.mapName, self.performanceMetrics['numberOfCellsVisited'],
                self.performanceMetrics['maximumLengthOfQueue'], self.performanceMetrics['totalAngleTurned'],
                self.performanceMetrics['pathTravelCost'], self.performanceMetrics['pathCardinality']]

        # If directory doesn't exist create directory
        if not os.path.exists(os.path.split(self.exportDirectory)[0]):
            os.makedirs(os.path.split(self.exportDirectory)[0])
        
        # If file doesn't exist create file
        if(os.path.isfile(self.exportDirectory)):
            isFileEmpty = (os.stat(self.exportDirectory).st_size == 0)
        else:
            isFileEmpty = True
        
        print(self.mapName)

        rowList=[]
        rowFound=False

        if isFileEmpty:
            with open(self.exportDirectory, 'w') as write_csvfile:
                # Instanstiate writer
                writer = csv.writer(write_csvfile)
                writer.writerow(column_headers)
                writer.writerow(data)
        else:
            with open(self.exportDirectory, 'r') as read_csvfile:
                    # Instantiate reader
                    reader = csv.reader(read_csvfile)
                    # Find if row already exists for that planner

                    for row in reader:
                        if(str(row[0])==str(self.plannerName) and str(row[1])==str(self.mapName)):
                            rowFound=True
                        else:
                            rowList.append(row)
            # If row was not found then append file.          
            if(not rowFound):
                with open(self.exportDirectory, 'a') as write_csvfile:
                    writer = csv.writer(write_csvfile)
                    writer.writerow(data)
            # If row was found then rewrite the whole file without the old row
            else:
                with open(self.exportDirectory, 'w') as write_csvfile:
                    rowList.append(data)
                    writer = csv.writer(write_csvfile)
                    for row in rowList:
                        writer.writerow(row)


        
            