#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import matplotlib.pyplot as plt
import time
import math
import os
import csv

# This is the base class of the controller which moves the robot to its goal.

class ControllerBase(object):

    def __init__(self, occupancyGrid):

        rospy.wait_for_message('/robot0/odom', Odometry)

        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Specification of accuracy. The first is the Euclidean
        # distance from the target within which the robot is assumed
        # to be there. The second is the angle. The latter is turned
        # into radians for ease of the controller.

        self.distanceErrorTolerance = rospy.get_param('distance_error_tolerance', 0.05)
        self.goalAngleErrorTolerance = math.radians(rospy.get_param('goal_angle_error_tolerance', 0.1))
        

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()

        # Store the occupany grid - used to transform from cell
        # coordinates to world driving coordinates.
        self.occupancyGrid = occupancyGrid
        
        # This is the rate at which we broadcast updates to the simulator in Hz.
        self.rospy_rate = 20.0
        self.rate = rospy.Rate(self.rospy_rate)

        self.exportDirectory = ""
        
        self.controllerVariables = {
            "distanceErrorGain" : {},
            "angleErrorGain" : {},
            "prevError" : 0,
            "errorIntegral" : 0
        }

        self.simulationTimeScaleFactor = rospy.get_param('time_scale_factor')

        self.goalNo = 0
        self.plannerName = ""
        self.pathMetrics = {
            "timeForPath" : 0.0,
            "distanceTravelled" : 0.0,
            "totalAngleTurned" : 0.0,
            "plannerPerformance" : {}
        }

    def pid_controller(self, error, controller_gains, afterFirst, delta_t=0.1):

        integral_term = 0
        derivative_term = 0

        if(afterFirst):
            integral_term = self.controllerVariables['errorIntegral'] + (error * delta_t)
            derivative_term = ((error-self.controllerVariables['prevError'])/delta_t)

        # Proportional term
        P = controller_gains['Kp'] * error

        # Integral term
        I = controller_gains['Ki'] * integral_term

        # Derivative term
        D = controller_gains['Kd'] * derivative_term

        # Update the values in controllerVariables
        self.controllerVariables['prevError'] = error
        self.controllerVariables['errorIntegral'] = integral_term

        return (P + I + D)

    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation
        
        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    # Handle the logic of rotating the robot to its final orientation
    def rotateToGoalOrientation(self, waypoint):
        raise NotImplementedError()

    def getAngle(self, parentCell, cell):
        
        del_y = cell.coords[1]-parentCell.coords[1]
        del_x = cell.coords[0]-parentCell.coords[0]

        angle = atan2(del_y,del_x) * (180/pi)

        return angle

    def simplifyPath(self, path):
        new_waypoints = []

        new_waypoints.append(path.waypoints[0])

        perviousTrajectory = self.getAngle(path.waypoints[0],path.waypoints[1])
    
        for waypointNumber in range(1, len(path.waypoints)-1):

            currentTrajectory = self.getAngle(path.waypoints[waypointNumber],path.waypoints[waypointNumber+1])

            turningAngle = abs(currentTrajectory - perviousTrajectory)
            perviousTrajectory = currentTrajectory
            if (turningAngle > 180):
                turningAngle = 360 - turningAngle

            if turningAngle != 0:
                new_waypoints.append(path.waypoints[waypointNumber])

        if path.waypoints[-1] not in new_waypoints:
            new_waypoints.append(path.waypoints[-1])

        return new_waypoints

    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goalOrientation, plannerDrawer, export=True):
        self.plannerDrawer = plannerDrawer

        self.goalNo += 1

        self.pathMetrics["timeForPath"] = 0.0
        self.pathMetrics["distanceTravelled"] = 0.0
        self.pathMetrics["totalAngleTurned"] = 0.0

        newPath = self.simplifyPath(path)

        with open("waypoint_export.txt", "w") as out_file:
            for cell in path.waypoints:
                out_file.writelines(str(cell.coords) + '\n')

        with open("waypoint_export_new", "w") as out_file:
            for cell in newPath:
                out_file.writelines(str(cell.coords) + '\n')

        rospy.loginfo('Driving path to goal with ' + str(len(path.waypoints)) + ' waypoint(s)' + str(len(newPath)))
        
        startTime = time.time()

        # Drive to each waypoint in turn
        for waypointNumber in range(0, len(newPath)):
            cell = newPath[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
            self.driveToWaypoint(waypoint)
            # Handle ^C
            if rospy.is_shutdown() is True:
                break

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')
        
        endTime = time.time()
        self.pathMetrics["timeForPath"] = (endTime-startTime)/(self.simulationTimeScaleFactor)
        
        if(export):
            self.exportPathMetrics()

        # Finish off by rotating the robot to the final configuration
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)

    def exportPathMetrics(self):        
        column_headers = ['PlanningAlgorithm','goalNumber','distanceTravelled','totalAngleTurned','timeForPath',
                          'plannerQueueLength','plannerCellsVisited','plannerPathCardinality','plannerPathCost','plannerAngleTurned']
        
        data = [self.plannerName, self.goalNo, self.pathMetrics['distanceTravelled'], self.pathMetrics['totalAngleTurned'], self.pathMetrics['timeForPath'],
                self.pathMetrics['plannerPerformance']['maximumLengthOfQueue'], self.pathMetrics['plannerPerformance']['numberOfCellsVisited'], 
                self.pathMetrics['plannerPerformance']['pathCardinality'], self.pathMetrics['plannerPerformance']['pathTravelCost'], 
                self.pathMetrics['plannerPerformance']['totalAngleTurned']
               ]

        # If directory doesn't exist create directory
        if not os.path.exists(os.path.split(self.exportDirectory)[0]):
            os.makedirs(os.path.split(self.exportDirectory)[0])
        
        # If file doesn't exist create file
        if(os.path.isfile(self.exportDirectory)):
            isFileEmpty = (os.stat(self.exportDirectory).st_size == 0)
        else:
            isFileEmpty = True
        
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
                        if(str(row[0])==str(self.plannerName) and str(row[1])==str(self.goalNo)):
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
                        