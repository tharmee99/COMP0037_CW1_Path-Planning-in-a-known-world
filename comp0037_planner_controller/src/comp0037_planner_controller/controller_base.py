#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import time
import math
import os
import json


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
        self.rate = rospy.Rate(10)

        self.exportDirectory = ""

        self.goalNo = 0
        self.plannerName = ""
        self.pathMetrics = {
            "timeForPath" : 0.0,
            "distanceTravelled" : 0.0,
            "totalAngleTurned" : 0.0,
            "plannerPerformance" : {}
        }

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

    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goalOrientation, plannerDrawer, export=True):
        self.plannerDrawer = plannerDrawer

        self.goalNo += 1

        self.pathMetrics["timeForPath"] = 0.0
        self.pathMetrics["distanceTravelled"] = 0.0
        self.pathMetrics["totalAngleTurned"] = 0.0

        rospy.loginfo('Driving path to goal with ' + str(len(path.waypoints)) + ' waypoint(s)')
        
        startTime = time.time()

        # Drive to each waypoint in turn
        for waypointNumber in range(0, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]
            waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
            self.driveToWaypoint(waypoint)
            # Handle ^C
            if rospy.is_shutdown() is True:
                break

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')
        
        endTime = time.time()
        self.pathMetrics["timeForPath"] = endTime-startTime
        
        if(export):
            self.exportPathMetrics()

        # Finish off by rotating the robot to the final configuration
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)

    def exportPathMetrics(self):
        data = {}

        if not os.path.exists(os.path.split(self.exportDirectory)[0]):
            os.makedirs(os.path.split(self.exportDirectory)[0])

        if(not os.path.isfile(self.exportDirectory)):
            with open(self.exportDirectory, 'w+') as json_file:
                json.dump(data, json_file, sort_keys=True, indent=4)
        
        with open(self.exportDirectory, 'w+') as json_file:
            try:
                data = json.load(json_file)
            except ValueError:
                json.dump(data, json_file, sort_keys=True, indent=4)

        if self.plannerName not in data:
            data[self.plannerName] = {}

        data[self.plannerName][self.goalNo] = self.pathMetrics

        with open(self.exportDirectory, 'w+') as json_file:
            json.dump(data, json_file, sort_keys=True, indent=4)
 
