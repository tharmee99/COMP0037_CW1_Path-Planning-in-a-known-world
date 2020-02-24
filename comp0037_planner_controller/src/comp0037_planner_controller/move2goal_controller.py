#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose
from math import pow,atan2,sqrt
from comp0037_planner_controller.planned_path import PlannedPath
from comp0037_planner_controller.controller_base import ControllerBase

import math
import angles
import os
import csv

# This sample controller works a fairly simple way. It figures out
# where the goal is. It first turns the robot until it's roughly in
# the correct direction and then keeps driving. It monitors the
# angular error and trims it as it goes.

class Move2GoalController(ControllerBase):

    def __init__(self, occupancyGrid):
        ControllerBase.__init__(self, occupancyGrid)

        # Get the proportional gain settings
        # self.distanceErrorGain = rospy.get_param('distance_error_gain', 1)
        # self.angleErrorGain = rospy.get_param('angle_error_gain', 4)

        # Original gain values for proportional controller
        # self.controllerVariables["distanceErrorGain"] = rospy.get_param('distance_gain', {'Kp':2,'Ki':0,'Kd':0})
        # self.controllerVariables["angleErrorGain"] = rospy.get_param('angle_gain', {'Kp':4,'Ki':0,'Kd':0})

        # Tuned values for PID controller
        self.controllerVariables["distanceErrorGain"] = rospy.get_param('distance_gain', {'Kp':3,'Ki':0,'Kd':0.05})
        self.controllerVariables["angleErrorGain"] = rospy.get_param('angle_gain', {'Kp':4,'Ki':0,'Kd':0.01})

        # Tolerance for the steering angle
        self.driveAngleErrorTolerance = math.radians(rospy.get_param('angle_error_tolerance', 1))
        
        # Data logging settings used for tuning of PID controller
        self.logData = True

        if (self.logData):
            self.firstLog = True

    
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def shortestAngularDistance(self, fromAngle, toAngle):
        delta = toAngle - fromAngle
        if (delta < -math.pi):
            delta = delta + 2.0*math.pi
        elif(delta > math.pi):
            delta = delta - 2.0*math.pi
        return delta
        
    def log_data(self, data):
        path_to_file = os.path.join(os.path.split(self.exportDirectory)[0],'log_data_1.csv')

        column_headers=['current_x','current_y','goal_x', 'goal_y', 'distance_error','theta', 'goal_theta', 'angle_error']
        with open(path_to_file, 'a') as write_csvfile:
            writer = csv.writer(write_csvfile)
            if(self.firstLog):
                writer.writerow(column_headers)
                self.firstLog = False
            writer.writerow(data)

    def driveToWaypoint(self, waypoint):
        vel_msg = Twist()

        dX = waypoint[0] - self.pose.x
        dY = waypoint[1] - self.pose.y

        distanceError = sqrt(dX * dX + dY * dY)
        angleError = self.shortestAngularDistance(self.pose.theta, atan2(dY, dX))
        
        afterFirstIteration = False
        delta_t = 0

        while (distanceError >= self.distanceErrorTolerance) & (not rospy.is_shutdown()):
            if self.logData:
                self.log_data([self.pose.x, self.pose.y,  waypoint[0], waypoint[1], distanceError,
                        self.pose.theta, atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x), angleError])
            
            # print("Distance Error: {}\nAngular Error: {}".format(distanceError, angleError))

            # Proportional Controller
            # Linear velocity in the x-axis: only switch on when the angular error is sufficiently small

            startX = self.pose.x
            startY = self.pose.y
            
            startTheta = self.pose.theta

            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            
            if math.fabs(angleError) < self.driveAngleErrorTolerance:
                vel_msg.linear.x = max(0.0, min(self.pid_controller(distanceError,self.controllerVariables["distanceErrorGain"],afterFirstIteration,1/self.rospy_rate), 10.0))
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.pid_controller(angleError,self.controllerVariables["angleErrorGain"],afterFirstIteration,1/self.rospy_rate), 5.0))


            #print("Linear Velocity: {}\nAngular Velocity: {}\n\n".format(vel_msg.linear.x, math.degrees(vel_msg.angular.z)))
            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()

            self.pathMetrics["distanceTravelled"] += self.get_distance(startX,startY)
            self.pathMetrics["totalAngleTurned"] += abs(self.shortestAngularDistance(startTheta, self.pose.theta)) * (180/math.pi)

            if(not afterFirstIteration):
                afterFirstIteration = True

            distanceError = sqrt(pow((waypoint[0] - self.pose.x), 2) + pow((waypoint[1] - self.pose.y), 2))
            angleError = self.shortestAngularDistance(self.pose.theta,atan2(waypoint[1] - self.pose.y, waypoint[0] - self.pose.x))

        # Make sure the robot is stopped once we reach the destination.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)

    def rotateToGoalOrientation(self, goalOrientation):
        vel_msg = Twist()

        goalOrientation = math.radians(goalOrientation)

        angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)


        afterFirstIteration = False
        delta_t = 0
        while (math.fabs(angleError) >= self.goalAngleErrorTolerance) & (not rospy.is_shutdown()):

            # angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = max(-5.0, min(self.pid_controller(angleError,self.controllerVariables["angleErrorGain"],afterFirstIteration,1/self.rospy_rate), 5.0))

            # Publishing our vel_msg
            self.velocityPublisher.publish(vel_msg)
            if (self.plannerDrawer is not None):
                self.plannerDrawer.flushAndUpdateWindow()
                
            self.rate.sleep()

            if (not afterFirstIteration):
                afterFirstIteration = True

            angleError = self.shortestAngularDistance(self.pose.theta, goalOrientation)

        # Stop movement once finished
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)
        