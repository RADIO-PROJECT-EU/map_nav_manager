#!/usr/bin/env python

"""
Copyright (c) 2017, Robotnik Automation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("interactive_markers")
import rospy, rospkg
import copy
import os

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl
from interactive_markers.menu_handler import *
from move_base_msgs.msg import * 

import actionlib
from geometry_msgs.msg import Pose2D, PoseStamped
from std_srvs.srv import Empty

# Client based on ActionServer to send goals to the purepursuit node
class MoveBaseClient():
	
	def __init__(self, planner_name):
		self.planner_name = planner_name
		# Creates the SimpleActionClient, passing the type of the action
		# (GoTo) to the constructor.
		self.client = actionlib.SimpleActionClient(planner_name, MoveBaseAction)

	## @brief Sends the goal to 
	## @param goal_pose as geometry_msgs/PoseStamped
	## @return 0 if OK, -1 if no server, -2 if it's tracking a goal at the moment
	def goTo(self, goal_pose):
		# Waits until the action server has started up and started
		# listening for goals.
		if self.client.wait_for_server(timeout = rospy.Duration(3.0) ):
			#if self.getState() != GoalStatus.LOST:
			#	rospy.loginfo('PurepursuitClient: planner is tracking a goal')
			#	return -2
				
			goal = MoveBaseGoal()
				
			#set goal
			goal.target_pose = goal_pose
			
			self.client.send_goal(goal)
			return 0
		else:
			rospy.logerr('MoveBaseClient: Error waiting for server')
			return -1
	
	## @brief cancel the current goal
	def cancel(self):		
		rospy.loginfo('MoveBaseClient: cancelling the goal')
		self.client.cancel_goal()
	
	## @brief Get the state information for this goal
    ##
    ## Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED,
    ## PREEMPTED, ABORTED, SUCCEEDED, LOST.
    ##
    ## @return The goal's state. Returns LOST if this
    ## SimpleActionClient isn't tracking a goal.
	def getState(self):
		return self.client.get_state()
	
	## @brief Returns ret if OK, otherwise -1
	def getResult(self):
		ret = self.client.get_result()
		if not ret:
			return -1
		
		else:
			return ret

## @brief Class to manage  the creation of a Waypoint base on InteractiveMarker
class PointPath(InteractiveMarker):
	
	def __init__(self, frame_id, name, description, is_manager = False, speed = 0.2):
		InteractiveMarker.__init__(self)

		marker_scale_x = rospy.get_param('~marker_scale_x', 1.0)
		marker_scale_y = rospy.get_param('~marker_scale_y', 0.2)
		marker_scale_z = rospy.get_param('~marker_scale_z', 0.2)
		print "Marker scales " + str(marker_scale_x) + ", " + str(marker_scale_y) + ", " + str(marker_scale_z)
		
		self.header.frame_id = frame_id
		self.name = name
		self.description = description
		self.speed = speed
		self.marker = Marker()
		self.marker.type = Marker.ARROW
		self.marker.scale.x = marker_scale_x
		self.marker.scale.y = marker_scale_y
		self.marker.scale.z = marker_scale_z
		#self.marker.pose.position.z = 0.20
		if is_manager:
			self.marker.color.r = 0.8
			self.marker.color.g = 0.0
			self.marker.color.b = 0.0
			self.marker.color.a = 0.5
		else:
			self.marker.color.r = 0.0
			self.marker.color.g = 0.8
			self.marker.color.b = 0.0
			self.marker.color.a = 0.5
			
		self.marker_control = InteractiveMarkerControl()
		self.marker_control.always_visible = True
		self.marker_control.orientation.w = 1
		self.marker_control.orientation.x = 0
		self.marker_control.orientation.y = 1
		self.marker_control.orientation.z = 0
		self.marker_control.name = "move_plane"
		self.marker_control.markers.append( self.marker )
		self.marker_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
		
		self.controls.append( self.marker_control )

		self.marker_control2 = InteractiveMarkerControl()
		self.marker_control2.orientation.w = 1
		self.marker_control2.orientation.x = 0
		self.marker_control2.orientation.y = 1
		self.marker_control2.orientation.z = 0
		self.marker_control2.name = "rotate_z"
		self.marker_control2.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

		self.controls.append( self.marker_control2 )
		
	## @brief method called every time that an interaction is received	
	def processFeedback(self, feedback):
		#p = feedback.pose.position
		self.pose = feedback.pose
		#print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)
		

## @brief Manages the creation of waypoints and how to send them to Purepursuit
class PointPathManager(InteractiveMarkerServer):
	
	def __init__(self, name, frame_id, planner):
		InteractiveMarkerServer.__init__(self, 'goto_interactive_marker')
		self.list_of_points = []
		self.frame_id = frame_id
		self.counter_points = 0
		
		# Menu handler to create a menu
		self.menu_handler = MenuHandler()
		h_first_entry = self.menu_handler.insert( "Waypoints" )
		h_second_entry = self.menu_handler.insert( "Path" )
		entry = self.menu_handler.insert( "Go", parent=h_second_entry, callback=self.startRouteCB)	# Send the path from the first point to the last one
		entry = self.menu_handler.insert( "Stop", parent=h_second_entry, callback=self.stopRouteCB)	# Stops the current path 
		
		# Creates the first point
		#self.list_of_points.append(PointPath(frame_id, 'p1', 'p1'))
		self.initial_point = PointPath(frame_id, 'GoTo', 'GoTo', True)
		self.insert(self.initial_point, self.initial_point.processFeedback)
		
		self.menu_handler.apply( self, self.initial_point.name )
		self.applyChanges()
		
		self.planner_client = MoveBaseClient(planner)
		
		# Locates and loads the UI file into the widget
		#rp = rospkg.RosPack()		
		# loads a ui file for the dialog
		#self.points_file_path = os.path.join(rp.get_path('robotnik_pp_planner'), 'config', 'waypoints.txt')
		
		#rospy.Timer(rospy.Duration(5), self.createNewPoint)
		#self._go_service = rospy.Service('~go',Empty, self.goService)
		#self._go_back_service = rospy.Service('~go_back', Empty, self.goBackService)
		#self._cancel_service = rospy.Service('~cancel', Empty, self.cancelService)
		
	
		
		
	## @brief Starts the route
	def startRouteCB(self, feedback):
		#goals = self.convertListOfPointPathIntoGoal()
		#print 'goals: %s'%(goals)
		#self.planner_client.goTo(goals)
		goal = PoseStamped()
		goal.header.stamp = rospy.Time.now()
		goal.header.frame_id =  self.frame_id
		goal.pose.position.x = self.initial_point.pose.position.x
		goal.pose.position.y = self.initial_point.pose.position.y
	
		goal.pose.orientation.x= self.initial_point.pose.orientation.x
		goal.pose.orientation.y = self.initial_point.pose.orientation.y
		goal.pose.orientation.z = self.initial_point.pose.orientation.z
		goal.pose.orientation.w = self.initial_point.pose.orientation.w
		
		self.planner_client.goTo(goal)
		rospy.loginfo('startRouteCB: Sending pose')
		return
	
	
	## @brief Stops the current route if it's started
	def stopRouteCB(self, feedback):
		self.planner_client.cancel()
		return
	
	
		
if __name__=="__main__":
	rospy.init_node("goto_interactive_marker")
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'frame_id': 'map',
	  'planner': 'move_base'
	}
	
	args = {}
	
	for name in arg_defaults:
		try:
			if rospy.search_param(name): 
				args[name] = rospy.get_param('~'+name) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerror('%s: %s'%(e, _name))
	
	server = PointPathManager(_name, frame_id = args['frame_id'], planner = args['planner'])
	

	t_sleep = 0.5
	running = True
	
	while not rospy.is_shutdown() and running:
		
		try:
			rospy.sleep(t_sleep)
		except rospy.exceptions.ROSInterruptException:
			rospy.loginfo('Main: ROS interrupt exception')
			running = False
			
		

