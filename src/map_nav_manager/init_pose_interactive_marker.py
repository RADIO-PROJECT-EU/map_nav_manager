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
from geometry_msgs.msg import PoseWithCovarianceStamped

import actionlib
from geometry_msgs.msg import Pose2D, PoseStamped
from std_srvs.srv import Empty

# Client based on ActionServer to send goals to the purepursuit node
class InitPoseClient():
	
	def __init__(self, topic_name):
		self.topic_name = topic_name
		# Creates a ROS publisher
		self.client = rospy.Publisher(topic_name, PoseWithCovarianceStamped, queue_size=10)

	## @brief Sends the pose 
	## @param goal_pose as geometry_msgs/PoseStamped
	## @return 0 if OK, -1 if no server, -2 if it's tracking a goal at the moment
	def setPose(self, pose):
		
		self.client.publish(pose)
		
		return

## @brief Class to manage  the creation of a Waypoint base on InteractiveMarker
class PointPath(InteractiveMarker):
	
	def __init__(self, frame_id, name, description, is_manager = False, speed = 0.2):
		InteractiveMarker.__init__(self)

		marker_scale_x = rospy.get_param('~marker_scale_x', 1.1)
		marker_scale_y = rospy.get_param('~marker_scale_y', 0.3)
		marker_scale_z = rospy.get_param('~marker_scale_z', 0.3)
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
			self.marker.color.r = 0.0
			self.marker.color.g = 0.8
			self.marker.color.b = 0.0
			self.marker.color.a = 0.5
		else:
			self.marker.color.r = 0.0
			self.marker.color.g = 1.0
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
class InitPoseManager(InteractiveMarkerServer):
	
	def __init__(self, name, frame_id, topic_name):
		InteractiveMarkerServer.__init__(self, 'init_pose_interactive_marker')
		self.list_of_points = []
		self.frame_id = frame_id
		self.counter_points = 0
		
		# Menu handler to create a menu
		self.menu_handler = MenuHandler()
		h_first_entry = self.menu_handler.insert( "Set Pose",callback=self.setPoseCB)
		
		# Creates the first point
		#self.list_of_points.append(PointPath(frame_id, 'p1', 'p1'))
		self.initial_point = PointPath(frame_id, 'Init Pose', 'Init Pose', True)
		self.insert(self.initial_point, self.initial_point.processFeedback)
		
		self.menu_handler.apply( self, self.initial_point.name )
		self.applyChanges()
		
		self.init_pose_client = InitPoseClient(topic_name)
		
		# The covariance set whenever setting the global pose
		self._default_pose_covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
		
		
	## @brief Starts the route
	def setPoseCB(self, feedback):		
		msg = PoseWithCovarianceStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = self.frame_id
		msg.pose.covariance = self._default_pose_covariance
		msg.pose.pose.position.x = self.initial_point.pose.position.x
		msg.pose.pose.position.y = self.initial_point.pose.position.y
		msg.pose.pose.position.z = 0.0
		msg.pose.pose.orientation.x = self.initial_point.pose.orientation.x
		msg.pose.pose.orientation.y = self.initial_point.pose.orientation.y
		msg.pose.pose.orientation.z = self.initial_point.pose.orientation.z
		msg.pose.pose.orientation.w = self.initial_point.pose.orientation.w
		
		self.init_pose_client.setPose(msg)
		rospy.loginfo('setPoseCB: setting pose')
		return
	
	
	
	
		
if __name__=="__main__":
	rospy.init_node("init_pose_interactive_marker")
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'frame_id': 'map',
	  'topic_name': 'initialpose'
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
	
	server = InitPoseManager(_name, frame_id = args['frame_id'], topic_name = args['topic_name'])
	

	t_sleep = 0.5
	running = True
	
	while not rospy.is_shutdown() and running:
		
		try:
			rospy.sleep(t_sleep)
		except rospy.exceptions.ROSInterruptException:
			rospy.loginfo('Main: ROS interrupt exception')
			running = False
			
		

