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
import math

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import InteractiveMarker, Marker, InteractiveMarkerControl
from interactive_markers.menu_handler import *
from move_base_msgs.msg import * 

import actionlib
from geometry_msgs.msg import Pose2D, PoseStamped
from std_srvs.srv import Empty
from poi_manager.srv import *
from poi_manager.msg import *

h_mode_last = 0

class PointPath(InteractiveMarker):
	
	def __init__(self, frame_id, name, description):
		InteractiveMarker.__init__(self)

		marker_scale_x = rospy.get_param('~marker_scale_x', 0.5)
		marker_scale_y = rospy.get_param('~marker_scale_y', 0.15)
		marker_scale_z = rospy.get_param('~marker_scale_z', 0.15)
		#print "Marker scales " + str(marker_scale_x) + ", " + str(marker_scale_y) + ", " + str(marker_scale_z)
		
		self.header.frame_id = frame_id
		self.name = name
		self.description = 'Description of %s'%description

		self.marker = Marker()
		self.marker.type = Marker.ARROW
		self.marker.scale.x = marker_scale_x
		self.marker.scale.y = marker_scale_y
		self.marker.scale.z = marker_scale_z
		self.marker.pose.position.z = 0.20	
		self.marker.color.r = 0.0
		self.marker.color.g = 0.8
		self.marker.color.b = 0.0
		self.marker.color.a = 0.5	
		self.marker_control = InteractiveMarkerControl()
		self.marker_control.name = 'menu'
		self.marker_control.always_visible = True
		self.marker_control.orientation.w = 1
		self.marker_control.orientation.x = 0
		self.marker_control.orientation.y = 1
		self.marker_control.orientation.z = 0
		self.marker_control.markers.append( self.marker )
		self.marker_control.interaction_mode = InteractiveMarkerControl.MENU
		
		self.controls.append( self.marker_control )
		
	## @brief method called every time that an interaction is received	
	def processFeedback(self, feedback):
		p = feedback.pose.position
		

## @brief Manages the creation of waypoints and how to send them to Purepursuit
class PointPathManager(InteractiveMarkerServer):
	
	def __init__(self, name, frame_id, poi_topic_name):
		InteractiveMarkerServer.__init__(self, 'poi_interactive_marker')
		self.list_of_points = []
		self.frame_id = frame_id
		self.counter_points = 0
		self.loadPointsCB(frame_id)

		# Locates and loads the UI file into the widget
		rp = rospkg.RosPack() 
	
	## @brief Function to load previous saved points
	def loadPointsCB(self, frame_id):				

		#~ ##call read_pois service
		rospy.wait_for_service('read_pois')
		try:
			resp = rospy.ServiceProxy('read_pois', ReadPOIs)
			poi_list = resp()		
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		# Menu handler to create a menu
		self.menu_handler = MenuHandler()
		self.h_first_entry = self.menu_handler.insert( "Menu" )
		self.entry_del = self.menu_handler.insert( "Delete", parent=self.h_first_entry, callback=self.deletePOI );
		
		for elem in poi_list.pose_list:
			
			self.new_point = PointPath(frame_id, elem.label, elem.label)
			self.new_point.pose.position.x = elem.pose.x
			self.new_point.pose.position.y = elem.pose.y			
			self.new_point.pose.orientation.x = 0
			self.new_point.pose.orientation.y = 0
			self.new_point.pose.orientation.z = math.sin(elem.pose.theta*0.5)
			self.new_point.pose.orientation.w = math.cos(elem.pose.theta*0.5)
			self.list_of_points.append(self.new_point)		
			self.insert(self.new_point, self.new_point.processFeedback)
			self.menu_handler.apply( self, self.new_point.name )			
			self.counter_points+=1
			self.applyChanges()	
	
	def update_pois(self):
		rospy.wait_for_service('update_pois')
		update_pois = rospy.ServiceProxy('update_pois', UpdatePOIs)
		newPOIs = []
		for i in self.list_of_points:	
			pose = Pose2D(i.pose.position.x, i.pose.position.y, 2*math.asin(i.pose.orientation.z))    
			newPOIs.append(LabeledPose(i.name,pose))
		try:
			resp = update_pois(newPOIs)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))				
		
	## @brief Callback called to delete point	
	def deletePOI(self, feedback):
		if self.counter_points > 0:						
			for i in self.list_of_points:				
				if i.name==feedback.marker_name:					
					self.list_of_points.remove(i)					
					self.erase(i.name)
					self.counter_points = self.counter_points - 1
					self.applyChanges()
					break
			#call update_pois service
			self.update_pois()
			 
if __name__=="__main__":
	rospy.init_node("poi_markers")
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'frame_id': 'map',
	  'poi_topic_name': 'initialpose',
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
	
	server = PointPathManager(_name, frame_id = args['frame_id'], poi_topic_name = args['poi_topic_name'])
	
	t_sleep = 0.5
	running = True
	
	while not rospy.is_shutdown() and running:
		
		try:
			rospy.sleep(t_sleep)
		except rospy.exceptions.ROSInterruptException:
			rospy.loginfo('Main: ROS interrupt exception')
			running = False
			
		

