#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import subprocess
import shlex
import os

import rospy
import rospkg
import rosparam
import roslaunch

import time
import threading

from map_nav_manager.msg import State as MapNavState
from robotnik_msgs.msg import State
from std_srvs.srv import *
from map_nav_manager.srv import *


DEFAULT_FREQ = 10.0
MAX_FREQ = 100.0


# Class Template of Robotnik component for Pyhton
class MapNavManagerNode:

	def __init__(self, args):

		self.node_name = rospy.get_name().replace('/','')
		self.desired_freq = args['desired_freq']
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ


		self.real_freq = 0.0

		# Saves the state of the component
		self.state = State.INIT_STATE
		# Saves the previous state
		self.previous_state = State.INIT_STATE
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 / self.desired_freq
		# State msg to publish
		self.msg_state = MapNavState()
		# Timer to publish state
		self.publish_state_timer = 1

		self.start_mapping = False
		self.stop_mapping = False
		self.start_navigation = False
		self.stop_navigation = False
		self.save_map = False
		self.load_map = False

		self.mapping = False
		self.navigation = False
		
		self.commands = {}
		self.commands['mapping'] = args['mapping']		
		self.commands['navigation'] = args['navigation']
		self.commands['map_saver'] = args['map_saver']
		self.commands['map_server'] = args['map_server']
		self.commands['localization'] = args['localization']
		
		self.gmapping_command = 'roslaunch summit_xl_localization slam_gmapping.launch prefix:=summit_xl_a_' #params?
		self.save_map_command = 'rosrun map_server map_saver -f '
		self.load_map_command = 'rosrun map_server map_server'
		self.navigation_command = 'roslaunch map_nav_manager navigation.launch' #params?
		self.map_name = ''


		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)

		#Service objects
		self.run_mapping_srv = None  # rospy.Service('~start_mapping_srv', Trigger, self.startMappingServiceCb)
		self.run_navigation_srv = None  # rospy.Service('~start_navigation_srv', Trigger, self.startNavigationServiceCb)
		self.stop_mapping_srv = None
		self.stop_navigation_srv = None
		self.save_map_srv = None
		self.load_map_srv = None

	# ===================================================================================================================================
	# ===================================================================================================================================

	def startMappingServiceCb(self, req):
		'''
			ROS start mapping server
			@param req: Required action
			@type req: std_srv/Empty
		'''

		rospy.loginfo('startMappingServiceCb: called!')
		if not self.mapping:
			self.start_mapping = True
			#self.stop_mapping = False
		return TriggerResponse(True, "Successfull!")

	# ===================================================================================================================================
	# ===================================================================================================================================

	def stopMappingServiceCb(self, req):
		rospy.loginfo('stopMappingServiceCb: called!')
		if self.mapping:
			self.stop_mapping = True
		return TriggerResponse(True, "Successfull!")

	# ===================================================================================================================================
	# ===================================================================================================================================

	# ===================================================================================================================================
	# ===================================================================================================================================
	def startNavigationServiceCb(self, req):
		rospy.loginfo('startNavigationServiceCb: called!')
		if not self.navigation:
			self.start_navigation = True
		return TriggerResponse(True, "Successfull!")

	# ===================================================================================================================================
	# ===================================================================================================================================

	def stopNavigationServiceCb(self, req):
		rospy.loginfo('stopNavigationServiceCb: called!')
		if self.navigation:
			self.stop_navigation = True
		#self.run_slam_gmapping = False
		return TriggerResponse(True, "Successfull!")

	#===================================================================================================================================
	#===================================================================================================================================

	def saveMapServiceCb(self, req):
		rospy.loginfo('saveMapServiceCb: called!')
		# self.run_slam_gmapping = False
		# si no esta mapping a true es poruqe no esta funcionando gmapping luego no hay topic /map para salvar
		if self.mapping:
			# rospy.loginfo(req.name)
			self.map_name = req.name
			self.save_map = True
			return SetFilenameResponse(True, "Map saved correctly")
		else:
			return SetFilenameResponse(False, "It's not possible to save the map")

	 #===================================================================================================================================
	#===================================================================================================================================
		  

	def loadMapServiceCb(self, req):
		rospy.loginfo('loadMapServiceCb: called!')
		if not self.navigation:
			# rospy.loginfo(req.name)
			self.map_name = req.name
			self.load_map = True
			return SetFilenameResponse(True, "El mapa se ha cargado correctamente")
		else:
			return SetFilenameResponse(False, "El proceso de navegacion esta ejecutandose")




	# ===================================================================================================================================
	# ===================================================================================================================================
	
	def startProcess(self, command_type):
		'''
			starts the process defined in the self.commands attribute
			returns 0 if OK, -1 if error
		'''
		if self.commands.has_key(command_type):
			if self.commands[command_type]['process'] != '':
				rospy.logwarn("startProcess: process for %s already running", command_type)
				return 0
				
			command = self.commands[command_type]['command'] + ' ' + self.commands[command_type]['params']
			command = shlex.split(command)
			try:
				self.commands[command_type]['process'] = subprocess.Popen(command)
				rospy.loginfo("startProcess: %s process started", command_type)
			except ValueError:
				rospy.logerr('startProcess: error running the command %s', self.commands[command_type]['command'] + ' ' + self.commands[command_type]['params'])
				return -1
		else:
			rospy.logerr('startProcess:: no %s commands defined', command_type)
			return -1
			
		return 0
	
	
	def stopProcess(self, command_type):
		'''
			stops the process defined in the self.commands attribute
			returns 0 if OK, -1 if error
		'''
		
		if self.commands.has_key(command_type):
			if self.commands[command_type]['process'] != '':
				try:
					self.commands[command_type]['process'].terminate()
					self.commands[command_type]['process'] = ''
					rospy.loginfo("stopProcess: %s process stopped", command_type)
				except ValueError:
					rospy.logerr('stopProcess: error terminating the command %s', self.commands[command_type]['command'] + ' ' + self.commands[command_type]['params'])
					return -1
					
			else:
				rospy.logwarn("stopProcess: process for %s not running", command_type)
				return 0
			
		else:
			rospy.logerr('stopProcess:: no %s commands defined', command_type)
			return -1
			
		return 0
		
		
	def waitProcess(self, command_type):
		'''
			waits the process defined in the self.commands attribute
			returns 0 if OK, -1 if error
		'''
		
		if self.commands.has_key(command_type):
			if self.commands[command_type]['process'] != '':
				try:
					self.commands[command_type]['process'].wait()
					self.commands[command_type]['process'] = ''
					rospy.loginfo("waitProcess: %s process stopped", command_type)
				except ValueError:
					rospy.logerr('waitProcess: error waiting the command %s', self.commands[command_type]['command'] + ' ' + self.commands[command_type]['params'])
					return -1
					
			else:
				rospy.logwarn("waitProcess: process for %s not running", command_type)
				return 0
			
		else:
			rospy.logerr('waitProcess:: no %s commands defined', command_type)
			return -1
			
		return 0
		
		
	# ===================================================================================================================================
	# ===================================================================================================================================
	
	def startMappingNodes(self):
		'''
			ROS start mapping nodes (gmapping)
			@param req: Required action
			@type req: std_srv/Empty
		'''
		
		if self.startProcess('mapping') == 0:
			self.mapping = True
		
		self.start_mapping = False
		

	# ===================================================================================================================================
	# ===================================================================================================================================

	def stopMappingNodes(self):
		'''
			ROS start mapping nodes (gmapping)
			@param req: Required action
			@type req: std_srv/Empty
		'''
		
		if self.stopProcess('mapping') == 0:
			self.mapping = False
		
		self.stop_mapping = False
		
		

	#===================================================================================================================================
	#===================================================================================================================================

	def startNavigationNodes(self):
		'''
			ROS start navigation nodes (amcl, move_base)
			@param req: Required actionS
			@type req: std_srv/Empty
		'''
		if self.startProcess('navigation') == 0:
			self.navigation = True
		
		self.start_navigation = False
		

	# ===================================================================================================================================
	# ===================================================================================================================================

	def stopNavigationNodes(self):
		
		if self.stopProcess('navigation') == 0:
			self.navigation = False
			
		self.stop_navigation = False

	#===================================================================================================================================
	#===================================================================================================================================

	def saveMap(self):
		'''
			ROS start navigation nodes (amcl, move_base)
			@param req: Required action
			@type req: std_srv/Empty
		'''
		if self.commands.has_key('map_saver'):
			self.commands['map_saver']['params'] = '-f ' + self.commands['map_saver']['maps_folder'] + '/' + self.map_name
		
		self.startProcess('map_saver')
		self.waitProcess('map_saver')
		self.save_map = False

	# ===================================================================================================================================
	# ===================================================================================================================================

	def loadMap(self):
		if self.load_map:
			command = self.load_map_command + self.map_name + ".yaml"
			command = shlex.split(command)
			subprocess.Popen(command)
			self.load_map = False

	# ===================================================================================================================================
	# ===================================================================================================================================

	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''

		self.initialized = True

		return 0


	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''

		self.setup()


		if self.ros_initialized:
			return 0

		# Publishers
		self._state_publisher = rospy.Publisher('~state', MapNavState, queue_size=10)
		# Subscribers
		# topic_name, msg type, callback, queue_size
		# self.topic_sub = rospy.Subscriber('topic_name', Int32, self.topicCb, queue_size = 10)
		# Service Servers
		# self.service_server = rospy.Service('~service', Empty, self.serviceCb)
		# Service Clients
		# self.service_client = rospy.ServiceProxy('service_name', ServiceMsg)
		# ret = self.service_client.call(ServiceMsg)

		self.run_mapping_srv = rospy.Service('~start_mapping_srv', Trigger, self.startMappingServiceCb)
		self.run_navigation_srv = rospy.Service('~start_navigation_srv', Trigger, self.startNavigationServiceCb)
		self.stop_mapping_srv = rospy.Service('~stop_mapping_srv', Trigger, self.stopMappingServiceCb)
		self.stop_navigation_srv = rospy.Service('~stop_navigation_srv', Trigger, self.stopNavigationServiceCb)
		self.save_map_srv = rospy.Service('~save_map_srv', SetFilename, self.saveMapServiceCb)
		self.load_map_srv = rospy.Service('~load_map_srv', SetFilename, self.loadMapServiceCb)

		self.ros_initialized = True

		self.publishROSstate()

		return 0


	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.initialized:
			return -1
		rospy.loginfo('%s::shutdown'%self.node_name)

		# Cancels current timers
		self.t_publish_state.cancel()

		self._state_publisher.unregister()

		self.initialized = False

		return 0


	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1

		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()

		self.ros_initialized = False

		return 0


	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False

		return 0


	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		self.rosSetup()

		if self.running:
			return 0

		self.running = True

		self.controlLoop()

		return 0


	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''
		rospy.loginfo('%s::controlLoop: Running...'%self.node_name)

		while self.running and not rospy.is_shutdown():
			t1 = time.time()

			if self.state == State.INIT_STATE:
				self.initState()

			elif self.state == State.STANDBY_STATE:
				self.standbyState()

			elif self.state == State.READY_STATE:
				self.readyState()

			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()

			elif self.state == State.FAILURE_STATE:
				self.failureState()

			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()

			self.allState()

			t2 = time.time()
			tdiff = (t2 - t1)


			t_sleep = self.time_sleep - tdiff

			if t_sleep > 0.0:
				try:
					rospy.sleep(t_sleep)
				except rospy.exceptions.ROSInterruptException:
					rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name)
					self.running = False

			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)

		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)

		return 0


	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''

		return 0


	def initState(self):
		'''
			Actions performed in init state
		'''

		if not self.initialized:
			self.setup()

		else:
			self.switchToState(State.STANDBY_STATE)


		return


	def standbyState(self):
		'''
			Actions performed in standby state
		'''

		self.switchToState(State.READY_STATE)

		return


	def readyState(self):
		'''
			Actions performed in ready state
		'''
		if self.start_mapping:
			self.startMappingNodes()
		if self.stop_mapping:
			self.stopMappingNodes()
		if self.start_navigation:
			self.startNavigationNodes()
		if self.stop_navigation:
			self.stopNavigationNodes()
		if self.save_map:
			self.saveMap()
		if self.load_map:
			self.loadMap()

		return


	def shutdownState(self):
		'''
			Actions performed in shutdown state 
		'''
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)

		return


	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''

		return


	def failureState(self):
		'''
			Actions performed in failure state
		'''


		return


	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.previous_state = self.state
			self.state = new_state
			rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))

		return


	def allState(self):
		'''
			Actions performed in all states
		'''
		self.rosPublish()

		return


	def stateToString(self, state):
		'''
			@param state: state to set
			@type state: State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'

		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'

		elif state == State.READY_STATE:
			return 'READY_STATE'

		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'

		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'

		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'
		else:
			return 'UNKNOWN_STATE'


	def publishROSstate(self):
		'''
			Publish the State of the component at the desired frequency
		'''
		self.msg_state.state.state = self.state
		self.msg_state.state.state_description = self.stateToString(self.state)
		self.msg_state.state.desired_freq = self.desired_freq
		self.msg_state.state.real_freq = self.real_freq
		self.msg_state.mapping = self.mapping
		self.msg_state.navigation = self.navigation
		self._state_publisher.publish(self.msg_state)

		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()

	"""
	def topicCb(self, msg):
		'''
			Callback for inelfe_video_manager state
			@param msg: received message
			@type msg: std_msgs/Int32
		'''
		# DEMO
		rospy.loginfo('RComponent:topicCb')


	def serviceCb(self, req):
		'''
			ROS service server
			@param req: Required action
			@type req: std_srv/Empty
		'''
		# DEMO
		rospy.loginfo('RComponent:serviceCb')	
	"""

def main():

    rospy.init_node("map_nav_manager_node")


    _name = rospy.get_name().replace('/','')

    arg_defaults = {
        'topic_state': 'state',
        'desired_freq': DEFAULT_FREQ,
        'navigation': {},
        'mapping': {},
        'map_server': {},
        'map_saver': {},
        'localization': {}
    }

    args = {}

    for name in arg_defaults:
        try:
            if rospy.search_param(name):
                args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
            else:
                args[name] = arg_defaults[name]
            #print name
        except rospy.ROSException, e:
            rospy.logerr('%s: %s'%(e, _name))


    map_nav_manager_node = MapNavManagerNode(args)

    rospy.loginfo('%s: starting'%(_name))

    map_nav_manager_node.start()


if __name__ == "__main__":
    main()
