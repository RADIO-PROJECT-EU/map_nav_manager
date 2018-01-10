# map_nav_manager

A ROS node to manage processes for performing navigation and mapping from a web-based user inteface

This packages was based on the package map_nav_manager from [Jose Rapado](https://github.com/JoseRobotnik/map_nav_manager.git)


## 1- Dependencies

* Firefox navigator 

## 2- Install & configuration


**1- Copy the repository into your catkin workspace.**

The web will need the model urdf of the robot you are going to visualize. To access the model you have to link the description package inside the web folder. To do it you can use the script located into the scripts folders:

```
> link_robot_description.sh turtlebot_description
```

**2- Edit the file global_config.js in the folder web/js**

There are some global variables that need to be configured:

```
var hostname = 'localhost'
var namespace = ''
var map_frame = 'map'
var map_topic = namespace+'/map'
```

* hostname
  * Hostname running the web server
* namespace
  * ROS namespace of the robot
* map_frame
  * Frame id used for map (global frame)
* map_topic
  * ROS topic to get the map

## 3- Startup

*Note: all the launch files (nodes) have to run under the same namespace than the robot.*

**1- Launch the servers and needed nodes:**

* SimpleHttpServer on port 8001 (by default)
* ros_bridge_websocket
* tf2_web_republisher
* robot_pose_publisher

```
> roslaunch map_nav_manager servers.launch 
```


**2- Launch the map_nav_manager node and markers**

It launches the node and Interactive Markers to send goals to move_base and to initialize the global pose (amcl)

```
> roslaunch map_nav_manager map_nav_manager.launch
```

Arguments for the launch file:

* global_frame_id
  * global frame id used to represent the model and command goals (default: map)


**3- Open your Firefox navigator and access the host with the correct port**

Example:

http://localhost:8001

**4- Creating a map

* Select Mapping & Navigation
* Press 'Start mapping'
* Move the robot around manually o by using the navigation marker
  * In case you want to use the navigation marker, please press the 'Start navigation' button
* When you want to save the map, set the name in the field 'Map name' and press 'Save map' button
  * If you want to use this map by default, set the checkbox 'Use this map by default'
* You have to stop the process of mapping by pressing 'Stop mapping' when the map is finished

**5- Localization & Navigation

* Select Localization & Navigation
* Press 'Start Map Server' to load a pre-saved map
  * You can specify the name of the map (without any extension like .yaml or .pgm), otherwise it will be used the default map specified in the config file
* Press 'Start Localization' to run the localization node (amcl)
  * Set the init pose of the robot by using the interactive marker and the option 'Init Pose'
* Press 'Start Navigation' to start the navigation node (move_base)
  * Use the interactive marker to command the robot to any position with the option 'Go' and 'Stop'
* Press 'Enable Autorun' to run all of the localization & navigation nodes by default whenever the robot/map_nav_manager starts 

## 4- Nodes

### 4.1 map_nav_manager_node

It is a process manager that controls the execution of external nodes involved in the mapping, localization and navigation tasks.

It manages the following nodes:
* slam_gmapping
* move_base
* amcl
* map_server
* map_saver

#### parameters

* desired_freq (double)
  * control loop frequency (10 hz default)
* navigation (dictionary)
  * list of params to run the navigation node (see subparams for further information)
* mapping (dictionary) 
  * list of params to run the navigation node (see subparams for further information)
* map_server (dictionary)
  * list of params to run the navigation node (see subparams for further information)
* map_saver (dictionary)
  * list of params to run the navigation node (see subparams for further information)
* localization (dictionary)
  * list of params to run the navigation node (see subparams for further information)
* config_params_file (string)
  * filename inside config folder to save/read the rosparams 


##### Subparameters

* command: system command/process to be called
* params: ROS params added to the command when running it
* args: Arguments added to the program
* autorun: flag to enable the autorun 
* maps_folder: path in the local system to save/load maps
* default_map: default name for saving/loading a map


#### topics

* ~state (map_nav_manager/State)
  * current control state

#### services

* ~save_map (map_nav_managar/SetFilename)
  * saves the current map with the specified name
* ~start_localization (std_srvs/Trigger)
* ~stop_localization (std_srvs/Trigger)
* ~start_map_server (map_nav_managar/SetFilename)
  * runs a map server with the specified map
* ~stop_map_server (std_srvs/Trigger)
* ~start_mapping (std_srvs/Trigger)
* ~stop_mapping (std_srvs/Trigger)
* ~start_navigation (std_srvs/Trigger)
* ~stop_navigation (std_srvs/Trigger)


### 4.2 map_nav_manager_interactive_marker

Interactive marker to send goals to move_base and initialize the pose of the robot

**GoTo**

Move the marker to the desired pose and press the 'Go' option.

To cancel the command press the 'Stop' option.

**Init pose**

Move the marker to the desired pose and press the 'Init Pose' option.

