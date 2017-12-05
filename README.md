# map_nav_manager

A ROS node to manage processes for performing navigation and mapping from a web-based user inteface

This packages was based on the package map_nav_manager from [Jose Rapado](https://github.com/JoseRobotnik/map_nav_manager.git)


## 1- Dependencies

* Firefox navigator 

## 2- Install & configuration


**1- Copy the repository into your catkin workspace.**

The web will need the model urdf of the robot you are going to visualize. To access the model you have to link the description package inside the web folder. To do it you can use the script located into the scripts folders:

```
> link_robot_description.sh robot_description_package
```

**2- Edit the file global_config.js in the folder web/js**

There are some global variables that need to be configured:

```
var hostname = 'localhost'
var namespace = '/summit_xl_a'
var map_frame = 'summit_xl_a_map'
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


**2- Launch the map_nav_manager node**

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
  * list of params to run the navigation node (see config/map_nav_manager_node.yaml for further information)
* mapping (dictionary) 
  * list of params to run the navigation node (see config/map_nav_manager_node.yaml for further information)
* map_server (dictionary)
  * list of params to run the navigation node (see config/map_nav_manager_node.yaml for further information)
* map_saver (dictionary)
  * list of params to run the navigation node (see config/map_nav_manager_node.yaml for further information)
* localization (dictionary)
  * list of params to run the navigation node (see config/map_nav_manager_node.yaml for further information)

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




