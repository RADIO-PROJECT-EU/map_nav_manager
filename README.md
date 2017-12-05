# map_nav_manager

A ROS node to manage processes for performing navigation and mapping from a web-based user inteface

This packages was based on the package map_nav_manager from [Jose Rapado](https://github.com/JoseRobotnik/map_nav_manager.git)


## Dependencies

* Firefox navigator 

## Install & configuration


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

## Startup

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

It launches the node and Interactive Markers to send goals to move_base

```
> roslaunch map_nav_manager map_nav_manager.launch
```

Arguments for the launch file:

* global_frame_id
  * global frame id used to represent the model and command goals (default: map)


**3- Open your Firefox navigator and access the host with the correct port**

Example:

http://localhost:8001


