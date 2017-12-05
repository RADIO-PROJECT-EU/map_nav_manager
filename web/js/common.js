var ros = new ROSLIB.Ros({
     url : 'ws://'+hostname+':9090'
});

var mapping = false
var navigation = false
var localization = false
var map_server = false
 
//jquery init
$(document).ready(function() {

	$('div#occupancygrid').width('70%');/* $( "#urdf" ).width(); */

	$('div#panel').width('25%');

	var anchodiv =  $('div#tabs-1').width();
	
	var anchonav = $('div#occupancygrid').width();

	var altonav = anchonav / 1.33;

	var anchopanel = anchodiv - anchonav - 1550;

	$('div#tabs-1').height(altonav);

	$(window).resize(function(){
		location.reload();
		}
	);

	//$('div#panel').width(anchopanel);


	//$('div#tabs-1').width(ancho);
	
	// Create the main viewer.
	var viewer = new ROS3D.Viewer({
  		divID : 'occupancygrid',
		width : anchonav,
		height : altonav - 250,
		antialias : true,
		background: '#002233',
		cameraPose : {x: 3, y: 3, z: 1}
	});

	// Add a grid.
	viewer.addObject(new ROS3D.Grid({
		color:'#0181c4',
  		cellSize: 0.5,
  		num_cells: 20
	}));

	// Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      fixedFrame : map_frame,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0,
      serverName: namespace+'/tf2_web_republisher',
      repubServiceName: namespace+'/republish_tfs'
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros : ros,
      param: namespace+'/robot_description',
      tfClient : tfClient,
      rootObject: viewer.scene
    });
    
    // Setup the marker client.
    var gridClient = new ROS3D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene,
      continuous: true,
      topic: map_topic
    });
    
    var interactiveMarkerClient = new ROS3D.InteractiveMarkerClient({
      ros : ros,
      camera : viewer.camera,
      rootObject : viewer.selectableObjects,
      tfClient : tfClient,
      topic: namespace+'/goto_interactive_marker'
    });
	
});

function startNavigation(){

	if(navigation == true){
		window.alert("Error: navigation already running");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name : namespace+'/map_nav_manager/start_navigation',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Response received");
	});	
	window.alert("Navigation process initiated. Press OK.");
	
	}

}

function stopNavigation(){

	if(navigation == false){
		window.alert("Error: navigation not running");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name :  namespace+'/map_nav_manager/stop_navigation',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Response received");
	});
	window.alert("Navigation process stopped. press OK.");
	
	}
}

// TOPIC SUBSCRIBERS
var map_nav_state_sub = new ROSLIB.Topic({
	ros : ros,
	name : namespace + '/map_nav_manager/state',
	messageType : 'map_nav_manager/State'
});
	
// Topic handlers
map_nav_state_sub.subscribe(function(message) {
	mapping = message.mapping;
	navigation = message.navigation;
	localization = message.localization;
	map_server = message.map_server;
	
	if(mapping){
		document.getElementById("mapping_status").style.color = "Green";
	}else{
		document.getElementById("mapping_status").style.color = "Red";
	}
	if(navigation){
		document.getElementById("navigation_status").style.color = "Green";
	}else{
		document.getElementById("navigation_status").style.color = "Red";
	}
	if(localization){
		document.getElementById("localization_status").style.color = "Green";
	}else{
		document.getElementById("localization_status").style.color = "Red";
	}
	if(map_server){
		document.getElementById("map_server_status").style.color = "Green";
	}else{
		document.getElementById("map_server_status").style.color = "Red";
	}
});	
