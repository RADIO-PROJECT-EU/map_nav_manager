var ros = new ROSLIB.Ros({
     url : 'ws://'+hostname+':9090'
});

var mapping = false
var navigation = false
var localization = false
var map_server = false
 

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
