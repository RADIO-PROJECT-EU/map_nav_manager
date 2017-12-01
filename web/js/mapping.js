var ros = new ROSLIB.Ros({
     url : 'ws://'+hostname+':9090'
});

var mapping = false
var navigation = false
var max_poi=10;

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
	
	updatePOI();
	
});

function startMapping(){

	if(mapping == true){
		window.alert("Error: mapping already running");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name : namespace+'/map_nav_manager/start_mapping_srv',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Respuesta Recibida");
	});	
	mapping = true;
	window.alert("Mapping process initiated. Press OK.");
	
	}

}


function startNavigation(){

	if(navigation == true){
		window.alert("Error: navigation already running");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name : namespace+'/map_nav_manager/start_navigation_srv',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Respuesta Recibida");
	});	
	navigation = true;
	window.alert("Navigation process initiated. Press OK.");
	
	}

}

function stopNavigation(){

	if(navigation == false){
		window.alert("Error: navigation not running");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name :  namespace+'/map_nav_manager/stop_navigation_srv',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Respuesta Recibida");
	});
	navigation = false;
	window.alert("Navigation process stopped. press OK.");
	
	}
}


function stopMapping(){

	if(mapping == false){
		window.alert("Error: mapping process not initiated");
	}else{
	var svc = new ROSLIB.Service({  
		ros : ros,
		name :  namespace+'/map_nav_manager/stop_mapping_srv',
		messageType : 'std_srv/Trigger'
	});

	svc.callService(function(res){
		console.log("Respuesta Recibida");
	});
	mapping = false;
	window.alert("Mapping process stopped. Press OK.");
	
	}
}

function saveMap(){

	if(mapping==false){

		window.alert("Error. Nodo SLAM-Gmapping no iniciado");

	}else{

		var file_name = $('#filename').val();

		if(file_name == ''){
			window.alert("Error. Escriba un nombre de archivo.");
		}else{

			console.log(file_name);
			
			var svc = new ROSLIB.Service({  
				ros : ros,
				name :  namespace+'/map_nav_manager/save_map_srv',
				messageType : 'map_nav_manager/SetFilename'
			});

			var data = new ROSLIB.ServiceRequest({
				name : file_name
			});

			svc.callService(data,function(res){
				console.log("Respuesta Recibida");
			});	

			window.alert("El mapa se ha guardado con éxito.");
		}
	}

}

function goIndex(){
	if(mapping==true){
		stopMapping();
	}
    window.location.href = "index.html";
}

function updatePOI(){
	var svc = new ROSLIB.Service({  
		ros : ros,
		name :  namespace+'/labeled_pose_list',	
	});

	var data = new ROSLIB.ServiceRequest({
	});
	
	//get the pointer of all the labels
	var radioitems=$('#radiopoi input:radio');
	//hide all the radios
	for(i = 0; i < radioitems.length; i++){
		$("label[for=radio"+i+"]").hide();
	}

	svc.callService(data, function(res){

		console.log("Respuesta Recibida");
     
		var newlabel;
		var POIid;
		
		//asign the value of the received data to the radio labels	
		for(i = 0; i < (res.pose_list.length) && i < max_poi; i++){
			newlabel= res.pose_list[i].label + ":" + res.pose_list[i].pose.x + ","+res.pose_list[i].pose.y + "," + res.pose_list[i].pose.theta;			
			radioname="radio"+i;
			$("label[for="+radioname+"]").html(""+newlabel);
			$("label[for="+radioname+"]").show();
			POIid = document.getElementById(radioname);
			POIid.setAttribute("value", newlabel);		
		}		
	});
}

function sendgoal(){
	if(navigation==false){
		window.alert("Navigation process is not running. Press OK.");
	}else{
		
		var ros = this.ros;
		var serverName = namespace + '/move_base';
		var actionName = 'move_base_msgs/MoveBaseAction';
		var POIposition = new ROSLIB.Vector3({x:0, y:0});
		var POIorientation = new ROSLIB.Quaternion({x:0, y:0, z:0, w:1});
		var POIvalue = document.querySelector('input[name = "radio"]:checked').value;
		console.log("Value selected:"+POIvalue);
		
		//splitting the string		
		var POITagAndPose= POIvalue.split(":");		
		var POIPose=POITagAndPose[1]+"";
		var POICoordinates=POIPose.split(",");		
		
		////create the pose from the radio selected

		//position
		POIposition.x=parseFloat(POICoordinates[0]);
		POIposition.y=parseFloat(POICoordinates[1]);

		//angle to quaternion
		var thetaRadians = POICoordinates[2];	
		var qz =  Math.sin(thetaRadians/2.0);
		var qw =  Math.cos(thetaRadians/2.0);
		POIorientation[2]=qz;
		POIorientation[3]=qw;		
	
		var pose = new ROSLIB.Pose({
			position : POIposition,
			orientation : POIorientation
		  });		
		
		// setup the actionlib client
  		var actionClient = new ROSLIB.ActionClient({
    		ros : ros,
    		actionName : actionName,
    		serverName : serverName
  		});

		  // create a goal
		var goal = new ROSLIB.Goal({
			actionClient : actionClient,
			goalMessage : {
			  target_pose : {
				header : {
				  frame_id : map_frame
				},
				pose : pose
			  }
			}
		  });
		  goal.send();
		  console.log("goal to:"+pose.position.x+":"+pose.position.y);
	}
}

function goNavigation(){
	if(mapping==true){
		stopMapping();
	}
    window.location.href = "navigation.html";
}
