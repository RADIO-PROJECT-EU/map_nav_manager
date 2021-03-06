
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
      topic: namespace+'/map_nav_manager_interactive_marker'
    });

});


function startMapping(){
	
	if(map_server == true){
		window.alert("Error. map_server has to be switched off before running a mapping");
		return;
	}
	if(mapping == true){
		window.alert("Error: mapping already running");
	}else{
		var svc = new ROSLIB.Service({  
			ros : ros,
			name : namespace+'/map_nav_manager/start_mapping',
			messageType : 'std_srv/Trigger'
		});

		svc.callService(function(res){
			console.log("Response received");
		});	
		window.alert("Mapping process initiated. Press OK.");
	
	}

}





function stopMapping(){

	if(mapping == false){
		window.alert("Error: mapping process not initiated");
	}else{
		var svc = new ROSLIB.Service({  
			ros : ros,
			name :  namespace+'/map_nav_manager/stop_mapping',
			messageType : 'std_srv/Trigger'
		});

		svc.callService(function(res){
			console.log("stopMapping: Response received");
		});
		window.alert("stopMapping:Mapping process stopped. Press OK.");
	
	}
}

function saveMap(){

	if(mapping==false){

		window.alert("Error. Node SLAM-Gmapping not initiated");

	}else{

		var file_name = $('#filename').val();
		var checkbox_default = $('#checkbox_map_default').is(":checked");
		//console.log("checbox = %d",checkbox_default);
		//console.log(checkbox_default);


		if(file_name == ''){
			window.alert("Error. Escriba un nombre de archivo.");
		}else{

			console.log("map name = %s, save as default = %d",file_name, checkbox_default);
			
			var svc = new ROSLIB.Service({  
				ros : ros,
				name :  namespace+'/map_nav_manager/save_map',
				messageType : 'map_nav_manager/SetFilename'
			});

			var data = new ROSLIB.ServiceRequest({
				name : file_name,
				use_it_by_default: checkbox_default
			});

			svc.callService(data,function(res){
				console.log("Response received");
			});	

			window.alert("El mapa se ha guardado con \u00e9xito.");
		}
	}

}

function goIndex(){
    window.location.href = "index.html";
}

function goNavigation(){
    window.location.href = "navigation.html";
}


	
