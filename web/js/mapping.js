

function startMapping(){

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

		window.alert("Error. Nodo SLAM-Gmapping no iniciado");

	}else{

		var file_name = $('#filename').val();

		if(file_name == ''){
			window.alert("Error. Escriba un nombre de archivo.");
		}else{

			console.log(file_name);
			
			var svc = new ROSLIB.Service({  
				ros : ros,
				name :  namespace+'/map_nav_manager/save_map',
				messageType : 'map_nav_manager/SetFilename'
			});

			var data = new ROSLIB.ServiceRequest({
				name : file_name
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


	
