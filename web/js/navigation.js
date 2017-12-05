
// MAP SERVER
function startMapServer(){
	if(mapping == true){
		window.alert("Error. Mapping has to be off before running a map server");
		return;
	}

	if(map_server==true){

		window.alert("Map server already running");

	}else{

		var file_name = $('#filename').val();

		if(file_name == ''){
			window.alert("startMapServer: loading default map");
		}else{

			console.log(file_name);
			
			var svc = new ROSLIB.Service({  
				ros : ros,
				name : namespace + '/map_nav_manager/start_map_server',
				messageType : 'map_nav_manager/SetFilename'
			});

			var data = new ROSLIB.ServiceRequest({
				name : file_name
			});

			svc.callService(data,function(res){
				console.log("startMapServer: Response received");
			});	

			window.alert("startMapServer: request successfully sent.");
		}
	}

}

function stopMapServer(){
	if(map_server==true){
		var svc = new ROSLIB.Service({  
				ros : ros,
				name : namespace + '/map_nav_manager/stop_map_server',
				messageType : 'std_srv/Trigger'
		});

		var data = new ROSLIB.ServiceRequest({
			
		});
		svc.callService(data,function(res){
			console.log("stopMapServer: Response received");
		});	
		window.alert("Stopping Map Server");

	}else{
		window.alert("Map Server not running");

	}
}

// LOCALIZATION
function startLocalization(){
	if(mapping == true){
		window.alert("Error. Mapping has to be off before running localization node");
		return;
	}

	if(map_server == false){
		window.alert("Localization node needs a map server");
	}
	
	if(localization==true){

		window.alert("Localization already running");

	}else{
	
		var svc = new ROSLIB.Service({  
			ros : ros,
			name : namespace + '/map_nav_manager/start_localization',
			messageType : 'std_srv/Trigger'
		});

		var data = new ROSLIB.ServiceRequest({
		});

		svc.callService(data,function(res){
			console.log("startLocalization: Response received");
		});	

		window.alert("startLocalization: request successfully sent.");
		
	}

}

function stopLocalization(){
	
	if(localization==true){
		
		var svc = new ROSLIB.Service({  
				ros : ros,
				name : namespace + '/map_nav_manager/stop_localization',
				messageType : 'std_srv/Trigger'
		});

		var data = new ROSLIB.ServiceRequest({
			
		});
		svc.callService(data,function(res){
			console.log("stopLocalization: Response received");
		});	
		window.alert("Stopping Localization node");

	}else{
		window.alert("Localization node is not running");

	}
}

function goIndex(){
    window.location.href = "index.html";
}

function goMapping(){
    window.location.href = "mapping.html";
}
