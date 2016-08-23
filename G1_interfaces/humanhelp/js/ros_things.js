var ros;
var viewer;
var cmd_sender;
var send_coords;
var state_checker;
var location_update_status_checker;
var location_update_confirm;
var program_send_srv;
var send_obj;
var request1;
var request2;
var request3;
var request4;
var request5;
var request6;
var request7;

// Poll 5 seconds to check for robot connection
function check_conn(){
  setInterval(function(){
    if(ros.isConnected == true){
      $("#conn_stat").html("Connected to Robot");
      $(".btn-group").find("#fetch_img").removeAttr("disabled");
      $(".btn-group").find("#fetch_img").attr("class","btn btn-info");
      $("#connect_btn").html("Disconnect");
      $("#connect_btn").attr("class","btn btn-success");
    }else{
      $("#connect_btn").html("Connect");
      $("#connect_btn").attr("class","btn btn-danger");
      $("#conn_stat").html("Disconnected from Robot");
      $("#fetch_img").attr("disabled", "disabled");
    }
  }, 5000);
}

// Send Robot Commands
function send_cmd(cmd_action, cmd_utensil_specifier, cmd_utensil, cmd_location_id){
  cmd_action = cmd_action.toLowerCase();
  cmd_utensil_specifier = cmd_utensil_specifier.toLowerCase();
  cmd_utensil = cmd_utensil.toLowerCase();
  cmd_location_id = cmd_location_id.toLowerCase();
  // Service for sending commands
  cmd_sender = new ROSLIB.Service({
    ros : ros,
    name : command_srv_name,
    serviceType : command_srv_type
  });
  request1 = new ROSLIB.ServiceRequest({
    action: cmd_action,
    utensil_specifier: cmd_utensil_specifier,
    utensil: cmd_utensil,
    location_id: cmd_location_id
  }); 
  cmd_sender.callService(request1, function(result) {
    var cmd_status = result.cmd_state;
    switch(cmd_status){
      case 0:
      err_type = 0;
      var cmd_err = "Sorry! Location " + location_id + " was not found. Please intervene now.";
      to_speak(cmd_err);
      $("#alert_1").text(cmd_err);
      $("#intervene").trigger('click');
      setTimeout(function(){
        $("#fetch_img").trigger('click');
      }, 5000);
      break;
      case 1:
      $("#alert_1").text(understand);
      to_speak(understand);
      break;
      default: break;
    }
  });
}

// Poll 1 second to check for logs
function fetch_logs(){
  // Service for getting state
  state_checker = new ROSLIB.Service({
    ros : ros,
    name : logs_srv_name,
    serviceType : logs_srv_type
  });
  setInterval(function(){
   // Request Packet
   request3 = new ROSLIB.ServiceRequest({
     get_state: 1
   });
    // Call Service
    state_checker.callService(request3, function(result) {
      append_logs(result.curr_state);
    });
    display_logs();
  },1000);
}

// Service call to send clicked point
function send_coord_serv(location_send, row_send, col_send){
  // Service for sending clicked points
  send_coords = new ROSLIB.Service({
    ros : ros,
    name : click_srv_name,
    serviceType : click_srv_type
  });
  // Request Packet
  request2 = new ROSLIB.ServiceRequest({
    row: row_send,
    col: col_send,
    location_id: location_send
  });
  // Call Service
  send_coords.callService(request2, function(result) {
    if(result.success == true){
      $("#alert_1").attr("class", "alert alert-success");
      $("#alert_1").text("Successfully saved your location.");
      to_speak("Thanks! Please command again.");
      $( "#command" ).trigger( "click" );
    }
  });
}

// Service call to send clicked point
function send_object_serv(obj_id_send, row_send, col_send){
  console.log('Sending object');
  // Service for sending clicked points
  send_obj = new ROSLIB.Service({
    ros : ros,
    name : obj_srv_name,
    serviceType : obj_srv_type
  });
  // Request Packet
  request7 = new ROSLIB.ServiceRequest({
    row: row_send,
    col: col_send,
    location_id: obj_id_send
  });
  // Call Service
  send_obj.callService(request7, function(result) {
    if(result.success == true){
      $("#alert_1").attr("class", "alert alert-success");
      $("#alert_1").text("Successfully updated your object.");
      to_speak("Thanks! I found your object.");
      $( "#command" ).trigger( "click" );
    }
  });
}

// Service call to update locations
function confirm_location_update(){
  location_update_confirm = new ROSLIB.Service({
    ros : ros,
    name : loc_confirm_srv_name,
    serviceType : loc_confirm_srv_type
  });
   // Request Packet
   request5 = new ROSLIB.ServiceRequest({
     set_loc: true
   });
    // Call Service
    location_update_confirm.callService(request5, function(result) {
      console.log(result);
      $("#alert_1").attr("class", "alert alert-info");
      $("#alert_1").text("Updated Location Successfully");
    });
  }

// Check for location update status
function fetch_location_update_status(){
  // Service for getting state
  location_update_status_checker = new ROSLIB.Service({
    ros : ros,
    name : loc_status_srv_name,
    serviceType : loc_status_srv_type
  });
   // Request Packet
   request4 = new ROSLIB.ServiceRequest({
     get_status: 1
   });
    // Call Service
    location_update_status_checker.callService(request4, function(result) {
      $("#alert_1").attr("class", "alert alert-info");
      $("#alert_1").text(result.upd_status);
    });
  }

// Set program
function send_program(task_name, send_prog_arr){
  var program_success = false;
  // Service for getting state
  program_send_srv = new ROSLIB.Service({
    ros : ros,
    name : prog_send_srv_name,
    serviceType : prog_send_srv_type
  });
   // Request Packet
   request6 = new ROSLIB.ServiceRequest({
     task_name: task_name,
     steps: send_prog_arr
   });
    // Call Service
    program_send_srv.callService(request6, function(result) {
      if(result.nonexist_ids.length > 0){
        program_success = false;
      }else{
        program_success = true;
      }
    });
    return program_success;
  }

// Connect to ROS.
function connect() {
  to_speak("Welcome visitor! How may I be of service today?");
  ros = new ROSLIB.Ros({
    url : 'ws://' + ROBOT_IP + ':' + ROBOT_PORT
  });

  // Check Connection Error
  ros.on('error', function(error) {
    $("#connect_btn").html("Connect");
    $("#connect_btn").attr("class","btn btn-danger");
    $("#conn_stat").html("Error Connecting To Robot");
    $("#fetch_img").attr("disabled", "disabled");
  });

  check_conn();
}

function do_ros(){
  // Create the main viewer.
  viewer = new ROS3D.Viewer({
    divID : 'urdf',
    width : 800,
    height : 600,
    antialias : false
  });

  // Add a grid.
  viewer.addObject(new ROS3D.Grid());

  // Setup a client to listen to TFs.
  var tfClient = new ROSLIB.TFClient({
    ros : ros,
    angularThres : 0.01,
    transThres : 0.01,
    rate : 10.0,
    fixedFrame : '/base'
  });

  // // Setup the URDF client.
  var urdfClient = new ROS3D.UrdfClient({
    ros : ros,
    tfClient : tfClient,
    path : '/humanhelp/',
    rootObject : viewer.scene,
    loader : ROS3D.COLLADA_LOADER_2
  });

  // Setup the marker client.
  var markerClient = new ROS3D.MarkerArrayClient({
   ros : ros,
   tfClient : tfClient,
   topic : '/world_model_visualizer',
   rootObject : viewer.scene
 });

  // Interactive Marker Client
  var intMarkerClient = new ROS3D.InteractiveMarkerClient({
    ros : ros,
    tfClient : tfClient,
    topic : '/location_markers',
    camera : viewer.camera,
    rootObject : viewer.selectableObjects
  });
}

function remove_viewer(){
  viewer = null;
}

// Kill ROS connection
function kill_ros(){
  ros.close()
}