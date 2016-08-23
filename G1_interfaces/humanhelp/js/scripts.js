var rows_send = 0;
var cols_send = 0;
var allow_click = false;
var prev_state = -6;
var logs_buffer = [];
var logs_counter = 0;
var program_array = [];
var err_type = -4;

// Initiate ROS Connection
$( document ).ready(function() {
	connect();
	fetch_logs();
});

// Disconnect ROS connection
$("#connect_btn").click(function(){
	if($("#connect_btn").attr("class") == "btn btn-danger"){
		connect();
	}else if($("#connect_btn").attr("class") == "btn btn-success"){
		kill_ros();
	}
});

// Clear up tabs and set new active
function clear_set_tabs(tab_id){
	remove_viewer();
	$('.nav-tabs li').each(function(i){
		$(this).attr('class', 'inactive');
	});
	$(tab_id).attr('class', 'active');
	$("#instructions").text("User Interaction Instructions.");
}

// Start Tour
$("#start_btn").click(function(){
	begin_tour();
});

// Get image from robot camera
$(document).on('click','#fetch_img', function() {
	$( "#intervene" ).trigger( "click" );
	for(var i = 0; i<3; i++){
		$('#user_img').error(function() {
			$("#user_img").attr("src","./img/placeholder.jpg");
			$("#instructions").html("No image received");
		}).attr("src","http://" + IMAGE_IP + ":" + IMAGE_PORT + "/snapshot?topic=" + image_topic + "&width=640&height=480");
		$("#instructions").html("Updated Image");
	}
	allow_click = true;
});

// Check if image exists
$(document).on('mouseenter','#user_img',function(){
	var haystack = $("#user_img").attr("src");
	var needle = 'placeholder';
	if(haystack.indexOf(needle) != -1){
		allow_click = false;
	}else{
		allow_click = true;
	}
});

// Get clicked point on image
$(document).on('click', '#user_img', function(evt){
	if(allow_click){
		evt.preventDefault();
		rows_send = evt.offsetY;
		cols_send = evt.offsetX;
		$("#instructions").html("Confirm Selection Below...")
		$("#alert_1").attr("class", "alert alert-info");
		$("#alert_1").text("You clicked on (" + rows_send + ", " + cols_send + ")");
	}
});

// Confirm click point
$(document).on('click', '#ok_click', function(){
	var curr_window = $("#main_content").children().attr("id");
	if(curr_window == "user_img"){
		switch(err_type){
			case 0:
			send_coord_serv(location_id,rows_send,cols_send);
			break;
			case 2: 
			send_object_serv(utensil,rows_send,cols_send);
			default:
			break;
		}
	}else if(curr_window == "speech_results"){
		var cmd_input = $("#text_cmd").find("#cmd_text").val();
		if(cmd_input != ''){
			nlp(cmd_input);
		}else{
			nlp(final_transcript);
		}
	}else if(curr_window == "urdf"){
		confirm_location_update();	
	}else if(curr_window == "text_task"){
		if(set_program()){
			$("#alert_1").attr("class", "alert alert-success");
			$("#alert_1").text("Valid Program Accepted");
			var task_in = $("#text_task").find("#task_text");
			task_in = task_in[0].value;
			if(task_in != ''){
				var prog_success = send_program(task_in, program_array);
				if(prog_success == false){
					$("#alert_1").attr("class", "alert alert-danger");
					$("#alert_1").text("Error.");
				}else{
					$("#alert_1").attr("class", "alert alert-success");
					$("#alert_1").text("Successfully Programmed");
				}
			}else{
				$("#alert_1").attr("class", "alert alert-danger");
				$("#alert_1").text("Invalid Program Name. Please enter.");
				$("#task_text").focus();
			}
		}else{
			$("#alert_1").attr("class", "alert alert-danger");
			$("#alert_1").text("Invalid Program. Please Check.");		
		}
	}
});

// Visualizer Tab
$("#visualize").click(function(){
	clear_set_tabs('#visualize');
	$("#main_content").html("<div id=\"urdf\"></div>")
	$("#start_speak").attr("disabled", "disabled");
	do_ros();
});

// Handle mouse and touch events for InteractiveMarker
$(document).on('mousedown mouseup touchstart touchend', '#urdf', function(evt){
	fetch_location_update_status();
});

// Intervention Tab
$("#intervene").click(function(){
	clear_set_tabs('#intervene');
	var intervene_img = '<img id="user_img" src="img/placeholder.jpg" />';
	$("#main_content").html(intervene_img)
	$("#start_speak").attr("disabled", "disabled");
});

// Commands Tab
$("#command").click(function(){
	clear_set_tabs('#command');
	$(".btn-group").find("#start_speak").removeAttr("disabled");
	var command_content = '<div id=\"speech_results\">Awaiting command...</div>';
	var command_text = '<div id=\"text_cmd\"><input type=\"text\" id=\"cmd_text\" placeholder=\"You may type in your command.\"></div>';
	$("#main_content").html(command_content);
	$(command_text).insertAfter("#speech_results");
});

// Logs Tab
$("#logs").click(function(){
	clear_set_tabs('#logs');
	$("#start_speak").attr("disabled", "disabled");
	var logs_content = '<div id=\"logs_display\"></div>';
	$("#main_content").html(logs_content);
	display_logs();
});

// Training Tab
$("#training").click(function(){
	clear_set_tabs('#training');
	$("#start_speak").attr("disabled", "disabled");
	var task_input = '<div id=\"text_task\"><input type=\"text\" id=\"task_text\" placeholder=\"Please Enter Task Being Programmed.\"/></div>';
	var training_content = '<div id=\"training_display\"></div>';
	$("#main_content").html(task_input);
	$("#main_content").append(training_content);
	var left_col = '<div id=\"left_col\"></div>';
	var right_col = '<div id=\"right_col\"></div>';
	$("#training_display").append(left_col);
	$("#training_display").append(right_col);
	init_program();
	populate_prog();
	$("#instructions").text("Please click step to remove from program");
});

// Clearing added step
$(document).on('click', '.step', function(){
	if($(this).parent().attr('class') == 'drop_area'){
		if($(this).attr('id') == 'start_prog'){
			add_prog("LEFT",$(this).html(),$(this).attr('id'),"success");	
		}else if($(this).attr('id') == 'end_prog'){
			add_prog("LEFT",$(this).html(),$(this).attr('id'),"danger");	
		}else{
			add_prog("LEFT",$(this).html(),$(this).attr('id'),"warning");
		}
		if($(this).attr('id')=='end_prog'){
			$(this).remove();
			$(".drop_area").last().remove();
			add_prog("RIGHT","","","","");
		}else{
			$(this).remove();
			$(".drop_area").last().remove();
			$(".drop_area").css("border-width","1px");
		}
	}
});

// Create new step
function add_prog(side, step_name, step_id, step_type){
	var allow_add = false;
	if(side == "LEFT"){
		var all_elems = $("#left_col").children();
		for(var x = 0; x < all_elems.length; x++){
			if($(all_elems[x]).attr('id') == step_id){
				allow_add = false;
			}else{
				allow_add = true;
			}
		}
		if(allow_add == true){
			var prog_btn = '<button type="button" id="'+step_id+'" draggable="true" ondragstart="drag(event)" class="btn btn-'+ step_type +' step">'+ step_name +'</button>';
			$("#left_col").append(prog_btn);
		}
	}else if(side == "RIGHT"){
		var drop_site = '<div class="drop_area" ondrop="drop(event)" ondragover="allowDrop(event)"></div>';
		$("#right_col").append(drop_site);	
	}
}

// Initialize program interface
function init_program(){
	var start_btn = '<button type="button" id="start_prog" draggable="true" ondragstart="drag(event)" class="btn btn-success step">Start Program</button>';
	var end_btn = '<button type="button" id="end_prog" draggable="true" ondragstart="drag(event)" class="btn btn-danger step">End Program</button>';
	var drop_site = '<div class="drop_area" ondrop="drop(event)" ondragover="allowDrop(event)"></div>';
	$("#left_col").append(start_btn);
	$("#right_col").append(drop_site);
	$("#left_col").append(end_btn);
}

// Add steps to program
function populate_prog(){
	add_prog("LEFT", "STOP/RESET", "stop", "warning");
	add_prog("LEFT", "ENTER TIME", "numpad","warning");
	add_prog("LEFT", "START", "start","warning");
	add_prog("LEFT", "WAIT", "wait","warning");
	add_prog("LEFT", "DEFROST", "defrost","warning");
	add_prog("LEFT", "EXPRESS", "quick","warning");
	add_prog("LEFT", "CENTER", "center","warning");
	add_prog("LEFT", "RECYCLE", "recycle","warning");
}

// Allow dropping
function allowDrop(evt){
	evt.preventDefault();
}

// Set ID on drag
function drag(ev) {
	ev.dataTransfer.setData("text", ev.target.id);
}

$(document).on('dragover dragenter','.drop_area',function(){
	$(this).css("border","1px solid #07c");
	$(this).css("box-shadow","0 0 10px #07c");
});
$(document).on('drop dragleave dragend','.drop_area',function(evt){
	$(this).css("border","none");
	$(this).css("box-shadow","none");
});

// Do drop functions
function drop(ev) {
	ev.preventDefault();
	var data = ev.dataTransfer.getData("text");
	ev.target.appendChild(document.getElementById(data));
	$(".drop_area").css("border-width","0px");
	if(data != "end_prog"){
		add_prog("RIGHT","");
	}
}

// Add to program array
function set_program(){
	var valid_prog = false;
	program_array = [];
	var step_list = $("#right_col").children();
	for(var x = 0; x < step_list.length; x++){
		program_array.push($(step_list[x]).children().attr('id'));
	}
	console.log(program_array);
	if($.inArray("end_prog",program_array) > 0){
		valid_prog = true;
	}else{
		valid_prog = false;
	}
	return valid_prog;
}

// $(document).on('touchstart touchend touchmove', '.step', function(evt){
// 	if(evt.type == 'touchmove'){
// 		var drop_x = evt.originalEvent.changedTouches[0].pageX;
// 		var drop_y = evt.originalEvent.changedTouches[0].pageY;
// 		$('#'+evt.currentTarget.id).offset({
// 			left:  drop_x,
// 			top:   drop_y
// 		});
// 	}
// 	if(evt.type == 'touchend'){
// 		var drop_x = evt.originalEvent.changedTouches[0].pageX;
// 		var drop_y = evt.originalEvent.changedTouches[0].pageY;
// 		var drop_place = document.elementFromPoint(drop_x, drop_y);
// 		if($(drop_place).attr('class') == 'drop_area'){
// 			if($(drop_place).children().length == 0){
// 				c_x = $(drop_place).offsetLeft;
// 				c_y = $(drop_place).offsetTop;
// 				$(drop_place).append($('#'+evt.currentTarget.id));
// 			}
// 		}
// 	}
// });

// Push to logs buffer and check for repeats
function append_logs(get_log_type){
	var log_type = parseInt(get_log_type);
	if(log_type != prev_state){
		speak_state_type(log_type);
		push_logs_buffer(get_log_type);
		prev_state = log_type;
	}
}

// Convert logs ID to text
function log_parser(log_type){
	var parsed_log = '';
	switch(log_type){
		case 0: parsed_log = "The robot is idle.";
		alert_user(false);
		break;
		case 1: parsed_log = "The robot has started moving task.";
		alert_user(false);
		break;
		case 2: parsed_log = "The robot is picking object.";
		alert_user(false);
		break;
		case 3: parsed_log = "The robot is placing object.";
		alert_user(false);
		break;
		case 44: parsed_log = "The robot has successfully finished task.";
		alert_user(false);
		break;
		case 11: parsed_log = "The robot is now closing the microwave.";
		alert_user(false);
		break;
		case 5: parsed_log = "The robot is now resetting.";
		alert_user(false);
		break;
		case 6: parsed_log = "The robot is now putting object into the microwave.";
		alert_user(false);
		break;
		case 7: parsed_log = "The microwave is too far.";
		alert_user(false);
		break;
		case 10: 
		parsed_log = "The robot is opening the microwave.";
		alert_user(false);
		break;
		case 21: parsed_log = "The robot is detecting the microwave.";
		alert_user(false);
		break;
		case 34: parsed_log = "The robot is looking for specified object.";
		alert_user(false);
		break;
		case 40: parsed_log = "The robot is detecting all the objects.";
		alert_user(false);
		break;
		case -10: 
		parsed_log = "Specified object not detected.";
		alert_user(false);
		break;
		case -2: 
		parsed_log = "The robot failed to open the microwave.";
		alert_user(true);
		break;
		case -21: 
		parsed_log = "The robot failed to pick up object";
		alert_user(true);
		break;
		case -22: 
		parsed_log = "The robot failed to place the object";
		alert_user(true);
		break;
		default: parsed_log = "Unknown Error. Contact Human.";
		alert_user(false);
		break;
	}
	return parsed_log;
}

// Add to logs buffer in circular fashion
function push_logs_buffer(to_push){
	logs_buffer.splice(logs_counter%logs_size, 0, to_push);
	logs_counter++;
}

// Clear and show updated logs
function display_logs(){
	$("#logs_display").html("");
	for(var i = logs_size; i >= 0; i--){
		var log_type = logs_buffer[i];
		if(log_type == 0){
			append_alert = '<div id="log_'+log_type+'" class="alert alert-danger" role="alert">'+log_parser(log_type)+'</div>'
		}		else{
			append_alert = '<div id="log_'+log_type+'" class="alert alert-info" role="alert">'+log_parser(log_type)+'</div>'
		}
		var logs_exists = $("#main_content").children().attr("id");
		if(logs_exists == "logs_display"){
			$("#logs_display").append(append_alert);
		}
	}
}

// Alert User
function alert_user(stat){
	if(SIM_FAIL){
		if(stat == true){
			alert_sound();
			$("#alert_1").attr("class", "alert alert-danger");
			$("#alert_1").text("Your ROBOT HAS AN ERROR.");
		}else{
			$("#alert_1").attr("class", "alert alert-success");
			$("#alert_1").text("Your ROBOT IS WORKING AGAIN!");
		}
	}
}

// Play Gong Alert
function alert_sound() {
	document.getElementById("audio").play();
}

// Fullscreen
$(document).on('click', '#fullscreener', function(){
	var haystack = $(this).html();
	var needle = 'glyphicon-resize-full';
	if(haystack.indexOf(needle) != -1){
		fullscreener();
		$(".visible-lg").find("#fullscreener").html('<em class="glyphicon glyphicon-resize-small"></em> Windowed');
		$(".hidden-lg").find("#fullscreener").html('<em class="glyphicon glyphicon-resize-small"></em>');		
	}else{
		exitFullscreen();
		$(".visible-lg").find("#fullscreener").html('<em class="glyphicon glyphicon-resize-full"></em> Fullscreen');
		$(".hidden-lg").find("#fullscreener").html('<em class="glyphicon glyphicon-resize-full"></em>');
	}
});

// Kill Fullscreen
function exitFullscreen() {
	if(document.exitFullscreen) {
		document.exitFullscreen();
	} else if(document.mozCancelFullScreen) {
		document.mozCancelFullScreen();
	} else if(document.webkitExitFullscreen) {
		document.webkitExitFullscreen();
	}
}

// Fullscreen experience
function fullscreener(){
	if (document.fullscreenEnabled || document.webkitFullscreenEnabled || document.mozFullScreenEnabled || document.msFullscreenEnabled){
		var i = document.documentElement;
		// go full-screen
		if (i.requestFullscreen) {
			i.requestFullscreen();
		} else if (i.webkitRequestFullscreen) {
			i.webkitRequestFullscreen();
		} else if (i.mozRequestFullScreen) {
			i.mozRequestFullScreen();
		} else if (i.msRequestFullscreen) {
			i.msRequestFullscreen();
		}
	}
}