var SIM_FAIL = false;

var ROBOT_IP = '128.8.140.73';
var ROBOT_PORT = '7890';

var image_topic = "/camera/rgb/image";
var IMAGE_IP = '128.8.140.73';
var IMAGE_PORT = '9974';

var MY_WIT_TOKEN = 'WMUH44ST5F4HSDURGQXOQG2ELCVIKVKA';

var command_srv_name = '/state_machine/cmd';
var command_srv_type = '/world_model_msgs/cmd';

var logs_srv_name = '/state_machine/current_state';
var logs_srv_type = '/world_model_msgs/exec_state';

var click_srv_name = '/interact/update_3d_position';
var click_srv_type = '/world_model_msgs/ImageCoordinate';

var loc_status_srv_name = '/location_update_status';
var loc_status_srv_type = '/interactive_marker_ui/location_update_status';

var loc_confirm_srv_name = '/location_update_confirm';
var loc_confirm_srv_type = '/interactive_marker_ui/location_update_confirm';

var prog_send_srv_name = '/state_machine/program_order';
var prog_send_srv_type = '/world_model_msgs/ProgramOrder';

var obj_srv_name = '/interact/object_choice';
var obj_srv_type = '/world_model_msgs/ImageCoordinate';

var logs_size = 7;