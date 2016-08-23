#include <g1_control/hand_controller.h>
//#include <std_srvs/Empty.h>
#include <reflex_msgs/SetTactileThreshold.h>
#include <reflex_msgs/SetTactileThresholdRequest.h>
#include <reflex_msgs/Command.h>
#include <reflex_msgs/PoseCommand.h>
#include <reflex_msgs/VelocityCommand.h>
#include <reflex_msgs/ForceCommand.h>
#include <reflex_msgs/FingerPressure.h>
#include <boost/array.hpp>
#include <signal.h>


void signal_callback_handler(int signum){
  printf("Caught signal %d\n",signum);
  // Cleanup and close up stuff here
  ROS_INFO_STREAM("++++++");
  // Terminate program
  exit(signum);
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "ReflexHandControllerTest", ros::init_options::NoSigintHandler);
  ros::NodeHandle node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, signal_callback_handler);


  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()){

    g1::control::HandController *reflexController = new g1::control::ReflexHandController(node);

    ROS_INFO_STREAM("HandController type:" << reflexController->getType());

       std::cout << "Calibration [y/N]? ";
       char input;
       std::cin >> input;
       if (input == 'y'){
         reflexController->calibrate();
         ROS_INFO("Calibrations are done!");
       } 

       ROS_INFO("open... [y/N]?");
       std::cin >> input;
       if (input == 'y'){
         reflexController->open();
         // ros::Duration(1).sleep();
       }


       // grasp_type = 0: wrap grasp with Preshape = 0; Horizontal; example: mug #1 
       // grasp_type = 1: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: bowl #1 the third finger will close
       // grasp_type = 2: pinch grasp (three fingers) with Preshape = pi; Horizontal; example: small(short) mug #? the third finger will NOT close
       // grasp_type = 3: power grasp (three fingers) with Preshape = 2*pi/3; vertical; example: ball
       ROS_INFO("preGrasp... [y/N]?");
       std::cin >> input;
       if (input == 'y'){
         ROS_INFO("Please choose grasp type:");
         ROS_INFO("Grasp type 0: Wrap grasp with Preshape = 0; Horizontal");
         ROS_INFO("Grasp type 1: Pinch grasp (three fingers, the third finger will ALSO Close) with Preshape = pi; Horizontal");
         ROS_INFO("Grasp type 2: Pinch grasp (three fingers, the third finger will NOT Close) with Preshape = pi; Horizontal");
         ROS_INFO("Grasp type 3: Power grasp (three fingers) with Preshape = 2*pi/3; vertical");
         int type;
         std::cin >> type;
         reflexController->preGrasp(type);
       } 

       
       std::cout << "Close [y/N]? ";
       ROS_INFO("Please choose close type:");
       ROS_INFO("Grasp_type == Close_type");
       ROS_INFO("if Grasp type == 2 ? The third finger will NOT close : Close the third finger");
       int type;
       std::cin >> type;

       if (type == 0 || type == 1 || type == 2 || type == 3){
        reflexController->close(type);
        // reflexController->close_pressure(type);
       }
       else{
        ROS_ERROR("close type is wrong!");
       }

       // ROS_INFO("close_pressure...");
       // reflexController.close_pressure();
       
       std::cout << "PreOpen [y/N]? ";
       std::cin >> input;
       if (input == 'y'){
         reflexController->preOpen(type);
         // ros::Duration(1).sleep();
       } 

      
      std::cout << "Open [y/N]? ";
       std::cin >> input;
       if (input == 'y'){
         reflexController->open();
         // ros::Duration(1).sleep();
       } 

       std::cout << "Reset [y/N]? ";
       std::cin >> input;
       if (input == 'y'){
         reflexController->reset();
       } 
      }

  
  return 0;
};
