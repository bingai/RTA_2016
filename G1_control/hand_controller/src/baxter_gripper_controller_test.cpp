#include <g1_control/hand_controller.h>
// #include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "BaxterGripperControllerTest");
  ros::NodeHandle node;

  // Allow the action server to receive and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  g1::control::HandController *gripperController = new g1::control::BaxterGripperController(node, "left");

  ROS_INFO_STREAM("HandController type:" << gripperController->getType() << " HandController side: " << gripperController->getSide() );

  while(ros::ok()){
    std::cout << "Calibration [y/N]? ";
    char input;
    std::cin >> input;
    if (input == 'y'){
      gripperController->calibrate();
    }

    ROS_INFO("open...");
    gripperController->open();

    std::cout << "Close [y/N]? ";
    std::cin >> input;
    if (input == 'y'){
      gripperController->close(0);
    }

    std::cout << "Open [y/N]? ";
    std::cin >> input;
    if (input == 'y'){
      gripperController->open();
    }
  }
  return 0;
}