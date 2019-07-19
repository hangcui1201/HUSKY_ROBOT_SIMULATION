#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");

  // Private node handle
  ros::NodeHandle nh_("~");

  // Create ROS node named husky_highlevel_controller
  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nh_);

  ros::spin();

  return 0;

}
