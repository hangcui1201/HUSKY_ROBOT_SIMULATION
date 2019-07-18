#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <ros/ros.h>

namespace husky_highlevel_controller {

	HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
	  nodeHandle_(nodeHandle) {

		// Check if the parameters can be loaded or not
    if (!(nodeHandle.getParam("/husky_highlevel_controller/husky/topic_name", topic_name) & 
    	  nodeHandle.getParam("/husky_highlevel_controller/husky/queue_size", queue_size)) ) {
	    ROS_ERROR("Could not load parameters.");
	    ros::requestShutdown();
    }

	    // Create a subscriber to subscribe topic_name
	    subscriber_ = nodeHandle_.subscribe(topic_name, queue_size,
	                                       &HuskyHighlevelController::laserScanCallback, this);

	    ROS_INFO("ROS node successfully launched.");

	}

	HuskyHighlevelController::~HuskyHighlevelController()
	{
	}

	// LaserScan.msg
	// Header header            # timestamp in the header is the acquisition time of 
  //                          # the first ray in the scan.
  //                          #
  //                          # in frame frame_id, angles are measured around 
  //                          # the positive Z axis (counterclockwise, if Z is up)
  //                          # with zero angle being forward along the x axis
                         
	// float32 angle_min        # start angle of the scan [rad]
	// float32 angle_max        # end angle of the scan [rad]
	// float32 angle_increment  # angular distance between measurements [rad]

	// float32 time_increment   # time between measurements [seconds] - if your scanner
	//                          # is moving, this will be used in interpolating position
	//                          # of 3d points
	// float32 scan_time        # time between scans [seconds]

	// float32 range_min        # minimum range value [m]
	// float32 range_max        # maximum range value [m]

	// float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
	// float32[] intensities    # intensity data [device-specific units].  If your
	//                          # device does not provide intensities, please leave
	//                          # the array empty.

	void HuskyHighlevelController::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    // Get the range of the laser scanner
    std::vector<float> laser_scan_range;
    laser_scan_range = msg->ranges;

    float min_distance = 10000;

    for(int i = 0; i < laser_scan_range.size(); i++){
			if(min_distance > laser_scan_range[i]) {
					min_distance = laser_scan_range[i];
			}
		}

			ROS_INFO_STREAM("Minimum distance: " + std::to_string(min_distance));

  }

} /* namespace */
