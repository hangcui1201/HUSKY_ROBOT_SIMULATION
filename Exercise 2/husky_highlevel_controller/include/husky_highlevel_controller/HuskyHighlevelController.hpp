#pragma once

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

namespace husky_highlevel_controller {

	/*!
	 * Class containing the Husky Highlevel Controller
	 */
	class HuskyHighlevelController {
	public:
		/*!
		 * Constructor.
		 */
		HuskyHighlevelController(ros::NodeHandle& nodeHandle);

		/*!
		 * Destructor.
		 */
		virtual ~HuskyHighlevelController();

	private:

		// ROS node handler
		ros::NodeHandle nodeHandle_;

		// ROS topic subscriber
		ros::Subscriber subscriber_;

		std::string topic_name;
		std::int32_t queue_size;

		void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	};

} /* namespace */
