#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <ros/ros.h>

namespace husky_highlevel_controller {

	HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
	  nodeHandle_(nodeHandle) {

		// Check if the parameters can be loaded or not

	  if(!nodeHandle.getParam("/husky_highlevel_controller/husky/topic_name", topic_name)) {
	    ROS_ERROR("Could not load topic_name parameter!");
	    ros::requestShutdown();
	  }

	  if (!nodeHandle.getParam("/husky_highlevel_controller/husky/queue_size", queue_size)) {
	    ROS_ERROR("Could not load queue_size parameter!");
	    ros::requestShutdown();
	  }

	  if (!nodeHandle.getParam("/husky_highlevel_controller/husky/p_gain", p_gain)) {
	    ROS_ERROR("Could not load p_gain parameter!");
	    ros::requestShutdown();
	  }

    // Create a subscriber to subscribe topic_name
    subscriber_ = nodeHandle_.subscribe(topic_name, queue_size,
                                       &HuskyHighlevelController::topicCallback, this);

    publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    visual_publisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

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

	void HuskyHighlevelController::topicCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    // Get the range of the laser scanner
    std::vector<float> laser_scan_range;
    laser_scan_range = msg->ranges;

    float min_distance = 10000;
    float min_index = 0;

    // Get minimum distance
    for(int i = 0; i < laser_scan_range.size(); i++){
			if(min_distance > laser_scan_range[i]) {
					min_distance = laser_scan_range[i];
					min_index = i;
			}
		}

		//ROS_INFO_STREAM("Minimum distance: " + std::to_string(min_distance));

		// Calculate x, y
	  float angle_increment = msg->angle_increment;
	  float angle_min = msg->angle_min;
	  float radian_min = angle_min + angle_increment * min_index;

    float x_min_coord, y_min_coord, z_min_coord;
	  x_min_coord = min_distance * cos(radian_min);
	  y_min_coord = min_distance * sin(radian_min);
	  z_min_coord = 1.0;

	  ROS_INFO("x: %f, y: %f\n", x_min_coord, y_min_coord);


    // Calculate control input
	  float z_angle_dot, x_dot;

	  z_angle_dot = HuskyHighlevelController::p_gain * (0 - radian_min);
	  x_dot = p_gain*0.5;

	  // Publish control input
	  geometry_msgs::Twist velocity_cmd;
	  velocity_cmd.linear.x = x_dot;
	  velocity_cmd.angular.z = z_angle_dot;

	  publisher_.publish(velocity_cmd);


    // Publish visualization marker
	  visualization_msgs::Marker marker;
	  marker.header.frame_id = "base_laser";
	  marker.header.stamp = ros::Time();
	  marker.ns = "my_namespace";
	  marker.id = 0;
	  marker.type = visualization_msgs::Marker::SPHERE;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.pose.position.x = x_min_coord;
	  marker.pose.position.y = y_min_coord;
	  marker.pose.position.z = z_min_coord;
	  marker.pose.orientation.x = 0.0;
	  marker.pose.orientation.y = 0.0;
	  marker.pose.orientation.z = 0.0;
	  marker.pose.orientation.w = 1.0;
	  marker.scale.x = 0.5;
	  marker.scale.y = 0.1;
	  marker.scale.z = 0.1;
	  marker.color.a = 1.0; // Don't forget to set the alpha!
	  marker.color.r = 0.0;
	  marker.color.g = 1.0;
	  marker.color.b = 0.0;

	  //Only if using a MESH_RESOURCE marker type:
	  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	  visual_publisher_.publish(marker);

  }

} /* namespace */
