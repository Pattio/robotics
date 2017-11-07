#include <ros/ros.h>

int main(int argc, char **argv) {
	// Initialize ROS and become ROS node
	ros::init(argc, argv, "rviz_draw");
	ros::NodeHandle nh;


	ros::Subscriber sub = nh.subscribe("map", 1000, &readOccupancyGrid);
	ros::spin();
}