#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


std::vector<geometry_msgs::Point> waypoints;

void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
	std::vector<geometry_msgs::Pose> poses = msg->poses;
	waypoints.clear();
	for(auto &pose : poses) {
		std::cout << pose.position.x << " " << pose.position.y << std::endl;
		waypoints.push_back(pose.position);
	}
}

// void driveTo()

int main(int argc, char **argv) {
	// Initialize ROS and become ROS node
	ros::init(argc, argv, "driver");
	ros::NodeHandle nh;

	ros::Subscriber waypoints_subscriber = nh.subscribe("/drive_waypoints", 100, waypointsCallback);


	// Subscribe to driver topic which takes ros_msgs:pints array
	// After receiving points array check if robot is in the starting position
	// Of waypoints if yes calculate distance and angle to next point and try to drive 
	// to that point, while driving calculate what's distance from point you are currently driving to the next point

	// If points array start is not equal to the current position try to find closest point in that waypoints arrray drive to that point and continue from there

	ros::spin();
}