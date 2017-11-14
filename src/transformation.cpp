#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <marker.hpp>

geometry_msgs::Pose realPose, odomPose;
geometry_msgs::Point mapOrigin;
visualization_msgs::MarkerArray realPoseMarkers, odomPoseMarkers;
std::vector<double> robot_start_pose;

void basePoseGroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char **argv) {
	// Initialize ROS and become ROS node
	ros::init(argc, argv, "transformation");
	ros::NodeHandle nh;

	// Get robot start pose
	nh.getParam("robot_start", robot_start_pose);

	// Create marker object
	Marker marker(nh, "real_and_odom_poses", "map");

	// Get map info
	nav_msgs::OccupancyGrid oG = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(10)));
	mapOrigin = oG.info.origin.position;

	// Subscribe to base_pose_ground_truth and odom
	ros::Subscriber base_pose_subscriber = nh.subscribe("base_pose_ground_truth", 1000, &basePoseGroundTruthCallback);
	ros::Subscriber odom_pose_subscriber = nh.subscribe("odom", 1000, &odometryCallback);

	// Draw movement changes every 1hz
	ros::Rate rate(1);
	while(ros::ok()) {
		marker.clear();
		marker.addArrow(realPose, "real_robot_pose");
		marker.addArrow(odomPose, "odom_robot_pose");
		marker.draw();
		ros::spinOnce();
		rate.sleep();
	}
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	odomPose = msg->pose.pose;
	// Just a sanity check that we got 3 coordinates
	if(robot_start_pose.size() == 3) {
		odomPose.position.x += robot_start_pose.at(0);
		odomPose.position.y += robot_start_pose.at(1);
	}
}

void basePoseGroundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	static tf::TransformBroadcaster broadcaster;
	realPose = msg->pose.pose;

	tf::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->pose.pose.position.x + fabs(mapOrigin.x), msg->pose.pose.position.y + fabs(mapOrigin.y), msg->pose.pose.position.z + mapOrigin.z));
	transform.setRotation(quaternion);
	broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/real_robot_pose"));
}