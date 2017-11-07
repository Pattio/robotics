#include "marker.hpp"

Marker::Marker(ros::NodeHandle nh, std::string topic, std::string frame) :
	nh(nh), topic(topic), frame(frame) 
{
	publisher = nh.advertise<visualization_msgs::MarkerArray>(topic, 1000);
};


void Marker::addMarker(geometry_msgs::Pose pose) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time();
	marker.ns = topic;
	marker.id = markers.size();
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose;
	marker.scale.x = 0.012;
	marker.scale.y = 0.012;
	marker.scale.z = 0.012;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	markers.push_back(marker);
}

void Marker::addWaypoint(geometry_msgs::Pose pose) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time();
	marker.ns = "WAYPOINT";
	marker.id = markers.size();
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose;
	marker.scale.x = 0.012;
	marker.scale.y = 0.012;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	markers.push_back(marker);
}

void Marker::addArrow(geometry_msgs::Pose pose, std::string arrow_namespace) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame;
	marker.header.stamp = ros::Time();
	marker.ns = arrow_namespace;
	marker.id = markers.size();
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = pose;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	markers.push_back(marker);
}

void Marker::draw() {
	visualization_msgs::MarkerArray mA;
	for(auto &marker : markers) {
		mA.markers.push_back(marker);
	}
	publisher.publish(mA);
}

void Marker::clear() {
	visualization_msgs::MarkerArray mA;
	for(auto &marker : markers) {
		marker.action = visualization_msgs::Marker::DELETE;
		mA.markers.push_back(marker);
	}
	publisher.publish(mA);
	markers.clear();
}