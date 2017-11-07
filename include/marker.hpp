#ifndef marker_hpp
#define marker_hpp

#include <ros/ros.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>


class Marker {
	public:

		Marker(ros::NodeHandle nh, std::string topic, std::string frame);
		void addMarker(geometry_msgs::Pose pose);
		void addArrow(geometry_msgs::Pose pose, std::string arrow_namespace);
		void draw();
		void clear();

	private:
		std::string topic, frame;
		std::vector<visualization_msgs::Marker> markers;
		ros::NodeHandle nh;
		ros::Publisher publisher;

};


#endif