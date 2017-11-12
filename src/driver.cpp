#include <ros/ros.h>
#include <AStar.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

enum DrivingMode { normal, bug, narrow };
bool obsticleLeft = false, obsticleRight = false;
bool closeObsticle = false;
ros::Publisher drive_publisher;
std::vector<geometry_msgs::Point> waypoints;
tf::TransformListener *listener;
AStar::Map *map = nullptr;
int indexas = 0;
DrivingMode drivingMode = DrivingMode::normal;
geometry_msgs::Point robotPose, lastObsticle;
#define robot_width 0.1

bool closeObsticleOnLeft = false;

// Convienient function for rounding values
double round(double x);
// Function to calculate angle between target and robot
double angle(double currentX, double currentY, double targetX, double targetY);
// Callback which receives waypoints which robot needs to visit
void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
// Transform function which transforms point from one frame to another
geometry_msgs::Point transformPoint(geometry_msgs::Point point, std::string baseFrame, std::string targetFrame);
// Laser scan callback
void laserscanCallback(const sensor_msgs::LaserScan& msg);
// Occupancy grid callback
void fillMap();
// Driving options
void normalDrive(double rotationDelta, double distance);
void bugDrive(geometry_msgs::Point targetPoint);

int main(int argc, char **argv) {
	// Initialize ROS and become ROS node
	ros::init(argc, argv, "driver");
	ros::NodeHandle nh;

	// Subscribe to custom topic which receives waypoints
	ros::Subscriber waypoints_subscriber = nh.subscribe("/drive_waypoints", 100, waypointsCallback);
	ros::Subscriber laserscan_subscriber = nh.subscribe("/base_scan", 100, laserscanCallback);
	drive_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	fillMap();
	listener = new tf::TransformListener;
	tf::StampedTransform transform;
	geometry_msgs::Point targetPoint;

	// Variables outside while loop for optimization
	double distance, robotYaw, targetAngle, rotationDelta;

	while(ros::ok()) {
		ros::spinOnce();

		// // Debug only
		// try {
		// 	listener->lookupTransform("map", "real_robot_pose", ros::Time(0), transform);
		// 	robotPose.x = transform.getOrigin().x();
		// 	robotPose.y = transform.getOrigin().y();
		// } catch (tf::TransformException ex) {
		// 	ROS_ERROR("%s",ex.what());
		// 	ros::Duration(1.0).sleep();
		// }

		// If no waypoints received, or last point reached continue, waiting for
		// new waypoints
		if(waypoints.size() == 0 || indexas == (waypoints.size())) continue;

		// Find current waypoint robot is heading towards
		targetPoint.x = waypoints.at(indexas).x;
		targetPoint.y = waypoints.at(indexas).y;
		
		// Using transfrom find where target is regarding robot frame
		geometry_msgs::Point targetDelta = transformPoint(targetPoint, "map", "real_robot_pose");
		distance = sqrt(targetDelta.x * targetDelta.x + targetDelta.y * targetDelta.y);

		// If you are at current target waypoint add next waypoint
		if(distance <= 0.1) indexas += 1;


		try {
			// Using transform find robot location and yaw regarding map frame
			listener->lookupTransform("map", "real_robot_pose", ros::Time(0), transform);
			robotPose.x = transform.getOrigin().x();
			robotPose.y = transform.getOrigin().y();
			robotYaw = tf::getYaw(transform.getRotation());
			targetAngle = angle(robotPose.x, robotPose.y, targetPoint.x, targetPoint.y);	
			rotationDelta = targetAngle - robotYaw;


			switch(drivingMode) {
				case DrivingMode::normal:
					normalDrive(rotationDelta, distance);
				break;
				case DrivingMode::bug:
					bugDrive(targetPoint);
				break;
			}


		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	}
}

void normalDrive(double rotationDelta, double distance) {
	geometry_msgs::Twist twist;
	double absoluteRotation = fabs(rotationDelta);

	// Adjust rotation
	if(rotationDelta > M_PI) {
		rotationDelta -= M_PI * 2;
	} else if(rotationDelta <= -M_PI) {
		rotationDelta += M_PI * 2;
	}
	if(absoluteRotation >= 0.3) {
		twist.angular.z = (rotationDelta < 0) ? -0.3 : 0.3;
	} else if (absoluteRotation >= 0.2) {
		twist.angular.z = (rotationDelta < 0) ? -0.2 : 0.2;
	} else if (absoluteRotation >= 0.1) {
		twist.angular.z = (rotationDelta < 0) ? -0.1 : 0.1;
	} else {
		if(distance >= 0.3) {
			twist.linear.x = 0.3;
		} else if(distance >= 0.2) {
			twist.linear.x = 0.2;
		} else {
			twist.linear.x = 0.1;
		}
	}
	drive_publisher.publish(twist);
}

void bugDrive(geometry_msgs::Point targetPoint) {
	geometry_msgs::Twist twist;

	// double deltaX = targetPoint.x - robotPose.x;
	std::cout << "close obsticle value is" << closeObsticle << std::endl;
	if(closeObsticle) {
		twist.angular.z = -0.1;
	} else if (robotPose.x - robot_width * 3 < lastObsticle.x && robotPose.y - robot_width < lastObsticle.y) {
		// There is a bug if lastObsticle is on the right of the map
		// but robot is going to left of the map robot will continue going straigh
		// without stoping, possible fix is to check with side of the map robot is facing and if robot is facing left map -pi then check 
		// if robotPose.x <= lasObsticle.x - robot_WIDTH
		std::cout << "OBSTICLE WAS AT x" << lastObsticle.x << "and y: " << lastObsticle.y << std::endl;
		std::cout << "ROBOT POSE IS AT x" << robotPose.x << " and y:" << robotPose.y << std::endl;
		std::cout << "bug wants to go straight" << std::endl;
		twist.linear.x = 0.1;
	} else {
		drivingMode = DrivingMode::normal;
	}

	drive_publisher.publish(twist);
}

void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
	std::vector<geometry_msgs::Pose> poses = msg->poses;
	waypoints.clear();
	for(auto &pose : poses) {
		std::cout << pose.position.x << " " << pose.position.y << std::endl;
		waypoints.push_back(pose.position);
	}
}

double angle(double currentX, double currentY, double targetX, double targetY) {
	return atan2(round(targetY) - round(currentY), round(targetX) - round(currentX));
}

double round(double x) { return floor(x * 100 + 0.5) / 100; }

geometry_msgs::Point transformPoint( geometry_msgs::Point point, 
	std::string baseFrame, std::string targetFrame) 
{
	geometry_msgs::PointStamped targetPoint;
	geometry_msgs::PointStamped basePoint;
	basePoint.header.stamp = ros::Time();
	basePoint.header.frame_id = baseFrame;
	basePoint.point = point;

	try {
		listener->transformPoint(targetFrame, basePoint, targetPoint);
		return targetPoint.point;
	} catch(tf::TransformException& ex) {
		ROS_ERROR("Received an exception while trying to transform point %s", ex.what());
	}
}

// TODO: Refactor to robot size
#define robot_height 0.1
#define robot_hypo 0.14
void laserscanCallback(const sensor_msgs::LaserScan& msg) {
	float start_angle = msg.angle_min;
	float increment = msg.angle_increment;
	geometry_msgs::Point normalPoint, transformedPoint;
	obsticleLeft = false;
	obsticleRight = false;
	int safeScans = 0;
	int test = 0;
	for(auto &range : msg.ranges) {
		normalPoint.x = range * cos(start_angle);
		normalPoint.y = range * sin(start_angle);
		transformedPoint = transformPoint(normalPoint, "/base_laser_link", "/base_link");
		// if(transformedPoint.y < 0 && fabs(transformedPoint.y) < (robot_width / 2) && transformedPoint.x < robot_hypo) {
		// 	std::cout << "OBSTICLE ON THE RIGHT" << std::endl;
		// 	obsticleRight = true;
		// }

		if(transformedPoint.y > 0 && transformedPoint.y < robot_width && transformedPoint.x <= robot_width / 2) {
			std::cout << "CLOSE OBSTICLE ON THE LEFT" << std::endl;
			closeObsticleOnLeft = true;
		}

		if(transformedPoint.y < 0 && fabs(transformedPoint.y) < robot_width && transformedPoint.x <= robot_width / 2) {
			std::cout << "CLOSE OBSTICLE ON THE RIGHT" << std::endl;
			closeObsticleOnLeft = true;
		}

		// if(transformedPoint.y > 0 && transformedPoint.y < robot_width && transformedPoint.x <= 0.05) {
		// 	std::cout << "OBSTICLE ON THE LEFT" << std::endl;
		// 	// ROS_INFO_STREAM("In robot frame obstacle is at x: " << transformedPoint.x << " y: " << transformedPoint.y);
		// 	obsticleLeft = true;
		// }

		// if((obsticleLeft && !obsticleRight) || (!obsticleLeft && obsticleRight)) drivingMode = DrivingMode::bug;
		// else 
		// 	drivingMode = DrivingMode::normal;

		// ROS_INFO_STREAM("REL " << test << " frame obstacle is at x: " << transformedPoint.x << " y: " << transformedPoint.y);
		if(fabs(transformedPoint.y) < (robot_width / 2) && transformedPoint.x < robot_width * 2) {
			closeObsticle = true;
			drivingMode = DrivingMode::bug;
			ROS_INFO_STREAM("In robot frame obstacle is at x: " << transformedPoint.x << " y: " << transformedPoint.y);
		} else {
			safeScans++;
		}
		test++;
		start_angle += increment;
	}
	std::cout << safeScans << std::endl;
	if(safeScans == 30) {
		std::cout << "meeee " << std::endl;
		// record current position
		if(closeObsticle == true) lastObsticle = robotPose;
		closeObsticle = false;
	}
}


void fillMap() {
	nav_msgs::OccupancyGrid oG = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(10)));
	AStar::PositionMap mapOrigin(oG.info.origin.position.x, oG.info.origin.position.y);
	// Cast occupancy grid to int vector
    std::vector<int> grid(oG.data.begin(), oG.data.end());
	map = new AStar::Map(oG.info.height, oG.info.width, grid, 0, mapOrigin, oG.info.resolution);
}
// void readOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
// 	 // Get map origin
//     AStar::PositionMap mapOrigin(msg->info.origin.position.x, msg->info.origin.position.y);
// 	// Cast occupancy grid to int vector
//     std::vector<int> grid(msg->data.begin(), msg->data.end());
// 	map = new AStar::Map(msg->info.height, msg->info.width, grid, 0, mapOrigin, msg->info.resolution);
// }