#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

enum DrivingMode { normal, bug, narrow };
bool obsticleLeft = false, obsticleRight = false;
bool closeObsticle = false;
ros::Publisher drive_publisher;
std::vector<geometry_msgs::Point> waypoints;
tf::TransformListener *listener;
int indexas = 0;
DrivingMode drivingMode = DrivingMode::normal;
geometry_msgs::Point robotPose, lastObsticle, drivingModeTransferPoint;
#define robot_width 0.1
#define robot_hypo 0.14
#define treshold 0.5


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
// Driving options
void normalDrive(double rotationDelta, double distance);
void bugDrive(geometry_msgs::Point targetPoint);
void narrowDrive(double robotYaw);

int main(int argc, char **argv) {
	// Initialize ROS and become ROS node
	ros::init(argc, argv, "driver");
	ros::NodeHandle nh;

	// Subscribe to custom topic which receives waypoints
	ros::Subscriber waypoints_subscriber = nh.subscribe("/drive_waypoints", 100, waypointsCallback);
	ros::Subscriber laserscan_subscriber = nh.subscribe("/base_scan", 100, laserscanCallback);
	drive_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	listener = new tf::TransformListener;
	tf::StampedTransform transform;
	geometry_msgs::Point targetPoint;

	// Variables outside while loop for optimization
	double distance, robotYaw, targetAngle, rotationDelta;

	while(ros::ok()) {
		ros::spinOnce();

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
				case DrivingMode::narrow:
					narrowDrive(robotYaw);
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

	// Find distance to point where last obsticle was encountered
	double xDelta = robotPose.x - drivingModeTransferPoint.x;
	double yDelta = robotPose.y - drivingModeTransferPoint.y;
	double distanceFromLastObsticle = sqrt(xDelta * xDelta + yDelta * yDelta);

	// If there are no obsticles do normal rotation
	// if there is an obsticle close rotate slowly and move forward a little
	// otherwise move forwarrd
	if(absoluteRotation >= 0.3 && !obsticleLeft && !obsticleRight && distanceFromLastObsticle >= treshold) {
		twist.angular.z = (rotationDelta < 0) ? -0.3 : 0.3;
	} else if (absoluteRotation >= 0.2 && !obsticleLeft && !obsticleRight && distanceFromLastObsticle >= treshold) {
		twist.angular.z = (rotationDelta < 0) ? -0.2 : 0.2;
	} else if (absoluteRotation >= 0.1 && !obsticleLeft && !obsticleRight && distanceFromLastObsticle >= treshold) {
		twist.angular.z = (rotationDelta < 0) ? -0.1 : 0.1;
	} else if (absoluteRotation >= 0.1) {
		twist.angular.z = (rotationDelta < 0) ? -0.05 : 0.05;
		twist.linear.x = 0.05;
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

	// If there is an obsticle rotate till you face obsticle with your side
	// then move forward while obstile is on your side
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

void narrowDrive(double robotYaw) {
	geometry_msgs::Twist twist;
	double target;

	// Find which way robot is heading and calibrate robot to move close between obsticles
	if((robotYaw >= 0 && robotYaw <= M_PI / 4) || (robotYaw < 0 && robotYaw >= -M_PI / 4)) {
		std::cout << "adjust to east" << std::endl;
		target = 0;
	} else if((robotYaw > M_PI / 4 && robotYaw <= M_PI / 2) || (robotYaw >= M_PI / 2 && robotYaw < M_PI * 3 / 4)) {
		std::cout << "adjust to north" << std::endl;
		target = M_PI / 2;
	} else if((robotYaw < -M_PI / 4 && robotYaw >= -M_PI / 2) || (robotYaw < -M_PI / 2 && robotYaw > -M_PI * 3 / 4)) {
		std::cout << "adjust to south" << std::endl;
		target = -M_PI / 2;
	} else {
		// adjust west
		std::cout << "adjust to west" << std::endl;
		target = -M_PI;
	}

	double rotationDelta = target - robotYaw;
	double absoluteRotation = fabs(rotationDelta);

	// Adjust rotation
	if(rotationDelta > M_PI) {
		rotationDelta -= M_PI * 2;
	} else if(rotationDelta <= -M_PI) {
		rotationDelta += M_PI * 2;
	}

	if(absoluteRotation >= 0.05) {
		twist.angular.z = (rotationDelta < 0) ? -0.05 : 0.05;
	} else {
		twist.linear.x = 0.1;
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

double round(double x) { return floor(x * 100 + 0.5) / 100; }
double angle(double currentX, double currentY, double targetX, double targetY) {
	return atan2(round(targetY) - round(currentY), round(targetX) - round(currentX));
}

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
void laserscanCallback(const sensor_msgs::LaserScan& msg) {
	float start_angle = msg.angle_min;
	float increment = msg.angle_increment;
	geometry_msgs::Point normalPoint, transformedPoint;
	obsticleLeft = false, obsticleRight = false;
	int safeScans = 0;

	for(auto &range : msg.ranges) {
		normalPoint.x = range * cos(start_angle);
		normalPoint.y = range * sin(start_angle);
		transformedPoint = transformPoint(normalPoint, "/base_laser_link", "/base_link");

		if(transformedPoint.y > 0 && transformedPoint.y < robot_width && transformedPoint.x <= robot_width / 2) {
			obsticleLeft = true;
		}

		if(transformedPoint.y < 0 && fabs(transformedPoint.y) < robot_width && transformedPoint.x <= robot_width / 2) {
			obsticleRight = true;
		}


		if(fabs(transformedPoint.y) < (robot_width / 2) && transformedPoint.x < robot_width * 2) {
			closeObsticle = true;
			drivingMode = DrivingMode::bug;
		} else {
			safeScans++;
		}
		start_angle += increment;
	}

	if(safeScans == 30) {
		if(closeObsticle == true) lastObsticle = robotPose;
		closeObsticle = false;
	}

	// If robot has obsticles on both sides change driving mode
	if(obsticleLeft && obsticleRight) {
		drivingMode = DrivingMode::narrow;
	} else if (drivingMode == DrivingMode::narrow){
		drivingModeTransferPoint = robotPose;
		drivingMode = DrivingMode::normal;
	}
}