#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>


ros::Publisher drive_publisher;
std::vector<geometry_msgs::Point> waypoints;
tf::TransformListener *listener;
int indexas = 0;

void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
geometry_msgs::Point transformPoint(geometry_msgs::Point point, std::string baseFrame, std::string targetFrame);

void drive(geometry_msgs::Point currentPoint, geometry_msgs::Point targetPoint);
void driveStraight(double speed, double distance);
double round(double x);
// void driveTo()

double angle(double currentX, double currentY, double targetX, double targetY);

int main(int argc, char **argv) {
	// Initialize ROS and become ROS node
	ros::init(argc, argv, "driver");
	ros::NodeHandle nh;

	ros::Subscriber waypoints_subscriber = nh.subscribe("/drive_waypoints", 100, waypointsCallback);
	drive_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

	listener = new tf::TransformListener;

	geometry_msgs::Point currentPoint, targetPoint;

	while(ros::ok()) {
		ros::spinOnce();
		if(waypoints.size() == 0 || indexas == (waypoints.size())) continue;

		targetPoint.x = waypoints.at(indexas).x;
		targetPoint.y = waypoints.at(indexas).y;
		geometry_msgs::Point obsticleDelta = transformPoint(targetPoint, "map", "real_robot_pose");
		double distance = sqrt(obsticleDelta.x * obsticleDelta.x + obsticleDelta.y * obsticleDelta.y);


		// If you are spawned at current path point skip it

		if(distance <= 0.1) indexas += 1;

		try {
			tf::StampedTransform transform;
			listener->lookupTransform("map", "real_robot_pose", ros::Time(0), transform);


			double yaw = tf::getYaw(transform.getRotation());
			currentPoint.x = transform.getOrigin().x();
			currentPoint.y = transform.getOrigin().y();
			double targetAngle = angle(currentPoint.x, currentPoint.y, targetPoint.x, targetPoint.y);
			
			std::cout << "robot angle" << yaw << " taget is : " << targetAngle << std::endl;

			geometry_msgs::Twist twist;
			double rotationDelta = targetAngle - yaw;

			if(fabs(rotationDelta) >= 0.1) {
				twist.angular.x = 0;
				// Add code to fix rotation to nearest side
				if(rotationDelta > M_PI) {
					rotationDelta -= M_PI * 2;
				} else if(rotationDelta <= -M_PI) {
					rotationDelta += M_PI * 2;
				}

				twist.angular.z = (rotationDelta < 0) ? -0.1 : 0.1;
			} else {
				if(distance >= 0.1) {
					twist.linear.x = 0.1;
				} else {
					// driving = false;
					std::cout << "reached goal" << indexas << std::endl;
					// indexas += 1;
				}
			}

			drive_publisher.publish(twist);

		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		// rate.sleep();
	}

	// Subscribe to driver topic which takes ros_msgs:pints array
	// After receiving points array check if robot is in the starting position
	// Of waypoints if yes calculate distance and angle to next point and try to drive 
	// to that point, while driving calculate what's distance from point you are currently driving to the next point

	// If points array start is not equal to the current position try to find closest point in that waypoints arrray drive to that point and continue from there
}

void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
	std::vector<geometry_msgs::Pose> poses = msg->poses;
	waypoints.clear();
	for(auto &pose : poses) {
		std::cout << pose.position.x << " " << pose.position.y << std::endl;
		waypoints.push_back(pose.position);
	}
}

void drive(geometry_msgs::Point currentPoint, geometry_msgs::Point targetPoint) {
	// double angle = atan2(targetPoint.y - currentPoint.y, targetPoint.x - currentPoint.x);
	double cPX = round(currentPoint.x);
	double cPY = round(currentPoint.y);
	double tPX = round(targetPoint.x);
	double tPY = round(targetPoint.y);
	double angle = atan2(tPY - cPY, tPX - cPX);
// 	dot = x1*x2 + y1*y2      # dot product between [x1, y1] and [x2, y2]
// det = x1*y2 - y1*x2      # determinant
// angle = atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
	std::cout << "aaanggle " << angle * 180 / M_PI << std::endl;
	//  if(transform.getOrigin().x() < goal.x) {
	// 	// 	geometry_msgs::Twist twist;
	// 	// 	twist.linear.x = 0.1;
	// 	// 	twist.angular.z = 0;
	// 	// 	drive_publisher.publish(twist);
	// }
}

double angle(double currentX, double currentY, double targetX, double targetY) {
	return atan2(round(targetY) - round(currentY), round(targetX) - round(currentX));
}

double round(double x) { return floor(x * 100 + 0.5) / 100; }

void driveStraight(double speed, double distance) {
	// distance = speed * time
	// time = distance / speed
	
	// double duration = distance / speed * 180 / PI;
	geometry_msgs::Twist twist;
	twist.linear.x = speed;
	twist.angular.z = 0;
	drive_publisher.publish(twist);
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
		// exit(-1);
	}
}