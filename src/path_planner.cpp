#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <AStar.hpp>
#include <marker.hpp>
#include <geometry.hpp>
#include <thread>


Marker *marker;
AStar::Map *myMap;
AStar::AStarAlgorithm *algorithm;
ros::Publisher move_publisher, map_publisher;

std::vector<std::vector<double>> goals;
std::vector<AStar::Position> paths[5], waypoints[5];

void drawMarkersForPath(const std::vector<AStar::Position> &path);
void drawWaypointsForPath(const std::vector<AStar::Position> &path);
void findPath(AStar::Position start, AStar::Position end, int pathIndex);
void makePathOptimal();
void readParameters(ros::NodeHandle nh);
void readOccupancyGrid(const nav_msgs::OccupancyGrid msg);
void publishWaypoints();
std::vector<AStar::Position> extractWaypoints(std::vector<AStar::Position> path);

int main(int argc, char **argv) {
	// Initialize ROS and become ROS node
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle nh;
	// Create marker object
	marker = new Marker(nh, "path", "map");
	// Create publisher which publishes waypoints to driver
	move_publisher = nh.advertise<geometry_msgs::PoseArray>("/drive_waypoints", 1);
	map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
	// Read parameters
	readParameters(nh);

	ros::Subscriber sub = nh.subscribe("map", 1000, &readOccupancyGrid);
	ros::spin();
	
	// Clean memory
	if(algorithm != nullptr) delete algorithm;
	if(myMap != nullptr) delete myMap;
	if(marker != nullptr) delete marker;
}

void drawMarkersForPath(const std::vector<AStar::Position> &path) {
	geometry_msgs::Pose pose;
	for(auto &position : path) {
		pose.position.x = myMap->origin.x + position.x * myMap->cellResolution;
		pose.position.y = myMap->origin.y + position.y * myMap->cellResolution;
		marker->addMarker(pose);
	}
	marker->draw();
}

void drawWaypointsForPath(const std::vector<AStar::Position> &path) {
	geometry_msgs::Pose pose;
	for(auto &position : path) {
		pose.position.x = myMap->origin.x + position.x * myMap->cellResolution;
		pose.position.y = myMap->origin.y + position.y * myMap->cellResolution;
		marker->addWaypoint(pose);
	}
	marker->draw();
}

void findPath(AStar::Position start, AStar::Position end, int pathIndex) {
	paths[pathIndex] = algorithm->findPath(start, end);
	waypoints[pathIndex] = extractWaypoints(paths[pathIndex]);
	drawWaypointsForPath(waypoints[pathIndex]);
	drawMarkersForPath(paths[pathIndex]);
}

// Using euclidean heuristic compare each point to sort points by 
// increasing order
void makePathOptimal() {
	std::vector<std::vector<double>> goalsCopy = goals;
	std::vector<std::vector<double>> heuristicGoals;
	heuristicGoals.push_back(goals.at(0));
	std::vector<double> currentPoint = goals.at(0);
	goalsCopy.erase (goalsCopy.begin());

	double minDistanceIndex = 0, heuristicDistance = 0;
	double minDistance = std::numeric_limits<double>::max();
	
	for(int j = 0; j < goals.size() - 1; j++) {
		for(int i = 0; i < goalsCopy.size(); i++) {
			heuristicDistance = sqrt(
				pow((goalsCopy.at(i).at(0) - currentPoint.at(0)), 2) + 
				pow((goalsCopy.at(i).at(1) - currentPoint.at(1)), 2)
			);
			if(heuristicDistance < minDistance) {
				minDistanceIndex = i;
				minDistance = heuristicDistance;
			}
		}
		currentPoint = goalsCopy.at(minDistanceIndex);
		heuristicGoals.push_back(currentPoint);
		goalsCopy.erase(goalsCopy.begin() + minDistanceIndex);
		minDistance = std::numeric_limits<double>::max();
	}
	goals = heuristicGoals;
}

void readParameters(ros::NodeHandle nh) {
	// Prepare vector for 6 goals
	goals.resize(6);
	// Get robot start position
	nh.getParam("robot_start", goals.at(0));
	// Get goal points
	for(int i = 1; i < 6; i++) {
		nh.getParam("goal" + std::to_string(i - 1), goals.at(i));
	}
}

void readOccupancyGrid(const nav_msgs::OccupancyGrid msg) {
    // Get map origin
    AStar::PositionMap mapOrigin(msg.info.origin.position.x, msg.info.origin.position.y);
    // Cast occupancy grid to int vector
    std::vector<int> grid(msg.data.begin(), msg.data.end());
    // Create map object with all the information
	myMap = new AStar::Map(msg.info.height, msg.info.width, grid, 0, mapOrigin, msg.info.resolution);
	// Inflate all obsticles
	myMap->inflateObsticles(robot_width, robot_height);
	
	// Publish inflated map
	nav_msgs::OccupancyGrid gridCopy = msg;
	gridCopy.data = myMap->getFlattenedMap();
	map_publisher.publish(gridCopy);

	// Make path optimal using euclidean heuristics
	makePathOptimal();
	
	// Create 5 threads for each of the goals
	std::thread threads[5];
	algorithm = new AStar::AStarAlgorithm(*myMap);

	// Find paths between each of the point pair
	AStar::Position startPosition = myMap->transformMapPositionToGridPosition(AStar::PositionMap(goals.at(0).at(0), goals.at(0).at(1)));
	for(int i = 1; i < goals.size(); i++) {
		AStar::Position endPosition = myMap->transformMapPositionToGridPosition(AStar::PositionMap(goals.at(i).at(0), goals.at(i).at(1)));
		threads[i - 1] = std::thread(findPath, startPosition, endPosition, i-1);
		startPosition = endPosition;
	}
	
	// Join all the threads
	for(int i = 0; i < 5; i++) {
		threads[i].join();
	}

	// Publish waypoints to driver after path is planned
	publishWaypoints();
	ROS_INFO("DONE PLANNING PATH, LET'S START ROLLING. GOOD LUCK.");
	exit(-1);
}

std::vector<AStar::Position> extractWaypoints(std::vector<AStar::Position> path) {
    std::vector<AStar::Position> waypoints;
 	// How accurate waypoints you need, currently set to the robot size
    int minWaypointDistance = robot_height / myMap->cellResolution;
    int currentWaypointDistance = 0;
    
    // Impossible values, so first point will be pushed
    int lastDeltaX = -999, lastDeltaY = -999;
    // Current loop delta values;
    int currentDeltaX, currentDeltaY;
    
    // Go through each point in path. If direction is changed and if
    // the distance between two point in different direction is big enough
    // add that point to waypoints vector
    for(int i = 1; i < path.size(); i++) {
        currentDeltaX = (path.at(i).x - path.at(i - 1).x);
        currentDeltaY = (path.at(i).y - path.at(i - 1).y);
        if(currentDeltaX == lastDeltaX && currentDeltaY == lastDeltaY) {
            // Increase the current waypoints distance if we are still moving
            // to same direction
            currentWaypointDistance++;
        } else if (currentWaypointDistance > minWaypointDistance || i == 1) {
            // Push first point as the waypoint, then only push points if
            // distance between changing delta's is bigger then min constant
            waypoints.push_back(path.at(i - 1));
            currentWaypointDistance = 0;
        }
        lastDeltaX = currentDeltaX;
        lastDeltaY = currentDeltaY;
    }
    // Push last point of the path
    if(!path.empty()) waypoints.push_back(path.back());
    return waypoints;
}

void publishWaypoints() {
	geometry_msgs::PoseArray poseArray;
	poseArray.header.stamp = ros::Time::now();
  	poseArray.header.frame_id = "/map";
	geometry_msgs::Pose pose;

	double lastX, lastY, currentX, currentY;
	for(int i = 0; i < 5; i++) {
		for(int j = 0; j < waypoints[i].size(); j++) {
			currentX = waypoints[i].at(j).x * myMap->cellResolution;
			currentY = waypoints[i].at(j).y * myMap->cellResolution;
			
			// Skip duplicate of endpoint -> startpoint
			if(lastX == currentX && lastY == currentY) continue;

			pose.position.x = currentX;
			pose.position.y = currentY;
			poseArray.poses.push_back(pose);
			lastX = currentX;
			lastY = currentY;
		}
	}
	move_publisher.publish(poseArray);
}