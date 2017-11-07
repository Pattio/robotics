#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <AStar.hpp>
#include <marker.hpp>
#include <thread>

Marker *marker;
AStar::Map *myMap;
AStar::AStarAlgorithm *algorithm;

std::vector<std::vector<double>> goals;
std::vector<AStar::Position> paths[5];

std::vector<AStar::Position> extractWaypoints(std::vector<AStar::Position> path);

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
	std::cout << "path planning started" << std::endl;
	paths[pathIndex] = algorithm->findPath(start, end);
	std::vector<AStar::Position> waypoints = extractWaypoints(paths[pathIndex]);
	drawWaypointsForPath(waypoints);
	drawMarkersForPath(paths[pathIndex]);
}

void readOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

    // Get map origin
    AStar::PositionMap mapOrigin(msg->info.origin.position.x, msg->info.origin.position.y);
    // Cast occupancy grid to int vector
    std::vector<int> grid(msg->data.begin(), msg->data.end());
    // Create map object with all the information
	myMap = new AStar::Map(msg->info.height, msg->info.width, grid, 0, mapOrigin, msg->info.resolution);
	// Inflate all obsticles by half of robot size
	myMap->inflateObsticles(5);
	
	// Create 5 threads for each of the goals
	std::thread threads[5];
	algorithm = new AStar::AStarAlgorithm(*myMap);

	// Make optimal path
	std::iter_swap(goals.begin() + 3, goals.begin() + 4);
	std::iter_swap(goals.begin() + 4, goals.begin() + 5);

	// Find paths between each of the point pair
	AStar::Position startPosition = myMap->transformMapPositionToGridPosition(AStar::PositionMap(goals.at(0).at(0), goals.at(0).at(1)));
	for(int i = 1; i < goals.size(); i++) {
		AStar::Position endPosition = myMap->transformMapPositionToGridPosition(AStar::PositionMap(goals.at(i).at(0), goals.at(i).at(1)));
		threads[i - 1] = std::thread(findPath, startPosition, endPosition, i);
		startPosition = endPosition;
	}

	
	// Join all the threads
	for(int i = 0; i < 5; i++) {
		threads[i].join();
	}

	ROS_INFO("DONE");
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

int main(int argc, char **argv) {

	// Initialize ROS and become ROS node
	ros::init(argc, argv, "path_planner");
	ros::NodeHandle nh;
	// Create marker object
	marker = new Marker(nh, "path", "map");
	// Read parameters
	readParameters(nh);

	// Use wait for message (practical)
	// rospy.wait_for_message(’/map’,OccupancyGrid,timeout=None)
	ros::Subscriber sub = nh.subscribe("map", 1000, &readOccupancyGrid);
	ros::spin();
	
	// Clean memory
	delete myMap;
	delete marker;
	delete algorithm;
}


std::vector<AStar::Position> extractWaypoints(std::vector<AStar::Position> path) {
    std::vector<AStar::Position> waypoints;
    
    // TODO: CHANGE FROM HARDCODED VALUE TO PARAMETER
    // Currently set to 4 as it is half of the robot size
    int minWaypointDistance = 4;
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