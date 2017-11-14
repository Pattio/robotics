//
//  AStar.cpp
//  AStar
//
//  Created by Edvinas on 03/11/2017.
//  Copyright © 2017 Edvinas. All rights reserved.
//

#include "AStar.hpp"
#include <algorithm>
#include <math.h>
#include <iostream>

using namespace AStar;

// Position struct implementation

PositionMap::PositionMap(double x, double y) :
x(x), y(y) {}

Position::Position(int x, int y) :
x(x), y(y) {}

bool Position::operator == (const Position &other) {
    return (x == other.x && y == other.y);
}

Position operator + (const Position &lhs, const Position &rhs) {
    return { lhs.x + rhs.x, lhs.y + rhs.y };
}

// Node struct implementation

Node::Node(Position position, Node *parent) :
position(position), parent(parent), costToReach(0), costToGoal(0) {}

const int Node::totalCost() const {
    return costToReach + costToGoal;
}

const int Node::estimatedDistance(const Position &destination) {
    // Return Euclidian Distance
    int xDelta = destination.x - position.x;
    int yDelta = destination.y - position.y;
    return sqrt(xDelta * xDelta + yDelta * yDelta);
}


// Map implementation

Map::Map(int height, int width, std::vector<int> data, int emptyCell, PositionMap origin, double cellResolution) :
    height(height), width(width), emptyCell(emptyCell), origin(origin), cellResolution(cellResolution)
{
    // Reserve space in vector for 2d array
    mapVector.resize(height);
    
    // Transform 1d array into 2d
    for(int i = 0; i < height * width; i++) {
        mapVector.at(i / width).push_back(data.at(i));
    }
}

void Map::debugMap() {
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            std::cout << mapVector.at(i).at(j) << " ";
        }
        std::cout << std::endl;
    }
}

std::vector<signed char> Map::getFlattenedMap() {
    std::vector<signed char> flattened;
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            flattened.push_back(mapVector.at(i).at(j));
        }
    }
    return flattened;
}


void Map::expandAroundException(int x, int y, int radius, bool exceptions[]) {
    int topLeftX = (x - radius < 0) ? 0 : x - radius;
    int topLeftY = (y - radius < 0) ? 0 : y - radius;
    int bottomRightX = (x + radius >= width) ? width - 1 : x + radius;
    int bottomRightY = (y + radius >= height) ? height - 1 : y + radius;

    for(int z = topLeftY; z <= bottomRightY; z++) {
        for(int x = topLeftX; x <= bottomRightX; x++) {
            if(mapVector.at(z).at(x) != emptyCell)
            exceptions[z * width  + x] = true;
        }
    }
}

void Map::inflateObsticles(double robotWidth, double robotHeight) {
    int robotWidthInPixels = std::ceil(robotWidth / cellResolution);
    int robotHeightInPixels = std::ceil(robotHeight / cellResolution);
    // int hypo = std::max(robotWidthInPixels, robotHeightInPixels);
    int radius = std::ceil(sqrt(robotWidth * robotWidth + robotHeight * robotHeight) / cellResolution);
    int exceptionRadius = radius * 2.5;
    
    int maxRobotSize = std::max(robotWidthInPixels, robotHeightInPixels);
    std::vector<std::vector<int>> newMap = mapVector;
    
    // Find inflate exceptions

    // Set all current Y empty spaces to 0
    int currentEmptySpaceInYDirection[width] = { 0 };
    bool exceptions[height][width] = { false };
    bool exceptionsReal[height][width] = { false };

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            if(mapVector.at(i).at(j) == emptyCell) {
                currentEmptySpaceInYDirection[j] += 1;
            } else {
                // If we found space which is smaller than double inflation,
                // but bigger than robots height add ends to exceptions
                if(currentEmptySpaceInYDirection[j] < radius * 2 &&
                    currentEmptySpaceInYDirection[j] >= robotHeightInPixels) {
                        exceptions[i - currentEmptySpaceInYDirection[j] - 1][j] = true;
                        exceptions[i][j] = true;
                        exceptionsReal[i - currentEmptySpaceInYDirection[j] - 1][j] = true;
                        exceptionsReal[i][j] = true;

                    // Find begining of the exception
                    if(j-1 >= 0 && exceptions[i][j-1] == false) {
                        // Expand everything about top exception point
                        expandAroundException(j, i, exceptionRadius, exceptionsReal[0]);
                        // Expand everything about bottom exception point
                         expandAroundException(j, i - currentEmptySpaceInYDirection[j] - 1, exceptionRadius, exceptionsReal[0]);

                    }

                }
                currentEmptySpaceInYDirection[j] = 0;
            }

            // Find end of the exception
            if(j-1 >= 0 && exceptions[i][j] == false && exceptions[i][j - 1]) {
                // Expand everything about top exception point
                expandAroundException(j - 1, i, exceptionRadius, exceptionsReal[0]);
                // Expand everything about bottom exception point
                expandAroundException(j - 1, i - currentEmptySpaceInYDirection[j] - 1, exceptionRadius, exceptionsReal[0]);
            }
        }
    }

    // Inflating obstacles
    int radiusCopy;
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            // Find cell which is not empty and 
            if(mapVector.at(i).at(j) != emptyCell) {

                if(exceptionsReal[i][j] == true) {
                    radiusCopy = maxRobotSize / 2 + 1; 
                } else {
                    radiusCopy = radius;
                }

                // Check that neighbour pixels exists
                if((i-1 > 0) && (i+1 < height -1) && (j-1 > 0) && (j+1 < width -1)) {
                    // Don't inflate pixels that have all neighbours
                    if(mapVector.at(i).at(j-1) != emptyCell &&
                        mapVector.at(i).at(j+1) != emptyCell &&
                        mapVector.at(i-1).at(j) != emptyCell &&
                        mapVector.at(i+1).at(j) != emptyCell) {
                        continue;
                    }
                }


                // Find top left and bottom right corners
                int topLeftX = (j - radiusCopy < 0) ? 0 : j - radiusCopy;
                int topLeftY = (i - radiusCopy < 0) ? 0 : i - radiusCopy;
                int bottomRightX = (j + radiusCopy >= width) ? width - 1 : j + radiusCopy;
                int bottomRightY = (i + radiusCopy >= height) ? height -1 : i + radiusCopy;

                if(exceptionsReal[i][j] == true) {
                    topLeftX -= radiusCopy * 2;
                    bottomRightX += radiusCopy * 2;
                }
                
                for(int z = topLeftY; z <= bottomRightY; z++) {
                    for(int x = topLeftX; x <= bottomRightX; x++) {
                        newMap.at(z).at(x) = -1;// mapVector.at(i).at(j)
                    }
                }
            }
        }
    }
    mapVector = newMap;
}

bool Map::obstacleExists(const Position &position) {
    if (position.x < 0 || position.x >= width ||
        position.y < 0 || position.y >= height ||
        mapVector.at(position.y).at(position.x) != emptyCell) {
        return true;
    }
    return false;
}


Position Map::transformMapPositionToGridPosition(const PositionMap &positionInMap) {
    int gridPositionX = (positionInMap.x + fabs(origin.x)) / cellResolution;
    int gridPositionY = (positionInMap.y + fabs(origin.y)) / cellResolution;
    return Position(gridPositionX, gridPositionY);
}

// A* implementation

AStarAlgorithm::AStarAlgorithm(Map map) :
map(map) {};

// Try to find neighbour on the heap and return pointer to it
Node* AStarAlgorithm::findNeighbour(std::vector<Node*> &nodeHeap, Position &position) {
    for(auto node : nodeHeap) {
        if(node->position == position) {
            return node;
        }
    }
    return nullptr;
}
struct HeapCompare {
    bool operator() (Node* &n1, Node* &n2) const {
        if(n1->totalCost() >= n2->totalCost()) return true;
        return false;
    }
};

std::vector<Position> AStarAlgorithm::findPath(const Position &startPosition, const Position &goalPosition) {
    Node *currentNode = nullptr;
    std::vector<Node*> openHeap;
    
    // Insert start node to heap
    openHeap.emplace(openHeap.begin(), new Node(startPosition));
    
    // Create cache table for closed nodes and set every node to null
    std::vector<std::vector<Node*>> closed;
    closed.resize(map.height);
    for(int i = 0; i < map.height; i++) {
        for(int j = 0; j < map.width; j++) {
            closed.at(i).push_back(nullptr);
        }
    }
    
    while (!openHeap.empty()) {
        
        // Get node which is currently closest to goal
        currentNode = openHeap.front();
        
        // If current node is equal to goal, stop search
        if (currentNode->position == goalPosition) {
            break;
        }
        
        // Insert current node to closed list and remove it from openset
        // because we are going to expand that node and carry search from neighbours
        closed.at(currentNode->position.y).at(currentNode->position.x) = currentNode;
        // Move biggest element to the end and remove it
        std::pop_heap(openHeap.begin(), openHeap.end(), HeapCompare());
        openHeap.pop_back();
        
        for (int i = 0; i < directions; i++) {
            Position neighbourPosition(currentNode->position.x + direction[i][0], currentNode->position.y + direction[i][1]);
            // If neighbour position is blocked continue try other neighbour
            if(map.obstacleExists(neighbourPosition) || closed.at(neighbourPosition.y).at(neighbourPosition.x)) {
                continue;
            }
            
            // Last 4 positions are diagonal so it costs sqrt(2) to reach, first 4
            // positions costs sqrt(1)
            int totalCost = currentNode->costToReach + ((i < 4) ? 10 : 14);
            
            Node *neighbour = findNeighbour(openHeap, neighbourPosition);
            if(neighbour == nullptr) {
                // If node doesn't exists create it and make current node
                // its parent
                neighbour = new Node(neighbourPosition, currentNode);
                neighbour->costToReach = totalCost;
                neighbour->costToGoal = neighbour->estimatedDistance(goalPosition);
                openHeap.push_back(neighbour);
                std::push_heap(openHeap.begin(), openHeap.end(), HeapCompare());
                
            } else if(totalCost < neighbour->costToReach) {
                neighbour->parent = currentNode;
                neighbour->costToReach = totalCost;
            }
        }
    }
    
    std::vector<Position> path;
    std::cout << currentNode->costToReach << " and to goal" << currentNode->costToGoal << std::endl;
    while (currentNode != nullptr) {
        path.push_back(currentNode->position);
        currentNode = currentNode->parent;
    }
    std::reverse(path.begin(), path.end());
    freeMemory(closed);
    return path;
}

void AStarAlgorithm::freeMemory(std::vector<std::vector<Node*>> nodesCache) {
    for(auto &vector : nodesCache) {
        for(auto &node : vector) {
            if(node != nullptr)
                delete node;
        }
    }
}